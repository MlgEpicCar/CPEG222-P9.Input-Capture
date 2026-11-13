// ****************************************************************
// * servo.c - Parallax Feedback 360 servo helper
// *   TIM4 CH1/CH3 : 50 Hz servo PWM on PD12 (left) / PD14 (right)
// *   TIM1 CH1-CH4 : input capture of the feedback signals
// *                  PE13 (left, CH3 rise + CH4 fall)
// *                  PE9  (right, CH1 rise + CH2 fall)
// ****************************************************************
#include "stm32f4xx.h"
#include "servo.h"

#define STOP_US        1500
#define MAX_CORR_US    220      /* 1500 +/- 220 = 1280..1720 = full speed   */
#define MAX_RPM        140

#define CYCLE_NOMINAL  1099     /* 910 Hz feedback period in us             */
#define CYCLE_TOL      150      /* ignore cycles outside nominal +/- this   */

typedef struct {
    int8_t   dir;               /* +1 forward, -1 reverse, 0 stop           */
    int8_t   sign;              /* +1: forward = pulse above 1500 us        */
    uint16_t correction;        /* 0..220 us away from 1500                 */
    uint16_t pulse;             /* pulse currently sent (us)                */
    uint16_t rpm;               /* last measured speed                      */
    /* written by capture ISR */
    volatile int32_t angle;     /* tenths of a degree, 0..3600              */
    volatile int32_t delta_sum; /* unwrapped angle change since last read   */
    volatile uint8_t have_angle;
} Servo;

static Servo servo[2] = {
    { 0, +1, 0, STOP_US, 0, 0, 0, 0 },   /* left : forward is above 1500   */
    { 0, -1, 0, STOP_US, 0, 0, 0, 0 }    /* right: mounted flipped         */
};

static void WritePulse(uint8_t i)
{
    if (i == SERVO_LEFT) TIM4->CCR1 = servo[i].pulse;
    else                 TIM4->CCR3 = servo[i].pulse;
}

static uint16_t FeedForward(uint16_t target_rpm)
{
    if (target_rpm > MAX_RPM) target_rpm = MAX_RPM;
    return (uint16_t)(((uint32_t)target_rpm * MAX_CORR_US + MAX_RPM / 2U) / MAX_RPM);
}

void Servo_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    /* ---- PWM: PD12 = TIM4_CH1, PD14 = TIM4_CH3, AF2 ---- */
    GPIOD->MODER   &= ~((3U << 24) | (3U << 28));
    GPIOD->MODER   |=  ((2U << 24) | (2U << 28));
    GPIOD->AFR[1]  &= ~((0xFU << 16) | (0xFU << 24));
    GPIOD->AFR[1]  |=  ((2U << 16) | (2U << 24));

    TIM4->CR1   = 0;
    TIM4->PSC   = 89U;                 /* 90 MHz / 90 = 1 MHz (1 us tick)   */
    TIM4->ARR   = 19999U;              /* 20 ms period = 50 Hz              */
    TIM4->CCR1  = STOP_US;
    TIM4->CCR3  = STOP_US;
    TIM4->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE;   /* PWM mode 1 */
    TIM4->CCMR2 = (6U << 4) | TIM_CCMR2_OC3PE;
    TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* ---- Feedback: PE9 = TIM1_CH1, PE13 = TIM1_CH3, AF1 ---- */
    GPIOE->MODER   &= ~((3U << 18) | (3U << 26));
    GPIOE->MODER   |=  ((2U << 18) | (2U << 26));
    GPIOE->PUPDR   &= ~((3U << 18) | (3U << 26));
    GPIOE->AFR[1]  &= ~((0xFU << 4) | (0xFU << 20));
    GPIOE->AFR[1]  |=  ((1U << 4) | (1U << 20));

    TIM1->CR1   = 0;
    TIM1->PSC   = 179U;                /* 180 MHz / 180 = 1 MHz             */
    TIM1->ARR   = 0xFFFFU;
    /* CH1/CH3 -> their own pin (rising); CH2/CH4 -> same pin, indirect (falling) */
    TIM1->CCMR1 = (1U << 0) | (2U << 8);
    TIM1->CCMR2 = (1U << 0) | (2U << 8);
    TIM1->CCER  = TIM_CCER_CC1E | (TIM_CCER_CC2E | TIM_CCER_CC2P) |
                  TIM_CCER_CC3E | (TIM_CCER_CC4E | TIM_CCER_CC4P);
    TIM1->DIER  = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->SR    = 0;
    NVIC_SetPriority(TIM1_CC_IRQn, 2);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    TIM1->CR1   = TIM_CR1_CEN;
}

/* ---- called from the TIM1 ISR: turn one feedback cycle into an angle ---- */
void Servo_ProcessCapture(uint8_t i, uint16_t rise, uint16_t fall, uint16_t prev_rise)
{
    uint16_t cycle = (uint16_t)(rise - prev_rise);
    uint16_t high  = (uint16_t)(fall - rise);

    if (cycle < (CYCLE_NOMINAL - CYCLE_TOL) || cycle > (CYCLE_NOMINAL + CYCLE_TOL) ||
        high >= cycle)
        return;                                   /* not a clean cycle: skip */

    int32_t duty  = ((int32_t)high * 1000) / cycle;        /* tenths of a % */
    int32_t angle = ((duty - 29) * 3600) / 942;            /* tenths of deg */
    if (angle < 0)    angle = 0;
    if (angle > 3600) angle = 3600;

    Servo *s = &servo[i];
    if (s->have_angle) {
        int32_t d = angle - s->angle;
        if (d >  1800) d -= 3600;                 /* crossed 360 -> 0 */
        if (d < -1800) d += 3600;                 /* crossed 0 -> 360 */
        s->delta_sum += d;
    }
    s->angle = angle;
    s->have_angle = 1;
}

void Servo_SetDirection(uint8_t i, int8_t dir)
{
    if (i > 1U) return;
    Servo *s = &servo[i];
    if (dir != s->dir) {
        s->dir = dir;
        s->correction = 0;                        /* re-seed on next update */
        s->pulse = STOP_US;
        WritePulse(i);
    }
}

void Servo_Update(uint16_t target_rpm)
{
    for (uint8_t i = 0; i < 2U; i++) {
        Servo *s = &servo[i];

        /* 1) measure: unwrapped angle moved during the last 100 ms */
        __disable_irq();
        int32_t delta = s->delta_sum;
        s->delta_sum = 0;
        __enable_irq();
        if (delta < 0) delta = -delta;
        /* tenths-deg per 0.1 s -> deg/s numerically; rpm = deg/s / 6 */
        uint32_t rpm = ((uint32_t)delta + 3U) / 6U;
        s->rpm = (uint16_t)(rpm > 999U ? 999U : rpm);

        /* 2) control */
        if (s->dir == 0 || target_rpm == 0U) {
            s->correction = 0;
            s->pulse = STOP_US;
        } else {
            if (s->correction == 0U)              /* just started: jump to estimate */
                s->correction = FeedForward(target_rpm);

            int32_t err  = (int32_t)target_rpm - (int32_t)s->rpm;
            int32_t step = (err * 2) / 5;         /* ~1.6 us per rpm * 0.25 gain */
            if (step == 0 && err != 0) step = (err > 0) ? 1 : -1;
            if (step >  15) step =  15;
            if (step < -15) step = -15;

            int32_t c = (int32_t)s->correction + step;
            if (c < 0) c = 0;
            if (c > MAX_CORR_US) c = MAX_CORR_US;
            if (c == 0) c = 1;                    /* keep "started" state */
            s->correction = (uint16_t)c;

            int32_t sign = s->sign * s->dir;      /* +1: above 1500, -1: below */
            s->pulse = (uint16_t)(STOP_US + sign * (int32_t)s->correction);
        }
        WritePulse(i);
    }
}

uint16_t Servo_GetPulseUs(uint8_t i) { return (i > 1U) ? STOP_US : servo[i].pulse; }
uint16_t Servo_GetRPM(uint8_t i)     { return (i > 1U) ? 0U : servo[i].rpm; }