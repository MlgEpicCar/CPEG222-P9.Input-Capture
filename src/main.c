// ****************************************************************
// * Carlos Munar
// * Project 9 - Input Capture - 11/13/2026
// ****************************************************************

#include "stm32f4xx.h"
#include "SysClock180MHz.h"
#include "OLED.h"
#include "ssd.h"
#include "servo.h"

#include <stdint.h>
#include <stdio.h>

/* ---------------- Buttons (active low) ---------------- */
#define BTN_UP      (1U << 0)   /* PF7 */
#define BTN_DOWN    (1U << 1)   /* PE5 */
#define BTN_LEFT    (1U << 2)   /* PF9 */
#define BTN_RIGHT   (1U << 3)   /* PE6 */
#define BTN_CENTER  (1U << 4)   /* PF8 */

typedef enum { MODE_STOP, MODE_FORWARD, MODE_REVERSE,
               MODE_LEFT_TURN, MODE_RIGHT_TURN } Mode;

static Mode    mode = MODE_STOP;
static uint8_t tank = 0;                 /* 1 = second consecutive LEFT/RIGHT */
static uint32_t last_press = 0;          /* last button counted (for "consecutive") */

static volatile uint32_t ms_ticks = 0;
static volatile uint8_t  control_due = 0;
static volatile uint8_t  display_due = 0;

static uint16_t target_rpm = 0;

/* ---------------- Interrupt handlers ---------------- */
void SysTick_Handler(void)
{
    ms_ticks++;
    if (ms_ticks % 100U == 0U) control_due = 1;
    if (ms_ticks % 150U == 0U) display_due = 1;
}

void TIM5_IRQHandler(void)               /* 100 Hz SSD multiplex refresh */
{
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

void TIM1_CC_IRQHandler(void)            /* servo feedback capture */
{
    /* Feedback3 (right): CH1 rising, CH2 falling.  Feedback1 (left): CH3 rising, CH4 falling. */
    static uint16_t r_now, r_prev, l_now, l_prev;
    static uint8_t  r_cnt, l_cnt;

    if (TIM1->SR & TIM_SR_CC1IF) {
        r_prev = r_now;
        r_now  = (uint16_t)TIM1->CCR1;   /* reading CCR clears the flag */
        if (r_cnt < 2U) r_cnt++;
    }
    if (TIM1->SR & TIM_SR_CC2IF) {
        uint16_t fall = (uint16_t)TIM1->CCR2;
        if (r_cnt >= 2U) Servo_ProcessCapture(SERVO_RIGHT, r_now, fall, r_prev);
    }
    if (TIM1->SR & TIM_SR_CC3IF) {
        l_prev = l_now;
        l_now  = (uint16_t)TIM1->CCR3;
        if (l_cnt < 2U) l_cnt++;
    }
    if (TIM1->SR & TIM_SR_CC4IF) {
        uint16_t fall = (uint16_t)TIM1->CCR4;
        if (l_cnt >= 2U) Servo_ProcessCapture(SERVO_LEFT, l_now, fall, l_prev);
    }
}

/* ---------------- Init helpers ---------------- */
static void SysTick_Start(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
}

static void SSD_Timer_Start(void)        /* TIM5: 90 MHz / 9000 / 100 = 100 Hz */
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->CR1  = 0;
    TIM5->PSC  = 8999U;
    TIM5->ARR  = 9U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1  = TIM_CR1_CEN;
}

static void Buttons_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN;
    GPIOF->MODER &= ~((3U << 14) | (3U << 16) | (3U << 18));   /* PF7 PF8 PF9 */
    GPIOF->PUPDR &= ~((3U << 14) | (3U << 16) | (3U << 18));
    GPIOF->PUPDR |=  ((1U << 14) | (1U << 16) | (1U << 18));
    GPIOE->MODER &= ~((3U << 10) | (3U << 12));                /* PE5 PE6 */
    GPIOE->PUPDR &= ~((3U << 10) | (3U << 12));
    GPIOE->PUPDR |=  ((1U << 10) | (1U << 12));
}

static void Pot_Init(void)               /* PC2 = ADC3_IN12 */
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    GPIOC->MODER |= (3U << 4);           /* analog */
    ADC->CCR   = (ADC->CCR & ~(3U << 16)) | (1U << 16);   /* PCLK2/4 */
    ADC3->CR1  = 0;
    ADC3->CR2  = 0;
    ADC3->SMPR1 |= (7U << 6);            /* ch12: 480 cycles */
    ADC3->SQR1 = 0;                      /* 1 conversion */
    ADC3->SQR3 = 12U;
    ADC3->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++) {}   /* ADC wake-up time */
}

static uint16_t Pot_ReadRaw(void)
{
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC3->DR;
}

/* ---------------- Buttons: polled, debounced, lock until release ---------------- */
static uint32_t Buttons_Read(void)       /* returns bitmask, 1 = pressed */
{
    uint32_t b = 0;
    if (!(GPIOF->IDR & (1U << 7))) b |= BTN_UP;
    if (!(GPIOE->IDR & (1U << 5))) b |= BTN_DOWN;
    if (!(GPIOF->IDR & (1U << 9))) b |= BTN_LEFT;
    if (!(GPIOE->IDR & (1U << 6))) b |= BTN_RIGHT;
    if (!(GPIOF->IDR & (1U << 8))) b |= BTN_CENTER;
    return b;
}

/* Returns one new button press (0 if none). A button must be released
   before it can fire again (the "lock"), and must be stable for 25 ms. */
static uint32_t Buttons_GetPress(void)
{
    static uint32_t prev = 0, stable = 0, locked = 0, changed_at = 0;
    uint32_t now = Buttons_Read();

    if (now != prev) { prev = now; changed_at = ms_ticks; }
    if ((ms_ticks - changed_at) < 25U || now == stable) return 0;

    stable = now;
    locked &= now;                       /* released buttons unlock */
    uint32_t fresh = now & ~locked;
    if (!fresh) return 0;
    uint32_t press = fresh & (~fresh + 1U);   /* lowest set bit */
    locked |= press;
    return press;
}

/* ---------------- Mode logic ---------------- */
static void HandlePress(uint32_t btn)
{
    if (btn == BTN_LEFT || btn == BTN_RIGHT) {
        if (last_press == btn) {         /* consecutive press of same button */
            if (tank) return;            /* third+ press: ignored */
            tank = 1;
        } else {
            tank = 0;
        }
        last_press = btn;
        mode = (btn == BTN_LEFT) ? MODE_LEFT_TURN : MODE_RIGHT_TURN;
    } else {
        last_press = btn;                /* any other button cancels the sequence */
        tank = 0;
        if      (btn == BTN_UP)     mode = MODE_FORWARD;
        else if (btn == BTN_DOWN)   mode = MODE_REVERSE;
        else if (btn == BTN_CENTER) mode = MODE_STOP;
    }
}

static void ApplyMode(void)
{
    int8_t l = 0, r = 0;
    switch (mode) {
    case MODE_FORWARD:    l =  1; r =  1; break;
    case MODE_REVERSE:    l = -1; r = -1; break;
    case MODE_LEFT_TURN:  l = tank ? -1 : 0; r = 1; break;   /* tank: left reverses   */
    case MODE_RIGHT_TURN: l = 1; r = tank ? -1 : 0; break;   /* tank: right reverses  */
    default: break;
    }
    Servo_SetDirection(SERVO_LEFT,  l);
    Servo_SetDirection(SERVO_RIGHT, r);
}

/* ---------------- Display ---------------- */
static void Oled_Refresh(void)
{
    const char *name = "STOP";
    switch (mode) {
    case MODE_FORWARD:    name = "FORWARD";                               break;
    case MODE_REVERSE:    name = "REVERSE";                               break;
    case MODE_LEFT_TURN:  name = tank ? "TANK LEFT"  : "LEFT TURN";       break;
    case MODE_RIGHT_TURN: name = tank ? "TANK RIGHT" : "RIGHT TURN";      break;
    default: break;
    }
    char buf[24];
    snprintf(buf, sizeof buf, "      %-20s", name);
    OLED_Print(3, 0, buf);
    snprintf(buf, sizeof buf, "L PWM %4u RPM %3u  ", Servo_GetPulseUs(SERVO_LEFT),  Servo_GetRPM(SERVO_LEFT));
    OLED_Print(6, 0, buf);
    snprintf(buf, sizeof buf, "R PWM %4u RPM %3u  ", Servo_GetPulseUs(SERVO_RIGHT), Servo_GetRPM(SERVO_RIGHT));
    OLED_Print(7, 0, buf);
}

int main(void)
{
    SystemClock_Config();                /* 180 MHz */
    SysTick_Start();

    OLED_Init();
    OLED_Clear();
    SSD_Init();
    SSD_Timer_Start();
    Buttons_Init();
    Pot_Init();
    Servo_Init();

    SSD_DisplayValue(0);
    Oled_Refresh();

    OLED_Print(0, 0, "CPEG222-F26 Project 9");
    OLED_Print(1, 0, "Carlos Munar 11/13/26");

    while (1) {
        uint32_t press = Buttons_GetPress();
        if (press) {
            HandlePress(press);
            ApplyMode();
        }

        if (control_due) {
            control_due = 0;

            uint32_t rpm = ((uint32_t)Pot_ReadRaw() * 140U + 2047U) / 4095U;
            target_rpm = (rpm < 5U) ? 0U : (uint16_t)rpm;
            SSD_DisplayValue(target_rpm);

            Servo_Update(target_rpm);

            char buf[24];
            snprintf(buf, sizeof buf, "Target RPM: %3u", target_rpm);
            OLED_Print(5, 0, buf);
        }

        if (display_due) {
            display_due = 0;
            Oled_Refresh();
        }
    }
}