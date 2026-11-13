
#include "stm32f4xx.h"
#include "ssd.h"

// Segment GPIO pins
#define PIN_A 9 // Port G
#define PIN_B 12 // Port F
#define PIN_C 13 // Port F
#define PIN_D 14 // Port G
#define PIN_E 8 // Port E
#define PIN_F 15 // Port F
#define PIN_G 4 // Port B
#define PIN_DecimalPoint 14 // Port F

// Digit select pins
#define PIN_DIGIT0 10 // Port E
#define PIN_DIGIT1 7 // Port E
#define PIN_DIGIT2 5 // Port B
#define PIN_DIGIT3 3 // Port B

static const uint8_t digitSegments[11] = {
    //ABCDEFG
    0b1000000, // 0
    0b1111001, // 1
    0b0100100, // 2
    0b0110000, // 3
    0b0011001, // 4
    0b0010010, // 5
    0b0000010, // 6
    0b1111000, // 7
    0b0000000, // 8
    0b0010000, // 9
    0b1111111  // Completely Blank
};

static void SSD_SetSegments(uint8_t pattern)
{
    // Bit 0 = A, Bit 1 = B, ...
    // A zero turns a segment ON (active-low).

    // Segment A: PG9
    if (pattern & (1U << 0))
        GPIOG->BSRR = (1U << PIN_A);
    else
        GPIOG->BSRR = (1U << (PIN_A + 16));

    // Segment B: PF12
    if (pattern & (1U << 1))
        GPIOF->BSRR = (1U << PIN_B);
    else
        GPIOF->BSRR = (1U << (PIN_B + 16));

    // Segment C: PF13
    if (pattern & (1U << 2))
        GPIOF->BSRR = (1U << PIN_C);
    else
        GPIOF->BSRR = (1U << (PIN_C + 16));

    // Segment D: PG14
    if (pattern & (1U << 3))
        GPIOG->BSRR = (1U << PIN_D);
    else
        GPIOG->BSRR = (1U << (PIN_D + 16));

    // Segment E: PE8
    if (pattern & (1U << 4))
        GPIOE->BSRR = (1U << PIN_E);
    else
        GPIOE->BSRR = (1U << (PIN_E + 16));

    // Segment F: PF15
    if (pattern & (1U << 5))
        GPIOF->BSRR = (1U << PIN_F);
    else
        GPIOF->BSRR = (1U << (PIN_F + 16));

    // Segment G: PB4
    if (pattern & (1U << 6))
        GPIOB->BSRR = (1U << PIN_G);
    else
        GPIOB->BSRR = (1U << (PIN_G + 16));
}

static void SSD_DisableDigits(void)
{
    // HIGH disables each common-anode digit transistor.
    GPIOE->BSRR = (1U << PIN_DIGIT0) |
                  (1U << PIN_DIGIT1);

    GPIOB->BSRR = (1U << PIN_DIGIT2) |
                  (1U << PIN_DIGIT3);
}

void SSD_Init(void)
{
    // Enable GPIO clocks: B, E, F, G
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN|
                    RCC_AHB1ENR_GPIOEEN|
                    RCC_AHB1ENR_GPIOFEN|
                    RCC_AHB1ENR_GPIOGEN;

    // SET SHIT TO OUTPUT
    GPIOB->MODER &= ~((3U << (PIN_DIGIT2 * 2))|
                    (3U << (PIN_DIGIT3 * 2))|
                    (3U << (PIN_G * 2)));
    GPIOB->MODER |= (1U << (PIN_DIGIT2 * 2))|
                    (1U << (PIN_DIGIT3 * 2))|
                    (1U << (PIN_G * 2));

    GPIOE->MODER &= ~((3U << (PIN_DIGIT0 * 2))|
                    (3U << (PIN_DIGIT1 * 2))|
                    (3U << (PIN_E * 2)));
    GPIOE->MODER |= (1U << (PIN_DIGIT0 * 2))|
                    (1U << (PIN_DIGIT1 * 2))|
                    (1U << (PIN_E * 2));

    GPIOF->MODER &= ~((3U << (PIN_B * 2))|
                    (3U << (PIN_C * 2))|
                    (3U << (PIN_F * 2))|
                    (3U << (PIN_DecimalPoint * 2)));
    GPIOF->MODER |= (1U << (PIN_B * 2))|
                    (1U << (PIN_C * 2))|
                    (1U << (PIN_F * 2))|
                    (1U << (PIN_DecimalPoint * 2));

    GPIOG->MODER &= ~((3U << (PIN_A * 2))|
                    (3U << (PIN_D * 2)));
    GPIOG->MODER |= (1U << (PIN_A * 2))|
                    (1U << (PIN_D * 2));

    SSD_DisableDigits();
    SSD_SetSegments(0b1111111);
}

static uint8_t current_digit = 0;
static uint8_t digits[4];

void SSD_DisplayValue(uint16_t value)
{
    // Sets the item in the list (digits) to the coresponding digit of the 4 digit number (eg: 4532)
    if (((value / 1000) % 10) == 0) {
        digits[0] = 10; // Clears the left-most digit if it's 0
    } else {
        digits[0] = (value / 1000) % 10;
    }
    digits[1] = (value / 100) % 10;
    digits[2] = (value / 10) % 10;
    digits[3] = value % 10;
}

void SSD_Refresh(void)
{
    // Disable all digits
    SSD_DisableDigits();

    // Set the segments for the current digit
    SSD_SetSegments(digitSegments[digits[current_digit]]);

    // Turns off Decimal
    GPIOF->BSRR = (1U << PIN_DecimalPoint);

    // Enable the selected digit.
    switch (current_digit)
    {
        case 0:
            GPIOE->BSRR = (1U << (PIN_DIGIT0 + 16));
            break;
        case 1:
            GPIOE->BSRR = (1U << (PIN_DIGIT1 + 16));
            //GPIOF->BSRR = (1U << (PIN_DecimalPoint + 16));
            break;
        case 2:
            GPIOB->BSRR = (1U << (PIN_DIGIT2 + 16));
            break;
        case 3:
            GPIOB->BSRR = (1U << (PIN_DIGIT3 + 16));
            break;
    }

    current_digit = (current_digit + 1) % 4;
}