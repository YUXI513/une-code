#include "oled.h"
#include "oled_font.h"

#define OLED_WIDTH  128U
#define OLED_PAGES  8U

static void OLED_SPI_Delay(void)
{
    /* Scale with SYSCLK so the 168 MHz PLL does not overclock software SPI.
     * The NOPs alone provide at least 1 us; loop overhead adds timing margin.
     */
    uint32_t cycles = (SystemCoreClock + 999999U) / 1000000U;
    while (cycles-- != 0U)
    {
        __NOP();
    }
}

static void OLED_WriteByte(uint8_t value)
{
    /* Idle-low clock, MSB first, data sampled on the rising edge. */
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
        HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin,
                          (value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        OLED_SPI_Delay();
        HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_SET);
        OLED_SPI_Delay();
        HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_RESET);
        OLED_SPI_Delay();
        value <<= 1;
    }
}

static void OLED_WriteCommand(uint8_t command)
{
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    OLED_WriteByte(command);
}

static void OLED_WriteData(uint8_t data)
{
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    OLED_WriteByte(data);
}

static void OLED_SetPosition(uint8_t column, uint8_t page)
{
    OLED_WriteCommand((uint8_t)(0xB0U | page));
    OLED_WriteCommand((uint8_t)(column & 0x0FU));
    OLED_WriteCommand((uint8_t)(0x10U | (column >> 4)));
}

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Hold the display in reset while configuring its interface. */
    HAL_GPIO_WritePin(GPIOB, OLED_SCK_Pin | OLED_DC_Pin | OLED_RES_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = OLED_SCK_Pin | OLED_DC_Pin | OLED_RES_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = OLED_MOSI_Pin;
    HAL_GPIO_Init(OLED_MOSI_GPIO_Port, &gpio);
}

void OLED_Clear(void)
{
    for (uint8_t page = 0U; page < OLED_PAGES; ++page)
    {
        OLED_SetPosition(0U, page);
        for (uint32_t column = 0U; column < OLED_WIDTH; ++column)
        {
            OLED_WriteData(0x00U);
        }
    }
}

void OLED_Init(void)
{
    /* Manufacturer's internal charge-pump settings, page addressing mode.
     * Datasheet: X096-2864KSWPG01-H30, section 4.4.2:
     * https://atta.szlcsc.com/upload/public/pdf/source/20231103/46D2C64E8B6339907535582F5378E168.pdf
     */
    static const uint8_t commands[] = {
        0xAE,             /* Display off during initialization. */
        0xD5, 0x90,       /* Display clock. */
        0xA8, 0x3F,       /* 64 multiplexed rows. */
        0xD3, 0x00,       /* No display offset. */
        0x40,             /* Start at row 0. */
        0xA1,             /* Segment remap. */
        0xC8,             /* Reverse COM scan direction. */
        0xDA, 0x12,       /* COM pin configuration for 128 x 64. */
        0x81, 0xB0,       /* Manufacturer's contrast setting. */
        0xD9, 0x22,       /* Pre-charge period. */
        0xDB, 0x30,       /* VCOMH level. */
        0x20, 0x02,       /* Page addressing mode. */
        0xA4,             /* Display follows RAM. */
        0xA6,             /* White pixels on a black background. */
        0x2E,             /* Disable scrolling. */
        0x8D, 0x14        /* Enable the internal charge pump. */
    };

    OLED_GPIO_Init();
    HAL_Delay(100U);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(10U);

    for (uint32_t index = 0U; index < sizeof(commands); ++index)
    {
        OLED_WriteCommand(commands[index]);
    }

    OLED_Clear();
    HAL_Delay(100U);
    OLED_WriteCommand(0xAFU);
}

void OLED_ShowOpenLoopDebug(void)
{
    /* Four 16 x 16 glyphs centered at x=32, y=24 on the 128 x 64 panel. */
    for (uint8_t half = 0U; half < 2U; ++half)
    {
        OLED_SetPosition(32U, (uint8_t)(3U + half));
        for (uint32_t glyph = 0U; glyph < 4U; ++glyph)
        {
            for (uint32_t column = 0U; column < 16U; ++column)
            {
                OLED_WriteData(oled_open_loop_glyphs[glyph][half * 16U + column]);
            }
        }
    }
}
