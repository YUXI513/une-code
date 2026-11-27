#include "oled.h"
#include "oled_font.h"
#include <string.h>

#define OLED_WIDTH  128U
#define OLED_PAGES  8U

static uint8_t text_target[OLED_PAGES][OLED_WIDTH];
static uint8_t text_shown[OLED_PAGES][OLED_WIDTH];
static uint32_t text_cursor;

/* Five columns, low bit at top. Digits followed by uppercase A-Z. */
static const uint8_t text_glyphs[36][5] = {
    {0x3e,0x51,0x49,0x45,0x3e}, {0x00,0x42,0x7f,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3c,0x4a,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1e},
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a}, {0x7f,0x08,0x08,0x08,0x7f},
    {0x00,0x41,0x7f,0x41,0x00}, {0x20,0x40,0x41,0x3f,0x01},
    {0x7f,0x08,0x14,0x22,0x41}, {0x7f,0x40,0x40,0x40,0x40},
    {0x7f,0x02,0x0c,0x02,0x7f}, {0x7f,0x04,0x08,0x10,0x7f},
    {0x3e,0x41,0x41,0x41,0x3e}, {0x7f,0x09,0x09,0x09,0x06},
    {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7f,0x01,0x01},
    {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f},
    {0x3f,0x40,0x38,0x40,0x3f}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static uint8_t TextColumn(char ch, uint32_t column)
{
    if (column == 5U) return 0U;
    if (ch >= '0' && ch <= '9') return text_glyphs[ch - '0'][column];
    if (ch >= 'A' && ch <= 'Z') return text_glyphs[ch - 'A' + 10][column];
    if (ch == '-') return 0x08U;
    if (ch == '~') return column == 1U ? 0x04U : (column == 3U ? 0x10U : 0x08U);
    if (ch == '+') return column == 2U ? 0x1cU : 0x08U;
    if (ch == '.') return column == 2U ? 0x60U : 0U;
    if (ch == ':') return column == 2U ? 0x36U : 0U;
    return 0U;
}

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
    memset(text_target, 0, sizeof(text_target));
    memset(text_shown, 0, sizeof(text_shown));
    text_cursor = 0U;
    for (uint8_t page = 0U; page < OLED_PAGES; ++page)
    {
        OLED_SetPosition(0U, page);
        for (uint32_t column = 0U; column < OLED_WIDTH; ++column)
        {
            OLED_WriteData(0x00U);
        }
    }
}

void OLED_SetTextLine(uint8_t row, const char *text)
{
    if (row >= OLED_PAGES || text == NULL) return;
    memset(text_target[row], 0, sizeof(text_target[row]));
    uint32_t x = 0U;
    const unsigned char *cursor = (const unsigned char *)text;
    while (*cursor != '\0' && x < OLED_WIDTH)
    {
        const uint8_t *glyph = NULL;
        uint32_t bytes = 1U;
        if (strncmp((const char *)cursor, "\xE8\xBE\x93", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[0], bytes = 3U; /* 输 */
        else if (strncmp((const char *)cursor, "\xE5\x85\xA5", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[1], bytes = 3U; /* 入 */
        else if (strncmp((const char *)cursor, "\xE7\x94\xB5", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[2], bytes = 3U; /* 电 */
        else if (strncmp((const char *)cursor, "\xE5\x8E\x8B", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[3], bytes = 3U; /* 压 */
        else if (strncmp((const char *)cursor, "\xE6\xB5\x81", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[4], bytes = 3U; /* 流 */
        else if (strncmp((const char *)cursor, "\xE5\x87\xBA", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[5], bytes = 3U; /* 出 */
        else if (strncmp((const char *)cursor, "\xEF\xBC\x9A", 3U) == 0)
            glyph = oled_measurement_chinese_glyphs[6], bytes = 3U; /* ： */

        if (glyph != NULL)
        {
            for (uint32_t column = 0U; column < 8U && x < OLED_WIDTH; ++column)
                text_target[row][x++] = glyph[column];
            cursor += bytes;
        }
        else if (*cursor < 0x80U)
        {
            char ch = (char)*cursor++;
            for (uint32_t column = 0U; column < 6U && x < OLED_WIDTH; ++column)
                text_target[row][x++] = TextColumn(ch, column);
        }
        else
        {
            /* Skip an unsupported UTF-8 code point without splitting its tail. */
            cursor += (cursor[0] & 0xF8U) == 0xF0U ? 4U
                    : (cursor[0] & 0xF0U) == 0xE0U ? 3U
                    : (cursor[0] & 0xE0U) == 0xC0U ? 2U : 1U;
        }
    }
}

void OLED_Process(void)
{
    /* Transfer one changed page column at a time. The main-loop scheduler
     * gives ADC sampling priority over this work. */
    for (uint32_t scanned = 0U; scanned < OLED_PAGES * OLED_WIDTH; ++scanned)
    {
        uint32_t row = text_cursor / OLED_WIDTH;
        uint32_t column = text_cursor % OLED_WIDTH;
        text_cursor = (text_cursor + 1U) % (OLED_PAGES * OLED_WIDTH);
        uint8_t pixels = text_target[row][column];
        if (pixels == text_shown[row][column]) continue;
        OLED_SetPosition((uint8_t)column, (uint8_t)row);
        OLED_WriteData(pixels);
        text_shown[row][column] = pixels;
        return;
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
