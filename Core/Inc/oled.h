#ifndef OLED_H
#define OLED_H

#include "stm32g4xx_hal.h"

/* X096-2864KSWPG01-H30 / SSD1315, schematic page 1.
 * Four-wire SPI: BS0/BS1/BS2 and CS# are wired to GND.
 * Software SPI keeps this driver independent of CubeMX SPI configuration.
 */
#define OLED_SCK_GPIO_Port   GPIOB
#define OLED_SCK_Pin         GPIO_PIN_3
#define OLED_MOSI_GPIO_Port  GPIOC
#define OLED_MOSI_Pin        GPIO_PIN_12
#define OLED_DC_GPIO_Port    GPIOB
#define OLED_DC_Pin          GPIO_PIN_5
#define OLED_RES_GPIO_Port   GPIOB
#define OLED_RES_Pin         GPIO_PIN_9

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowOpenLoopDebug(void);
/* 21 characters per 8-pixel text row. Queue text, then service one dirty
 * character per call from main; never refresh a full screen in an ISR. */
void OLED_SetTextLine(uint8_t row, const char *text);
void OLED_Process(void);

#endif /* OLED_H */
