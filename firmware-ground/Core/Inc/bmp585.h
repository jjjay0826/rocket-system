/*
 * bmp585.h — BMP5 family SPI driver (BMP585 / BMP581)
 */
#ifndef __BMP585_H
#define __BMP585_H

#include "stm32f4xx_hal.h"

/* BMP585 CS 腳位：共用 SPI3 (PB12=SCK, PB4=MISO, PB5=MOSI)
   如果你的 CSB 接的不是 PB10，改這裡 */
#define BMP_CS_PORT  GPIOB
#define BMP_CS_PIN   GPIO_PIN_0

uint8_t  BMP585_Init(SPI_HandleTypeDef *hspi); // 回傳 chip_id (0x51=BMP585, 0x50=BMP581, 0=無回應)
float    BMP585_ReadPressure(void);            // 回傳氣壓 (hPa)
float    BMP585_ReadAltitude(float seaLevel_hPa); // 計算高度 (m)
uint32_t BMP585_GetLastRaw(void);              // 回傳最後一次讀到的 24-bit 原始值（偵錯用）

#endif
