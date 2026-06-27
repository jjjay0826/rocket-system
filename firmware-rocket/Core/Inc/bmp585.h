/*
 * bmp585.h — BMP5 family SPI driver (BMP585 / BMP581)
 */
#ifndef __BMP585_H
#define __BMP585_H

#include "stm32f4xx_hal.h"
#include "main.h"      /* CubeMX User Label 生成的 BARO_CS_Pin / BARO_CS_GPIO_Port */

/* BMP585 CS 腳位 = CubeMX label「BARO_CS」(PB12)；換板只改 CubeMX，這裡自動跟著走 */
#define BMP_CS_PORT  BARO_CS_GPIO_Port
#define BMP_CS_PIN   BARO_CS_Pin

uint8_t  BMP585_Init(SPI_HandleTypeDef *hspi); // 回傳 chip_id (0x51=BMP585, 0x50=BMP581, 0=無回應)
float    BMP585_ReadPressure(void);            // 回傳氣壓 (hPa)
float    BMP585_ReadAltitude(float seaLevel_hPa); // 計算高度 (m)
uint32_t BMP585_GetLastRaw(void);              // 回傳最後一次讀到的 24-bit 原始值（偵錯用）

#endif
