/*
 * gnss.h
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#ifndef __GNSS_H
#define __GNSS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

extern char gnss_time[16];

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;    // km/h
    uint8_t num_sats;     // satellites used in fix (GGA field 7, 0 before fix)
    uint8_t sats_in_view; // satellites visible in sky (GSV, non-zero even before fix)
    bool valid;
} GNSS_Data;

void GNSS_Init(UART_HandleTypeDef *huart);
void GNSS_Process(void);   /* USART1 ISR：只收字元，不解析 */
void GNSS_Poll(void);      /* 主迴圈：解析已收到的 NMEA 行（含 atof，禁止在 ISR 呼叫）*/
GNSS_Data GNSS_GetData(void);
uint32_t GNSS_GetByteCnt(void);
uint32_t GNSS_GetLineCnt(void);

#endif

