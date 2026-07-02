/*
 * gnss.c
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#include "main.h"
#include "gnss.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef* gps_uart;
static char rx_char;
static char nmea_buf[100];
static uint8_t idx = 0;

static GNSS_Data current_data;
static uint32_t gnss_byte_cnt  = 0;   /* 收到的總 bytes */
static uint32_t gnss_line_cnt  = 0;   /* 收到的完整行數 */

static void gnss_parse(const char *nmea);  /* forward declaration */

static float nmea_to_deg(const char* val, const char dir) {
    /* NMEA 格式：DDDMM.MMMM，前兩（或三）位是度，其餘是分 */
    float raw = atof(val);
    int   deg_i = (int)(raw / 100.0f);
    float min_f = raw - (float)(deg_i * 100);
    float decimal = (float)deg_i + min_f / 60.0f;
    return (dir == 'S' || dir == 'W') ? -decimal : decimal;
}

/* 按逗號切 NMEA，保留空欄位。strtok 會把連續逗號當「一個」分隔符、跳過空
 * 欄位 → 失去定位時（GGA 經緯度欄位變空）後面欄位整排錯位、解出垃圾座標。
 * 本函式就地把 ',' 改 '\0'，parts[] 指向每一欄（含空字串），回傳欄位數。*/
static int nmea_split(char *s, char **parts, int max) {
    int n = 0;
    if (n < max) parts[n++] = s;
    for (; *s && n < max; s++) {
        if (*s == ',') { *s = '\0'; parts[n++] = s + 1; }
    }
    return n;
}

char gnss_time[16] = "000000";  // 初始值，可根據 GNSS 更新

void GNSS_Init(UART_HandleTypeDef *huart) {
    gps_uart = huart;
    HAL_UART_Receive_IT(gps_uart, (uint8_t*)&rx_char, 1);
}

void GNSS_ProcessLine(const char* line) {
    /* Accept $GP/$GN (GPS/multi) and $BD (BeiDou/北斗 ATGM336H-5N31)
     * All talker IDs are 2 chars after '$', sentence type starts at offset 3
     * e.g. $GNGGA → type="GGA"   $BDGSV → type="GSV"   $GPGSV → type="GSV" */
    int is_g  = (strncmp(line, "$G",  2) == 0);
    int is_bd = (strncmp(line, "$BD", 3) == 0);
    if (!is_g && !is_bd) return;
    const char *type = line + 3;  /* skip talker id, point to sentence type */

    if (strncmp(type, "GGA", 3) == 0) {
        char* parts[16];
        char buf[100];
        strncpy(buf, line, sizeof(buf));
        buf[sizeof(buf)-1] = '\0';
        int i = nmea_split(buf, parts, 16);   /* 保留空欄位，欄位位置固定 */

        if (i >= 8) {
            current_data.num_sats = (uint8_t)atoi(parts[7]); /* field 7 = sats used */
        }
        /* field 6 = fix quality：'0' 或空 = 無定位 → 不更新座標（避免解垃圾）*/
        if (i >= 10 && parts[6][0] != '0' && parts[6][0] != '\0') {
            current_data.latitude  = nmea_to_deg(parts[2], parts[3][0]);
            current_data.longitude = nmea_to_deg(parts[4], parts[5][0]);
            current_data.altitude  = atof(parts[9]);
            current_data.valid = true;
        } else {
            current_data.valid = false;
        }
    } else if (strncmp(type, "GSV", 3) == 0) {
        /* $GPGSV,totalMsg,msgNum,totalSats,...  field[3] = total sats in view */
        char buf2[100]; char* p2[6];
        strncpy(buf2, line, sizeof(buf2)); buf2[sizeof(buf2)-1]='\0';
        int j = nmea_split(buf2, p2, 6);
        if (j >= 4 && p2[3][0] >= '0' && p2[3][0] <= '9') {
            uint8_t sv = (uint8_t)atoi(p2[3]);
            /* 累加各星系的 GSV（GPS+BeiDou+GLONASS），只在第一條訊息時更新 */
            if (p2[2][0] == '1') {  /* msgNum==1: 第一條，重設累加 */
                current_data.sats_in_view = sv;
            }
        }
    } else if (strncmp(type, "VTG", 3) == 0) {
        char* parts[10];
        char buf[100];
        strncpy(buf, line, sizeof(buf));
        buf[sizeof(buf)-1] = '\0';
        int i = nmea_split(buf, parts, 10);
        if (i >= 8) {
            current_data.speed = atof(parts[7]);  /* km/h field */
        }
    } else if (strncmp(type, "RMC", 3) == 0) {
        gnss_parse(line);  /* extract gnss_time[] from $GNRMC / $GPRMC */
    }
}

void GNSS_Process(void) {
    gnss_byte_cnt++;
    if (rx_char == '\n') {
        nmea_buf[idx] = '\0';
        if (idx > 0) gnss_line_cnt++;
        GNSS_ProcessLine(nmea_buf);
        idx = 0;
    } else if (rx_char != '\r') {
        if (idx < sizeof(nmea_buf)-1)
            nmea_buf[idx++] = rx_char;
    }

    HAL_UART_Receive_IT(gps_uart, (uint8_t*)&rx_char, 1);
}

/* 供 main.c / cmd.c 查詢用 */
uint32_t GNSS_GetByteCnt(void) { return gnss_byte_cnt; }
uint32_t GNSS_GetLineCnt(void) { return gnss_line_cnt; }

GNSS_Data GNSS_GetData(void) {
    return current_data;
}


static void gnss_parse(const char *nmea) {
    int ok = (strncmp(nmea, "$G",  2) == 0) || (strncmp(nmea, "$BD", 3) == 0);
    if (ok && strncmp(nmea + 3, "RMC", 3) == 0) {
        // GPRMC 會長這樣：$GPRMC,hhmmss.00,A,....  → 取 field 1 = time
        char buf[100]; char *parts[3];
        strncpy(buf, nmea, sizeof(buf)); buf[sizeof(buf)-1] = '\0';
        int n = nmea_split(buf, parts, 3);
        if (n >= 2 && parts[1][0] != '\0') {
            strncpy(gnss_time, parts[1], 6);
            gnss_time[6] = '\0';
        }
    }
}

