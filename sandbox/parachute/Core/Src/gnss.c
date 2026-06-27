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
        char* token;
        char* parts[15];
        char buf[100];
        strncpy(buf, line, sizeof(buf));
        buf[sizeof(buf)-1] = '\0';

        int i = 0;
        token = strtok(buf, ",");
        while (token && i < 15) {
            parts[i++] = token;
            token = strtok(NULL, ",");
        }

        if (i >= 8) {
            current_data.num_sats = (uint8_t)atoi(parts[7]); /* field 7 = sats used */
        }
        if (i >= 10 && parts[6][0] != '0') {
            current_data.latitude  = nmea_to_deg(parts[2], parts[3][0]);
            current_data.longitude = nmea_to_deg(parts[4], parts[5][0]);
            current_data.altitude  = atof(parts[9]);
            current_data.valid = true;
        } else {
            current_data.valid = false;
        }
    } else if (strncmp(type, "GSV", 3) == 0) {
        /* $GPGSV,totalMsg,msgNum,totalSats,...  field[3] = total sats in view */
        char buf2[100]; char* p2[5]; int j=0;
        strncpy(buf2, line, sizeof(buf2)); buf2[sizeof(buf2)-1]='\0';
        char* tok2 = strtok(buf2, ",");
        while (tok2 && j < 5) { p2[j++] = tok2; tok2 = strtok(NULL, ","); }
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

        int i = 0;
        char* token = strtok(buf, ",");
        while (token && i < 10) {
            parts[i++] = token;
            token = strtok(NULL, ",");
        }
        if (i >= 7) {
            current_data.speed = atof(parts[7]);  /* km/h field */
        }
    } else if (strncmp(type, "RMC", 3) == 0) {
        gnss_parse(line);  /* extract gnss_time[] from $GNRMC / $GPRMC */
    }
}

/* ── ISR → 主迴圈 交接緩衝 ─────────────────────────────────────
 * 重要：GNSS_ProcessLine 內含 strtok/atof，newlib 的 atof(strtod)
 * 會呼叫 malloc。主迴圈的 snprintf("%f") 同樣會 malloc，兩者若在
 * ISR/主迴圈並行執行 → heap 損毀 → 隨機記憶體破壞
 * （實測症狀：FIL 物件失效 fw=9、KF 狀態被污染等，發生時間正好
 *   是 GNSS 收到衛星、NMEA 欄位開始有數字之後 ~15-30s）。
 * 因此 ISR 只負責收完整行，解析一律由主迴圈 GNSS_Poll() 執行。  */
static volatile uint8_t line_pending = 0;
static char pending_line[100];

void GNSS_Process(void) {
    gnss_byte_cnt++;
    if (rx_char == '\n') {
        nmea_buf[idx] = '\0';
        if (idx > 0) {
            gnss_line_cnt++;
            if (!line_pending) {   /* 前一行尚未被取走 → 丟棄本行 */
                memcpy(pending_line, nmea_buf, (size_t)idx + 1);
                line_pending = 1;
            }
        }
        idx = 0;
    } else if (rx_char != '\r') {
        if (idx < sizeof(nmea_buf)-1)
            nmea_buf[idx++] = rx_char;
    }

    HAL_UART_Receive_IT(gps_uart, (uint8_t*)&rx_char, 1);
}

/* 主迴圈每圈呼叫：在安全 context 解析 NMEA（可安心用 strtok/atof）*/
void GNSS_Poll(void) {
    if (!line_pending) return;
    GNSS_ProcessLine(pending_line);
    line_pending = 0;   /* 解析完才釋放，ISR 期間不會覆寫 pending_line */
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
        // GPRMC 會長這樣：$GPRMC,hhmmss.00,A,....
        const char *time_ptr = strchr(nmea, ',');
        if (!time_ptr) return;   /* 無逗號的畸形句 → 防 NULL+1 野指標 */
        time_ptr++;
        strncpy(gnss_time, time_ptr, 6);
        gnss_time[6] = '\0';
    }
}

