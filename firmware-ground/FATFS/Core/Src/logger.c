/*
 * logger.c
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#include "logger.h"
#include "gnss.h"      // 提供 gnss_time
#include <string.h>
#include <stdio.h>
#include "sdcard.h"
#include "main.h"      // 用於 uart1_write
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

/* 使用 CubeMX 正確的 FATFS 物件（SDFatFS/SDPath 來自 fatfs.h）*/
extern FIL file;
FRESULT fres;

/* 外部宣告：用於輸出讀到的內容 */
extern void uart1_write(const char *s);
/* 讀取時旗標：避免同一時間寫入 */
static volatile uint8_t logger_reading = 0;
/* SD 卡是否就緒旗標：失敗時跳過所有寫入，避免卡住 */
static volatile uint8_t sd_ok = 0;
/* FATFS 驅動只綁定一次 */
static uint8_t fatfs_linked = 0;

void logger_init(void) {
    sd_ok = 0;

    /* 首次呼叫才綁定 SDIO 驅動（MX_FATFS_Init 只能呼叫一次）*/
    if (!fatfs_linked) {
        MX_FATFS_Init();
        fatfs_linked = 1;
    }

    /* opt=1：立即掛載，無卡時直接返回 FR_NODISK，不需等到 f_open 才超時 */
    fres = f_mount(&SDFatFS, SDPath, 1);
    if (fres != FR_OK) { return; }

    fres = f_open(&file, "log.txt", FA_WRITE | FA_OPEN_APPEND | FA_OPEN_ALWAYS);
    sd_ok = (fres == FR_OK) ? 1 : 0;
}

uint8_t logger_is_ready(void) { return sd_ok; }

void logger_write(const char *data) {
    char line[128];
    UINT bw;

    /* SD 卡未就緒或正在讀取，跳過 */
    if (!sd_ok || logger_reading) return;

    // 組合內容格式：[hhmmss] 資料內容
    snprintf(line, sizeof(line), "[%s] %s\r\n", gnss_time, data);

    // 寫入檔案
    fres = f_write(&file, line, strlen(line), &bw);
    if (fres == FR_OK) {
        f_sync(&file); // 保證立即寫入 SD 卡
    }
}

void logger_close(void) {
    f_close(&file);
}

/* ===== 讀取函式 ===== */
int logger_open_for_read(void) {
    // 關閉先前的文件（如果是寫模式）
    f_close(&file);

    // 以唯讀模式重新開啟日誌檔案
    if (!sd_ok) { uart1_write("ERR: SD not ready\r\n"); return -1; }
    fres = f_open(&file, "log.txt", FA_READ);
    if (fres != FR_OK) {
        uart1_write("ERR: Cannot open log.txt for reading\r\n");
        return -1;
    }
    return 0;
}

int logger_read_line(char *buffer, uint16_t max_len) {
    UINT br = 0;  // Bytes read
    char c;
    uint16_t idx = 0;

    if (!buffer || max_len == 0) return -1;

    // 逐字元讀取，直到換行或檔案結尾
    while (idx < max_len - 1) {
        fres = f_read(&file, &c, 1, &br);
        
        // 讀取失敗或 EOF
        if (fres != FR_OK || br == 0) {
            buffer[idx] = '\0';
            return (fres == FR_OK && idx > 0) ? idx : -1;  // EOF 或錯誤
        }

        // 遇到換行符，結束讀取
        if (c == '\n') {
            // 移除可能的 \r（CRLF 格式）
            if (idx > 0 && buffer[idx-1] == '\r') {
                idx--;
            }
            buffer[idx] = '\0';
            return idx;
        }

        buffer[idx++] = c;
    }

    // 緩衝滿，確保 null 終止
    buffer[idx] = '\0';
    return idx;
}

void logger_read_all_uart(void) {
    static char line[256];
    static char formatted[300];
    int line_num = 1;
    /* 標記為正在讀取，阻止 logger_write 寫入 */
    logger_reading = 1;

    // 開啟檔案以供讀取
    if (logger_open_for_read() != 0) {
        uart1_write("Failed to open log.txt\r\n");
        logger_reading = 0;
        return;
    }
    uart1_write("===== SD Card Log Content =====\r\n");

    /* 逐行讀取並輸出；最多 10000 行，防止 SD 卡損壞時無限迴圈 */
    while (line_num <= 10000 && logger_read_line(line, sizeof(line)) >= 0) {
        // 過濾開頭 timestamp: 如果以 '[' 開頭，找到第一個 ']' 並跳過
        char *payload = line;
        if (payload[0] == '[') {
            char *p = strchr(payload, ']');
            if (p) {
                payload = p + 1;
                while (*payload == ' ' || *payload == '\t') payload++;
            }
        }

        int n = snprintf(formatted, sizeof(formatted), "%s\r\n", payload);
        if (n < 0) {
            continue;
        }
        if ((size_t)n >= sizeof(formatted)) formatted[sizeof(formatted)-1] = '\0';

        /* 輸出到 UART1 */
        uart1_write(formatted);

        /* 輸出到 USB CDC (chunked) */
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            const uint8_t *p = (const uint8_t*)formatted;
            size_t remaining = strlen(formatted);
            while (remaining > 0) {
                uint16_t chunk = (uint16_t)(remaining > 64 ? 64 : remaining);
                uint32_t t0 = HAL_GetTick();
                while (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_BUSY) {
                    if (HAL_GetTick() - t0 > 200) break;
                    HAL_Delay(2);
                }
                p += chunk;
                remaining -= chunk;
            }
        }
        line_num++;
    }

    uart1_write("===== End of Log =====\r\n");
    f_close(&file);

    /* 讀取結束，解除阻止，並重新開啟日誌供寫入 */
    logger_reading = 0;
    logger_init();
}

