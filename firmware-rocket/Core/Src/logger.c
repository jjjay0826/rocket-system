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

/* 使用 CubeMX 正確的 FATFS 物件（User-defined 模式：USERFatFS/USERPath 來自 fatfs.h）*/
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
/* 當前 log 檔名：每次上電遞增（log1.csv, log2.csv, ...），不覆寫舊飛行資料 */
static char cur_logname[16] = "log1.csv";
/* 8MB 預分配是否成功：飛行「零 sync」政策的前提。失敗（卡空間不足等）
 * 時 main.c 的 sync 政策必須退回週期 sync（code-review C6）。 */
static uint8_t prealloc_ok = 0;
/* 檔案世代：每次成功開新 logN.csv 遞增。main.c 據此判斷「新檔要重寫
 * CSV header」——CLEAR 滾新檔後不再漏標頭（code-review C1）。 */
static uint8_t file_gen = 0;
/* READ 期間保存的寫入位置：讀完重開後 seek 回原位。
 * 不能用 FA_OPEN_APPEND——預分配檔 fsize=8MB+1，APPEND 會把寫入位置
 * 跳到檔尾，之後全部寫在預分配區外＝零 sync 前提崩壞（code-review C3）*/
static FSIZE_t saved_wpos = 0;

void logger_init(void) {
    sd_ok = 0;
    
    extern void SD_disk_deinit(void);
    SD_disk_deinit(); /* Force hardware SPI re-init to prevent long timeouts */

    /* 首次呼叫才綁定 SDIO 驅動（MX_FATFS_Init 只能呼叫一次）*/
    if (!fatfs_linked) {
        MX_FATFS_Init();
        fatfs_linked = 1;
    }

    /* opt=1：立即掛載，無卡時直接返回 FR_NODISK，不需等到 f_open 才超時 */
    fres = f_mount(&USERFatFS, USERPath, 1);
    if (fres != FR_OK) { return; }

    /* 找下一個沒用過的 logN.csv → 每次上電開新檔，不覆寫上一次飛行資料。
     * 用 FA_CREATE_NEW 逐號嘗試：檔案已存在會回 FR_EXIST 就換下一號。*/
    {
        int idx;
        for (idx = 1; idx <= 9999; idx++) {
            snprintf(cur_logname, sizeof(cur_logname), "log%d.csv", idx);
            fres = f_open(&file, cur_logname, FA_WRITE | FA_CREATE_NEW);
            if (fres == FR_OK)    break;   /* 建立成功＝此編號還沒用過 */
            if (fres != FR_EXIST) return;  /* 非「已存在」的錯誤（如無卡）→ 放棄 */
        }
        if (fres != FR_OK) return;         /* 9999 號都用光（幾乎不可能）*/
    }
    /* 檔案剛建立，大小=0、append_pos=0；預分配從 byte 0 開始一次配滿 8MB */

    /* ── 預分配磁碟空間（減少飛行中 SD GC 阻塞）────────────────────
     * FA_CREATE_ALWAYS 後檔案大小 = 0，append_pos = 0，
     * 預分配從 byte 0 開始一次配滿 LOG_PREALLOC_BYTES 個連續 cluster。
     * 飛行中寫入資料全部落在已分配區域，FAT 無需更新 → GC 不觸發。
     * f_sync 只在地面初始化執行一次，飛行中不再有 GC 風險。
     *
     * 容量估算（每行 ≈280B，500ms 輸出一行）：
     *   4 小時全速  = 4×3600×2×280 = 8,064,000 B ≈ 7.7 MB  → 8MB 足夠
     *   落地後 5s 間隔，後半段消耗大幅降低
     * ──────────────────────────────────────────────────────────────*/
    /* LOG_PREALLOC_BYTES 已移至 logger.h（main.c 的 sync 政策需要它）*/
    {
        prealloc_ok = 0;
        if (f_lseek(&file, LOG_PREALLOC_BYTES) == FR_OK) {
            UINT bw = 0;
            uint8_t zero = 0;
            if (f_write(&file, &zero, 1, &bw) == FR_OK && bw == 1
                && f_sync(&file) == FR_OK
                && f_lseek(&file, 0) == FR_OK) {
                prealloc_ok = 1;
            }
        }
        if (!prealloc_ok) {
            /* 卡空間不足等：不擋記錄，但 main.c 會退回週期 sync 政策，
             * 且明確告警——舊版靜默吞掉，零 sync 飛行斷電＝整趟丟失 */
            f_lseek(&file, 0);
            uart1_write("LOG: PREALLOC FAIL (low space?) - fallback to periodic sync\r\n");
        }
    }

    file_gen++;              /* 新檔世代：main.c 據此重寫 CSV header */
    sd_ok = 1;
}

uint8_t logger_prealloc_ok(void) { return prealloc_ok; }
uint8_t logger_file_gen(void)    { return file_gen; }

/* 寫入失敗復原（code-review C4）：FatFS 在 f_write 失敗時鎖定 fp->err
 * （ff.c ABORT，僅 f_open 能清），不重開則之後所有寫入立即失敗＝
 * 整趟飛行資料靜默丟失。關檔→重開→seek 回原寫入位置。 */
int logger_reopen(void) {
    FSIZE_t pos = f_tell(&file);
    f_close(&file);
    fres = f_open(&file, cur_logname, FA_WRITE);   /* 勿用 APPEND（見 saved_wpos 註解）*/
    if (fres == FR_OK) fres = f_lseek(&file, pos);
    sd_ok = (fres == FR_OK) ? 1 : 0;
    return sd_ok ? 0 : -1;
}

uint8_t logger_is_ready(void) { return sd_ok; }

void logger_write(const char *data) {
    char line[128];
    UINT bw;
    static uint16_t write_count = 0;   /* 偵測寫入次數，每 16 筆才 sync 一次 */

    /* SD 卡未就緒或正在讀取，跳過 */
    if (!sd_ok || logger_reading) return;

    // 組合內容格式：[hhmmss] 資料內容
    snprintf(line, sizeof(line), "[%s] %s\r\n", gnss_time, data);

    // 寫入檔案
    fres = f_write(&file, line, strlen(line), &bw);
    if (fres == FR_OK) {
        write_count++;
        /* 每 16 筆 sync 一次（約 8 秒一次）賸免飛行中高頻觸發 FAT GC */
        if ((write_count & 0x0F) == 0) {
            f_sync(&file);
        }
    }
}

void logger_close(void) {
    f_sync(&file);   /* 強制同步未寫入的緩衝資料 */
    f_close(&file);
}

/* 截掉 8MB 預分配尾巴 → 檔案收斂成實際資料長度（Excel 直接開，無垃圾）。
 * ⚠ 截掉後預分配就沒了：僅供桌面測試收尾用，截完就該斷電拔卡；
 *    若之後才起飛，飛行中寫入會邊寫邊配 cluster（有 GC 風險）。*/
int logger_trunc(void) {
    if (!sd_ok) return -1;
    if (f_truncate(&file) != FR_OK) return -1;
    if (f_sync(&file)     != FR_OK) return -1;
    return 0;
}

/* ===== 讀取函式 ===== */
int logger_open_for_read(void) {
    /* 先記下寫入位置（此刻 file 仍是寫模式），讀完重開後 seek 回來 */
    saved_wpos = f_tell(&file);
    // 關閉先前的文件（如果是寫模式）
    f_close(&file);

    // 以唯讀模式重新開啟日誌檔案
    if (!sd_ok) { uart1_write("ERR: SD not ready\r\n"); return -1; }
    fres = f_open(&file, cur_logname, FA_READ);
    if (fres != FR_OK) {
        uart1_write("ERR: Cannot open log for reading\r\n");
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
        uart1_write("Failed to open log.csv\r\n");
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

        /* 輸出到 USB CDC (chunked)。
         * 注意：uart1_write 已重導成 cdc_write，這裡不可再多叫一次（會每行印兩次）*/
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

    /* 讀完重開「同一個」log 檔並 seek 回原寫入位置：繼續寫、不清空、
     * 不建新檔。⚠ 勿用 FA_OPEN_APPEND：預分配檔 fsize=8MB+1，APPEND
     * 會把 fptr 跳到檔尾 → 之後全寫在預分配區外、零 sync 政策下斷電
     * 整趟孤兒化，且 TRUNC 也削不掉 8MB 垃圾間隙（code-review C3）。 */
    fres = f_open(&file, cur_logname, FA_WRITE);
    if (fres == FR_OK) fres = f_lseek(&file, saved_wpos);
    sd_ok = (fres == FR_OK) ? 1 : 0;   /* 重開失敗要讓寫入路徑知道 */
    logger_reading = 0;
}

