/*
 * logger.h
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "fatfs.h"
#include <stdint.h>

extern FIL file;

/* 8MB 預分配大小：main.c 的飛行 sync 政策需檢查寫入位置是否仍在區內 */
#define LOG_PREALLOC_BYTES  (8UL * 1024UL * 1024UL)

/* 寫入相關 */
void logger_init(void);
void logger_write(const char *data);
void logger_close(void);
uint8_t logger_is_ready(void);
uint8_t logger_prealloc_ok(void);  /* 8MB 預分配成功?（零 sync 政策前提）*/
uint8_t logger_file_gen(void);     /* 檔案世代：開新 logN.csv 時遞增     */
int     logger_reopen(void);       /* 寫入失敗復原：重開+seek 回原位     */
/* 把檔案截到目前寫入長度（去掉 8MB 預分配尾巴）。0=OK，-1=失敗/SD未就緒。
 * 飛行時 LANDED 會自動做；桌面測試用 TRUNC 命令手動觸發。*/
int  logger_trunc(void);

/* 讀取相關 */
// 讀取單行（返回讀到的字元數；< 0 表示 EOF 或錯誤）
int logger_read_line(char *buffer, uint16_t max_len);

// 讀取整個檔案並輸出到 UART
void logger_read_all_uart(void);

// 重新開啟日誌檔案以供讀取（從頭開始）
int logger_open_for_read(void);

#endif // __LOGGER_H__
