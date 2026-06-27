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

/* 寫入相關 */
void logger_init(void);
void logger_write(const char *data);
void logger_close(void);
uint8_t logger_is_ready(void);

/* 讀取相關 */
// 讀取單行（返回讀到的字元數；< 0 表示 EOF 或錯誤）
int logger_read_line(char *buffer, uint16_t max_len);

// 讀取整個檔案並輸出到 UART
void logger_read_all_uart(void);

// 重新開啟日誌檔案以供讀取（從頭開始）
int logger_open_for_read(void);

#endif // __LOGGER_H__
