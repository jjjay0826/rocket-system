/*
 * lora_e22.h - EBYTE E22-900T22D LoRa 模組驅動（UART 透傳模式）
 *
 * rocket_v2 板：LoRa 改用 E22 UART 模組（取代舊板 SX1268 SPI 裸驅動）
 *   接線：MCU USART1  PA9(TX) → E22 RXD
 *                     PA10(RX)← E22 TXD
 *   模式：M0=M1=0（透傳模式，假設硬體已接 GND；若接 MCU GPIO 見下方 #define）
 *
 * API 與舊 lora.h 完全相同 → main.c 只需改 #include "lora_e22.h"。
 * 透傳模式下「傳送」= 從 UART 送 bytes，E22 自動經 RF 發出；
 *               「接收」= E22 把收到的 RF 資料從 UART 吐回，驅動以 ring buffer 收。
 */
#ifndef __LORA_E22_H
#define __LORA_E22_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 初始化（透傳模式下僅啟動 UART 接收 IT；若 M0/M1 接 MCU 則拉低設透傳）
 * 回傳 0=成功 */
int  LoRa_Init(void);

/* 阻塞傳送（給開機訊息用，內部 HAL_UART_Transmit）回傳 0=成功 */
int  LoRa_Send(const uint8_t *data, uint8_t len);
int  LoRa_SendStr(const char *str);

/* ---- 非阻塞 TX（UART IT）---- */
/*  0=已啟動  -1=上次未完成(busy)  -2=參數錯誤 */
int  LoRa_SendAsync(const uint8_t *data, uint8_t len);
/*  1=完成(或無待傳)  0=傳送中  -1=超時 */
int  LoRa_PollTx(void);
uint8_t LoRa_IsBusy(void);

/* ---- 接收（UART IT ring buffer）---- */
/* 啟動連續接收（LoRa_Init 已呼叫，可重複呼叫重啟）*/
int  LoRa_StartRx(void);
/* 取出一行（以 '\n' 為界，符合本專案封包格式）
 *  >0 = 取出的 byte 數（不含 '\n'，已補 '\0'）
 *   0 = 尚無完整一行
 *  -1 = 參數錯誤 */
int  LoRa_Receive(uint8_t *buf, uint8_t max_len);

/* ---- ISR hook（由 stm32f4xx_it.c 的 UART 回呼依 instance 呼叫）---- */
void LoRa_OnTxDone(void);   /* USART1 TX 完成 */
void LoRa_OnRxByte(void);   /* USART1 收到 1 byte */

#endif /* __LORA_E22_H */
