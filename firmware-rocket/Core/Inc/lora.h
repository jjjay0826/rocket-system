/*
 * lora.h - SX1268S433N0S1 LoRa driver
 * SPI3 shared bus (PB12=SCK, PB4=MISO, PB5=MOSI)
 * CS=PB10, BUSY=GND(always ready), RST=GND(no HW reset), DIO1=NC
 */
#ifndef __LORA_H
#define __LORA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 初始化 SX1268，433 MHz，LoRa SF7 BW125 CR4/5 */
/* 回傳 0=成功，非0=失敗 */
int  LoRa_Init(void);

/* 傳送資料，len 最多 255 bytes */
/* 回傳 0=成功，非0=失敗 */
int  LoRa_Send(const uint8_t *data, uint8_t len);

/* 傳送字串 */
int  LoRa_SendStr(const char *str);

/* ---- 非阻塞 TX（輪詢式，不需 DIO 腳位）---- */
/* 啟動傳送，立即返回（不等射頻完成）
 *   0  = 已啟動
 *  -1  = 上次 TX 仍未完成（busy）
 *  -2  = 參數錯誤
 */
int  LoRa_SendAsync(const uint8_t *data, uint8_t len);

/* 主迴圈每次呼叫，輪詢 SX1268 TxDone IRQ flag
 *   1  = 完成（或無待傳任務）
 *   0  = 仍在傳送中
 *  -1  = 超時（硬體異常，已強制 Standby）
 */
int  LoRa_PollTx(void);

/* 是否有 TX 正在進行中 */
uint8_t LoRa_IsBusy(void);

/* ---- 接收模式 ---- */
/* 進入連續接收模式（呼叫一次後持續監聽）*/
int  LoRa_StartRx(void);

/* 輪詢是否收到封包
 *   buf     : 接收緩衝區
 *   max_len : 緩衝區大小
 *   回傳值  :  >0 = 收到的 byte 數
 *              0  = 尚未收到
 *             -1  = CRC 錯誤
 */
int  LoRa_Receive(uint8_t *buf, uint8_t max_len);

#endif /* __LORA_H */
