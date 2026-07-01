/*
 * lora_e22.c - EBYTE E22-900T22D 透傳模式驅動（USART1）
 *
 * 設計重點：
 *  - 透傳模式：UART 進 → RF 出，不需封包/暫存器操作（比 SX1268 簡單很多）
 *  - 非阻塞 TX：HAL_UART_Transmit_IT，TxCplt 回呼清 busy 旗標
 *  - E22 內部有 ~1000 byte 緩衝，本專案 ~90B/500ms（佔空比 < RF 抽取速率）
 *    不會溢位，故不需逐包等 RF airtime
 *  - 非阻塞 RX：單 byte HAL_UART_Receive_IT 持續重掛 → ring buffer
 *
 * 整合注意：
 *  1) CubeMX 需啟用「USART1 global interrupt」(NVIC)
 *  2) 回呼集中在 stm32f4xx_it.c 依 instance 分派：
 *     USART1→LoRa_OnTxDone()/LoRa_OnRxByte()、USART2→GNSS_Process()
 *  3) 若 M0/M1 接到 MCU GPIO（非硬接 GND），定義下方 LORA_M0/M1 巨集
 */
#include "lora_e22.h"
#include "usart.h"          /* huart1 */
#include <string.h>

extern UART_HandleTypeDef huart1;
#define LORA_UART   (&huart1)

/* ── 若 M0/M1 接 MCU GPIO，取消註解並填腳位（透傳模式 = 兩腳皆 LOW）──
#define LORA_M0_PORT  GPIOx
#define LORA_M0_PIN   GPIO_PIN_x
#define LORA_M1_PORT  GPIOx
#define LORA_M1_PIN   GPIO_PIN_x
 */

#define LORA_TX_TIMEOUT_MS  500U   /* UART IT 傳送超時保護 */
#define LORA_RXBUF_SIZE     256U   /* 必須為 2 的次方 */
#define LORA_RXBUF_MASK     (LORA_RXBUF_SIZE - 1U)

/* ── E22 AUX 狀態腳（rocket_v7：PB9，HIGH=閒置可送、LOW=忙）──────────
 * 啟用前提：PB9 已在 CubeMX 設為 GPIO_Input（否則讀到的是輸出暫存器=錯）。
 * 取消下行註解即啟用「送前確認 E22 閒置」，避免灌爆 E22 內部緩衝。      */
//#define LORA_USE_AUX
#ifdef LORA_USE_AUX
/* 用 CubeMX label「LORA_AUX」(PB9)，前提 PB9 已設 GPIO_Input（建議 Pull-up）*/
static inline uint8_t lora_aux_idle(void)
{ return (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET); }
#else
static inline uint8_t lora_aux_idle(void) { return 1; }  /* 未啟用：永遠視為閒置 */
#endif

/* ── TX 狀態 ── */
static volatile uint8_t  tx_busy   = 0;
static volatile uint32_t tx_start  = 0;
static uint8_t  tx_buf[256];        /* 送出前先複製，IT 期間不可被覆寫 */

/* ── RX ring buffer ── */
static volatile uint8_t  rx_ring[LORA_RXBUF_SIZE];
static volatile uint16_t rx_head = 0;   /* ISR 寫入 */
static volatile uint16_t rx_tail = 0;   /* 主迴圈讀取 */
static uint8_t  rx_byte;                 /* 單 byte IT 暫存 */

/* ══════════════════════════════════════════════════════ */
int LoRa_Init(void)
{
#ifdef LORA_M0_PORT
    /* 透傳模式：M0=0, M1=0 */
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);   /* 等 E22 進入模式 */
#endif
    tx_busy = 0;
    rx_head = rx_tail = 0;
    LoRa_StartRx();
    return 0;
}

/* ── 阻塞傳送（開機訊息用）── */
int LoRa_Send(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return -1;
    /* 等任何進行中的 IT 傳送結束，避免 HAL busy */
    uint32_t t0 = HAL_GetTick();
    while (tx_busy) { if (HAL_GetTick() - t0 > LORA_TX_TIMEOUT_MS) { tx_busy = 0; break; } }
    return (HAL_UART_Transmit(LORA_UART, (uint8_t*)data, len, 200) == HAL_OK) ? 0 : -1;
}

int LoRa_SendStr(const char *str)
{
    if (!str) return -1;
    return LoRa_Send((const uint8_t*)str, (uint8_t)strlen(str));
}

/* ── 非阻塞 TX ── */
int LoRa_SendAsync(const uint8_t *data, uint8_t len)
{
    if (tx_busy)          return -1;
    if (!lora_aux_idle()) return -1;   /* E22 仍忙（AUX=LOW）→ 本輪不送 */
    if (!data || len == 0) return -2;
    /* len 為 uint8_t（≤255），tx_buf[256] 必容得下，不需再夾限 */

    memcpy(tx_buf, data, len);
    tx_busy  = 1;
    tx_start = HAL_GetTick();
    if (HAL_UART_Transmit_IT(LORA_UART, tx_buf, len) != HAL_OK) {
        tx_busy = 0;
        return -1;
    }
    return 0;
}

int LoRa_PollTx(void)
{
    if (!tx_busy) return 1;                                   /* 完成或無任務 */
    if (HAL_GetTick() - tx_start > LORA_TX_TIMEOUT_MS) {      /* 超時保護 */
        HAL_UART_AbortTransmit_IT(LORA_UART);
        tx_busy = 0;
        return -1;
    }
    return 0;                                                 /* 傳送中 */
}

uint8_t LoRa_IsBusy(void) { return (uint8_t)(tx_busy || !lora_aux_idle()); }

/* ── 接收 ── */
int LoRa_StartRx(void)
{
    /* 啟動單 byte IT；RxCplt 內會重掛，形成連續接收 */
    return (HAL_UART_Receive_IT(LORA_UART, &rx_byte, 1) == HAL_OK) ? 0 : -1;
}

int LoRa_Receive(uint8_t *buf, uint8_t max_len)
{
    if (!buf || max_len == 0) return -1;
    uint16_t idx = 0;
    /* 從 ring buffer 取到 '\n' 為止 */
    while (rx_tail != rx_head && idx < (uint16_t)(max_len - 1)) {
        uint8_t c = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) & LORA_RXBUF_MASK);
        if (c == '\n') { buf[idx] = '\0'; return (int)idx; }   /* 取得完整一行 */
        if (c != '\r') buf[idx++] = c;
    }
    buf[idx] = '\0';
    return 0;   /* 尚無完整一行（資料留在 ring buffer 等下次）*/
}

/* ══════════════════════════════════════════════════════
 * ISR hook：由 stm32f4xx_it.c 的 HAL_UART_*CpltCallback 依 instance 分派呼叫。
 * ★不在此覆寫 HAL 弱回呼，否則會與 GPS(USART2) 的回呼重複定義 → link 失敗。
 * ══════════════════════════════════════════════════════ */
void LoRa_OnTxDone(void)    /* USART1 TX 完成時呼叫 */
{
    tx_busy = 0;
}

void LoRa_OnRxByte(void)    /* USART1 收到 1 byte 時呼叫 */
{
    uint16_t next = (uint16_t)((rx_head + 1) & LORA_RXBUF_MASK);
    if (next != rx_tail) {            /* 未滿才寫入，滿則丟棄該 byte */
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    HAL_UART_Receive_IT(LORA_UART, &rx_byte, 1);   /* 重掛，持續接收 */
}
