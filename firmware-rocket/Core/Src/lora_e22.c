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

/* ── M0/M1 模式腳（換頻需要，2026-07-26）───────────────────────────────
 * 🔴 硬體前提：E22 的 M0/M1 必須已從 GND 切離、改接到下面兩支 MCU 腳。
 *    ⚠ 若模組 M0/M1 仍焊死在 GND 而啟用此巨集 → MCU 推挽輸出對地短路
 *      → 燒 GPIO。硬體改好之前，這行必須保持註解。
 *
 *    PB7 = 原 SIG_1（禁用 LoRa 的跳線，已廢除）→ M0
 *    PB5 = 原 SIG_3（本來就沒用到）           → M1
 *    改線後 gpio.c 的 PB7 也要從輸入改成推挽輸出（見 LoRa_ModePinsInit）。
 *
 * 為什麼要做：規範 4.1.4.3 要求決賽前完成不同頻道通訊測試以避開他隊同頻
 * 干擾。M0/M1 硬接地時只能拆下模組、接 USB-TTL、開上位機才改得了頻率，
 * 比賽現場來不及；接到 GPIO 後地面站打一行指令就換完。
 */
#define LORA_MODE_PINS      /* 2026-07-27 啟用：兩塊板的 M0/M1 已切離 GND 改接 MCU */

#ifdef LORA_MODE_PINS
#define LORA_M0_PORT  GPIOB
#define LORA_M0_PIN   GPIO_PIN_7
#define LORA_M1_PORT  GPIOB
#define LORA_M1_PIN   GPIO_PIN_5
#endif

#define LORA_TX_TIMEOUT_MS  500U   /* UART IT 傳送超時保護 */
#define LORA_RXBUF_SIZE     256U   /* 必須為 2 的次方 */
#define LORA_RXBUF_MASK     (LORA_RXBUF_SIZE - 1U)

/* ── E22 AUX 狀態腳（PB9，HIGH=閒置可送、LOW=忙/不存在）──────────────
 * 2026-07-20 啟用：netlist（Netlist_Schematic1_2026-06-26.tel）證實接線
 * LORA.5(AUX) → STM32.17 = PB9。解「LoRa 假存活」——透傳模式下 UART TX
 * 單向推，沒裝模組也 mod.lora=1、LTX 100% 假成功；AUX 是唯一能證明
 * E22 活著的訊號（上電自檢完成後拉 HIGH）。
 * PB9 由 LoRa_Init 就地設 input+PULL-DOWN（不依賴 .ioc）：
 *  - 模組不在/沒電 → pulldown 讀 LOW → init 500ms 逾時 → mod.lora=0（誠實）
 *  - 模組在 → AUX 推挽 HIGH 壓得過內部 ~40k pulldown → 正常
 * ⚠ 勿改 PULL-UP：浮空讀 HIGH＝假存活重演。 */
#define LORA_USE_AUX
#ifdef LORA_USE_AUX
#define LORA_AUX_PORT  GPIOB
#define LORA_AUX_PIN   GPIO_PIN_9
static inline uint8_t lora_aux_idle(void)
{ return (HAL_GPIO_ReadPin(LORA_AUX_PORT, LORA_AUX_PIN) == GPIO_PIN_SET); }
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
/* M0/M1 設為推挽輸出並拉低＝透傳模式。可重複呼叫（冪等）。
 * ★必須在 main() 早期就呼叫一次：CubeMX 產生的 gpio.c 把 PB7 設成
 *   input+PULLUP（它原本是 SIG_1 跳線腳），於是從 MX_GPIO_Init 到 LoRa_Init
 *   之間（中間夾著 USB 枚舉的 1 秒延遲）M0=1、M1=0 ＝ E22 的 WOR 發送模式。
 *   那段時間不發資料所以無害，但沒有理由讓模組在開機時待在一個非預期模式。*/
void LoRa_ModePinsInit(void)
{
#ifdef LORA_MODE_PINS
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* 先寫再切模式：避免切成輸出的瞬間輸出未定義電位 */
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
    gi.Pin   = LORA_M0_PIN | LORA_M1_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
#endif
}

int LoRa_Init(void)
{
#ifdef LORA_MODE_PINS
    LoRa_ModePinsInit();   /* 冪等；早期已呼叫過，這裡確保狀態正確 */
    HAL_Delay(10);         /* 等 E22 進入模式 */
#endif
#ifdef LORA_USE_AUX
    /* PB9 就地 init 為 input+pulldown（不依賴 .ioc，同 LED 的做法）*/
    {
        GPIO_InitTypeDef gi = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gi.Pin  = LORA_AUX_PIN;
        gi.Mode = GPIO_MODE_INPUT;
        gi.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(LORA_AUX_PORT, &gi);
    }
#endif
    tx_busy = 0;
    rx_head = rx_tail = 0;
    LoRa_StartRx();

    /* ⚠ 透傳模式下 STM32 看不到 E22 的 RF 狀態，「回 0」本質只代表 UART/驅動
     * 已初始化，不保證 E22 有接/有電/RF 能發（上層 mod.lora 同理）。唯一能驗證
     * E22 存活的是 AUX 腳（上電自檢完成後拉高）。若有接 AUX（啟用 LORA_USE_AUX），
     * 這裡等它就緒，等不到就回 -1 → 上層 mod.lora=0，反映 E22 沒回應。*/
#ifdef LORA_USE_AUX
    {
        uint32_t t0 = HAL_GetTick();
        while (!lora_aux_idle()) {
            if (HAL_GetTick() - t0 > 500) return -1;   /* E22 沒回應（沒接/沒電）*/
        }
    }
#endif
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
    /* ★2026-07-20 重寫：舊版沒湊滿一行就 return 0，但 rx_tail 已前進＝
     * 已讀字元直接丟失、下次從零開始。9600 baud 每字元隔 ~1ms 到、主迴圈
     * 每幾百 µs poll 一次 → "ARM\r\n" 被逐字吃掉丟棄，最後 '\n' 交出空行
     * → 上行命令 100% 失效（首次外場 ARM 實測炸出）。
     * 修法：部分行累積在 static buffer 跨呼叫保留，收齊 '\n' 才交行。 */
    static uint8_t  line_acc[96];   /* 部分行累積（跨呼叫保留）*/
    static uint16_t acc_len = 0;

    if (!buf || max_len == 0) return -1;
    while (rx_tail != rx_head) {
        uint8_t c = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) & LORA_RXBUF_MASK);
        if (c == '\n') {                       /* 完整一行 → 交出並清累積 */
            uint16_t n = acc_len;
            if (n > (uint16_t)(max_len - 1)) n = (uint16_t)(max_len - 1);
            memcpy(buf, line_acc, n);
            buf[n] = '\0';
            acc_len = 0;
            return (int)n;                     /* 空行回 0，呼叫端 >0 檢查自然忽略 */
        }
        if (c != '\r' && acc_len < sizeof(line_acc) - 1)
            line_acc[acc_len++] = c;           /* 超長（雜訊流）：丟尾保頭，等 '\n' 收行 */
    }
    return 0;   /* 尚無完整一行（部分行暫存於 line_acc，真的有留）*/
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

/* ══════════════════════════════════════════════════════
 * 換頻（設定模式 M0=1,M1=1 → 寫 REG2 頻道 → 回透傳）
 *
 * E22-900T22D：實際頻率 = 850.125 MHz + CH(MHz)，CH72 = 922.125 MHz（現用）。
 * 命令 C0 05 01 <CH>：從暫存器位址 05H 起寫 1 byte，C0＝寫入並保存（斷電不失效）。
 * 回應 C1 05 01 <CH>。
 *
 * ⚠ 這段會短暫阻塞（~200ms：模式切換等待 + UART 阻塞收發）。只允許在
 *   FLIGHT_IDLE 呼叫——飛行中停掉遙測接收 200ms 不可接受。呼叫端負責把關。
 * ══════════════════════════════════════════════════════ */
int LoRa_SetChannel(uint8_t ch)
{
#ifndef LORA_MODE_PINS
    (void)ch;
    return -2;      /* 硬體未改線：M0/M1 仍接地，無法進設定模式 */
#else
    uint8_t cmd[4]  = { 0xC0, 0x05, 0x01, ch };
    uint8_t resp[4] = { 0 };
    int ret = -1;
    uint32_t t0;

    /* 停 IT 接收：設定模式的回應要用阻塞式讀，兩者不能並存 */
    HAL_UART_AbortReceive_IT(LORA_UART);

    /* 進設定模式（M0=1, M1=1），等 AUX 回到閒置 */
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    t0 = HAL_GetTick();
    while (!lora_aux_idle() && (HAL_GetTick() - t0) < 100) { }

    if (HAL_UART_Transmit(LORA_UART, cmd, sizeof(cmd), 200) == HAL_OK) {
        if (HAL_UART_Receive(LORA_UART, resp, sizeof(resp), 500) == HAL_OK) {
            /* 回應必須是 C1 05 01 <寫進去的 ch>，否則視為失敗 */
            if (resp[0] == 0xC1 && resp[1] == 0x05 && resp[3] == ch) ret = 0;
        }
    }

    /* 回透傳模式（M0=0, M1=0），等 AUX 閒置後重掛 IT 接收 */
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    t0 = HAL_GetTick();
    while (!lora_aux_idle() && (HAL_GetTick() - t0) < 100) { }

    rx_head = rx_tail = 0;   /* 丟掉設定模式殘留的位元組 */
    tx_busy = 0;
    LoRa_StartRx();
    return ret;
#endif
}
