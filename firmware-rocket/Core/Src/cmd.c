/*
 * cmd.c
 *
 * Command system — input via UART1 or USB CDC.
 * Key design: cmd_handle_char() is called from ISR/USB context, so it must
 * never block. Heavy work (SD read, printf) runs in cmd_execute_pending()
 * from the main loop.
 */
#include "cmd.h"
#include <string.h>
#include <stdio.h>
#include "logger.h"   /* includes fatfs.h → f_unlink, FIL, etc. */
#include "main.h"
#include "usart.h"
#include "lora_e22.h" /* BRIDGE 模式轉發用 LoRa_SendStr */

/* Provided by main.c (non-static); both go to their respective buses */
extern void uart1_write(const char *s);
extern void cdc_write(const char *s);

/* ---- Deferred echo ring buffer ----------------------------------------
 * cmd_handle_char() (USB/UART ISR) pushes raw bytes here.
 * cmd_flush_echo()  (main loop)    drains them in bulk to UART1 + CDC.
 * 8-bit atomic head/tail: safe for single-producer / single-consumer.
 * ----------------------------------------------------------------------- */
#define ECHO_Q_SIZE 128
static volatile uint8_t echo_q[ECHO_Q_SIZE];
static volatile uint8_t echo_head = 0;
static volatile uint8_t echo_tail = 0;

static void echo_push(uint8_t c)
{
  uint8_t next = (uint8_t)((echo_head + 1u) % ECHO_Q_SIZE);
  if (next != echo_tail) { echo_q[echo_head] = c; echo_head = next; }
}

static void echo_push_str(const char *s)
{
  while (*s) echo_push((uint8_t)*s++);
}

/* ---- Command line buffer ----------------------------------------------- */
#define CMD_BUF_SIZE 192   /* 128→192 (2026-07-20)：BRIDGE 模式要裝下 ~150 字元遙測行 */
static char cmd_buffer[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

/* Single-slot pending command (written by ISR, read by main loop) */
static volatile uint8_t cmd_pending = 0;
static char pending_cmd[CMD_BUF_SIZE];

/* ── BRIDGE 模式（2026-07-20）：USB CDC → LoRa 原樣轉發 ─────────────────
 * 用途：把火箭板當「USB-TTL + E22」dongle 用——地面站模擬回放
 * （tools/sim_replay.py --port <火箭板COM>）不需額外硬體。
 * 進入：BRIDGE 命令；退出：EXITBRIDGE 一行（或 reset）。
 * 模式中：每一行原樣 +\r\n 轉發 LoRa；main.c 據 cmd_bridge_active()
 * 靜音自身 2Hz 遙測（避免真假兩股資料混流）。 */
static volatile uint8_t bridge_mode = 0;
uint8_t cmd_bridge_active(void) { return bridge_mode; }

/* ---- Helper: command output → USB CDC ----------------------------------
 * 舊板 debug 走實體 UART1，曾「UART1+CDC 各送一份」；本板 uart1_write 已
 * 重導成 cdc_write，再成對呼叫會把每段字印兩次 → 只留一份。*/
static void cmd_out(const char *s)
{
  cdc_write(s);
}

/* ======================================================================
 * cmd_handle_char — called from ISR / USB receive context.
 * Must be non-blocking: only push to buffers, never call HAL_Delay or
 * CDC_Transmit_FS (CDC Transmit in CDC Receive ISR causes deadlock).
 * ====================================================================== */
void cmd_handle_char(uint8_t c)
{
  /* Discard garbage / non-printable bytes (e.g. floating UART1 RX noise).
   * Accept: printable ASCII 0x20-0x7E, CR, LF, BS, DEL. */
  if (!( c == 0x08 || c == 0x0A || c == 0x0D || c == 0x7F
         || (c >= 0x20 && c <= 0x7E) )) {
    return;
  }

  /* Overflow guard */
  if (cmd_idx >= CMD_BUF_SIZE - 1) {
    cmd_idx = 0;
    return;   /* silently reset — overflow can only happen with noise now */
  }

  /* Backspace / DEL */
  if (c == 0x08 || c == 0x7F) {
    if (cmd_idx > 0) {
      cmd_idx--;
      echo_push_str("\b \b");
    }
    return;
  }

  /* Echo the character back (deferred — drains in main loop) */
  echo_push(c);

  cmd_buffer[cmd_idx++] = c;

  /* Line complete */
  if (c == '\r' || c == '\n') {
    cmd_buffer[cmd_idx] = '\0';

    /* ★空行（\r\n 的 \n 那次觸發）絕對不能碰 pending_cmd 單槽——
     * 第一版修復只擋了 cmd_pending 旗標，但 pending_cmd[0]='\0' 照樣把
     * \r 那次剛存好的整行洗成空字串（ISR 連續處理 \r\n，主迴圈來不及
     * 消化）→ BRIDGE 拿到「旗標=1、內容=空」＝還是什麼都沒轉發
     * （2026-07-20 二修：地面 FT232 RX 仍不閃才揪到）。
     * 現在：有內容才複製、才設旗標；空行只重置行緩衝。 */
    int ii = 0, jj = 0;
    while (cmd_buffer[ii] == ' ' || cmd_buffer[ii] == '\t') ii++;
    if (cmd_buffer[ii] && cmd_buffer[ii] != '\r' && cmd_buffer[ii] != '\n') {
      while (cmd_buffer[ii] && cmd_buffer[ii] != '\r' && cmd_buffer[ii] != '\n'
             && jj < CMD_BUF_SIZE - 1)
        pending_cmd[jj++] = cmd_buffer[ii++];
      pending_cmd[jj] = '\0';
      cmd_pending = 1;
    }

    cmd_idx = 0;
    echo_push_str("\r\n");   /* ensure terminal goes to new line */
  }
}

/* ======================================================================
 * cmd_flush_echo — drain deferred echo buffer (main loop, safe context).
 * Collects all pending bytes into a local array and sends as one batch.
 * ====================================================================== */
void cmd_flush_echo(void)
{
  uint8_t h = echo_head;
  if (echo_tail == h) return;   /* nothing queued */

  char tmp[ECHO_Q_SIZE + 1];
  uint8_t n = 0;
  while (echo_tail != h) {
    tmp[n++] = (char)echo_q[echo_tail];
    echo_tail = (uint8_t)((echo_tail + 1u) % ECHO_Q_SIZE);
  }
  tmp[n] = '\0';

  cdc_write(tmp);
}

/* ======================================================================
 * cmd_show_help — show available commands on both UART1 and CDC.
 * ====================================================================== */
void cmd_show_help(void)
{
  cmd_out("\r\n");
  cmd_out("========== Rocket Avionics Commands ==========\r\n");
  cmd_out("READ   - Dump current SD log to terminal\r\n");
  cmd_out("TRUNC  - Trim log to real size (run before pulling card on bench)\r\n");
  cmd_out("CLEAR  - Close current log, start a new one\r\n");
  cmd_out("STATUS - System status summary\r\n");
  cmd_out("PINTEST- Sensor-bus GPIO probe (B13/B15 slow toggle, ~16s, then reboot)\r\n");
  cmd_out("PINHOLD- Static pin hold 3x60s phases for DMM current probing, then reboot\r\n");
  cmd_out("BUSFLOAT- Release SPI2 pins to Hi-Z until reboot (probe-safe mode)\r\n");
  cmd_out("-- Manual deploy (FIRES IGNITER - keep it disconnected on bench!) --\r\n");
  cmd_out("ARM        - arm manual deploy (replies a 4-digit code)\r\n");
  cmd_out("FIRE <code>- fire deploy within 10s of ARM (code must match)\r\n");
  cmd_out("SAFE       - disarm manual deploy\r\n");
  cmd_out("HELP   - Show this message\r\n");
  cmd_out("==============================================\r\n");
  cmd_out("Note: SD card saves data automatically.\r\n");
  cmd_out("      To retrieve log without USB, remove SD\r\n");
  cmd_out("      card and read log.txt on a PC.\r\n");
  cmd_out("==============================================\r\n\r\n");
  cmd_out("> ");
}

/* ======================================================================
 * process_command_exec — runs in main loop (blocking OK here).
 * ====================================================================== */
static void process_command_exec(const char *cmd)
{
  if (!cmd || cmd[0] == '\0') {
    cmd_out("> ");
    return;
  }

  if (strncmp(cmd, "READ", 4) == 0)
  {
    cmd_out("\r\n===== Reading SD Card Log (log.txt) =====\r\n");
    logger_read_all_uart();
    cmd_out("===== End of Log =====\r\n\r\n> ");
  }
  else if (strncmp(cmd, "HELP", 4) == 0)
  {
    cmd_show_help();
  }
  else if (strncmp(cmd, "CLEAR", 5) == 0)
  {
    /* 語意＝HELP 所述「關閉目前 log、滾動開下一個 logN.csv」。
     * 不刪任何舊檔（滾動檔名本來就不覆寫飛行資料）。
     * 舊版 f_unlink("log.txt") 是死操作：滾動檔名時代該檔不存在。
     * 關檔前先截斷 8MB 預分配尾巴，舊檔收斂成乾淨 CSV。 */
    cmd_out("Closing current log, starting a new one...\r\n");
    if (logger_trunc() != 0)
      cmd_out("[WARN] truncate failed - old log keeps 8MB tail\r\n");
    logger_close();
    logger_init();
    cmd_out(logger_is_ready() ? "New log started. SD ready.\r\n\r\n> "
                              : "[ERR] New log failed (SD not ready).\r\n\r\n> ");
  }
  else if (strncmp(cmd, "TRUNC", 5) == 0)
  {
    /* 桌面測試收尾：截掉 8MB 預分配尾巴，之後斷電拔卡即得乾淨 CSV */
    if (logger_trunc() == 0)
      cmd_out("\r\nLog truncated to real size. Power off & pull the card.\r\n\r\n> ");
    else
      cmd_out("\r\n[ERR] TRUNC failed (SD not ready?)\r\n\r\n> ");
  }
  else if (strncmp(cmd, "BRIDGE", 6) == 0)
  {
    /* USB→LoRa 透傳橋接：火箭板變 dongle（sim_replay.py 直插用）。
     * 自身遙測靜音由 main.c 讀 cmd_bridge_active() 處理。 */
    bridge_mode = 1;
    cmd_out("\r\nBRIDGE ON: every line -> LoRa verbatim. Own telemetry muted.\r\n"
            "Type EXITBRIDGE (or reset) to leave.\r\n");
  }
  else if (strncmp(cmd, "STATUS", 6) == 0)
  {
    char sb[128];
    snprintf(sb, sizeof(sb),
             "\r\n=== Status ===\r\n"
             "SD: %s\r\n"
             /* 舊板(imu 專案)是 SPI3/PB1；rocket_v2 兩顆感測器共用 SPI2，
              * IMU CS=PA8、BMP585 CS=PB12。字串留舊值會誤導故障排除。 */
             "IMU: LSM6DSOTR (SPI2, CS=PA8)\r\n"
             "Baro: BMP585 (SPI2)\r\n"
             "GNSS: ATGM336H (UART2, PA3)\r\n\r\n> ",
             logger_is_ready() ? "OK (log.txt open)" : "NOT READY");
    cmd_out(sb);
  }
  else if (strncmp(cmd, "PINTEST", 7) == 0)
  {
    /* 感測器匯流排腳位診斷（雙感測器同時無回應的硬體分案工具）：
     * 把 SEN_SCK(PB13)/SEN_MOSI(PB15) 暫時改成 1Hz 方波 GPIO，用三用電表
     * 就能看「MCU 推不推得動這條線」——分辨 MCU 輸出級損壞 vs 線上被
     * 外部元件鉗位（例如感測器輸入端 ESD 損傷漏電）。
     * 兩顆 CS（PB12/PA8）全程壓 HIGH：感測器都不被選中 → SEN_MISO(PB14)
     * 保證無人驅動；本測試也刻意不驅動 PB14，避免匯流排打架。
     * 結束以 NVIC_SystemReset() 復原全部周邊組態（SPI2 設定不用手動還原）。*/
    cmd_out("\r\nPINTEST: B13/B15 1Hz square wave x8 cycles (~16s)\r\n");
    cmd_out("Meter DC-V: B13 & B15 = 3.3V on HIGH, 0V on LOW\r\n");
    cmd_out("B12 & A8 (CS) held HIGH; B14 not driven\r\n");

    /* CS 先寫高再確保輸出模式（原本就是輸出高，重寫無害且自我防衛） */
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_N_GPIO_Port, IMU_CS_N_Pin, GPIO_PIN_SET);

    GPIO_InitTypeDef gi = {0};
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = GPIO_PIN_13 | GPIO_PIN_15;   /* SEN_SCK / SEN_MOSI 由 AF 轉手動 */
    HAL_GPIO_Init(GPIOB, &gi);

    for (int i = 1; i <= 8; i++) {
      char pb[48];
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
      snprintf(pb, sizeof(pb), "[%d/8] HIGH (expect 3.3V)\r\n", i);
      cmd_out(pb);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
      snprintf(pb, sizeof(pb), "[%d/8] LOW  (expect 0V)\r\n", i);
      cmd_out(pb);
      HAL_Delay(1000);
    }
    cmd_out("PINTEST done -> reboot to restore SPI2\r\n");
    if (logger_is_ready()) logger_close();   /* reset 打斷寫入會把卡鎖死（需斷電才復活）*/
    HAL_Delay(200);
    NVIC_SystemReset();
  }
  else if (strncmp(cmd, "PINHOLD", 7) == 0)
  {
    /* PINTEST 的靜態版（cross-check 修正：1Hz 方波可能被電表平均，
     * 且鉗位定位需要「跨 R23/R24 量壓降=電流」的穩態窗口）。
     * Phase1 60s：B13=HIGH、B15=LOW  → 量 R23 兩端對地 + 跨阻 mV
     * Phase2 60s：B13=LOW、B15=HIGH → 量 R24 兩端對地 + 跨阻 mV
     * Phase3 60s：B13/B14/B15 全改 analog 高阻抗（MCU 完全放手），CS 維持 HIGH
     *            → 匯流排自然電位（BMP 板內建 10k 上拉應能拉高健康的線）
     * 全程兩顆 CS 壓 HIGH；結束軟重開復原 SPI2。 */
    GPIO_InitTypeDef gi = {0};

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_N_GPIO_Port, IMU_CS_N_Pin, GPIO_PIN_SET);

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = GPIO_PIN_13 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gi);

    cmd_out("\r\nPINHOLD Phase1 (60s): B13=HIGH, B15=LOW\r\n");
    cmd_out("  Measure: R23 both ends vs GND + across R23 (mV)\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    for (int t = 60; t > 0; t -= 10) {
      char pb[32];
      snprintf(pb, sizeof(pb), "  P1 %ds left\r\n", t);
      cmd_out(pb);
      HAL_Delay(10000);
    }

    cmd_out("PINHOLD Phase2 (60s): B13=LOW, B15=HIGH\r\n");
    cmd_out("  Measure: R24 both ends vs GND + across R24 (mV)\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    for (int t = 60; t > 0; t -= 10) {
      char pb[32];
      snprintf(pb, sizeof(pb), "  P2 %ds left\r\n", t);
      cmd_out(pb);
      HAL_Delay(10000);
    }

    cmd_out("PINHOLD Phase3 (60s): B13/B14/B15 = analog Hi-Z (MCU released)\r\n");
    cmd_out("  Measure: bus SCK/SDI/SDO natural levels (BMP 10k pullups)\r\n");
    gi.Mode = GPIO_MODE_ANALOG;
    gi.Pull = GPIO_NOPULL;
    gi.Pin  = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gi);
    for (int t = 60; t > 0; t -= 10) {
      char pb[32];
      snprintf(pb, sizeof(pb), "  P3 %ds left\r\n", t);
      cmd_out(pb);
      HAL_Delay(10000);
    }

    cmd_out("PINHOLD done -> reboot\r\n");
    if (logger_is_ready()) logger_close();   /* 同 PINTEST：防 reset 打斷寫入鎖死卡 */
    HAL_Delay(200);
    NVIC_SystemReset();
  }
  else if (strncmp(cmd, "BUSFLOAT", 8) == 0)
  {
    /* B 板保護/量測模式：SPI2 三線改 analog Hi-Z、兩 CS 拉高，維持到重開機。
     * 背景：匯流排存在 ~1V 二極體式鉗位時，正常韌體 SPI2 閒置（SCK 恆高）
     * 會持續灌 ~46mA（遠超 GPIO 25mA 額定）。量測/剪腳驗證前先下此令：
     * 匯流排只剩模組自身上拉在餵，可安全探測自然電位。 */
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_N_GPIO_Port, IMU_CS_N_Pin, GPIO_PIN_SET);
    {
      GPIO_InitTypeDef gi = {0};
      gi.Mode = GPIO_MODE_ANALOG;
      gi.Pull = GPIO_NOPULL;
      gi.Pin  = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
      HAL_GPIO_Init(GPIOB, &gi);
    }
    cmd_out("\r\nBUSFLOAT: B13/B14/B15 -> analog Hi-Z, CS held HIGH (until reboot)\r\n"
            "Bus now fed only by module pullups - safe to probe.\r\n\r\n> ");
  }
  else if (strncmp(cmd, "ARM", 3) == 0 ||
           strncmp(cmd, "FIRE", 4) == 0 ||
           strncmp(cmd, "SAFE", 4) == 0)
  {
    /* 手動開傘（USB 通道）：共用 main.c 的安全核心；回覆走 CDC(cmd_out) */
    ManualDeploy_HandleLine(cmd, cmd_out);
    cmd_out("\r\n> ");
  }
  else
  {
    cmd_out("[ERR] Unknown: '");
    cmd_out(cmd);
    cmd_out("'  Type HELP\r\n\r\n> ");
  }
}

/* ======================================================================
 * cmd_is_typing — returns 1 if user has started but not yet submitted a line.
 * Main loop can use this to suppress telemetry while user is mid-command.
 * ====================================================================== */
uint8_t cmd_is_typing(void)
{
  return (cmd_idx > 0) ? 1u : 0u;
}

/* ======================================================================
 * cmd_execute_pending — call from main loop every iteration.
 * ====================================================================== */
void cmd_execute_pending(void)
{
  if (!cmd_pending) return;
  cmd_pending = 0;
  /* BRIDGE 模式：整行原樣轉發 LoRa（不進命令解析），EXITBRIDGE 退出 */
  if (bridge_mode) {
    if (strncmp(pending_cmd, "EXITBRIDGE", 10) == 0) {
      bridge_mode = 0;
      cmd_out("BRIDGE OFF - normal telemetry resumed\r\n\r\n> ");
    } else if (pending_cmd[0]) {
      char fwd[CMD_BUF_SIZE + 2];
      snprintf(fwd, sizeof(fwd), "%s\r\n", pending_cmd);
      LoRa_SendStr(fwd);
    }
    pending_cmd[0] = '\0';
    return;
  }
  process_command_exec(pending_cmd);
  pending_cmd[0] = '\0';
}
