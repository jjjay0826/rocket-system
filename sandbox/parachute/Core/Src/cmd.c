/*
 * cmd.c
 *
 * Command system — input via USB CDC (UART1 on this board is the LoRa header;
 * uart1_write is aliased to CDC in main.c).
 * Key design: cmd_handle_char() is called from ISR/USB context, so it must
 * never block. Heavy work (SD read, printf) runs in cmd_execute_pending()
 * from the main loop.
 */
#include "cmd.h"
#include <string.h>
#include <stdio.h>
#include "logger.h"   /* includes fatfs.h → f_unlink, FIL, etc. */
#include "main.h"

/* Provided by main.c (non-static) */
extern void uart1_write(const char *s);
extern void cdc_write(const char *s);

/* ---- Deferred echo ring buffer ----------------------------------------
 * cmd_handle_char() (USB ISR)  pushes raw bytes here.
 * cmd_flush_echo()  (main loop) drains them in bulk to CDC.
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
#define CMD_BUF_SIZE 128
static char cmd_buffer[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

/* Single-slot pending command (written by ISR, read by main loop) */
static volatile uint8_t cmd_pending = 0;
static char pending_cmd[CMD_BUF_SIZE];

/* ---- Helper: output to USB CDC ----------------------------------------- */
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
  /* Discard garbage / non-printable bytes.
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

    /* Copy trimmed command to pending slot */
    int ii = 0, jj = 0;
    while (cmd_buffer[ii] == ' ' || cmd_buffer[ii] == '\t') ii++;
    while (cmd_buffer[ii] && cmd_buffer[ii] != '\r' && cmd_buffer[ii] != '\n'
           && jj < CMD_BUF_SIZE - 1)
      pending_cmd[jj++] = cmd_buffer[ii++];
    pending_cmd[jj] = '\0';

    cmd_idx = 0;
    echo_push_str("\r\n");   /* ensure terminal goes to new line */
    cmd_pending = 1;
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
 * cmd_show_help — show available commands.
 * ====================================================================== */
void cmd_show_help(void)
{
  cmd_out("\r\n");
  cmd_out("====== Parachute Drop-Test Commands ======\r\n");
  cmd_out("READ   - Dump current LOG_xxx.TXT to terminal\r\n");
  cmd_out("CLEAR  - Erase current log file on SD card\r\n");
  cmd_out("STATUS - System status summary\r\n");
  cmd_out("HELP   - Show this message\r\n");
  cmd_out("===========================================\r\n");
  cmd_out("Note: SD card saves data automatically.\r\n");
  cmd_out("      IMU_xxx.CSV (500Hz) + LOG_xxx.TXT pair\r\n");
  cmd_out("      per boot. Remove card to read on a PC.\r\n");
  cmd_out("===========================================\r\n\r\n");
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
    cmd_out("\r\n===== Reading SD Card Log =====\r\n");
    logger_read_all_uart();
    cmd_out("===== End of Log =====\r\n\r\n> ");
  }
  else if (strncmp(cmd, "HELP", 4) == 0)
  {
    cmd_show_help();
  }
  else if (strncmp(cmd, "CLEAR", 5) == 0)
  {
    cmd_out("Clearing SD log...\r\n");
    logger_close();
    f_unlink(logger_name());   /* 刪目前序號檔（舊版寫死 "log.txt" 刪不到）*/
    logger_init();
    cmd_out(logger_is_ready() ? "Log cleared. SD ready.\r\n\r\n> "
                              : "Log cleared (SD not ready).\r\n\r\n> ");
  }
  else if (strncmp(cmd, "STATUS", 6) == 0)
  {
    char sb[160];
    snprintf(sb, sizeof(sb),
             "\r\n=== Status (black pill carrier) ===\r\n"
             "SD:   %s (%s, SPI1 CS=PA4)\r\n"
             "IMU:  LSM6DS3 (GY module, SPI2, CS=PA8)\r\n"
             "Baro: BMP585 (SPI2, CS=PB12)\r\n\r\n> ",
             logger_is_ready() ? "OK" : "NOT READY",
             logger_name());
    cmd_out(sb);
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
  process_command_exec(pending_cmd);
  pending_cmd[0] = '\0';
}
