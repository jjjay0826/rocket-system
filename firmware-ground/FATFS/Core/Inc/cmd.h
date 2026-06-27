/*
 * cmd.h
 * 
 * Command system for SD card operations
 */
#ifndef __CMD_H__
#define __CMD_H__

#include <stdint.h>

/* Process character input from UART */
void cmd_handle_char(uint8_t c);

/* Show help message */
void cmd_show_help(void);

/* Execute any pending command (call regularly from main loop) */
void cmd_execute_pending(void);

/* Flush deferred echo to UART1 + USB CDC (call regularly from main loop) */
void cmd_flush_echo(void);

/* Returns 1 if user has started typing a command but not yet pressed Enter */
uint8_t cmd_is_typing(void);

#endif // __CMD_H__
