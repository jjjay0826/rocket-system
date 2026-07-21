/**
  ******************************************************************************
  * @file           : lora_bridge.h
  * @brief          : Bidirectional forwarding bridge between USB CDC and E28 LoRa (USART2)
  ******************************************************************************
  */

#ifndef __LORA_BRIDGE_H
#define __LORA_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LORA_BRIDGE_BUF_SIZE 1024

void LoraBridge_Init(void);
void LoraBridge_Process(void);
void LoraBridge_UsbRxCallback(const uint8_t *buf, uint32_t len);
void LoraBridge_UartRxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_BRIDGE_H */
