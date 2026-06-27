/*
 * lora.c
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#include "main.h"
#include "lora.h"
#include <stdint.h>
#include <string.h>
extern SPI_HandleTypeDef hspi1;
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_NSS_Pin       GPIO_PIN_4

void LoRa_Select()   { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); }
void LoRa_Unselect() { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); }

void LoRa_Init(void)
{
  // 模擬初始化，可替換為實際 LoRa SX1268 初始化程式
  LoRa_Unselect();
  HAL_Delay(10);
}

void LoRa_SendString(const char* str)
{
  LoRa_Select();
  HAL_SPI_Transmit(&hspi1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
  LoRa_Unselect();
}

