/*
 * lora.h
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#ifndef __LORA_H
#define __LORA_H

#include "stm32f4xx_hal.h"

void LoRa_Init(void);
void LoRa_SendString(const char* str);

#endif

