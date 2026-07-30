/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "cmd.h"
#include "gnss.h"
#include "lora_e22.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* CSS(HSE 飛行中失效)會走 NMI 進來:清旗標並呼叫 HAL_RCC_CSSCallback
   * (main.c)降級 HSI 84MHz → 飛控續跑。 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  /* ★不可 while(1) 卡死:CSS 處理完必須返回,開傘狀態機才能繼續 */
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* ★2026-07-31：故障時重開機，不要停在原地 ────────────────────────────
   * 原本這裡是 `while (1) {}`，而本專案**沒有啟用任何看門狗**
   * （HAL_IWDG/WWDG_MODULE_ENABLED 都是註解掉的，.ioc 也沒配）。
   * 所以飛行中任何一次 hard fault → MCU 永遠卡住 → 傘不開 → 以終端速度墜落。
   *
   * 而 main.c 早就寫好了「異常重啟後的墜落救援」：非上電重置會設
   * postreset_watch=1，開機 3 秒後只要量到持續 >15 m/s 的下降就開傘。
   * 那條路徑本來就是為了「飛行中被打斷」設計的 —— 但沒有任何東西會讓
   * hard fault 產生 reset，所以它對最可能的觸發原因完全無效。
   *
   * 重開機把那條救援路徑接上了。這個改動不可能讓情況變糟：現況是永遠卡死，
   * 任何替代方案都比它好。而且它只在 MCU 已經失效時才動作，正常飛行碰不到。
   *
   * 沒有改用 IWDG 的理由：要挑一個不會誤觸的逾時值很難 ——
   * LoRa_SendStr 阻塞 650ms、logger_init 1s、sd_unstick_card 1.1s，
   * 疊起來得留 3~4 秒餘裕。飛行中誤觸看門狗比 hard fault 更可能發生。
   *
   * 代價：故障原因不會留下來。要除錯的話可在此把 SCB->CFSR/HFSR/BFAR
   * 存到 .noinit 區再重開，本次賽前不做（多的碼＝多的風險）。
   * 重開後 RST= 欄會顯示 SOFT，開機報告看得到。*/
  NVIC_SystemReset();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* 理由同 HardFault_Handler：沒有看門狗，卡住就永遠不會開傘。
   * 重開機讓 main.c 的墜落救援（postreset_watch + 持續下降偵測）接手。*/
  NVIC_SystemReset();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* 理由同 HardFault_Handler：沒有看門狗，卡住就永遠不會開傘。
   * 重開機讓 main.c 的墜落救援（postreset_watch + 持續下降偵測）接手。*/
  NVIC_SystemReset();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* 理由同 HardFault_Handler：沒有看門狗，卡住就永遠不會開傘。
   * 重開機讓 main.c 的墜落救援（postreset_watch + 持續下降偵測）接手。*/
  NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* ★ USARTx_IRQHandler 已由 CubeMX 生成（上方），不可在此重複定義，否則 link 衝突。
 * rocket_v2 UART 角色：USART1 = LoRa E22、USART2 = GPS。
 * 回呼依 instance 分派： */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* USART2 = GPS：GNSS_Process() 內部會重新 arm HAL_UART_Receive_IT */
    GNSS_Process();
  }
  else if (huart->Instance == USART1)
  {
    /* USART1 = LoRa E22 上行接收 */
    LoRa_OnRxByte();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* USART1 = LoRa E22 TX 完成 → 清 busy（非阻塞 TX）*/
    LoRa_OnTxDone();
  }
}

/* ★2026-07-31：UART 錯誤後把接收重新掛回去 ──────────────────────────────
 * HAL 把溢位（ORE）當成 blocking error：stm32f4xx_hal_uart.c 裡
 *
 *     if ((huart->ErrorCode & HAL_UART_ERROR_ORE) || dmarequest) {
 *         UART_EndRxTransfer(huart);      // 關掉 RXNE 中斷
 *         HAL_UART_ErrorCallback(huart);  // weak，本專案原本沒實作
 *     }
 *
 * 也就是說：一次溢位就讓接收**永久停止**，而且沒有任何錯誤訊息。
 *   · USART1（LoRa 上行）死掉 → 緊急 /dpl 從此打不進去。而遙測下行是另一個
 *     方向、還在跑，所以**鏈路看起來完全正常**，要等到按下開傘鈕看到
 *     UNCONFIRMED 才會發現。
 *   · USART2（GPS）死掉 → 位置資料中斷。
 *
 * 9600 baud 一個 byte 是 1.04ms，要溢位得有超過 1ms 的中斷延遲 —— 飛行中
 * （USB 拔掉、USART1 優先權 5、沒什麼在搶）機率低，桌測時（USB OTG 優先權 0
 * 會搶佔）比較高。機率不高但後果嚴重且靜默，而這個回呼只在接收「已經停掉」
 * 之後才會被叫到，加了不可能讓正常運作變糟。*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 清掉黏住的旗標（ORE 要讀 SR 再讀 DR 才會清）*/
  __HAL_UART_CLEAR_OREFLAG(huart);
  volatile uint32_t dummy = huart->Instance->SR;
  dummy = huart->Instance->DR;
  (void)dummy;
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  if (huart->Instance == USART1)      LoRa_RearmRx();
  else if (huart->Instance == USART2) GNSS_RearmRx();
}

/* USER CODE END 1 */
