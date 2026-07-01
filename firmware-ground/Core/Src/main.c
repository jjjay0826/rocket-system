/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c   —— firmware-ground（地面接收端）
 * @brief   E22 LoRa 透傳接收 → USB CDC 轉發到電腦
 *
 *  硬體與火箭端相同：WeAct 黑丸版 STM32F411（HSE 25MHz）。
 *    USART1 (PA9=TX→E22 RXD, PA10=RX←E22 TXD) = LoRa E22（透傳，M0/M1 接 GND）
 *    USB CDC = 對電腦（虛擬 COM）
 *
 *  火箭端每 500ms 送一行遙測（以 '\n' 結尾）；本端收到整行就「原樣轉發」，
 *  電腦串口即可看到：N=.. T=.. P=.. RH=.. KH=.. G=.. S=.. M=..
 *
 *  註：此專案由火箭端複製精簡而來，已移除感測器/SD/GPS/開傘等飛行邏輯。
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "lora_e22.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

void SystemClock_Config(void);

/* ── USB CDC 輸出（與火箭端相同做法：分塊 64B + 重試 3 次）───────────── */
void cdc_write(const char *s)
{
  if (!s || hUsbDeviceFS.pClassData == NULL) return;
  if (hUsbDeviceFS.dev_state < 3U) return;   /* 未 CONFIGURED 不送 */
  const uint8_t *p = (const uint8_t*)s;
  size_t remaining = strlen(s);
  while (remaining > 0)
  {
    uint16_t chunk = (uint16_t)(remaining > 64 ? 64 : remaining);
    for (int r = 0; r < 3; r++)
    {
      if (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_OK) break;
      HAL_Delay(5);
    }
    p += chunk;
    remaining -= chunk;
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();   /* E22 LoRa */
  MX_USB_DEVICE_Init();    /* 對電腦的 USB CDC */
  LoRa_Init();             /* 啟動 USART1 接收（ring buffer）*/

  HAL_Delay(500);
  cdc_write("=== Ground RX ready (E22 -> USB CDC) ===\r\n");

 
  char line[128];
  uint32_t last_debug_time = HAL_GetTick();

  while (1)
  {
     if (HAL_GetTick() - last_debug_time > 2000)
    {
      last_debug_time = HAL_GetTick();
      
      // 外部宣告 lora_e22.c 內部的 head 與 tail 指標
      extern volatile uint16_t rx_head;
      extern volatile uint16_t rx_tail;
      
      char debug_info[128];
      uint32_t sr = huart1.Instance->SR; // 取得 UART1 狀態暫存器
      
      sprintf(debug_info, "[Debug] Head:%d, Tail:%d, SR:0x%08lX, State:0x%02X\r\n", 
              rx_head, rx_tail, sr, huart1.RxState);
      cdc_write(debug_info);
    }

    /* 2. 原本的接收邏輯 */
    int n = LoRa_Receive((uint8_t*)line, sizeof(line));
    if (n > 0)
    {
      cdc_write("[DATA] ");
      cdc_write(line);
      cdc_write("\r\n");
    }
    
  }
}

/**
  * @brief System Clock Configuration（HSE 25MHz → 84MHz，與火箭端一致）
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* 不 disable_irq，保 USB 中斷存活；LED 快閃示警 */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_B10_GPIO_Port, LED_B10_Pin);
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
