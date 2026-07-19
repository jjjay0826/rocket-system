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

/* ── USB→LoRa 上行（手動開傘命令通道）─────────────────────────────────
 * 電腦在串口打的字（CDC_Receive_FS，USB ISR）進此 ring；主迴圈組成整行後
 * 經 LoRa 送給火箭端（火箭端 ManualDeploy 核心解析 ARM/FIRE/SAFE）。
 * ISR 單寫 head、主迴圈單讀 tail，8-bit 索引無鎖安全。*/
#define GND_URX_SIZE 128u
static volatile uint8_t  g_urx[GND_URX_SIZE];
static volatile uint16_t g_urx_head = 0;   /* CDC_Receive_FS(ISR) 寫 */
static uint16_t          g_urx_tail = 0;   /* 主迴圈讀 */

/* 由 usbd_cdc_if.c 的 CDC_Receive_FS 呼叫（USB ISR context，只推 ring 不阻塞）*/
void Ground_OnUsbRx(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((g_urx_head + 1u) % GND_URX_SIZE);
    if (next == g_urx_tail) break;          /* 滿：丟棄剩餘 */
    g_urx[g_urx_head] = buf[i];
    g_urx_head = next;
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
  cdc_write("=== Ground ready: E22<->USB (RX telemetry + TX uplink) ===\r\n");
  cdc_write("    Uplink: type a line (e.g. ARM / FIRE <code> / SAFE) -> rocket\r\n");

  char line[128];
  char ul[80]; uint8_t ul_len = 0;   /* USB→LoRa 上行組行緩衝 */
  uint32_t last_debug_time = HAL_GetTick();

  while (1)
  {
     if (HAL_GetTick() - last_debug_time > 2000)
    {
      last_debug_time = HAL_GetTick();
      
      // 外部宣告 lora_e22.c 內部的 head 與 tail 指標
      extern volatile uint16_t rx_head;
      extern volatile uint16_t rx_tail;
      extern volatile uint8_t rx_ring[];

      char debug_info[128];
      uint32_t sr = huart1.Instance->SR; // 取得 UART1 狀態暫存器
      
      sprintf(debug_info, "[Debug] Head:%d, Tail:%d, SR:0x%08lX, State:0x%02X\r\n", 
              rx_head, rx_tail, sr, huart1.RxState);
      cdc_write(debug_info);

      if (rx_head != rx_tail) {
          cdc_write("Mystery Bytes: ");
          char hex_str[8];
          uint16_t curr = rx_tail;
          // LORA_RXBUF_MASK 假設你是 256 大小，所以是 255 (0xFF)
          while (curr != rx_head) {
              sprintf(hex_str, "%02X ", rx_ring[curr]);
              cdc_write(hex_str);
              curr = (curr + 1) & 255; 
          }
          cdc_write("\r\n");
      }
    }

    /* 2. 原本的接收邏輯（下行遙測）*/
    int n = LoRa_Receive((uint8_t*)line, sizeof(line));
    if (n > 0)
    {
      cdc_write("[DATA] ");
      cdc_write(line);
      cdc_write("\r\n");
    }

    /* 3. 上行：把電腦串口打的整行（\r/\n 為界）經 LoRa 送給火箭端。
     *    火箭端 ManualDeploy 核心解析 ARM / FIRE <碼> / SAFE。
     *    加 '\n' 讓火箭端 LoRa_Receive 以此斷行。⚠ 與下行遙測共用 RF，
     *    偶發碰撞→命令沒回應時操作員重送即可（ARM 會回碼當隱式 ACK）。*/
    while (g_urx_tail != g_urx_head)
    {
      char c = (char)g_urx[g_urx_tail];
      g_urx_tail = (uint16_t)((g_urx_tail + 1u) % GND_URX_SIZE);
      if (c == '\r' || c == '\n')
      {
        if (ul_len > 0)
        {
          ul[ul_len] = '\0';
          cdc_write("[UPLINK] "); cdc_write(ul); cdc_write("\r\n");
          ul[ul_len] = '\n'; ul[ul_len + 1] = '\0';   /* 補斷行給 RF */
          LoRa_SendStr(ul);
          ul_len = 0;
        }
      }
      else if (ul_len < (uint8_t)(sizeof(ul) - 2))
      {
        ul[ul_len++] = c;
      }
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
