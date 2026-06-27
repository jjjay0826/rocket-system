/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LoRa Receiver - SX1268S433N0S1
  *
  * 功能：433 MHz LoRa 接收，SF7 BW125 CR4/5
  * 收到封包後透過 USART1（PA15, 9600bps）輸出到 PC Serial Monitor
  * 同樣可用 USB CDC 查看
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lora.h"

/* USB CDC（選用，若不接 USB 可刪掉） */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef  hspi3;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint32_t rx_count = 0;   /* 收到封包總數 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ---- UART 輸出小工具 ---- */
static void uart_write(const char *s)
{
    if (!s) return;
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 500);
}

/* ---- USB CDC 輸出（選用） ---- */
static void cdc_write(const char *s)
{
    if (!s || hUsbDeviceFS.pClassData == NULL) return;
    if (hUsbDeviceFS.dev_state < 3U) return;
    const uint8_t *p = (const uint8_t*)s;
    size_t rem = strlen(s);
    while (rem > 0) {
        uint16_t chunk = (uint16_t)(rem > 64 ? 64 : rem);
        for (int r = 0; r < 3; r++) {
            if (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_OK) break;
            HAL_Delay(5);
        }
        p += chunk; rem -= chunk;
        if (rem > 0) HAL_Delay(5);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
    /* MCU 初始化 */
    HAL_Init();
    SystemClock_Config();

    /* 周邊初始化 */
    MX_GPIO_Init();
    MX_SPI3_Init();
    MX_USART1_UART_Init();
    MX_USB_DEVICE_Init();   /* 若不用 USB 可刪此行 */

    /* USER CODE BEGIN 2 */

    HAL_Delay(500);  /* 等待 USB 穩定 */

    /* ---- LoRa 初始化 ---- */
    {
        int lr = LoRa_Init();
        char b[48];
        snprintf(b, sizeof(b), "\r\nLORA RX INIT: %s\r\n", lr == 0 ? "OK" : "FAIL");
        uart_write(b);
        cdc_write(b);
    }

    /* ---- 進入連續接收模式 ---- */
    LoRa_StartRx();
    uart_write("Listening on 433 MHz SF7 BW125...\r\n");
    cdc_write("Listening on 433 MHz SF7 BW125...\r\n");

    /* USER CODE END 2 */

    /* ================================================================
     * 主迴圈：每 10ms 輪詢一次 SX1268 IRQ 旗標
     * ================================================================ */
    while (1)
    {
        /* USER CODE BEGIN WHILE */

        uint8_t rx_buf[256];
        int n = LoRa_Receive(rx_buf, sizeof(rx_buf) - 1);

        if (n > 0)
        {
            rx_buf[n] = '\0';   /* 字串結尾，方便 printf */
            rx_count++;

            char out[320];
            int len = snprintf(out, sizeof(out),
                               "[#%lu] %s\r\n",
                               (unsigned long)rx_count,
                               (char*)rx_buf);

            if (len > 0 && len < (int)sizeof(out)) {
                uart_write(out);
                cdc_write(out);
            }
        }
        else if (n == -1)
        {
            /* CRC 錯誤 */
            uart_write("[CRC ERR]\r\n");
            cdc_write("[CRC ERR]\r\n");
        }

        HAL_Delay(10);  /* 10ms 輪詢間隔 */

        /* USER CODE END WHILE */
    }
}

/**
  * @brief System Clock Configuration
  * @note  與 rocket sensor 完全相同的時脈設定（100MHz, HSE 25MHz）
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 200;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
