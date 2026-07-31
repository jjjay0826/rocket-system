/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c   —— firmware-ground（地面接收端）
 * @brief   E22 LoRa 透傳接收 → USB CDC 轉發到電腦
 *
 *  硬體與火箭端相同：WeAct 黑丸版 STM32F411（HSE 25MHz）。
 *    USART2 = LoRa E22（透傳）           ← 見 usart.c 的 MX_USART2_UART_Init
 *    PA0 = LORA_M2（拉高進透傳模式）      ← 見 lora_bridge.c LoraBridge_Init
 *    PA1 = LORA_AUX（低電位=模組忙）
 *    USB CDC = 對電腦（虛擬 COM）
 *
 *  ⚠ 2026-07-31 更正：此處原本寫「USART1 (PA9=TX, PA10=RX)」，那是錯的
 *    —— 全專案只初始化 USART2，PA9/PA10 根本沒有被設定。程式一直是對的，
 *    錯的只有這段註解，但它會在查「收不到遙測」時把人帶去量錯的腳位，
 *    而且量到的會是浮接的腳，看起來就像模組壞了。
 *
 *  火箭端每 500ms 送一行遙測（以 "\r\n" 結尾）；本端收到整行就「原樣轉發」，
 *  不解析、不改動任何位元組——真正的解析器是電腦上的 Python 地面站。
 *  電腦串口會看到（欄位定義見 ../../shared/protocol.h）：
 *    T28386 SQ42 AX+0.007 ... ST:0 MOD:F GPS:1,8 C:0 VF8.12 VA7.98 LAT.. LON..
 *  （2026-07-30 更新：此處原本寫的 "N=.. T=.. P=.. S=.. M=.." 是 2026-06 之前
 *    的舊格式，早已不存在。本端是純橋接，格式換了也不用改碼，但註解會誤導人。）
 *
 *  註：此專案由火箭端複製精簡而來，已移除感測器/SD/GPS/開傘等飛行邏輯。
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora_bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ★2026-07-31：中斷優先權重排。
   *
   * CubeMX 把 OTG_FS 和 USART2 【都設成 0】。同優先權彼此無法搶佔，
   * 也就是「誰先進來誰做完」。USB 的 ISR 在列舉、控制傳輸時可以跑
   * 超過 1ms，而 9600 baud 下一個位元組正好是 1.04ms —— USART2 只有
   * 一個位元組的硬體緩衝，USB ISR 只要壓過 1.04ms，那個位元組就沒了
   * （ORE），而且【不會重傳】。
   *
   * 火箭端的排法（USB=0 最高）在那裡是對的：飛行時 USB 根本沒插，
   * 而 LoRa/GPS 的 ISR 比較重。地面端的取捨完全相反：
   *
   *     USART2 收到的位元組 = 這趟飛行唯一的資料，掉了就沒了
   *     USB 慢一點          = CDC 自己會重試，看不出差別
   *
   * 所以這裡讓 USART2 搶贏 USB。UART 的 ISR 只做「丟進 ring buffer +
   * 重新掛上接收」，量測不到的短（幾 µs），對 USB 的時序沒有實質影響。
   *
   * 寫在 USER CODE 區塊內而不是去改 usbd_conf.c / usart.c 的生成碼，
   * 是為了讓 CubeMX 重新產生時不會被洗掉。這裡跑在兩個 MX_*_Init
   * 之後，所以會蓋掉它們設的值。 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);   /* 最高：不能掉位元組 */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 0);   /* 次之：慢一點無所謂 */

  LoraBridge_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LoraBridge_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* ★2026-07-31：原本是 __disable_irq() + while(1)。
   * SystemClock_Config() 的兩個失敗分支都走到這裡，也就是說
   * 【HSE 晶振沒起來 = 這塊板永久磚死】，而且連 USB 都不會列舉，
   * 現場看到的就是「插上去沒有 COM port」。
   *
   * 改成重開機。若 HSE 真的壞了會變成重開迴圈 —— 但那反而是
   * 明確的症狀（COM port 反覆出現消失），比一片死寂好判讀太多。
   * 若只是上電瞬間的暫態（低溫、電壓爬升慢），重開一次就過了。 */
  NVIC_SystemReset();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
