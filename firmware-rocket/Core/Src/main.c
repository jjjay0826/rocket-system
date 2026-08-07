/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (USB CDC + robust clock + IMU test)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "lora_e22.h"
#include "gnss.h"
#include "sdcard.h"
#include "logger.h"
#include "bmp585.h"
#include "cmd.h"

/* USB CDC */
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
/* ---- 開傘參數（改這裡即可調整） ---- */
/* 開傘發火腳 = CubeMX label「FIRE_7V_2」(PA1 → 7V_OUT2 一路)
 * ★2026-07-31 起這個巨集不再是唯一的點傘腳位 —— 氣囊移除後 PA0 也併進了
 *   傘迴路，兩支腳必須同時驅動。所有點火請走下面的 deploy_fire_on/off()。*/
#define DEPLOY_PORT     FIRE_7V_2_GPIO_Port
#define DEPLOY_PIN      FIRE_7V_2_Pin

/* ★★★ 2026-07-31 硬體改動：氣囊取消，PA0（原 7V_OUT1 氣囊路）已改接進
 *     降落傘發火迴路 —— 傘要 PA0 與 PA1 【同時】被驅動才會點著。
 *
 * 因此所有點傘路徑一律走下面這兩個函式，不要再單獨操作任何一支腳。
 * 原本散在五個地方各寫一次 HAL_GPIO_WritePin，改硬體時漏掉任何一處，
 * 那條路徑就會靜靜地變成「拉了腳但傘不會開」—— 而且測不出來，因為
 * 遙測只看得到 flight_state 有進 DEPLOYING。集中成一個函式才擋得住。
 *
 * 收尾同時拉低兩支：脈衝長度沿用 DEPLOY_PULSE_MS。            */
static inline void deploy_fire_on(void)
{
  HAL_GPIO_WritePin(FIRE_7V_1_GPIO_Port, FIRE_7V_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DEPLOY_PORT,         DEPLOY_PIN,    GPIO_PIN_SET);
}
static inline void deploy_fire_off(void)
{
  HAL_GPIO_WritePin(FIRE_7V_1_GPIO_Port, FIRE_7V_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DEPLOY_PORT,         DEPLOY_PIN,    GPIO_PIN_RESET);
}
#define LAUNCH_AZ_G  2.5f       /* 離架偵測合加速度閾值 (g)。1.3→2.5 (2026-07-20)：
                                 * OpenRocket 模擬推力段峰值 6g、2.5g 約 t+1.7s 達成
                                 * （比 1.3g 僅晚 ~0.35s，20s 備援起點幾乎不動）；
                                 * 1.3g 地面搬運甩動就可能連續 200ms 超標＝誤判離架
                                 * →20s 後備援點火（無 arming 下極危險），2.5g 甩不出來 */
#define DEPLOY_PULSE_MS  1000UL   /* GPIO HIGH 持續時間 ms (所有點火/降落傘/氣囊/桌測統一由本變數決定) */
/* ---- 開傘觸發條件（改這裡調整） ----
 * 邏輯：(A AND B) OR C
 *   A: 氣壓高度低於最高點 DEPLOY_DROP_M 以上
 *   B: 垂直速度持續向下超過 DEPLOY_VZ_NEG_MS ms
 *   C: 飛行時間超過 DEPLOY_TB_MS（備援）
 */
#define DEPLOY_DROP_M      10.0f    /* A: 最高點後下墜觸發落差 (m) */
#define DEPLOY_PEAK_MIN_M  20.0f    /* A: 最高點需達此高度才啟用，防地面誤觸 (m) */
#define DEPLOY_VZ_NEG_THR  -0.5f    /* B: kf2_v 低於此值視為「持續向下」(m/s) */
#define DEPLOY_VZ_NEG_MS   1500UL   /* B: 速度向下持續門檻 (ms) */
#define DEPLOY_TB_MS       18000UL  /* C: 備援強制觸發時間 (ms)。20s→18s (2026-07-20
                                     * 定案，逐狀態驗證過)：Pioneer-5K 實測曲線 ±10%
                                     * 包絡模擬 apogee=16.1~17.95s（風況<0.1s 除名）；
                                     * LAUNCHED 判定實測 ≈1.3s（2.5g+200ms，曲線爬升陡）
                                     * → C 點火 ≈19.3s＝最晚 apogee+1.34s（防上升中點火）
                                     * ＝最早 apogee+3.2s（21.7m/s）。+10% 端 C 比主路徑
                                     * A∧B 早 0.2s 以 12m/s 開傘（良性交叉保護）；兩端
                                     * 實際開傘速度皆 ≤14m/s。 */
#define AIRBAG_IMPACT_G  5.0f     /* 落海/觸地撞擊偵測：DEPLOYED 下 total_g 超過
                                   * 此值＝觸水/觸地減速 spike → 自動充氣氣囊。
                                   * 下降穩定 ≈1g、傘擺盪 ≤3g；落水(終端 6.2m/s、
                                   * 減速 ~0.3m)≈6.5g → 5.0 有 margin 又防空中擺盪
                                   * 誤觸。實際落水/落地測試後可校。 */
/* ── 氣囊撞擊偵測的兩道防誤觸（2026-07-30）───────────────────────────
 * 【主】ARM_DELAY：開傘衝擊必定超過 5g，而且時序剛好撞在一起。
 *   開傘瞬間下降速度 6~37 m/s（81 組模擬），傘繩拉伸行程約 0.25m
 *   → a = v²/2s = 7g ~ 279g。連最溫和的那組都超過門檻。
 *   而 DEPLOYING→DEPLOYED 只等 DEPLOY_PULSE_MS(1s)，傘完全張開卻要
 *   0.5~1.5s ——衝擊峰值有相當機率落在 DEPLOYED 之後，直接誤觸，
 *   氣囊在數百公尺高空充氣，而 airbag_auto_fired 是一次性的：
 *   真正落海時就沒氣囊了。
 *   本延遲從「點火時刻」起算（deploy_time_ms），涵蓋整段開傘過程。
 *   代價：傘下 12m/s × 5s ≈ 少 60m 高度。開傘高度 800~1300m，可忽略；
 *   落海發生在數十秒之後，完全不受影響。
 * 【副】IMPACT_MS：濾掉單一樣本毛刺與自旋暫態。落水減速持續約 100ms
 *   （6.2m/s / 0.3m），IMU 每 10ms 更新一次 → 30ms 需連續 3 筆，抓得到。
 *   注意這道防護擋不掉開傘衝擊（同為數十~百 ms 等級事件），
 *   真正擋開傘衝擊的是上面的 ARM_DELAY。 */
#define AIRBAG_ARM_DELAY_MS 5000UL /* 進 DEPLOYED 後再靜默這麼久才開始監看撞擊 */
#define AIRBAG_IMPACT_MS      30UL /* total_g 需持續超標這麼久才算撞擊 */
#define IMU_ARM_G       1.5f      /* IMU 積分啟動閾值 (g)，測試用可調低 */
#define MAH_2KP         1.0f      /* Mahony 比例增益 × 2 */
#define MAH_2KI         0.01f     /* Mahony 積分增益 × 2 */

/* ---- IMU 量程刻度 ----
 * ±16g（CTRL1_XL=0x64）：1g = 2048 LSB
 * ★ 火箭推力段 5~20g，±2g 會飽和削平 → 速度積分從點火就錯，故必用 ±16g */
#define IMU_ACC_SCALE  2048.0f

/* ---- 氣壓 1D Kalman（壓力平滑）---- */
#define KF_Q  0.5f    /* 過程雜訊方差 (hPa²)：越大追蹤越快 */
#define KF_R  0.01f   /* 量測雜訊方差 (hPa²)：BMP585 RMS≈0.03hPa */

/* ---- 2D Kalman 高度/速度估計器（移植自 imu 專案，飛行適配）----
 * 狀態 x = [高度 kf2_h, 速度 kf2_v]
 * 預測步（100Hz）用 IMU 垂直加速度，更新步（50Hz）用氣壓高度。 */
#define KF2_Q_H   0.0001f   /* 高度過程雜訊 */
#define KF2_Q_V   0.001f    /* 速度過程雜訊（imu 專案實證值，原 0.10 會讓 K1 爆炸）*/
#define KF2_R_H   0.25f     /* 氣壓高度量測雜訊方差 (±0.5m RMS) */
/* ── 飛行專屬適配（投放測試版沒有，照抄會出事）── */
#define KF2_R_HIGHG_MULT  25.0f   /* 高g(推力段)膨脹 R：震動/穿音速時少信氣壓 */
#define KF2_RESET_THR_M    5.0f   /* 大偏差重置門檻 (m) */
#define KF2_RESET_GMAX     1.5f   /* 只在 total_g < 此值才允許重置→對齊氣壓
                                   * （推力段氣壓是垃圾，禁止對齊，全信 IMU）*/
#define KF2_AZ_CLAMP     160.0f   /* lin_az sanity 限幅 (m/s²,≈±16g)
                                   * 只擋解碼錯誤，不砍真實推力（不可設 ±3g！）*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;  /* USART1 = LoRa E22（debug 改走 USB CDC）*/
extern UART_HandleTypeDef huart2;  /* USART2 = GPS */
/* 感測器 SPI：BMP585 + LSM6DSO 共用 SPI2 */
extern SPI_HandleTypeDef hspi2;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ── pyro 電源監測 ADC（PB1=保險絲後端、PB0=arming 開關後端）─────────────
 * 全部寫在 USER CODE 區塊內（含 GPIO/ADC 初始化），CubeMX regenerate 不會蓋掉。
 * 分壓 22k/10k：8.4V(2S 滿電) → 2.63V，安全落在 3.3V 以下。
 *  PB1(IN9)：保險絲是否熔斷。誤觸發時電流走 safety shunt 燒斷保險絲＝點火頭
 *            沒被點著（人安全），但整條 pyro 電源同時死亡——發射台上外觀看
 *            不出來，飛上去才發現開不了傘。此值 0V 即為熔斷證據。
 *  PB0(IN8)：arming 開關後端＝「已武裝」的實體證據。滿足競賽規範 4.6.7
 *            「驗證啟動狀態時人員無需靠近火箭 100mm 內（可透過電腦連線確認）」。
 *  ※ 純量測、不驅動任何東西：讀錯也不會讓 pyro 走火，安全仍由機械 arming
 *     開關實體斷路提供（規範 4.6.3 要求的第 2 個「獨立」事件）。 */
/* 🔴 分壓電路（22k/10k/0.1uF ×2 組）實際焊上去之前，保持註解！
 *    未接線時 PB0/PB1 浮空，ADC 讀到的是隨機雜訊：可能落在 0V 附近 →
 *    地面站誤報「保險絲熔斷」，也可能落在高處 → 誤報「已武裝」。兩種都是
 *    比沒有功能更糟的假訊號。停用時遙測送 -1（＝本板無此量測能力），
 *    地面站完全不顯示。硬體裝好後取消註解、重編燒錄即可。 */
/* ★2026-08-01 關閉 —— 分壓【實際上沒有焊】。
 * 原註解寫「2026-07-27 已焊上」，經現場確認是錯的。
 * 開著的話 PB0/PB1 讀到浮接雜訊（0.0~0.5V），遙測會把它當成真實電壓送出去
 * —— 而 5.0V 以下就是「保險絲熔斷」，等於永久謊報一個最該告警的故障。
 * 關掉之後 PyroADC_Init 直接 return，v_fuse/v_arm 維持 -1，
 * 遙測送 -1 =「沒有量測能力」，與「量到 0V＝熔斷」明確區分。*/
// #define PYRO_ADC_FITTED

static uint8_t  pyro_adc_ok = 0;      /* ADC 初始化成功旗標 */
static float    v_fuse      = -1.0f;  /* PB1 保險絲後端電壓（-1=尚未量測）*/
static float    v_arm       = -1.0f;  /* PB0 arming 開關後端電壓 */
static uint32_t t_pyro_adc  = 0;      /* 上次量測時刻（1Hz 足夠，電壓變化慢）*/

/* 分壓還原倍率：(R1+R2)/R2 = (22k+10k)/10k = 3.2 */
#define PYRO_DIV_RATIO   ((22000.0f + 10000.0f) / 10000.0f)
/* 出廠校正的 VREFINT 原始值（3.3V 下量得），用來反推「現在 VDDA 到底幾伏」。
 * 直接假設 3.3V 會帶進穩壓器 ±1.5% 誤差；判斷熔斷無妨，看電池餘量就會偏。*/
#define VREFINT_CAL_ADDR ((uint16_t*)0x1FFF7A2AU)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── pyro 電源監測 ADC 實作（直接暫存器操作，不經 HAL ADC 模組）─────────
 * ★ 為什麼不用 HAL：本專案 stm32f4xx_hal_conf.h 的 HAL_ADC_MODULE_ENABLED
 *   是註解掉的（CubeMX 專案沒啟用 ADC），呼叫 HAL_ADC_* 會直接編譯失敗。
 *   改用暫存器後整個功能自足於 USER CODE 區塊：不必改 hal_conf.h、不必動
 *   .ioc，CubeMX regenerate 也不會把這功能弄掉。ADC 暫存器本身很單純。
 * 時鐘：PCLK2 = 84MHz，ADCPRE=/4 → 21MHz（F411 上限 36MHz）。
 *       112+12 cycles ÷ 21MHz ≈ 5.9us/次。 */
#define ADC_SMP_112CYC   5U   /* 取樣時間欄位編碼：101 = 112 cycles → 5.33us @21MHz */
#define ADC_SMP_480CYC   7U   /* 111 = 480 cycles → 22.9us @21MHz */
#define ADC_CH_FUSE      9U   /* PB1 */
#define ADC_CH_ARM       8U   /* PB0 */
#define ADC_CH_VREFINT  17U
#define ADC_RAW_INVALID 0xFFFFU  /* 轉換失敗（≠ 量到 0V，兩者不可混為一談）*/

static uint16_t pyro_adc_raw(uint32_t channel);   /* 開機自檢要先用到 */

/* PB0/PB1 設為類比輸入 + 啟用 ADC1。失敗時 pyro_adc_ok=0，量測值維持 -1
 * （遙測送 -1 = 「沒有量測能力」，與「量到 0V＝熔斷」明確區分）。*/
static void PyroADC_Init(void)
{
#ifndef PYRO_ADC_FITTED
  pyro_adc_ok = 0;   /* 硬體未裝：不初始化，v_fuse/v_arm 維持 -1（無能力）*/
  return;
#else
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* PB0/PB1 → 類比模式（MODER = 0b11），只動這兩支腳的欄位 */
  GPIOB->MODER |= (3U << (0U * 2U)) | (3U << (1U * 2U));

  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE_Msk) | (1U << ADC_CCR_ADCPRE_Pos); /* /4 */
  ADC->CCR |= ADC_CCR_TSVREFE;      /* 開內部參考電壓通道 */

  ADC1->CR1 = 0;                    /* 12-bit、不掃描 */
  ADC1->CR2 = 0;                    /* 右對齊、單次轉換、軟體觸發 */
  ADC1->SQR1 = 0;                   /* 序列長度 = 1 */

  /* 取樣時間：SMPR2 管 ch0-9、SMPR1 管 ch10-18 */
  ADC1->SMPR2 = (ADC1->SMPR2 & ~((7U << (ADC_CH_ARM * 3U)) | (7U << (ADC_CH_FUSE * 3U))))
              | (ADC_SMP_112CYC << (ADC_CH_ARM * 3U))
              | (ADC_SMP_112CYC << (ADC_CH_FUSE * 3U));
  /* ★VREFINT 必須用 480 cycles：F411 datasheet 規定內部參考電壓的最小取樣
   *   時間 TS_vrefint = 10us，而 112 cycles 在 21MHz 下只有 5.33us —— 取樣
   *   電容充不飽 → 讀值偏低 → VDDA 被高估 → 兩路電壓一起被放大。
   *   480 cycles = 22.9us，符合規格。外部分壓走 112 cycles 即可（源阻抗
   *   6.9k 遠低於內部參考的等效阻抗）。*/
  ADC1->SMPR1 = (ADC1->SMPR1 & ~(7U << ((ADC_CH_VREFINT - 10U) * 3U)))
              | (ADC_SMP_480CYC << ((ADC_CH_VREFINT - 10U) * 3U));

  ADC1->CR2 |= ADC_CR2_ADON;        /* 上電 */
  HAL_Delay(1);                     /* ADC + VREFINT 穩定時間（需 >10us）*/

  /* ★開機自檢：ADON 寫下去不代表 ADC 真的會動。先試轉一次 VREFINT，讀值
   *   落在合理帶內才認定可用。否則 pyro_adc_ok 恆為 1，任何轉換失敗都會
   *   被下游當成「量到 0V」＝謊報保險絲熔斷。
   *   VREFINT 標稱 1.21V → 1.21/3.3*4095 ≈ 1500 counts；連同 ±3% 元件公差
   *   與 VDDA 漂移取寬帶 1200~1900，只求排除「完全沒在動」的情況。*/
  pyro_adc_ok = 1;                  /* 暫時設 1，讓下面的試轉能執行 */
  { uint16_t t = pyro_adc_raw(ADC_CH_VREFINT);
    pyro_adc_ok = (t != ADC_RAW_INVALID && t > 1200U && t < 1900U) ? 1U : 0U; }
#endif
}

/* 單通道取樣。112 cycles：分壓源阻抗 22k∥10k≈6.9k 偏高，取樣電容要夠時間
 * 充飽，短取樣時間會讀出偏低值。迴圈上限保護：轉換僅 ~6us，這裡給約兩個
 * 數量級餘裕就跳出，避免 ADC 異常時卡死主迴圈（開傘狀態機不能停）。*/
static uint16_t pyro_adc_raw(uint32_t channel)
{
  /* guard 5000：480 cycles 的轉換 @21MHz 約 23.4us，本迴圈每圈約 10 cycles
   * @84MHz → 5000 圈約 0.6ms，仍有 25 倍餘裕，卻把 ADC 故障時的最壞停頓從
   * 舊值(100000 圈≈12ms/次、每秒三次≈36ms)壓到 1.8ms。*/
  uint32_t guard = 5000U;
  if (!pyro_adc_ok) return ADC_RAW_INVALID;
  ADC1->SQR3 = channel & 0x1FU;
  ADC1->SR  &= ~ADC_SR_EOC;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC) && --guard) { __NOP(); }
  /* ★超時回傳專用哨兵值，不可回 0——0 在下游是「量到 0V」＝保險絲熔斷，
   *   把「量不到」謊報成「熔斷」會讓發射台上的人去換一顆好的保險絲。*/
  if (!guard) return ADC_RAW_INVALID;
  return (uint16_t)(ADC1->DR & 0x0FFFU);
}

/* 用內部參考電壓反推實際 VDDA（穩壓器不會剛好 3.300V）*/
static float pyro_adc_vdda(void)
{
  uint16_t cal = *VREFINT_CAL_ADDR;
  uint16_t raw = pyro_adc_raw(ADC_CH_VREFINT);
  if (raw == ADC_RAW_INVALID || raw == 0 || cal == 0 || cal == 0xFFFF)
    return 3.3f;                              /* 讀不到→退回標稱值 */
  return 3.3f * (float)cal / (float)raw;
}

/* 讀一路分壓並還原成電池端真實電壓。回 -1 = 讀不到（與 0V 熔斷不同）。*/
static float pyro_read_volt(uint32_t channel, float vdda)
{
  if (!pyro_adc_ok) return -1.0f;
  uint16_t raw = pyro_adc_raw(channel);
  if (raw == ADC_RAW_INVALID) return -1.0f;
  return ((float)raw / 4095.0f) * vdda * PYRO_DIV_RATIO;
}

/* 1Hz 更新兩路電壓。電壓變化很慢、熔斷是一次性事件，不需要跟著 100Hz 主迴圈。*/
static void PyroADC_Poll(uint32_t now)
{
  if (!pyro_adc_ok) return;
  if (now - t_pyro_adc < 1000UL) return;
  t_pyro_adc = now;
  float vdda = pyro_adc_vdda();
  v_fuse = pyro_read_volt(ADC_CH_FUSE, vdda);   /* PB1 */
  v_arm  = pyro_read_volt(ADC_CH_ARM,  vdda);   /* PB0 */
}

/* ---- 飛行狀態機 ---- */
typedef enum {
    FLIGHT_IDLE,       /* 地面靜置，等待離架 */
    FLIGHT_LAUNCHED,   /* 離架後，監視高度與時間 */
    FLIGHT_DEPLOYING,  /* 開傘訊號輸出中（由 DEPLOY_PULSE_MS 決定，預設 1 秒） */
    FLIGHT_DEPLOYED,   /* 開傘完成，等待落地 */
    FLIGHT_LANDED      /* 落地確認，進入低功耗記錄模式 */
} FlightState_t;

static FlightState_t flight_state   = FLIGHT_IDLE;
static uint32_t      launch_time_ms = 0;   /* HAL_GetTick() at launch */
static uint32_t      deploy_time_ms = 0;   /* HAL_GetTick() at deploy */

/* ---- 落地偵測參數 ---- */
/* 【③】氣壓計存活證明：離架後這麼久，高度必須爬過下面的門檻，否則判定
 * 「讀數合理但不跟隨高度」（氣孔遮蔽）→ baro_untrusted，備援退回純計時。
 * 動力段 5 秒早已數百公尺，10m 門檻極寬鬆，正常飛行不可能誤判。*/
#define BARO_PROOF_MS      5000UL   /* 離架後多久開始要求證明 */
#define BARO_PROOF_MIN_M   10.0f    /* 此時峰值高度至少要有這麼多 */

/* 【④】異常重啟後的「墜落救援」──────────────────────────────────────
 * 飛行中一次 reset（撞擊電源瞬斷，本專案已實證）會把 flight_state 打回 IDLE，
 * 而重新偵測離架需要 2.5g/200ms —— 引擎早燒完了，滑行段與下墜段都 ≤1g，
 * 於是**永遠回不到 LAUNCHED**，18 秒備援根本不計時，傘再也不會開。
 *
 * ★關鍵洞見：救援不需要任何「撐過斷電的記憶體」。
 *   RTC 備份暫存器在本板救不了主要情境（VBAT 直接接 VDD，沒有獨立電池，
 *   撞擊斷電時備份domain 一起斷），SRAM 保留也不可靠。
 *   但**下降速度是「變化率」，不依賴 ref_press 是否正確**——就算重開機把
 *   空中的氣壓當成了「地面」（使 rel_alt≈0），d(rel_alt)/dt 仍然忠實反映
 *   真實的垂直速度。所以：重啟後若持續量到高速下降，那就是一枚正在墜落的
 *   火箭，直接開傘。
 * 門檻選擇：傘下降速率依規範 4.2.5 必須 <12 m/s（高空空氣稀薄約 12.7 m/s），
 *   自由落體則是 30~100 m/s。取 15 m/s 可乾淨分離兩者：已開傘者不會被誤觸，
 *   自由落體必然命中。地面上要誤觸得從 11 公尺高摔下來。*/
#define POSTRESET_FALL_VZ  -15.0f   /* 下降快於此（m/s，負值向下）*/
#define POSTRESET_FALL_MS   1000UL  /* 需持續這麼久 */
#define POSTRESET_ARM_MS    3000UL  /* 開機後這麼久才開始監看（等濾波器穩定）*/
/* 【②】推測式離架的撤銷：氣壓計復活且高度確實還在地面，持續這麼久 → 退回 IDLE */
#define REVOKE_HOLD_MS     3000UL
#define REVOKE_ALT_M       10.0f

#define LAND_G_DEV_THR    0.15f     /* |total_g - 1g| < 此值視為靜止 */
#define LAND_ALT_THR      30.0f     /* rel_alt < 此值才考慮落地（排除彈出傘在高空靜止）*/
#define LAND_STABLE_MS    10000UL   /* 需連續 10s 滿足條件才確認落地 */
#define LAND_LOG_INTERVAL 5000UL    /* 落地後每 5s 寫一筆 LoRa beacon */
static uint32_t land_stable_start = 0;  /* 靜止條件開始計時的 tick */
static float         ref_press      = 1013.25f; /* 地面氣壓，啟動時初始化 */
/* 時鐘健康旗標：0=HSE 正常、1=開機 HSE 起振失敗已降級 HSI、2=飛行中 CSS 觸發降級。
 * 實案：黑丸版 HSE 晶振故障 → 舊碼死在 Error_Handler(燈都沒設)=全暗磚死無從診斷 */
volatile uint8_t     clk_hsi_fallback = 0;
/* 匯流排健檢旗標：1=開機時 SPI2 三線有線被鉗低（晶片電源軌塌陷/模組未裝），
 * 三線已鎖 analog Hi-Z、感測器初始化跳過。實案：B 板 VDDIO 鉗位 46mA 事故 */
static uint8_t       bus_clamped    = 0;
/* 【③】氣壓計讀值不跟隨高度（氣孔遮蔽）→ 1。單向閂鎖，見 BARO_PROOF_MS。*/
static uint8_t       baro_untrusted = 0;
/* 【②】此次離架是「推測」的（降級路徑）還是 2.5g 實測到的。
 * 推測式離架若被證明是誤判，可由下方邏輯撤銷退回 IDLE——否則 flight_state
 * 全檔沒有任何回 IDLE 的路徑，一次誤判就永久閂死，真正發射時 peak 一過 20m
 * 立刻觸發備援 → 在推力段約 20m、數十 m/s 開傘 → 結構解體。*/
static uint8_t       launch_inferred = 0;
static uint32_t      revoke_start    = 0;
/* 【④】上次不是正常上電（BROWNOUT/SOFT/IWDG/NRST）→ 1，啟用墜落救援監看 */
static uint8_t       postreset_watch = 0;
static uint32_t      fall_start_ms   = 0;
static float         rel_alt        = 0.0f;     /* 相對高度（m） */

/* =========================
   Debug helpers
   ========================= */
void cdc_write(const char *s);   /* 前向宣告，下方定義 */
/* rocket_v2：USART1 改作 LoRa，debug 不能再走 UART → 一律導向 USB CDC。
 * 保留 uart1_write 名稱，cmd.c/logger.c 等舊呼叫點無需改動。 */
void uart1_write(const char *s)
{
  if (!s) return;
  cdc_write(s);
}


void cdc_write(const char *s)
{
  if (!s || hUsbDeviceFS.pClassData == NULL) return;
  /* CONFIGURED=3, SUSPENDED=4：兩者都允許傳送 */
  if (hUsbDeviceFS.dev_state < 3U) return;

  const uint8_t *p = (const uint8_t*)s;
  size_t remaining = strlen(s);

  /* ★2026-07-31：主機不在時不要空轉 ──────────────────────────────────────
   * 舊碼每個 64-byte chunk 最多燒 3×5 + 5 = 20ms。USB 拔掉之後這筆開銷不會
   * 消失：本板關掉了 VBUS sensing，拔線後 dev_state 落在 SUSPENDED(4)，
   * 而上面的判斷是 `< 3 才 return`，也就是 SUSPENDED 仍會嘗試傳送。
   * 沒有主機把 IN 端點的資料取走，CDC_Transmit_FS 從第二次起固定回 BUSY
   * → 每個 chunk 都跑滿重試。500ms 一次的狀態輸出約 200 字元＝4 個 chunk
   * → **飛行中每 500ms 有 80ms 卡在這裡**。
   *
   * 開傘的計時邏輯全部用 HAL_GetTick() 差值，不會因為迴圈變慢而算錯，
   * 但 IMU 每 10ms 取樣會連續漏掉 8 筆，卡爾曼積分跟著變粗。
   *
   * 修法：連續失敗到一定次數就認定「沒有主機在讀」，之後每個 chunk 只試一次
   * 不再等待；任何一次成功就立刻恢復完整重試。主機在的時候行為完全不變。*/
  static uint8_t cdc_dead = 0;      /* 連續失敗計數，達 CDC_DEAD_N 後進省略模式 */
  #define CDC_DEAD_N 6

  while (remaining > 0)
  {
    uint16_t chunk = (uint16_t)(remaining > 64 ? 64 : remaining);
    int tries = (cdc_dead >= CDC_DEAD_N) ? 1 : 3;
    int ok = 0;
    for (int r = 0; r < tries; r++)
    {
      if (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_OK) { ok = 1; break; }
      if (r + 1 < tries) HAL_Delay(5);
    }
    if (ok) cdc_dead = 0;
    else if (cdc_dead < CDC_DEAD_N) cdc_dead++;

    p += chunk;
    remaining -= chunk;
    if (remaining > 0 && cdc_dead < CDC_DEAD_N) HAL_Delay(5);
  }
} 

/* =========================
   LSM6DSOTR via SPI2, CS = PA8 (active low)  [rocket_v2]
   ========================= */
#define IMU_CS_PORT IMU_CS_N_GPIO_Port   /* CubeMX label「IMU_CS_N」= PA8 */
#define IMU_CS_PIN  IMU_CS_N_Pin
static inline void IMU_CS_LOW(void)  { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static inline void IMU_CS_HIGH(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);   }

/* 讀取單一暫存器
 * SPI 協議：位元 7 = 1 表示讀取，位元 7 = 0 表示寫入
 * 先送暫存器地址（含讀取位），再收 1 byte 資料（存在 rx[1]） */
static uint8_t lsm6_read_reg(uint8_t reg)
{
  uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };  /* bit7=1：讀取模式 */
  uint8_t rx[2] = { 0 };
  IMU_CS_LOW();
  (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  IMU_CS_HIGH();
  return rx[1];  /* rx[0] 為送出地址時的虛擬回傳，忽略 */
}

/* 寫入單一暫存器
 * bit7 = 0：寫入模式；val 為要寫入的值 */
static void lsm6_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };  /* bit7=0：寫入模式 */
  IMU_CS_LOW();
  (void)HAL_SPI_Transmit(&hspi2, tx, 2, 100);
  IMU_CS_HIGH();
}

/* 連續讀取多個暫存器（Burst Read）
 * 需先在 CTRL3_C 設定 IF_INC=1，才能自動遞增地址
 * start_reg：起始暫存器地址；buf：接收緩衝；len：讀取 byte 數
 * 回傳 0=成功，-1=SPI 錯誤，-9=緩衝超出限制 */
static int lsm6_read_burst(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx[32];
  uint8_t rx[32];
  if (len + 1 > sizeof(tx)) return -9;  /* 防止 tx[] 溢位 */

  tx[0] = (uint8_t)(start_reg | 0x80);  /* bit7=1：讀取，CTRL3_C.IF_INC 負責地址自動遞增 */
  memset(&tx[1], 0x00, len);             /* 補零（SPI 主機送 0x00 以驅動時脈） */

  IMU_CS_LOW();
  HAL_StatusTypeDef ok = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1), 200);
  IMU_CS_HIGH();

  if (ok != HAL_OK) return -1;
  memcpy(buf, &rx[1], len);  /* rx[0] 為地址 byte 的虛擬回傳，跳過 */
  return 0;
}

static int lsm6_init(void)
{
  HAL_Delay(20);

  uint8_t who = lsm6_read_reg(0x0F);
  char b[64];
  int n = snprintf(b, sizeof(b), "WHO=%02X\r\n", who);
  if (n < 0) {
    memcpy(b, "WHO=ERR\r\n", sizeof("WHO=ERR\r\n"));
    b[sizeof(b)-1] = '\0';
  } else if ((size_t)n >= sizeof(b)) {
    b[sizeof(b)-1] = '\0';
  }
  cdc_write(b);

  /* WHO_AM_I 接受整個 LSM6DS 家族（暫存器圖相容）：
   * 0x69=LSM6DS3, 0x6A=LSM6DS3TR-C/DSL, 0x6B=LSM6DSR, 0x6C=LSM6DSO/DSOX
   * v7 用 GY-LSM6DS3 模組，晶片可能是 0x69/0x6A，不能只認 0x6C。 */
  if (who < 0x69 || who > 0x6C) return -10;

  lsm6_write_reg(0x12, 0x01);  /* CTRL3_C: SW_RESET=1，軟體重置，清除所有暫存器 */
  HAL_Delay(50);               /* 等待重置完成（資料表建議 ≥ 50µs，用 50ms 較保守） */

  lsm6_write_reg(0x12, 0x44); /* CTRL3_C: BDU=1（讀完整組才更新輸出）, IF_INC=1（Burst Read 地址自動遞增）*/
  lsm6_write_reg(0x10, 0x64); /* CTRL1_XL: ODR_XL=416Hz, FS_XL=±16g（火箭推力段不飽和；1g=2048LSB）*/
  lsm6_write_reg(0x11, 0x4C); /* CTRL2_G:  ODR_G=104Hz,  FS_G=2000dps（火箭翻滾不超量程）*/
  /* ── CTRL9_XL / CTRL10_C 只對 LSM6DS3 世代有效（2026-07-30）──────────
   * 0x20~0x2D 的資料區整個 LSM6DS 家族相容，但這兩個控制暫存器不相容：
   *   LSM6DS3/DS3TR-C/DSL (WHO 0x69/0x6A/0x6B)
   *     CTRL9_XL bit5:3 = Zen/Yen/Xen_XL   → 0x38 = 開啟加速度三軸
   *     CTRL10_C bit5:3 = Zen/Yen/Xen_G    → 0x38 = 開啟陀螺三軸
   *     （兩者都是 SW_RESET 後的預設值，這裡等於明示重申）
   *   LSM6DSO/DSOX (WHO 0x6C)
   *     每軸開關已取消（三軸恆開），位址改作他用：
   *     CTRL9_XL 預設 0xE0，寫 0x38 會清掉 DEN_X/Y/Z 並開啟 DEN_XL_EN
   *     資料戳記；CTRL10_C 只剩 bit5=TIMESTAMP_EN，0x38 還會踩到兩個
   *     保留位元。→ 這顆晶片不該寫，跳過即可（三軸本來就開著）。
   * 本載具 BOM 是 LSM6DS3，走上面那條；保留分支讓兩種晶片都能上板。 */
  if (who <= 0x6B) {
    lsm6_write_reg(0x18, 0x38); /* CTRL9_XL: Zen/Yen/Xen_XL=1，啟用加速度計三軸輸出 */
    lsm6_write_reg(0x19, 0x38); /* CTRL10_C: Zen/Yen/Xen_G=1，啟用陀螺儀三軸輸出 */
  }
  return 0;
} 

// 讀 0x20 開始：TEMP(2) + G(6) + A(6) ＝ 14 bytes
static int lsm6_read_raw_LSM6DSOTR(int16_t *temp,
                                  int16_t *gx, int16_t *gy, int16_t *gz,
                                  int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t d[14] = {0};

  // OUT_TEMP_L = 0x20
  int rr = lsm6_read_burst(0x20, d, 14);
  if (rr != 0) return rr;

  *temp = (int16_t)((d[1]  << 8) | d[0]);   // TEMP
  *gx   = (int16_t)((d[3]  << 8) | d[2]);   // Gx
  *gy   = (int16_t)((d[5]  << 8) | d[4]);   // Gy
  *gz   = (int16_t)((d[7]  << 8) | d[6]);   // Gz
  *ax   = (int16_t)((d[9]  << 8) | d[8]);   // Ax
  *ay   = (int16_t)((d[11] << 8) | d[10]);  // Ay
  *az   = (int16_t)((d[13] << 8) | d[12]);  // Az

  return 0;
}

/* ======================================================
   Mahony 互補濾波器（全局狀態）
   ====================================================== */
static float q0=1.f,q1=0.f,q2=0.f,q3=0.f;   /* 四元數 */
static float mah_ix=0.f,mah_iy=0.f,mah_iz=0.f; /* 積分項 */

/* Mahony 互補濾波器更新
 * 原理：用加速度計量測的「重力方向」校正陀螺儀積分，
 *       以 PI 控制器消除四元數姿態的緩慢漂移。
 * ax,ay,az：原始加速度（g）；gx,gy,gz：角速度（rad/s）；dt：時間步長（s） */
static void mahony_update(float ax,float ay,float az,
                          float gx,float gy,float gz,float dt)
{
  /* ① 加速度計正規化：轉為單位向量（純方向，大小不影響校正）*/
  float n=sqrtf(ax*ax+ay*ay+az*az);
  if(n<0.001f)return;  /* 近零重力（自由落體）：跳過，避免除零 */
  n=1.f/n; ax*=n;ay*=n;az*=n;

  /* ② 由當前四元數推算「重力在體座標系的預期方向」（旋轉矩陣第三列）*/
  float vx=2.f*(q1*q3-q0*q2);
  float vy=2.f*(q0*q1+q2*q3);
  float vz=q0*q0-q1*q1-q2*q2+q3*q3;

  /* ③ 加速度計量測值 × 預期重力方向 → 叉積誤差（體座標系下的姿態偏差）*/
  float ex=ay*vz-az*vy;
  float ey=az*vx-ax*vz;
  float ez=ax*vy-ay*vx;

  /* ④ PI 校正：積分項（I）消除靜態偏差，比例項（P）提供即時修正 */
  mah_ix+=ex*MAH_2KI*dt; mah_iy+=ey*MAH_2KI*dt; mah_iz+=ez*MAH_2KI*dt;
  gx+=MAH_2KP*ex+mah_ix; gy+=MAH_2KP*ey+mah_iy; gz+=MAH_2KP*ez+mah_iz;

  /* ⑤ 四元數微分方程積分（半角速度形式，q̇ = 0.5 × q ⊗ ω）*/
  gx*=0.5f*dt; gy*=0.5f*dt; gz*=0.5f*dt;
  float qa=q0,qb=q1,qc=q2;
  q0+=(-qb*gx-qc*gy-q3*gz);
  q1+=(qa*gx+qc*gz-q3*gy);
  q2+=(qa*gy-qb*gz+q3*gx);
  q3+=(qa*gz+qb*gy-qc*gx);

  /* ⑥ 四元數正規化：補償數值誤差累積，保持 |q|=1 */
  n=1.f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0*=n;q1*=n;q2*=n;q3*=n;
}

/* 旋轉體座標系加速度到世界系，提取垂直分量（Z軸向上） */
static float world_az(float ax,float ay,float az)
{
  return 2.f*(q1*q3-q0*q2)*ax
        +2.f*(q2*q3+q0*q1)*ay
        +(q0*q0-q1*q1-q2*q2+q3*q3)*az;
}

/* ======================================================
   IMU 積分狀態
   ====================================================== */
static float  az_bias_g   = 0.0f;  /* 靜止時垂直加速度偏差 (g) */
static uint8_t imu_armed  = 0;     /* g > 閾值後鎖定，持續積分 */
static uint32_t g2_count  = 0;     /* 連續 >LAUNCH_AZ_G 的 10ms 計數 */
static uint32_t sd_write_cnt = 0;  /* SD 成功寫入次數 */
static uint8_t  imu_ok    = 0;     /* IMU 初始化是否成功 */

/* ── 2D Kalman 估計器狀態（取代原 PI 互補 imu_hz/imu_vz）──────
 * kf2_h/kf2_v：融合高度/速度；kf2_pXX：2×2 協方差矩陣 */
static float kf2_h   = 0.0f, kf2_v   = 0.0f;
static float kf2_p00 = 1.0f, kf2_p01 = 0.0f, kf2_p11 = 1.0f;
static float vz_baro_lp = 0.0f;        /* 氣壓微分速度 (LP τ=1s)：KF2 重置初值 + 交叉驗證 */

/* ── 開傘條件 A/B 狀態 ──────────────────────────────── */
static float    peak_rel_alt    = 0.0f; /* 飛行中氣壓最高點 (m) */
static uint32_t vz_neg_start_ms = 0;   /* kf2_v 轉負的起始 tick */
static uint8_t  cond_A          = 0;   /* A: 氣壓高度低於最高點 10m（裸氣壓）*/
static uint8_t  cond_B          = 0;   /* B: KF2 速度持續向下 1.5s */

/* ---- 模組存活狀態 ---- */
typedef struct {
    uint8_t bmp585  : 1;   /* 氣壓計 */
    uint8_t imu     : 1;   /* IMU */
    uint8_t lora    : 1;   /* LoRa */
    uint8_t sdcard  : 1;   /* SD 卡 */
} ModStatus_t;
static ModStatus_t mod      = {0};   /* 預設全部失效 */
static uint8_t bmp_err_cnt  = 0;     /* BMP 連續失敗計數 */
static uint8_t imu_err_cnt  = 0;     /* IMU 連續失敗計數 */
static uint8_t  lora_err_cnt = 0;     /* LoRa 連續失敗計數 */
static uint32_t lora_seq     = 0;     /* 傳送序號（接收端可偵測掉包）*/
static uint32_t lora_ok      = 0;     /* 成功傳送次數 */
static uint32_t lora_fail    = 0;     /* 失敗傳送次數 */
static uint32_t main_loop_cnt = 0;     /* 主迴圈計數器 */
static uint8_t  is_boosting  = 0;     /* 是否處於推力段 (BOOST) */
#define MOD_ERR_MAX  5               /* 連續 N 次失敗 → 標記死亡 */

/* ══════════════════════════════════════════════════════════════════════
 * 開傘條件的「有效值」：感測器故障時的退化規則
 * ══════════════════════════════════════════════════════════════════════
 * 這條公式原本在三個地方各抄一份（開傘決策、遙測 C 欄、USB 狀態顯示），
 * 抽成函式，避免日後改一處漏兩處讓遙測說謊。
 *
 * ── 2026-07-30：兩個方向的退化風險不對稱，不該再對稱處理 ──────────
 * 舊碼是 `mod.bmp585 ? cond_A : (mod.imu ? 1 : 0)`，也就是「氣壓計死掉就
 * 把 A 視為成立，讓 cond_B 單獨決定開傘」。但兩顆感測器的可信度差一個
 * 數量級：
 *
 *   · IMU 死 → cond_A 單獨守門。cond_A 是純氣壓的「低於峰值 10m」，不含
 *     任何積分，沒有漂移；peak_rel_alt 是單調閂鎖，假高點只會把門檻推高
 *     ＝讓開傘變晚（安全方向）；而且氣壓路徑有突變閘門(10m/20ms)、防鎖死
 *     (連拒 25 次強制重錨)、凍結看門狗三層防護。→ 可以信任，維持原樣。
 *
 *   · 氣壓死 → cond_B 單獨守門。cond_B 用的 kf2_v 是「IMU 積分 ＋ 氣壓
 *     修正」的融合值，氣壓一死就失去唯一的修正源。更糟的是滑行段接近自由
 *     落體，加速度計讀值 ≈0，Mahony 連重力參考都沒有，姿態只剩陀螺積分
 *     十幾秒。此時 kf2_v 是累積漂移量，不是量測值。用它一票決定開傘，最壞
 *     情況是在推力段誤觸 → 解體。→ 不可信，關閉主路徑。
 *
 * 關掉之後由備援 C 接手：C 在 t_det+18s 觸發，81 組模擬實測落在頂點後
 * 0.58~3.80s，比一個漂移中的濾波器可預測得多。這是拿「不確定」換「確定」，
 * 不是失去一條路徑。
 * 兩顆皆死時兩者都是 0（避免立即觸發），同樣只靠 C。 */
static inline int deploy_A_eff(void) { return mod.bmp585 ? cond_A : 0; }
static inline int deploy_B_eff(void) { return mod.imu ? cond_B : (mod.bmp585 ? 1 : 0); }

/* ======================================================
   氣壓計 Kalman 濾波器
   ====================================================== */
static float kf_p_est  = 1013.25f;  /* 估計氣壓 */
static float kf_p_err  = 1.0f;      /* 估計誤差 */

/* 一維 Kalman 濾波器更新（用於氣壓值平滑）
 * 狀態：kf_p_est = 估計氣壓；kf_p_err = 估計誤差方差
 * meas：BMP585 新量測值（hPa）；回傳：濾波後氣壓（hPa） */
static float kf_update(float meas)
{
  kf_p_err += KF_Q;                        /* 預測步：誤差方差因過程雜訊增大 */
  float K = kf_p_err / (kf_p_err + KF_R); /* Kalman 增益：量測可信度越高 K 越大 */
  kf_p_est += K * (meas - kf_p_est);      /* 更新步：用量測殘差修正估計值 */
  kf_p_err *= (1.f - K);                  /* 更新誤差方差（K 越大，方差收斂越快）*/
  return kf_p_est;
}

/* ═══════════════════════════════════════════════════════════════════════
 * 手動開傘（USB CDC + LoRa 共用核心）
 * 兩段式安全：ARM → 回覆「動態 4 位通關碼」→ 限時 10s 內 'FIRE <碼>' 才點火。
 *   · 防單一亂碼/RF 雜訊誤觸（需先 ARM，且碼必須精確吻合）
 *   · 防重放（碼每次 ARM 由 tick 產生而變動；逾時自動解除）
 *   · 已在 DEPLOYING/DEPLOYED 則拒絕重複觸發
 * 點火走與自動開傘「相同」的 DEPLOYING 路徑 → 共用 DEPLOY_PULSE_MS 脈衝收尾邏輯。
 * ⚠ 手動開傘「刻意」繞過自動開傘的 1.3g/高度安全閘門——這是人工 override
 *   的本質，責任在操作員。地面測試務必電火頭斷開；真正的地面安全靠「電火頭
 *   發射台最後接」＋兩段式流程。未桌面驗證前勿信賴。
 * ═══════════════════════════════════════════════════════════════════════ */
static volatile uint8_t manual_armed   = 0;
static uint32_t         manual_arm_code = 0;
static uint32_t         manual_arm_time = 0;
/* ── ARM 逾時採狀態感知（2026-07-20：緊急開傘時效 vs 誤觸防護的折衷）──
 * 地面(IDLE) 30s：發射倒數時 ARM 備妥，容忍短暫 hold；
 * 飛行中 300s：ARM 存活整段飛行 → 緊急時「單發 FIRE <碼>」即開傘（~1-2s），
 * 不必在下墜的 10 秒裡跑完兩步。動態碼驗證完整保留——同頻他隊/雜訊發的
 * FIRE 沒有碼，照樣被拒。威脅模型：逾時防的是「操作員 ARM 後分心殘留」，
 * 飛行中 armed 待命正是備援要的狀態，放長是特性不是漏洞。 */
#define MANUAL_ARM_TIMEOUT_IDLE_MS    30000UL
#define MANUAL_ARM_TIMEOUT_FLIGHT_MS 300000UL
static uint32_t manual_arm_timeout(void)
{
  return (flight_state == FLIGHT_IDLE) ? MANUAL_ARM_TIMEOUT_IDLE_MS
                                       : MANUAL_ARM_TIMEOUT_FLIGHT_MS;
}

/* LoRa 回覆包裝：LoRa_SendStr 回傳 int，包成 reply 回呼要的 void(const char*) */
static void lora_cmd_reply(const char *s) { (void)LoRa_SendStr(s); }

/* ★2026-07-31：氣囊整組移除。
 * 原本這裡有 abg_active / abg_fire_ms / airbag_auto_fired 三個狀態，
 * 以及撞擊偵測與 LANDED 兩個自動充氣觸發點，全部刪除。
 *
 * 刪掉而不是「留著但不觸發」的理由：PA0 現在是降落傘發火迴路的一半。
 * 任何殘存的「單獨拉 PA0」路徑都會在錯誤的時機半驅動傘迴路 —— 就算
 * 這次接線下它點不著，也是一條沒有人會再驗證的活路徑。寧可讓編譯器
 * 幫忙找出所有引用點。
 */

/* ── 【①】地面測試模式（2026-07-28）────────────────────────────────────
 * 原始用途（2026-07-28）是擋住「桌測跑到 LANDED 就自動充氣氣囊」。
 * ★2026-07-31 氣囊移除後，自動充氣那條路徑已經不存在，但這個模式仍要
 *   保留，因為它現在扛兩件事：
 *     ① PB6 手動發火鈕的三道閘門之一（IDLE + ARM + GNDTEST）
 *     ② cmd.c 維修指令（PINTEST/BRIDGE…）的地面限定
 *   同時它仍是規範 4.5.3「回收系統應在模擬觸發條件下地面測試」的入口。
 * 10 分鐘自動逾時不變 —— 忘了關也不會帶上發射台。 */
#define GND_TEST_WINDOW_MS  600000UL   /* 10 分鐘後自動失效，忘了關也不會帶上發射台 */
static uint32_t gnd_test_until = 0;
/* 給 cmd.c 用：維修指令（PINTEST/PINHOLD/BUSFLOAT/READ/CLEAR/TRUNC/BRIDGE）
 * 只有在地面靜置時才准跑。它們會阻塞主迴圈 16~180 秒，或永久靜音遙測，
 * 而開傘狀態機、18 秒備援計時、感測器更新全都在同一個迴圈裡。
 * flight_state 是 static，所以用函式而不是 extern 變數。*/
uint8_t flight_is_idle(void) { return (uint8_t)(flight_state == FLIGHT_IDLE); }

static inline uint8_t gnd_test_active(void)
{
  return (gnd_test_until != 0) && (HAL_GetTick() < gnd_test_until);
}

/* ═══════════════════════════════════════════════════════════════════════
 * ★★★ 桌測解禁開關（2026-07-20 使用者要求）★★★
 * 定義時：dpl/abg 跳過全部閘門（IDLE ARM 解鎖、上升 10s、already
 * deployed/landed），且 dpl 改走「裸脈衝」（不動 flight_state）＝正式 key
 * 可無限重複點火。TEST_FIRE 命令已移除，測試直接用正式 key。
 * ★ 復原＝把下面這行註解掉重編譯（#warning 會在每次編譯提醒；開機
 *   訊息也會廣播 UNRESTRICTED 警示，防止帶著解禁版上天）。
 * ═══════════════════════════════════════════════════════════════════════ */
/* ★2026-08-01 發射前關閉。開著時 dpl/abg 沒有任何閘門 —— 不需要 ARM、
 * 不管飛行狀態、可無限重複點火。那是桌測用的。
 * 關閉後恢復：IDLE 需先 ARM、離架後 10 秒內拒收、已開傘/已落地拒收。*/
// #define REMOTE_CMD_UNRESTRICTED
#ifdef REMOTE_CMD_UNRESTRICTED
#warning "REMOTE_CMD_UNRESTRICTED is ACTIVE - dpl/abg have NO safety gates. NOT FLIGHT SAFE."
#endif

/* dpl 裸脈衝狀態（解禁模式用）：不動 flight_state，2s 由 Poll 收尾 */
static volatile uint8_t dpl_pulse_active = 0;
static uint32_t         dpl_pulse_ms     = 0;

/* 主迴圈每輪呼叫：ARM 逾時自動解除，縮小誤觸窗口；獨立脈衝收尾 */
void ManualDeploy_Poll(void)
{
  if (manual_armed && (HAL_GetTick() - manual_arm_time) > manual_arm_timeout())
    manual_armed = 0;
  if (dpl_pulse_active && (HAL_GetTick() - dpl_pulse_ms) >= DEPLOY_PULSE_MS) {
    deploy_fire_off();
    dpl_pulse_active = 0;
  }
}

/* 處理一行命令（ARM / FIRE <碼> / SAFE）。回覆經 reply 回呼送回來源通道
 * （USB→cmd_out / LoRa→lora_cmd_reply）。非命令行忽略（LoRa 雜訊不誤動作）。*/
void ManualDeploy_HandleLine(const char *line, void (*reply)(const char *))
{
  if (!line || !reply) return;
  char rb[96];

  /* ARM：手打短令與地面站秘鑰版皆可（#CMD 版=格式統一，2026-07-20）*/
  if ((strncmp(line, "ARM", 3) == 0 &&
       (line[3] == '\0' || line[3] == '\r' || line[3] == '\n' || line[3] == ' '))
      || strcmp(line, "#CMD:ARM_SYSTEM_SALT7763#") == 0) {
    manual_arm_code = HAL_GetTick() % 10000UL;
    manual_arm_time = HAL_GetTick();
    manual_armed    = 1;
    snprintf(rb, sizeof(rb),
      "MSG INFO MANUAL ARMED - FIRE %04lu within %lus (SAFE aborts)\r\n",
      (unsigned long)manual_arm_code,
      (unsigned long)(manual_arm_timeout() / 1000UL));
    reply(rb);
    return;
  }
  if (strncmp(line, "SAFE", 4) == 0) {
    manual_armed = 0;
    reply("MSG INFO MANUAL SAFE (disarmed)\r\n");
    return;
  }
  /* RECAL：氣壓零點重校（發射台歸零高度用，對應地面站 /cal）。
   * 只在 IDLE 受理——飛行中重設 ref_press = 高度基準亂掉 = 開傘判斷毀滅，
   * 此閘門連 REMOTE_CMD_UNRESTRICTED 都不跳過。取 8 筆有效樣本平均
   * （約 80ms 阻塞，IDLE 靜置下無害）；BMP 死掉時誠實回 ERROR。*/
  if (strcmp(line, "#CMD:RECAL_SALT5566#") == 0) {
    if (flight_state != FLIGHT_IDLE) {
      reply("MSG WARN REJECT recal only allowed in IDLE\r\n");
      return;
    }
    float psum = 0.f; int pn = 0;
    for (int i = 0; i < 8; i++) {
      float p = BMP585_ReadPressure();
      if (p > 800.f && p < 1100.f) { psum += p; pn++; }
      HAL_Delay(10);
    }
    if (pn < 4) { reply("MSG ERROR RECAL failed - BMP no valid pressure\r\n"); return; }
    ref_press = psum / (float)pn;
    kf_p_est  = ref_press;  kf_p_err = 1.0f;
    kf2_h = 0.0f; kf2_v = 0.0f;
    kf2_p00 = 1.0f; kf2_p01 = 0.0f; kf2_p11 = 1.0f;
    snprintf(rb, sizeof(rb), "MSG SUCCESS RECAL ref_press=%.2f hPa (n=%d)\r\n",
             ref_press, pn);
    reply(rb);
    return;
  }
  /* 【①】GNDTEST：地面測試模式，暫時解除「自動充氣需真的飛過」閘門。
   * 對應規範 4.5.3(回收系統感測器須在模擬觸發條件下地面測試)。
   * 只在 IDLE 受理;10 分鐘自動失效;偵測到真的離架時立即解除(見離架偵測)。
   * ⚠ 開著這個模式時，桌上做 ARM→FIRE 會照常在 11 秒後點燃氣囊那一路——
   *   那正是你要測的東西，但兩顆電火頭都必須先斷開。*/
  if (strcmp(line, "#CMD:GNDTEST_SALT3310#") == 0) {
    if (flight_state != FLIGHT_IDLE) {
      reply("MSG WARN REJECT gndtest only allowed in IDLE\r\n");
      return;
    }
    gnd_test_until = HAL_GetTick() + GND_TEST_WINDOW_MS;
    snprintf(rb, sizeof(rb),
      "MSG WARN GROUND TEST MODE ON for %lus - PB6 manual fire ENABLED, "
      "disconnect BOTH igniters\r\n", (unsigned long)(GND_TEST_WINDOW_MS / 1000UL));
    reply(rb);
    return;
  }
  if (strcmp(line, "#CMD:GNDTEST_OFF#") == 0) {
    gnd_test_until = 0;
    reply("MSG INFO Ground test mode OFF - PB6 manual fire disabled again\r\n");
    return;
  }
  /* SETCH：換 LoRa 頻道（#CMD:SETCH_72# → 922.125MHz）。
   * 規範 4.1.4.3 要求決賽前完成不同頻道通訊測試、避開他隊同頻干擾。
   * ⚠ 換頻會阻塞約 200ms 且期間收不到遙測 → 只在 IDLE 受理（此閘不因
   *   REMOTE_CMD_UNRESTRICTED 而放寬）。且換完後地面端也必須跟著換，
   *   否則這塊板就此失聯——所以先回報再執行，讓操作員至少收到最後一句。*/
  if (strncmp(line, "#CMD:SETCH_", 11) == 0) {
    if (flight_state != FLIGHT_IDLE) {
      reply("MSG WARN REJECT setch only allowed in IDLE\r\n");
      return;
    }
    const char *p = line + 11;
    uint32_t ch = 0; int nd = 0;
    /* 最多吃 3 位數：uint32 累加無溢位之虞（舊版無位數上限，十位以上的輸入
     * 會回繞，可能落回 0..80 而繞過下面的範圍檢查）。多於 3 位 → 下一個字元
     * 不是 '#' → 照樣被擋。*/
    while (*p >= '0' && *p <= '9' && nd < 3) { ch = ch * 10u + (uint32_t)(*p - '0'); p++; nd++; }
    if (nd == 0 || *p != '#' || ch > 80u) {   /* 900T22D 有效頻道 0..80 */
      reply("MSG WARN REJECT bad channel - use #CMD:SETCH_72# (0-80)\r\n");
      return;
    }
    /* 合規提示（僅警示、不阻擋——使用者明確要求不加硬限制）：
     * NCC LP0002 只准 920-925MHz，對應 CH70~74；模組本身可到 0..80。 */
    if (ch < 70u || ch > 74u) {
      reply("MSG WARN CH outside 920-925MHz band (legal CH70-74) - proceeding anyway\r\n");
    }
    snprintf(rb, sizeof(rb),
      "MSG WARN Switching to CH%lu (%.3f MHz) - ground must follow NOW\r\n",
      (unsigned long)ch, 850.125f + (float)ch);
    reply(rb);
    HAL_Delay(50);                     /* 讓上面那句先送出去 */
    int r = LoRa_SetChannel((uint8_t)ch);
    if (r == 0) {
      snprintf(rb, sizeof(rb), "MSG SUCCESS CH%lu active (%.3f MHz)\r\n",
               (unsigned long)ch, 850.125f + (float)ch);
    } else if (r == -2) {
      snprintf(rb, sizeof(rb), "MSG ERROR SETCH unsupported - M0/M1 not wired to MCU\r\n");
    } else {
      snprintf(rb, sizeof(rb), "MSG ERROR SETCH failed - module did not confirm\r\n");
    }
    reply(rb);
    return;
  }
  /* ── 遠端緊急命令（2026-07-20，對接 rocket_side_requirements.md）────
   * 地面站打 dpl/abg → 自動下傳隊伍秘鑰字串、burst 4 次×700ms（半雙工避障：
   * 本端 2Hz 發遙測，發送時收不到東西，700ms 間隔確保 4 次至少錯開一次 TX 窗；
   * 重複命中由下方 already-* 檢查吸收）。★此處原註「50ms」是錯的，實際值見
   * ground_side/src/core/lora_protocol.py 的 burst_interval=0.7——50ms 會讓
   * 四發全落在同一個 TX 窗內一起被吃掉，整個避障論證不成立。
   * 單發單向設計論證見 git 歷史
   * （兩段式=3 段 RF 串聯，緊急時失效率翻倍；明文口令用過即被竊聽）。
   * 閘門（比 .md 的 boottime 10s 閘更嚴）：
   *  - IDLE：需 ARM 中（30s 窗）＝桌測解鎖路徑「ARM → dpl」；未 armed 一律
   *    拒收 → 發射台誤觸/他隊（字串在 public repo）最壞情境免疫。
   *    發射倒數不必 ARM——飛行中本來就免 ARM 單發。
   *  - 飛行：LAUNCHED+10s 後受理（上升段誤開傘=解體，擋掉；burnout 5.9s、
   *    apogee 16.6s，最早也要 apogee 後才需要）。
   *  - 開傘另擋 DEPLOYING/DEPLOYED/LANDED；氣囊允許 DEPLOYING/DEPLOYED
   *    （下降段正是充氣緩衝的使用時機）、擋 LANDED。 */
  {
    uint8_t is_dpl = (strcmp(line, "#CMD:FORCE_DPL_SALT9981#") == 0);
    uint8_t is_abg = (strcmp(line, "#CMD:OPEN_ABG_SALT8872#") == 0);
    /* #CMD: 開頭但比對不中＝打錯字/字串版本不符——回饋而非死寂
     * （桌測手打 24 字元一字錯就全滅，沒回饋根本不知道錯在自己）*/
    if (!is_dpl && !is_abg && strncmp(line, "#CMD:", 5) == 0) {
      reply("MSG WARN Unknown CMD - check exact secret string\r\n");
      return;
    }
    if (is_dpl || is_abg) {
#ifndef REMOTE_CMD_UNRESTRICTED
      /* 拒收訊息一律自報指令名（dpl / abg）：地面站才分得出被拒的是哪一道。
       * 舊版所有 REJECT 文字都一樣，地面站只能用「頻道」比對 → 一句 RECAL
       * 的拒收會把待確認的開傘指令一起清掉，並把紅色告警貼上錯誤的標籤。*/
      const char *act = is_dpl ? "dpl" : "abg";
      /* ── 正式閘門（解禁時整段跳過；復原＝關掉上方 #define）── */
      if (flight_state == FLIGHT_IDLE && !manual_armed) {
        snprintf(rb, sizeof(rb),
          "MSG WARN REJECT %s - IDLE and not armed - ARM first (bench unlock)\r\n", act);
        reply(rb);
        return;
      }
      if (flight_state != FLIGHT_IDLE
          && (HAL_GetTick() - launch_time_ms) < 10000UL) {
        snprintf(rb, sizeof(rb),
          "MSG WARN REJECT %s - ascent guard (<10s after launch)\r\n", act);
        reply(rb); return;
      }
      if (flight_state == FLIGHT_LANDED) {
        snprintf(rb, sizeof(rb), "MSG WARN REJECT %s - already landed\r\n", act);
        reply(rb); return;
      }
#endif
      if (is_dpl) {
#ifdef REMOTE_CMD_UNRESTRICTED
        /* 解禁：裸脈衝（不動 flight_state）＝正式 key 可無限重複點火 */
        if (dpl_pulse_active) { reply("MSG WARN Deploy pulse already active\r\n"); return; }
        dpl_pulse_active = 1;
        dpl_pulse_ms     = HAL_GetTick();
        deploy_fire_on();
        reply("MSG SUCCESS Parachute deployed successfully\r\n");
#else
        /* ★2026-08-01：地面測試要能【重複】。
         *
         * 走正常路徑的話 flight_state 會被推進 DEPLOYING → DEPLOYED →
         * （靜止 10 秒）→ LANDED，而 LANDED 是終點狀態、回不去 ——
         * 一次上電只能測一發，之後全部回「already deployed / landed」。
         * 桌測要量兩支腳的電壓、要驗上行鏈路、要換人看，一發不夠。
         *
         * 所以在【IDLE 且已 ARM】時改走裸脈衝：只拉腳、不動狀態機、
         * 也不解除 ARM，所以 ARM 的 30 秒窗口內可以一直測。
         *
         * 這不是把解禁模式開回來 —— 那個版本【完全沒有閘門】。這裡：
         *   · 必須先 ARM（刻意動作，30 秒後自動失效）
         *   · 必須在 IDLE —— 離架偵測一成立（2.5g×200ms）這條路就消失
         *   · 脈衝進行中不重複觸發（dpl_pulse_active 擋著）
         * 訊息保留 successfully 字樣，地面站的下行確認才會亮。 */
        if (flight_state == FLIGHT_IDLE) {
          if (dpl_pulse_active) {
            reply("MSG WARN Deploy pulse already active\r\n");
            return;
          }
          dpl_pulse_active = 1;
          dpl_pulse_ms     = HAL_GetTick();
          deploy_fire_on();
          reply("MSG SUCCESS Parachute deployed successfully (ground test - repeatable while ARM holds)\r\n");
          return;
        }
        if (flight_state == FLIGHT_DEPLOYING || flight_state == FLIGHT_DEPLOYED) {
          /* 語意上這是「傘已經開了」＝好消息，不是失敗。burst 的第 2~4 發
           * 必然命中這裡；若 SUCCESS 那幀掉包，地面站只會看到這句，因此
           * 措辭必須讓它把此句當成「已開傘」的證據，而不是紅色的指令失敗。*/
          reply("MSG WARN REJECT dpl - already deployed (chute is already out)\r\n"); return;
        }
        manual_armed   = 0;
        deploy_time_ms = HAL_GetTick();
        flight_state   = FLIGHT_DEPLOYING;      /* 走自動開傘同一 DEPLOY_PULSE_MS 脈衝收尾 */
        deploy_fire_on();
        reply("MSG SUCCESS Parachute deployed successfully\r\n");
#endif
      } else {
        /* ★2026-07-31：氣囊已移除，PA0 併入降落傘發火迴路。
         * 單獨拉 PA0 只會半驅動傘迴路 —— 點不著，卻讓操作員以為做了事。
         * 所以明確拒收並說明去路，不要靜默忽略。
         * 指令字串本身保留（地面站舊版仍可能送出），只是不再有動作。*/
        /* ⚠ 這句【不可以】出現 "dpl" 三個字母。地面站的拒收分派是拿
         * 訊息內容做子字串比對（main_window.py:1495），一旦命中 "dpl"
         * 就會把正在等待確認的開傘指令清掉並標成紅色 REJECTED。*/
        reply("MSG WARN REJECT abg - airbag removed 2026-07-31; "
              "PA0 now drives the parachute circuit - use chute command\r\n");
      }
      return;
    }
  }
  if (strncmp(line, "FIRE", 4) == 0) {
    if (!manual_armed) { reply("MSG WARN REJECT not armed - send ARM first\r\n"); return; }
    if ((HAL_GetTick() - manual_arm_time) > manual_arm_timeout()) {
      manual_armed = 0; reply("MSG WARN REJECT ARM expired - send ARM again\r\n"); return;
    }
    /* 手動解析 FIRE 後的數字（避開 sscanf 連結成本）*/
    const char *p = line + 4;
    while (*p == ' ') p++;
    uint32_t code = 0; int ndig = 0;
    while (*p >= '0' && *p <= '9') { code = code * 10u + (uint32_t)(*p - '0'); p++; ndig++; }
    if (ndig == 0)               { reply("MSG WARN REJECT need FIRE <code>\r\n"); return; }
    if (code != manual_arm_code)  { reply("MSG WARN REJECT wrong code\r\n"); return; }
    if (flight_state == FLIGHT_DEPLOYING || flight_state == FLIGHT_DEPLOYED) {
      manual_armed = 0; reply("MSG WARN REJECT already deployed\r\n"); return;
    }
    /* 300s 長窗可能蓋到落地後——墜毀殘骸上點火比不點更危險，擋掉 */
    if (flight_state == FLIGHT_LANDED) {
      manual_armed = 0; reply("MSG WARN REJECT already landed\r\n"); return;
    }
    /* ★2026-08-01：IDLE 時同樣走裸脈衝（理由見上面 /dpl 那段）。
     * ARM 不解除 → 同一個四位數碼在窗口內可以重複 FIRE。 */
    if (flight_state == FLIGHT_IDLE) {
      if (dpl_pulse_active) {
        reply("MSG WARN Deploy pulse already active\r\n");
        return;
      }
      dpl_pulse_active = 1;
      dpl_pulse_ms     = HAL_GetTick();
      deploy_fire_on();
      reply("MSG SUCCESS Parachute deployed successfully (ground test - repeatable while ARM holds)\r\n");
      return;
    }
    /* ── 飛行中：走自動開傘同一 DEPLOYING 路徑 ── */
    manual_armed   = 0;
    deploy_time_ms = HAL_GetTick();
    flight_state   = FLIGHT_DEPLOYING;
    deploy_fire_on();
    reply("MSG SUCCESS Parachute deployed successfully\r\n");
    return;
  }
  /* 其他行忽略 */
}

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
  /* ── 提前啟動 USB（時鐘完成後立刻，早於 USART/TIM/SPI/I2C init）──
   * 若後續任何 init 失敗進入 Error_Handler，USB IRQ 仍持續運作，
   * COM4 不會消失，方便診斷。
   * usb_device.c 內有 guard，防止 CubeMX 生成的第二次呼叫重複初始化。 */
  MX_GPIO_Init();           /* USB D+/D- (PA11/PA12) 需要 GPIOA 時鐘  */
  MX_USB_DEVICE_Init();     /* 先啟動 USB CDC                          */
  HAL_Delay(500);           /* 給 Windows 500ms 開始枚舉               */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 發火腳預設低（最優先確保）— rocket_v7 只有 2 個 7V 發火通道：PA0/PA1
   * 配合硬體 BJT 基極 10kΩ 下拉，上電/MCU 浮空時 AOD4185 保持關閉。
   * ★ PB8/PB9 在 v7 不是發火腳（PB9=LORA_AUX 輸入、PB8 未用）→ 絕不可在此驅動，
   *   否則 PB9 會與 E22 的 AUX 輸出對撞。 */
  HAL_GPIO_WritePin(FIRE_7V_1_GPIO_Port, FIRE_7V_1_Pin, GPIO_PIN_RESET);  /* 7V 發火① PA0 */
  HAL_GPIO_WritePin(FIRE_7V_2_GPIO_Port, FIRE_7V_2_Pin, GPIO_PIN_RESET);  /* 7V 發火② PA1 (DEPLOY) */

  /* pyro 電源監測 ADC（PB0/PB1）——純量測，不影響任何點火路徑 */
  PyroADC_Init();

  /* E22 模式腳（PB7=M0、PB5=M1）立刻拉低＝透傳。必須在此，不能等 LoRa_Init：
   * gpio.c 把 PB7 設成 input+PULLUP，中間又隔著 USB 枚舉的 1 秒延遲，
   * 那段時間模組會待在 WOR 發送模式。 */
  LoRa_ModePinsInit();

  /* ---- SPI2 感測匯流排開機健檢(B 板 VDDIO 鉗位事故的制度化防護)----
   * 背景:匯流排上若有晶片內部電源軌塌陷,訊號腳會被其保護二極體鉗在
   * ~0.5V;SPI 推挽閒置(SCK 恆高)頂著鉗位 = 持續 ~46mA,遠超 GPIO 25mA
   * 額定。健康匯流排三線由 BMP 板上 10k 上拉,輸入採樣皆應為高。
   * 任一線連續 3 次採樣為低 → 三線鎖 analog Hi-Z、跳過感測器初始化、
   * 開機報告印 BUS=CLAMPED(也涵蓋 BMP 模組未裝=無上拉的情況)。 */
  {
    GPIO_InitTypeDef gi = {0};
    gi.Pin  = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_Delay(2);                     /* 讓模組上拉把線位拉穩 */
    uint8_t low_rounds = 0;
    for (int s = 0; s < 3; s++) {
      if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) ||
          !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) ||
          !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) low_rounds++;
      HAL_Delay(1);
    }
    if (low_rounds >= 3) {
      bus_clamped = 1;
      gi.Mode = GPIO_MODE_ANALOG;     /* 鎖 Hi-Z:不推挽、不灌流 */
      HAL_GPIO_Init(GPIOB, &gi);
    } else {
      HAL_SPI_DeInit(&hspi2);         /* 強制重設 HAL SPI 狀態機，確保下行重新設定 GPIO AF 模式 */
      MX_SPI2_Init();                 /* 健康 → 還原 AF 推挽組態 */
    }
  }

  /* 等 USB CDC 完整枚舉 */
  HAL_Delay(1000);

  /* ---- IMU 初始化（重試 3 次;匯流排鉗位時跳過,避免無意義觸發）---- */
  for (int _i = 0; !bus_clamped && _i < 3; _i++) {
    if (lsm6_init() == 0) { mod.imu = 1; imu_ok = 1; break; }
    HAL_Delay(200);
  }

  /* ---- LoRa 初始化（重試 3 次）---- */
  for (int _i = 0; _i < 3; _i++) {
    if (LoRa_Init() == 0) { mod.lora = 1; break; }
    HAL_Delay(200);
  }

  /* ---- BMP585 初始化（重試 3 次;匯流排鉗位時跳過）---- */
  for (int _i = 0; !bus_clamped && _i < 3; _i++) {
    uint8_t bid = BMP585_Init(&hspi2);
    if (bid != 0) { mod.bmp585 = 1; break; }
    HAL_Delay(200);
  }

  /* ---- 地面氣壓基準 ---- */
  { float p = BMP585_ReadPressure();
    if (p > 800.f && p < 1100.f) { ref_press = p; kf_p_est = p; } }

  /* ---- GNSS 初始化（USART2）---- */
  GNSS_Init(&huart2);

  /* ---- 開機狀態整合報告(含時鐘來源 + 重啟原因,異常鑑識用)----
   * CLK: HSE=正常 / HSI-FB=HSE 故障已降級(板子晶振要查!)
   * RST: 上次重啟原因。BROWNOUT/POWER-ON 突然出現=電源瞬斷(撞擊/接觸不良),
   *      IWDG=看門狗(若未啟用卻出現=異常),SOFT=軟體重啟,NRST-PIN=按鍵。*/
  { char b[224];   /* 160→224：加了 VF/VA 電壓與 FUSE BLOWN 警示 */
    const char *rst =
      __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? "IWDG" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? "WWDG" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? "LPWR" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  ? "SOFT" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)  ? "POWER-ON" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)  ? "BROWNOUT" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  ? "NRST-PIN" : "?";
    /* 【④】不是正常上電 ⇒ 上一輪可能死在空中：武裝墜落救援監看。
     * POWER-ON 才代表「這是全新的一次通電」，其餘（BROWNOUT/SOFT/IWDG/
     * NRST-PIN/?）都可能是飛行途中被打斷。誤判成本極低——監看只在
     * 「持續量到 >15m/s 下降」時才動作，地面上不可能發生。 */
    postreset_watch = !__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    /* 開機立刻量一次 pyro 電源，讓開機報告就能看到保險絲/武裝狀態
     * （t_pyro_adc 保持 0 → 主迴圈首輪仍會照常更新）*/
    { float vd = pyro_adc_vdda();
      v_fuse = pyro_read_volt(ADC_CH_FUSE, vd);
      v_arm  = pyro_read_volt(ADC_CH_ARM,  vd); }
    snprintf(b, sizeof(b),
      "MOD: BMP=%d IMU=%d LORA=%d  REF_PRESS=%.2f hPa  CLK=%s  RST=%s%s  VF=%.2fV VA=%.2fV%s\r\n",
      mod.bmp585, mod.imu, mod.lora, ref_press,
      clk_hsi_fallback ? "HSI-FB(HSE FAIL!)" : "HSE", rst,
      bus_clamped ? "  BUS=CLAMPED!(SPI2 Hi-Z)" : "",
      v_fuse, v_arm,
      (pyro_adc_ok && v_fuse >= 0.0f && v_fuse < 5.0f) ? "  FUSE BLOWN?!" : "");
    cdc_write(b);
    /* ★PB7 的「接地禁用 LoRa」跳線已廢除（2026-07-26）：該腳改作 E22 的 M0
     * 模式腳。透傳模式下 M0 本來就是低電平，舊邏輯會把它讀成「使用者要求
     * 禁用」→ 遙測整個停掉。飛行中沒有任何理由關掉唯一的下行鏈路。 */
    if (mod.lora) LoRa_SendStr(b);
#ifdef REMOTE_CMD_UNRESTRICTED
    /* 解禁版警示廣播：USB＋LoRa 都吼一聲，帶著這版上發射台前一定看得到 */
    cdc_write("MSG ERROR UNRESTRICTED MODE - dpl/abg gates OFF - NOT FLIGHT SAFE\r\n");
    if (mod.lora)
      LoRa_SendStr("MSG ERROR UNRESTRICTED MODE - dpl/abg gates OFF - NOT FLIGHT SAFE\r\n");
#endif
  }

  /* ---- IMU 校準（僅在 IMU 成功初始化時執行）---- */
  if (mod.imu) {
    /* 收斂階段（1 秒）*/
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/IMU_ACC_SCALE, a_y=ray/IMU_ACC_SCALE, a_z=raz/IMU_ACC_SCALE;
        float g_x=rgx/14.286f*0.017453293f;
        float g_y=rgy/14.286f*0.017453293f;
        float g_z=rgz/14.286f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
      }
    }
    /* 校準階段（1 秒，取 world_az 平均）*/
    float bias_sum2 = 0.f; int bias_cnt2 = 0;
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/IMU_ACC_SCALE, a_y=ray/IMU_ACC_SCALE, a_z=raz/IMU_ACC_SCALE;
        float g_x=rgx/14.286f*0.017453293f;
        float g_y=rgy/14.286f*0.017453293f;
        float g_z=rgz/14.286f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
        bias_sum2 += world_az(a_x,a_y,a_z);
        bias_cnt2++;
      }
    }
    if (bias_cnt2 > 0) {
      az_bias_g = (bias_sum2 / (float)bias_cnt2) - 1.0f;
      char b[64];
      snprintf(b, sizeof(b), "AZ_BIAS=%.5f g (n=%d)\r\n", az_bias_g, bias_cnt2);
      cdc_write(b);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_imu = 0, t_bmp = 0, t_out = 0, t_lora = 0, t_csv = 0;
  uint8_t  sd_init_done  = 0;
  uint8_t  sd_init_tries = 0;   /* SD init 重試計數（第一次上電偶發 cmd0 亂碼，隔 3s 再試會好）*/
  uint8_t  lora_tx_pending = 0;   /* 非阻塞 TX：上一封包是否仍在傳送中 */
  float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;
  float press = ref_press, total_g = 1;
  float tc = 25.f;
  uint8_t cf_init = 0;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    main_loop_cnt++;

    /* ═══════════════════════════════════════════════════════════════
     * ★ 最高優先：開傘狀態機（無任何阻塞，每次迴圈必執行）
     *   此區塊不呼叫任何 HAL_Delay / SPI / I2C，純 GPIO + 比較
     * ═══════════════════════════════════════════════════════════════ */
    uint32_t now = HAL_GetTick();

    /* [A] DEPLOYING → DEPLOYED：脈衝結束後 (DEPLOY_PULSE_MS) 關閉 GPIO */
    if (flight_state == FLIGHT_DEPLOYING &&
        (now - deploy_time_ms) >= DEPLOY_PULSE_MS) {
      deploy_fire_off();
      flight_state = FLIGHT_DEPLOYED;
      land_stable_start = 0;   /* 重置落地計時器 */
      cdc_write("*** DEPLOYED ***\r\n");
    }

    /* [A2] DEPLOYED → LANDED：落地偵測
     * 條件：total_g 接近 1g（靜止）且 rel_alt < 30m，持續 10 秒
     * 目的：進入低功耗模式，減少 SD 寫入，保留 LoRa beacon 供尋回 */
    if (flight_state == FLIGHT_DEPLOYED) {
      /* ★2026-07-31：原本這裡是「落海撞擊偵測 → 自動充氣氣囊」。
       * 氣囊已移除，整段刪掉（AIRBAG_IMPACT_G / AIRBAG_ARM_DELAY_MS /
       * AIRBAG_IMPACT_MS 三個 #define 保留但已無引用，供日後恢復參考）。
       * 落海浮力現在完全靠箭身本身，見 doc/ 的浮力評估。               */

      /* ★落地判斷必須有「活著的感測器」背書（2026-07-28 全盤審查）：
       * total_g 與 press/rel_alt 宣告在主迴圈之外，感測器一旦死掉，它們就
       * 停在最後一次成功讀到的值——舊碼直接拿那個死值當證據。
       * 最糟的組合：氣壓計開機就沒起來（IMU 正常，火箭照常飛、照常開傘），
       * rel_alt 恆為 0，而傘下等速下降時 total_g≈1.0 —— 兩個條件同時成立，
       * 10 秒後宣告「落地」，氣囊在數百公尺的空中充氣，而 LANDED 是終點狀態。
       * 改成任一半都要有活的感測器背書；沒有就不判定落地（fail-safe：寧可
       * 不宣告落地，也不要在空中放氣囊）。落海氣囊的主路徑是上面那行撞擊
       * 偵測，它本來就檢查了 mod.imu。*/
      int g_still   = mod.imu    && (fabsf(total_g - 1.0f) < LAND_G_DEV_THR);
      int alt_low   = mod.bmp585 && (rel_alt < LAND_ALT_THR);
      int land_cond = g_still && alt_low;
      if (land_cond) {
        if (land_stable_start == 0) land_stable_start = now;
        if ((now - land_stable_start) >= LAND_STABLE_MS) {
          flight_state = FLIGHT_LANDED;
          char lmsg[64];
          snprintf(lmsg, sizeof(lmsg),
            "*** LANDED alt=%.1fm t_fly=%lus ***\r\n",
            rel_alt, (unsigned long)((now - launch_time_ms) / 1000UL));
          cdc_write(lmsg);   /* 不寫 SD：避免插入非 CSV 行（落地由 state 欄=4 標記）*/
          /* 截掉 8MB 預分配的尾端，log.csv 收斂為實際資料長度（落地在地面，GC 可接受）*/
          if (logger_is_ready()) { f_truncate(&file); f_sync(&file); }
        }
      } else {
        land_stable_start = 0;   /* 條件中斷（可能降落傘仍在搖擺），重置計時 */
      }
    }

    /* [A2] 【④】異常重啟後的墜落救援：IDLE + 持續高速下降 = 這是一枚正在
     *      墜落的火箭，上一輪的飛行狀態被 reset 洗掉了。直接開傘。
     *      條件全部要成立才動作：
     *        - 上次不是正常上電（postreset_watch）
     *        - 現在在 IDLE（真正在飛的話狀態機自己會處理）
     *        - 開機已滿 3 秒（等氣壓濾波器穩定，避免開機瞬間的假速度）
     *        - 氣壓計活著且可信
     *        - 下降快於 15 m/s 且持續滿 1 秒
     *      傘已開的情況下降速率 ≤12.7 m/s，不會命中；自由落體 30~100 m/s 必中。*/
    if (postreset_watch && flight_state == FLIGHT_IDLE
        && now > POSTRESET_ARM_MS && mod.bmp585 && !baro_untrusted) {
      if (vz_baro_lp < POSTRESET_FALL_VZ) {
        if (fall_start_ms == 0) fall_start_ms = now;
        else if ((now - fall_start_ms) >= POSTRESET_FALL_MS) {
          char fb[104];
          postreset_watch = 0;            /* 一次性 */
          deploy_time_ms  = now;
          flight_state    = FLIGHT_DEPLOYING;
          deploy_fire_on();
          /* 重建最低限度的飛行狀態，讓後續 DEPLOYED→LANDED 與氣囊能正常走 */
          imu_armed      = 1;
          launch_time_ms = now;
          launch_inferred = 0;            /* 這是實測到的墜落，不是推測 */
          peak_rel_alt   = rel_alt;
          snprintf(fb, sizeof(fb),
            "MSG ERROR POST-RESET FALL RESCUE - deploying (vz=%.1f m/s)\r\n", vz_baro_lp);
          cdc_write(fb);
          if (mod.lora) LoRa_SendStr(fb);
        }
      } else {
        fall_start_ms = 0;
      }
    }

    /* [B] LAUNCHED → DEPLOYING
     *   主觸發：(A) 氣壓高度低於最高點 10m  AND  (B) 垂直速度持續向下 1.5s
     *   備  援：(C) 飛行時間 ≥ 20s，無條件強制開傘
     *
     *   A/B 同時成立 → 頂點後確認下降，最精確
     *   C 單獨成立   → 感測器異常時保底觸發
     */
    if (flight_state == FLIGHT_LAUNCHED) {
      uint32_t t_fly = now - launch_time_ms;

      /* ── 【②】推測式離架的撤銷（2026-07-28）──────────────────────────
       * flight_state 全檔沒有任何寫回 FLIGHT_IDLE 的路徑，所以一次誤判就
       * 永久閂死。最陰險的後果不是當下點火（那有 20m 閘門擋著），而是：
       * 誤判閂住之後 t_fly 一路累積遠超 18 秒，等到**真正發射**時，高度一過
       * 20m 立刻滿足備援 → 在推力段約 20m、數十 m/s 開傘 → 結構解體。
       * 這條路徑完全在「電火頭最後接」SOP 的保護範圍之外（那時早已武裝完成）。
       * 撤銷條件刻意保守：只撤銷「推測」來的離架（2.5g 實測到的絕不撤銷），
       * 且必須氣壓計活著、可信、高度確實在地面、並持續 3 秒。*/
      if (launch_inferred && mod.bmp585 && !baro_untrusted
          && rel_alt < REVOKE_ALT_M && peak_rel_alt < DEPLOY_PEAK_MIN_M) {
        if (revoke_start == 0) revoke_start = now;
        else if ((now - revoke_start) >= REVOKE_HOLD_MS) {
          flight_state    = FLIGHT_IDLE;   /* 唯一一條回得去 IDLE 的路 */
          imu_armed       = 0;
          launch_inferred = 0;
          revoke_start    = 0;
          peak_rel_alt    = 0.0f;
          vz_neg_start_ms = 0;
          cond_A = 0; cond_B = 0;
          cdc_write("MSG WARN FORCE-LAUNCH revoked - still on ground, back to IDLE\r\n");
          if (mod.lora)
            LoRa_SendStr("MSG WARN FORCE-LAUNCH revoked - still on ground, back to IDLE\r\n");
        }
      } else {
        revoke_start = 0;
      }

      /* ── 【③】氣壓計「正向存活證明」（2026-07-28）────────────────────
       * mod.bmp585 只證明「SPI 有回應、讀值落在 800~1100 hPa」，**證明不了
       * 讀值會跟著高度變化**。氣孔被密封膠/膠帶遮住時（本載具已實證的硬體
       * 問題），氣壓計會一直回報一個完全合理、卻不隨高度移動的數值：
       *   - bmp_err_cnt 每筆有效樣本都歸零 → mod.bmp585 永遠是 1
       *   - 凍結看門狗比對 bit-exact 相同，而 24-bit raw 的雜訊底有數十 LSB
       *     → 永不觸發
       *   - peak_rel_alt≈0 → cond_A 死、deploy_bkup 的 20m 閘門也死
       *   → 三條自動路徑同時失效，火箭以終端速度落海。
       * 存活證明：離架後 BARO_PROOF_MS 內，高度必須爬過 BARO_PROOF_MIN_M。
       * 動力段 5 秒早已數百公尺（模擬:burnout 5.9s、apogee 16.6s），若此時
       * 峰值仍不到 10m，唯一的解釋就是氣壓計沒在跟隨高度 → 標為不可信，
       * 備援退回純計時。門檻刻意訂得極寬鬆，正常飛行不可能誤判。
       * 只判定一次（單向閂鎖）：中途復活也不改回，因為已經證明它不可靠。*/
      if (!baro_untrusted && mod.bmp585 && t_fly > BARO_PROOF_MS
          && peak_rel_alt < BARO_PROOF_MIN_M) {
        baro_untrusted = 1;
        cdc_write("MSG ERROR BARO NOT TRACKING ALTITUDE - backup deploy now time-only\r\n");
        if (mod.lora)
          LoRa_SendStr("MSG ERROR BARO NOT TRACKING ALTITUDE - backup deploy now time-only\r\n");
      }

      /* 退化規則見 deploy_A_eff/deploy_B_eff 的說明（不對稱：氣壓可單獨守門、
       * IMU 不可）。三處使用同一組函式，遙測顯示與實際決策保證一致。 */
      int cond_A_eff = deploy_A_eff();
      int cond_B_eff = deploy_B_eff();

      int deploy_main = (cond_A_eff && cond_B_eff);
      /* C 備援：t>DEPLOY_TB_MS(18s)。★地面安全閘門（防組裝/搬運誤判 LAUNCH 後，純計時
       *   在地面點火——原本此路無高度確認）：氣壓計活著時，額外要求「曾爬升過
       *   DEPLOY_PEAK_MIN_M(20m)」才准觸發。地面高度永不爬 → peak_rel_alt≈0
       *   → 備援被擋。這與 cond_A 用的是同一道 20m 地面防護閘門，風險姿態一致。
       *   氣壓計死時（mod.bmp585=0，正是備援設計要保底的感測器故障情境）退回
       *   純計時，保留「感測器全死也開傘」原意；此路殘留地面風險由「電火頭最後
       *   接」SOP 兜底（force-launch(60s) 亦走此路，同受 SOP 保護）。
       * ⚠ 未桌面驗證前勿信賴。需灌假資料驗三情境：①爬升過→20s 觸發 ②沒爬升
       *   →擋住 ③氣壓計標死→退回純計時仍於 20s 觸發。 */
      /* ── 2026-07-28 全盤審查後的兩處補強（★兩者必須同時存在）──────────
       * 【②】`!bus_clamped`：開機自檢就判定 SPI2 異常的板子，兩個感測器根本
       *   沒被初始化過。舊碼的三元閘門在 mod.bmp585=0 時整個打開 → 配合下面
       *   force-launch 的 60s 計時，一顆鬆掉的焊點就會讓板子在上電 78 秒後
       *   無條件點燃降落傘、89 秒後點燃氣囊，人還站在旁邊。
       *   「從來沒初始化成功」≠「飛行中壞掉」：前者代表這塊板沒準備好，
       *   它該做的是安靜下來報告故障，讓另一塊板完成回收（雙板熱備援的意義），
       *   而不是憑一個計時器自己點火。遠端手動開傘不受此限，仍可救。
       * 【③】`!baro_untrusted`：氣孔遮蔽時氣壓計會回報「合理但不變」的讀值，
       *   mod.bmp585 永遠是 1，於是 peak_rel_alt≈0 把這道 20m 閘門永久關死——
       *   而 cond_A 同樣需要 peak≥20m，三條自動路徑一起失效，火箭直落。
       *   baro_untrusted 由下方的「離架後高度必須有變化」存活證明設立，
       *   一旦判定氣壓計在說謊，備援就退回純計時（原本設計要對付感測器故障
       *   的行為）。 */
      int baro_gate_ok = (mod.bmp585 && !baro_untrusted);
      int deploy_bkup = (t_fly >= DEPLOY_TB_MS) && !bus_clamped
                        && (baro_gate_ok ? (peak_rel_alt >= DEPLOY_PEAK_MIN_M) : 1);

      if (deploy_main || deploy_bkup) {
        deploy_time_ms = now;
        flight_state   = FLIGHT_DEPLOYING;
        deploy_fire_on();
        if (deploy_main) {
          /* 觸發訊息含診斷數據，便於事後分析。同步下傳 MSG 事件
           * （地面站規範格式）——自動開傘是最關鍵事件，先前只上 USB、
           * 地面只能從 ST 欄位變化推斷。LoRa_SendStr 阻塞最壞 ~650ms，
           * 此刻點火已發生、決策已完成，可接受。 */
          char msg[96];
          snprintf(msg, sizeof(msg),
            "MSG SUCCESS Parachute deployed (auto A+B pk=%.1fm now=%.1fm vz=%.2fm/s)\r\n",
            peak_rel_alt, rel_alt, kf2_v);
          cdc_write(msg);
          if (mod.lora) LoRa_SendStr(msg);
        } else {
          /* 訊息含實際計時值與診斷數據：舊版寫死「T>20s」，但 DEPLOY_TB_MS
           * 早在 2026-07-20 改成 18s，發射當天照著字面判讀會誤導。 */
          char msg[112];
          snprintf(msg, sizeof(msg),
            "MSG SUCCESS Parachute deployed (backup timer T>%lus pk=%.1fm now=%.1fm)\r\n",
            (unsigned long)(DEPLOY_TB_MS / 1000UL), peak_rel_alt, rel_alt);
          cdc_write(msg);
          if (mod.lora) LoRa_SendStr(msg);
        }
      }
    }
    /* ═══════════════════════════════════════════════════════════════ */

    /* ═══════════════════════════════════════════════════════════════
     * LoRa TX 完成輪詢（每次主迴圈執行，不受 500ms 節拍限制）
     * 原因：TX airtime ≈ 250ms，若只在 500ms LoRa block 裡呼叫，
     *       HAL_GetTick()-lora_tx_start ≈ 500ms > 400ms 超時門檻
     *       → PollTx 永遠先超時，TxDone 來不及被偵測。
     * 修正：每迴圈輪詢，約在 TX 後 250ms 即可偵測到 TxDone。
     * ═══════════════════════════════════════════════════════════════ */
    if (lora_tx_pending && mod.lora) {
      int poll = LoRa_PollTx();
      if (poll == 1) {                     /* TxDone 確認 */
        lora_ok++;
        lora_err_cnt    = 0;
        lora_tx_pending = 0;
      } else if (poll == -1) {             /* 超時：硬體異常 */
        lora_fail++;
        lora_tx_pending = 0;
        if (++lora_err_cnt >= MOD_ERR_MAX) {
          mod.lora = 0;
          cdc_write("WARN: LORA DEAD\r\n");
        }
      }
      /* poll == 0：仍在傳送，下次迴圈再查 */
    }
    /* ═══════════════════════════════════════════════════════════════ */

    /* ─── 指令系統 ─── */
    cmd_execute_pending();
    cmd_flush_echo();

    /* ─── 手動開傘：ARM 逾時自動解除 ＋ LoRa 上行命令 ───
     * LoRa_Receive 取一整行（'\n' 為界、已去 '\r'）→ 餵入共用安全核心；
     * 回覆走 LoRa 送回地面。非 ARM/FIRE/SAFE 的行被核心忽略（雜訊安全）。*/
    ManualDeploy_Poll();
    if (mod.lora) {
      static uint8_t lrx[80];
      int lr = LoRa_Receive(lrx, (uint8_t)sizeof(lrx));
      if (lr > 0) {
        /* 上行內容 echo 到 USB：火箭端 COM 直接看得到收到什麼（2026-07-20
         * 首次 ARM 實測「全聾無回應」後加的診斷窗口——沒有它，上行鏈路
         * 斷在哪一層完全不可觀察）。火箭 RX 平時無流量，不會洗版。 */
        char lora_rx_dbg[100];
        snprintf(lora_rx_dbg, sizeof(lora_rx_dbg), "LORA RX: %s\r\n", (char *)lrx);
        cdc_write(lora_rx_dbg);
        ManualDeploy_HandleLine((char *)lrx, lora_cmd_reply);
      }
    }

    /* ─── SD 卡延遲初始化（啟動後 3 秒；失敗每 3s 自動重試，最多 5 次）───
     * 實測：第一次上電偶發 cmd0 回亂碼(如 0x3F)，是上電初期電源/訊號未穩；
     * 過幾秒重試即成功（先前要手動按 reset 才好 → 這裡改成韌體自動重來）。*/
    if (!sd_init_done && now >= 3000UL + (uint32_t)sd_init_tries * 3000UL) {
      logger_init();
      if (logger_is_ready()) {
        sd_init_done = 1;
        mod.sdcard = 1;
        cdc_write("SD: OK\r\n");
      } else {
        extern FRESULT fres;
        extern volatile uint8_t SD_dbg_cmd0, SD_dbg_cmd8, SD_dbg_spi_status;
        extern volatile uint32_t SD_dbg_spierr, SD_dbg_spi_errcode;
        sd_init_tries++;
        if (sd_init_tries >= 5) sd_init_done = 1;   /* 5 次都失敗才放棄 */
        char e[112];
        snprintf(e, sizeof(e),
                 "SD: FAIL %u/5%s (fres=%d cmd0=%02X cmd8=%02X spierr=%lu st=%u ec=0x%lX)\r\n",
                 (unsigned)sd_init_tries, (sd_init_tries >= 5) ? " GIVE-UP" : " retrying",
                 (int)fres, (unsigned)SD_dbg_cmd0, (unsigned)SD_dbg_cmd8,
                 (unsigned long)SD_dbg_spierr, (unsigned)SD_dbg_spi_status,
                 (unsigned long)SD_dbg_spi_errcode);
        cdc_write(e);
      }
    }

    /* ─── SD 卡動態重試與熱插拔恢復機制 ───
     * ⚠ LAUNCHED/DEPLOYING 期間跳過：logger_init 是阻塞式（半死卡下
     * ACMD41 預算 1s＋深度救援 ~1.1s），會在開傘決策窗口打出秒級空窗。
     * 上升段 SD 掉了就掉了，DEPLOYED（傘已開）之後才恢復記錄。 */
    if (sd_init_done && !logger_is_ready()
        && flight_state != FLIGHT_LAUNCHED
        && flight_state != FLIGHT_DEPLOYING) {
      static uint32_t sd_reinit_t = 0;
      if (now - sd_reinit_t >= 5000UL) {
        sd_reinit_t = now;
        logger_init();
        if (logger_is_ready()) {
          mod.sdcard = 1;
          cdc_write("SD: OK (RECOVERED)\r\n");
        } else {
          /* 失敗也要出聲：拔卡實測時「全程靜默」被誤判成救援沒在跑。
           * fres 定位 FatFS 層死點（3=NOT_READY→disk_init 敗、13=NO_FILESYSTEM、
           * 4=NO_FILE…）；cmd0/cmd8 定位 SPI 層死點（01=有回應、FF=沒回）。
           * 實測 2026-07-20：熱插回卡 cmd0=01 但 init 快速失敗＝死在 CMD8 之後，
           * 需要這組完整參數分案。 */
          extern volatile uint8_t SD_dbg_cmd0, SD_dbg_cmd8;
          extern FRESULT fres;
          char e[64];
          snprintf(e, sizeof(e), "SD: REINIT FAIL (fres=%d cmd0=%02X cmd8=%02X) retry 5s\r\n",
                   (int)fres, (unsigned)SD_dbg_cmd0, (unsigned)SD_dbg_cmd8);
          cdc_write(e);
        }
      }
    }

    /* ─── SD 預算水位自動滾檔（IDLE 限定，＝自動 CLEAR、資料不丟失）───
     * IDLE 2Hz 也在啃預分配預算（~1.2MB/h），待機夠久會把飛行段
     * 「零 sync＋無配置 GC」的前提吃光。吃過半 → 自動 trunc→close→init
     * 滾新檔＋重配滿血預算；舊檔收斂保留、新檔標頭由 file_gen 機制補寫。
     * 「靜止」門檻擋住唯一風險：init 阻塞（~0.5-1s）撞上點火瞬間——
     * 只在 |g-1|<0.05 時執行，起飛加速一開始就不會觸發。 */
    if (flight_state == FLIGHT_IDLE && logger_is_ready()
        && f_tell(&file) >= (LOG_PREALLOC_BYTES / 2U)
        && fabsf(total_g - 1.0f) < 0.05f) {
      cdc_write("SD: AUTO-ROLL (budget refill)\r\n");
      logger_trunc();
      logger_close();
      logger_init();
    }

    /* ─── IMU 每 10ms ─── */
    if (now - t_imu >= 10) {
      float dt = (float)(now - t_imu) * 0.001f;
      if (dt > 0.05f) dt = 0.05f;
      t_imu = now;

      if (mod.imu) {
        int16_t raw_t,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az;
        if (lsm6_read_raw_LSM6DSOTR(&raw_t,&raw_gx,&raw_gy,&raw_gz,
                                     &raw_ax,&raw_ay,&raw_az) == 0) {
          imu_err_cnt = 0;

          /* ── IMU 凍結看門狗（2026-07-30）─────────────────────────────
           * 氣壓計早有這道防護，IMU 卻只檢查 SPI 回傳碼——一顆「SPI 正常
           * 回應但已停止轉換」的晶片完全偵測不到，而後果比氣壓計凍結更糟：
           *   lin_az 變成常數 → kf2_v 線性發散 → 若該常數扣掉重力為負，
           *   cond_B 在 1.5 秒後「必然」成立（不是可能，是必然）。
           * 只比對加速度三軸，不看陀螺：ODR_G=104Hz 與本迴圈 100Hz 讀取率
           * 太接近，同一筆陀螺樣本被讀兩次是正常現象，會誤判。
           * ODR_XL=416Hz 遠高於讀取率，每次必為新樣本；±16g 下 1g=2048LSB、
           * 雜訊底約數 LSB，三軸同時 bit-exact 連續 50 次(0.5s)不可能發生。
           * 重試上限 3 次（每次 lsm6_init 阻塞約 70ms），仍凍結就判死——
           * 判死是正確的 fail-safe：IMU 死 → cond_A（純氣壓）單獨守門，
           * 那條路徑無積分無漂移，可以信任。 */
          {
            static int16_t f_ax = 0, f_ay = 0, f_az = 0;
            static uint8_t f_same = 0, f_retry = 0;
            if (raw_ax == f_ax && raw_ay == f_ay && raw_az == f_az) {
              if (++f_same >= 50) {
                f_same = 0;
                if (++f_retry > 3) {
                  mod.imu = 0; imu_ok = 0;
                  cdc_write("WARN: IMU FROZEN - DEAD\r\n");
                } else {
                  lsm6_init();
                  cdc_write("IMU: FREEZE REINIT\r\n");
                }
              }
            } else { f_same = 0; f_ax = raw_ax; f_ay = raw_ay; f_az = raw_az; }
          }

          ax = (float)raw_ax / IMU_ACC_SCALE;
          ay = (float)raw_ay / IMU_ACC_SCALE;
          az = (float)raw_az / IMU_ACC_SCALE;
          const float deg2rad = 0.017453293f;
          /* 陀螺 ±2000dps 靈敏度 = 70 mdps/LSB（LSM6DS3 DS Table 3）
           * → 1/0.070 = 14.286 LSB/dps。舊值 16.384（=32768/2000）
           *   讓角速度偏低 12.8% → 翻滾時姿態滯後、world_az 投影誤差。 */
          gx = (float)raw_gx / 14.286f * deg2rad;
          gy = (float)raw_gy / 14.286f * deg2rad;
          gz = (float)raw_gz / 14.286f * deg2rad;
          tc = (float)raw_t / 256.f + 25.f;
          total_g = sqrtf(ax*ax + ay*ay + az*az);
          mahony_update(ax,ay,az,gx,gy,gz,dt);

          /* 引擎燒完偵測：起飛後，當合加速度降回 1.15g 以下，視為進入慣性上升段 (LAUNCH) */
          if (flight_state == FLIGHT_LAUNCHED && is_boosting) {
            if (total_g < 1.15f) {
              is_boosting = 0;
              cdc_write("*** BURNOUT (Coasting) ***\r\n");
            }
          }

          /* 發射偵測：total_g > 閾值持續 200ms */
          if (total_g >= LAUNCH_AZ_G) {
            g2_count++;
            if (!imu_armed && g2_count >= 20) {
              imu_armed      = 1;
              launch_time_ms = now;
              flight_state   = FLIGHT_LAUNCHED;
              is_boosting    = 1;
              /* 2.5g 實測到的離架＝真憑實據，絕不可被撤銷邏輯退回 IDLE */
              launch_inferred = 0;
              revoke_start    = 0;
              /* 【①】真的飛起來了 → 地面測試模式立即失效，不可能帶著上天 */
              gnd_test_until  = 0;
              /* ★2026-07-31：BRIDGE 同理。它會把本板自己的遙測完全靜音
               * （main.c 的 TX 閘門看 cmd_bridge_active()），而原本只有從 USB
               * 打 EXITBRIDGE 或 reset 能退出 —— USB 拔掉之後就沒救了。
               * 忘了關就帶上天 = 整場零遙測，而地面站只會看到「沒訊號」。*/
              cmd_exit_bridge();
              /* KF2 從乾淨起點：地面高度=0、速度=0、協方差復位 */
              kf2_h = 0.0f; kf2_v = 0.0f;
              kf2_p00 = 1.0f; kf2_p01 = 0.0f; kf2_p11 = 1.0f;
              vz_baro_lp = 0.0f;
              /* 重置開傘條件狀態 */
              peak_rel_alt    = 0.0f;
              vz_neg_start_ms = 0;
              cond_A          = 0;
              cond_B          = 0;
              cdc_write("*** LAUNCH! ***\r\n");
            }
          } else {
            if (!imu_armed) g2_count = 0;
          }

          /* ── KF2 預測步（100Hz，用 IMU 垂直加速度）僅 imu_armed 後執行 ──
           * 狀態轉移 F = [[1,dt],[0,1]]
           *   h_pred = h + v·dt + ½·a·dt²
           *   v_pred = v + a·dt
           *   P_pred = F·P·Fᵀ + Q
           * 更新步在 BMP 區塊（50Hz）用氣壓高度修正。 */
          if (imu_armed) {
            float az_w   = world_az(ax,ay,az) - az_bias_g;  /* 世界系垂直加速度（g）*/
            float lin_az = (az_w - 1.0f) * 9.80665f;        /* 扣重力(-1g)轉 m/s² */
            /* sanity 限幅 ±16g：只擋解碼錯誤，★不可設 ±3g（會砍真實推力）*/
            if      (lin_az >  KF2_AZ_CLAMP) lin_az =  KF2_AZ_CLAMP;
            else if (lin_az < -KF2_AZ_CLAMP) lin_az = -KF2_AZ_CLAMP;

            float dt2 = dt * dt;
            kf2_h += kf2_v * dt + 0.5f * lin_az * dt2;
            kf2_v += lin_az * dt;
            float p00 = kf2_p00 + dt*(2.0f*kf2_p01) + dt2*kf2_p11 + KF2_Q_H;
            float p01 = kf2_p01 + dt * kf2_p11;
            float p11 = kf2_p11 + KF2_Q_V;
            kf2_p00 = p00; kf2_p01 = p01; kf2_p11 = p11;

            /* ── 開傘條件 B：KF2 垂直速度持續向下 DEPLOY_VZ_NEG_MS ──
             * kf2_v < VZ_NEG_THR 持續 1.5s → cond_B = 1
             * 速度短暫回正（KF 抖動）→ 重置計時 */
            if (kf2_v < DEPLOY_VZ_NEG_THR) {
              if (vz_neg_start_ms == 0) vz_neg_start_ms = now;
              cond_B = ((now - vz_neg_start_ms) >= DEPLOY_VZ_NEG_MS) ? 1 : 0;
            } else {
              vz_neg_start_ms = 0;  /* 速度回正：重置計時 */
              cond_B = 0;
            }
          }
        } else {
          /* IMU 讀取連續失敗 → 標記死亡 */
          if (++imu_err_cnt >= MOD_ERR_MAX) {
            mod.imu = 0; imu_ok = 0;
            cdc_write("WARN: IMU DEAD\r\n");
          }
        }
      } else if (flight_state == FLIGHT_IDLE && now > 15000UL && !imu_armed) {
        /* IMU 死亡降級：需交叉確認確實在飛行中，避免地面誤動作
         *
         * ┌──────────┬──────────┬────────────────────────────────┐
         * │ IMU      │ BMP      │ 處置                           │
         * ├──────────┼──────────┼────────────────────────────────┤
         * │ 死亡     │ 正常     │ rel_alt>30m → 確認飛行，降級   │
         * │ 死亡     │ 正常     │ rel_alt≤30m → 地面故障，不觸發 │
         * │ 死亡     │ 也死亡   │ ★不觸發（見下）                │
         * └──────────┴──────────┴────────────────────────────────┘
         *
         * ── 2026-07-30：刪除「兩者皆死 → 60 秒後推測離架」────────────────
         * 舊碼在兩顆感測器都死掉、且開機超過 60 秒時無條件宣告離架，再 18 秒
         * 點燃降落傘。問題在於**「開機 60 秒」不是「在高空」的證據**——正常
         * 流程本來就會在發射台待機遠超 60 秒。它真正的觸發條件其實只有
         * 「兩顆感測器都死」，而那在地面同樣會發生：BMP585 與 IMU 共用 SPI2，
         * 而 SPI2 焊點是本載具已實證的弱點（室外飛測 BMP 間歇、敲擊可重現）。
         * 在發射台曝曬、風吹、震動越久，掉出來的機會越大 → 78 秒後火箭靜止
         * 在架上點火，人可能還在旁邊。
         *
         * 而它保護的情境窄到近乎不存在：本區塊的外層條件是
         * `flight_state == FLIGHT_IDLE && !imu_armed`，也就是**還沒偵測到離架**。
         *   · 兩顆在離架「前」死 → 火箭在地上，本來就不該推測
         *   · 兩顆在離架「後」死 → imu_armed 早已是 1、flight_state 早已
         *     LAUNCHED → 這段程式碼根本不會被評估
         * 唯一還能用到它的，是「兩顆感測器在離架的同一個 200ms 窗內同時死掉」
         * 這個巧合。用地面誤點火的風險去換這個，不划算。
         *
         * 兩顆全滅時的真正冗餘是**雙板完全獨立熱備援**（另一塊板照常完成回收），
         * 不是同一塊板憑一個計時器自己猜。遠端手動開傘也不受影響，仍可救。
         * （BOTH_DEAD_HOLD_MS 與 both_dead_start 一併移除；bus_clamped 仍在別處使用。）
         */
        int baro_confirms  = (mod.bmp585 && rel_alt > 30.0f);
        /* 兩顆全滅的計時推測路徑已於 2026-07-30 移除，理由見上方說明。
         * 現在唯一的降級離架依據是「氣壓計還活著且說我們在 30 公尺以上」
         * ——那是**高度**的證據，不是時間的。 */
        if (baro_confirms) {
          /* 由降級路徑「推測」的離架，不是 2.5g 實測到的。記下來，
           * 讓下方的撤銷邏輯可以在證明是誤判時退回 IDLE。
           * ★2026-07-30：以前這裡寫 `both_dead_bkup ? 1 : 0`，於是 baro_confirms
           *   這條路被標成「非推測」而喪失撤銷資格。刪掉 both_dead 之後，本區塊
           *   剩下的每一條都是推測，一律標 1，讓撤銷邏輯繼續有作用。 */
          launch_inferred = 1;
          imu_armed      = 1;
          launch_time_ms = now;
          flight_state   = FLIGHT_LAUNCHED;
          is_boosting    = 0;
          /* ── 修正 1：KF2 對齊當前氣壓高度 ──────────────────────────
           * 若 kf2_h 留在 0，更新步會以為「從地面追到 30m」，
           * 造成 kf2_v 暴衝，完全失去參考價值。
           * 以 rel_alt 為起點，y ≈ 0，後續下降才能正確驅動 kf2_v。
           * ──────────────────────────────────────────────────────── */
          kf2_h   = rel_alt; kf2_v = 0.0f;
          kf2_p00 = 1.0f; kf2_p01 = 0.0f; kf2_p11 = 1.0f;
          vz_baro_lp = 0.0f;
          /* ── 修正 2：peak_rel_alt 對齊當前高度 ─────────────────────
           * 若留在 0，cond_A 會立即成立（rel_alt=30 > 0+20 且 30 < 0-10 → 不成立）
           * 實際上應以「確認飛行當下的高度」為最高點基準。
           * ──────────────────────────────────────────────────────── */
          peak_rel_alt    = rel_alt;
          vz_neg_start_ms = 0;
          cond_A          = 0;
          cond_B          = 0;
          if (baro_confirms) {
            char b[56];
            snprintf(b, sizeof(b), "WARN: IMU DEAD, BARO-CONFIRM alt=%.1fm\r\n", rel_alt);
            cdc_write(b);
          } else {
            cdc_write("WARN: IMU+BMP DEAD, FORCE-LAUNCH(60s)\r\n");
          }
        }
      }
    }

    /* ─── BMP585 每 20ms（50Hz）─── */
    if (now - t_bmp >= 20) {
      float dt_baro = (float)(now - t_bmp) * 0.001f;
      if (dt_baro > 0.5f) dt_baro = 0.5f;
      t_bmp = now;

      if (mod.bmp585) {
        float p_raw = BMP585_ReadPressure();

        /* ── 凍結看門狗：raw 連續 50 次(1s)完全相同 = 感測器卡死 ──
         * 真實氣壓在 OSR 下必有 ±幾 LSB 雜訊，完全不變只可能是轉換停止
         * （暫存器被 SPI 毛刺改寫 / 狀態機卡死）→ 重新初始化。 */
        {
          static uint32_t bmp_raw_prev = 0;
          static uint8_t  bmp_same_cnt = 0;
          uint32_t raw_now = BMP585_GetLastRaw();
          if (raw_now == bmp_raw_prev) {
            if (++bmp_same_cnt >= 50) {
              bmp_same_cnt = 0;
              /* ★2026-07-31：先軟體重置再 init。只呼叫 Init 等於重寫兩個設定
               * 暫存器，對「狀態機卡住」這種真正需要救援的情況沒有作用。
               * 並檢查回傳值 —— 舊碼無論成敗都印 FREEZE REINIT，看起來像修好了。*/
              BMP585_SoftReset();
              uint8_t rid = BMP585_Init(&hspi2);
              cdc_write(rid ? "BMP: FREEZE REINIT ok\r\n"
                            : "BMP: FREEZE REINIT FAILED\r\n");
            }
          } else { bmp_same_cnt = 0; bmp_raw_prev = raw_now; }
        }

        if (p_raw > 800.f && p_raw < 1100.f) {
          /* ── 氣壓突變保護（plausibility gate，含防鎖死）─────────
           * 候選高度與上一有效值差 >10m/20ms（=500m/s）→ 拒收，攔截 SPI 毛刺。
           * 防鎖死：連續拒收 25 次(0.5s) → 強制重新錨定（寧可跳一次，不可永久失明）。*/
          float alt_cand = 44330.f * (1.f - powf(p_raw / ref_press, 0.1903f));
          float alt_jump = alt_cand - rel_alt;
          static uint8_t baro_rej_cnt = 0;
          int baro_valid = (now < 2000UL) || (fabsf(alt_jump) < 10.0f);
          if (!baro_valid && ++baro_rej_cnt >= 25) baro_valid = 1;

          if (baro_valid) {
            baro_rej_cnt = 0;
            bmp_err_cnt  = 0;
            press   = kf_update(p_raw);  /* 1D Kalman 平滑壓力（hPa）*/
            rel_alt = 44330.f * (1.f - powf(press / ref_press, 0.1903f));
            if (!cf_init) { cf_init = 1; }

            /* ── 氣壓微分速度（LP τ=1s）：KF2 重置初值 + 交叉驗證 ── */
            {
              static float   rel_alt_prev   = 0.0f;
              static uint8_t vz_baro_inited = 0;
              if (!vz_baro_inited) { vz_baro_inited = 1; rel_alt_prev = rel_alt; }
              else {
                float vz_raw = (rel_alt - rel_alt_prev) / dt_baro;
                float alpha  = dt_baro / (dt_baro + 1.0f);  /* τ=1s LP */
                vz_baro_lp  += alpha * (vz_raw - vz_baro_lp);
                rel_alt_prev = rel_alt;
              }
            }

            if (imu_armed) {
              /* IMU 死亡備援：等速模型預測（IMU 正常時預測由 10ms 區塊執行）
               * 否則預測停跑 → p01 不增長 → K1→0 → kf2_v 永久凍結。*/
              if (!mod.imu) {
                kf2_h   += kf2_v * dt_baro;
                kf2_p00 += dt_baro*2.0f*kf2_p01 + dt_baro*dt_baro*kf2_p11 + KF2_Q_H;
                kf2_p01 += dt_baro * kf2_p11;
                kf2_p11 += 0.05f;
              }

              /* ── 大偏差重置：★只在低g（滑行/下降，氣壓可信）才對齊 ──
               * 推力段（total_g≥1.5）氣壓是垃圾，禁止重置，全信 IMU。 */
              if (total_g < KF2_RESET_GMAX &&
                  fabsf(rel_alt - kf2_h) > KF2_RESET_THR_M) {
                kf2_h   = rel_alt;
                kf2_v   = vz_baro_lp;
                kf2_p00 = 4.0f; kf2_p01 = 0.0f; kf2_p11 = 4.0f;
              }

              /* ── KF2 更新步（50Hz，量測=氣壓高度）──────────────
               * H=[1,0]; y=baro-h_pred; S=P00+R; K=P·Hᵀ/S
               * ★高g 時膨脹 R：推力段震動/穿音速，少信氣壓多信 IMU。*/
              float r_h = (total_g > 2.0f) ? (KF2_R_H * KF2_R_HIGHG_MULT) : KF2_R_H;
              float y   = rel_alt - kf2_h;
              float S   = kf2_p00 + r_h;
              float K0  = kf2_p00 / S;
              float K1  = kf2_p01 / S;
              kf2_h += K0 * y;
              kf2_v += K1 * y;
              float p00n = (1.0f - K0) * kf2_p00;
              float p01n = (1.0f - K0) * kf2_p01;
              float p11n = kf2_p11 - K1 * kf2_p01;
              kf2_p00 = p00n; kf2_p01 = p01n;
              kf2_p11 = (p11n > 1e-4f) ? p11n : 1e-4f;  /* 防協方差負數 */

              /* ── 開傘條件 A：裸氣壓高度低於最高點（混合方案，不經 KF）──
               * 與 cond_B(KF2 速度) 半獨立 → AND 閘防禦縱深 */
              if (rel_alt > peak_rel_alt) peak_rel_alt = rel_alt;
              cond_A = (peak_rel_alt >= DEPLOY_PEAK_MIN_M &&
                        rel_alt < peak_rel_alt - DEPLOY_DROP_M) ? 1 : 0;
            } else {
              /* 未起飛：KF2 凍結對齊地面，等待發射 */
              kf2_h = rel_alt; kf2_v = 0.0f;
            }
          }
        } else {
          /* BMP 讀值超範圍 → 連續失敗標記死亡 */
          if (++bmp_err_cnt >= MOD_ERR_MAX) {
            mod.bmp585 = 0;
            cdc_write("WARN: BMP DEAD\r\n");
          }
        }
      } else {
        /* ── BMP 死亡自動復活：每 5s 重新初始化（一次瞬斷不賠掉整趟）── */
        static uint32_t bmp_reinit_t = 0;
        if (now - bmp_reinit_t >= 5000UL) {
          bmp_reinit_t = now;
          uint8_t bid = BMP585_Init(&hspi2);
          if (bid != 0) {
            mod.bmp585 = 1; bmp_err_cnt = 0;
            cdc_write("BMP: REINIT OK\r\n");
          }
        }
      }
    }

    /* ═══════════════════════════════════════════════════════════════
     * LoRa 每 500ms（2 Hz，所有飛行狀態）
     * ★ 非阻塞架構：SendAsync 立即返回，PollTx 每輪迴圈輪詢一次
     *   TX airtime（~50-250ms）期間主迴圈持續執行，開傘狀態機不中斷
     * ═══════════════════════════════════════════════════════════════ */
    /* pyro 電源監測（內部自帶 1Hz 節流）*/
    PyroADC_Poll(now);

    if (now - t_lora >= 500) {
      t_lora = now;

      /* BRIDGE 模式：靜音自身遙測，空口只留 sim_replay 轉發的資料
       * （PB7 禁用跳線已廢除，見上方註解——該腳現為 E22 M0）*/
      if (mod.lora && !cmd_bridge_active()) {
        /* TX 空閒時發送新封包（poll 已移至主迴圈頂部，每次迴圈執行）*/
        if (!LoRa_IsBusy()) {
          lora_seq++;   /* 序號遞增（接收端用來偵測掉包）*/
          /* ── LoRa 遙測封包格式 ───────────────────────────────────────────
           * ⚠ 此格式是「火箭端 ↔ 地面端」共用協定。
           * 改動下面任一欄位 → 必須同步更新地面解析端與對接協議。
           * ───────────────────────────────────────────────────────────────── */
          char lora_pkt[256];
          int lora_n;
          int ca_eff = deploy_A_eff();
          int cb_eff = deploy_B_eff();
          GNSS_Data gd = GNSS_GetData();

          /* 【①】地面測試模式開著時，遙測每一幀都吼一聲——這個模式會解除
           * 氣囊自動充氣的閘門，絕不能在沒人注意的情況下留著。*/
          if (gnd_test_active() && !lora_tx_pending)
            LoRa_SendStr("MSG WARN GROUND TEST MODE ACTIVE - PB6 manual fire enabled\r\n");

          uint8_t mod_hex = (mod.bmp585 << 3) | (mod.imu << 2) | (mod.lora << 1) | (mod.sdcard << 0);
          uint8_t pyro_hex = (cond_A << 3) | (ca_eff << 2) | (cond_B << 1) | (cb_eff << 0);

          /* VF=保險絲後端電壓、VA=arming 開關後端電壓（-1.00 = ADC 不可用）。
           * 地面端以正則 key-value 解析，新欄位向後相容（舊版解析器直接忽略）。*/
          if (gd.valid) {
            lora_n = snprintf(lora_pkt, sizeof(lora_pkt),
              "T%lu SQ%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f ST:%d MOD:%X GPS:1,%u C:%X VF%.2f VA%.2f LAT%+0.5f LON%+0.5f\r\n",
              (unsigned long)now, (unsigned long)lora_seq, ax, ay, az,
              gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
              press, rel_alt, kf2_h, kf2_v, total_g,
              (int)flight_state, mod_hex, (unsigned)gd.num_sats, pyro_hex,
              v_fuse, v_arm,
              gd.latitude, gd.longitude);
          } else {
            lora_n = snprintf(lora_pkt, sizeof(lora_pkt),
              "T%lu SQ%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f ST:%d MOD:%X GPS:0,0 C:%X VF%.2f VA%.2f\r\n",
              (unsigned long)now, (unsigned long)lora_seq, ax, ay, az,
              gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
              press, rel_alt, kf2_h, kf2_v, total_g,
              (int)flight_state, mod_hex, pyro_hex,
              v_fuse, v_arm);
          }

          if (lora_n > 0 && lora_n < (int)sizeof(lora_pkt)) {
            if (LoRa_SendAsync((uint8_t*)lora_pkt, (uint8_t)lora_n) == 0) {
              lora_tx_pending = 1;   /* TX 已啟動，下次 poll 確認結果 */
            } else {
              lora_fail++;           /* SendAsync 失敗（不應發生）*/
            }
          }
        }
      }
      /* ★ post-lora 補查已移除：TX 非阻塞，主迴圈頂部狀態機已完整覆蓋 */
    }
    /* ═══════════════════════════════════════════════════════════════ */

    /* ─── SD CSV 記錄：狀態感知取樣率（獨立於 USB/LoRa 節拍）──────────────
     * IDLE 2Hz（桌面測試檔案小）→ 飛行中 50Hz（20ms，對齊氣壓/KF2 更新率）
     * → LANDED 每 5s。
     * 負載實算：每列 ~150-180B（25 欄含兩個 %.5f）→ 50Hz ≈ 7-9KB/s，
     * 656kHz 實效 ~55-65KB/s → 餘裕 ~6-9 倍；8MB 預分配 ≈ 15-19 分鐘飛行。
     * ⚠ 已知取捨（code-review C8, PLAUSIBLE）：50Hz 使 SD sector 落盤/
     *   GC 停頓（10~250ms）落在開傘決策迴圈的機率比 2Hz 高 ~25 倍，
     *   最壞讓開傘晚一個停頓長度（~2.5m @10m/s）。要保守可把 20UL 調回
     *   40~50UL（25~20Hz）換一半曝險。 */
    mod.sdcard = logger_is_ready();   /* 以 logger 即時狀態為準：CLEAR 救回
                                       * SD 後不再卡死為 0（code-review C2）*/
    uint32_t csv_interval;
    switch (flight_state) {
      case FLIGHT_LAUNCHED:
      case FLIGHT_DEPLOYING:
      case FLIGHT_DEPLOYED:  csv_interval = 20UL;              break;  /* 50Hz */
      case FLIGHT_LANDED:    csv_interval = LAND_LOG_INTERVAL; break;  /* 5s   */
      default:               csv_interval = 500UL;             break;  /* 2Hz  */
    }
    if (now - t_csv >= csv_interval && logger_is_ready()) {
      t_csv = now;
      /* 每個 log 檔「世代」寫一次 CSV header：CLEAR 滾新檔後
       * logger_file_gen 遞增 → 自動補寫新檔標頭；寫失敗不鎖定、
       * 下個 tick 重試（code-review C1/C5）。 */
      static uint8_t csv_hdr_gen = 0;
      if (csv_hdr_gen != logger_file_gen()) {
        const char *hdr =
          "time_ms,state,ax,ay,az,gx,gy,gz,press,rel_alt,kf_h,kf_v,"
          "total_g,tc,fix,lat,lon,sats,bmp,imu,lora,sd,condA,condB,peak\r\n";
        UINT bwh = 0;
        if (f_write(&file, hdr, (UINT)strlen(hdr), &bwh) == FR_OK
            && bwh == (UINT)strlen(hdr)) {
          csv_hdr_gen = logger_file_gen();
        }
      }
      /* 組一筆 CSV 資料行
       * state 欄：0=IDLE 1=LAUNCHED 2=DEPLOYING 3=DEPLOYED 4=LANDED
       * condA/condB 為原始量測值（故障容錯後的 eff 值可由 MOD 欄推得）*/
      GNSS_Data gd_csv = GNSS_GetData();
      char csv[256];
      int cn = snprintf(csv, sizeof(csv),
        "%lu,%d,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,"
        "%.1f,%.1f,%d,%.5f,%.5f,%u,%d,%d,%d,%d,%d,%d,%.1f\r\n",
        (unsigned long)now, (int)flight_state,
        ax, ay, az,
        gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
        press, rel_alt, kf2_h, kf2_v,
        total_g, tc,
        (int)gd_csv.valid, gd_csv.latitude, gd_csv.longitude, (unsigned)gd_csv.num_sats,
        mod.bmp585, mod.imu, mod.lora, mod.sdcard,
        cond_A, cond_B, peak_rel_alt);

      if (cn > 0 && cn < (int)sizeof(csv)) {
        UINT bw_sd = 0;
        FRESULT fw = f_write(&file, csv, (UINT)cn, &bw_sd);
        static uint8_t csv_reopen_fails = 0;   /* reopen 循環斷路器計數 */
        if (fw == FR_OK && bw_sd == (UINT)cn) {
          csv_reopen_fails = 0;                /* 真正寫成功才歸零 */
          sd_write_cnt++;
          /* ── sync 策略：只有地面/落地做顯式 sync ──
           * 飛行中(LAUNCHED/DEPLOYING/DEPLOYED)零 sync：8MB 預分配讓目錄/FAT
           * 在 init 就定案（fsize 固定），資料窗每 512B 自然落盤 →
           * 斷電最多丟 ~512B（50Hz 下 ≈0.1s 資料）；顯式 f_sync 反而每次
           * 多寫目錄 sector（15~40ms 阻塞），高頻下傷開傘狀態機。
           * ★ 零 sync 的前提＝「預分配成功且寫入仍在區內」。前提破功
           *   （卡空間不足 prealloc 失敗；或落地未偵測——掛樹/高地形——
           *   寫爆 8MB）時退回週期 sync，否則邊配的 cluster 永不落 FAT，
           *   斷電後 8MB 之外全部變 lost cluster（code-review S1/C6）。 */
          switch (flight_state) {
            case FLIGHT_LAUNCHED:
            case FLIGHT_DEPLOYING:
            case FLIGHT_DEPLOYED:
              if (!logger_prealloc_ok()
                  || f_tell(&file) >= LOG_PREALLOC_BYTES) {
                if ((sd_write_cnt % 50U) == 0U) f_sync(&file);  /* ~1s */
              }
              break;                     /* 前提成立：飛行中不 sync */
            default:
              f_sync(&file);             /* IDLE / LANDED：每筆落盤 */
              break;
          }
        } else {
          static uint8_t sd_err_cnt = 0;
          if (sd_err_cnt < 5) {
            char e[64];
            snprintf(e,sizeof(e),"SD_ERR fw=%d bw=%u cn=%d\r\n",(int)fw,(unsigned)bw_sd,cn);
            cdc_write(e); sd_err_cnt++;
          }
          /* ── 復原（code-review C4）：FatFS 寫入失敗會鎖定 fp->err
           * （僅 f_open 能清），不重開則之後每筆立即失敗＝其餘飛行
           * 全部靜默丟失。節流 2s：重開＋seek 回原寫入位置。
           * ── 斷路器（2026-07-20 室外實測）：卡半死時 reopen 會「假成功」
           * （f_open 吃 FatFS 快取回 OK，實際 I/O 全掛）→ 寫失敗↔reopen
           * 無限循環，永遠輪不到完整 remount。連續 3 輪（~6s）沒有任何
           * 一筆寫成功 → 強制標記失效，交給熱插拔恢復做 SD_disk_deinit
           * ＋f_mount 全套（含深度救援 sd_unstick_card）。          */
          static uint32_t csv_reopen_t = 0;
          if (now - csv_reopen_t >= 2000UL) {
            csv_reopen_t = now;
            if (++csv_reopen_fails >= 3) {
              csv_reopen_fails = 0;
              logger_mark_failed();
              cdc_write("SD: REOPEN LOOP -> FULL REINIT\r\n");
            } else if (logger_reopen() == 0) {
              cdc_write("SD: REOPEN OK\r\n");
            }
          }
        }
      }
    }

    /* ─── UART/CDC 輸出：一般每 500ms；LANDED 後降頻到每 5s ───
     * （僅遙測輸出；SD 寫入節拍由上方 t_csv 區塊獨立控制）
     * BRIDGE 模式跳過：USB 讓給 sim_replay 輸入，不再狂寫自身遙測
     * （否則螢幕亂＋無人讀時 cdc_write 空轉吃 CPU）。 */
    uint32_t out_interval = (flight_state == FLIGHT_LANDED) ? LAND_LOG_INTERVAL : 500UL;
    if (now - t_out >= out_interval && !cmd_bridge_active()) {
      t_out = now;
      /* LED_B10 toggle removed to serve as dedicated LoRa TX flicker */

      GNSS_Data gd = GNSS_GetData();

      char state_str[24];
      switch (flight_state) {
        case FLIGHT_IDLE:      snprintf(state_str,sizeof(state_str),"IDLE"); break;
        case FLIGHT_LAUNCHED:  snprintf(state_str,sizeof(state_str),"LAUNCHED T%lu",(unsigned long)(now-launch_time_ms)); break;
        case FLIGHT_DEPLOYING: snprintf(state_str,sizeof(state_str),"DEPLOYING"); break;
        case FLIGHT_LANDED:    snprintf(state_str,sizeof(state_str),"LANDED"); break;
        default:               snprintf(state_str,sizeof(state_str),"DEPLOYED"); break;
      }

      /* 輸出格式說明：
       * T        = 系統時間 (ms)
       * Ax/Ay/Az = 加速度 (g)
       * Gx/Gy/Gz = 角速度 (deg/s)
       * P        = 濾波後氣壓 (hPa)
       * RelH     = 相對起飛點高度 (m，裸氣壓)
       * KfH      = KF2 融合估算高度 (m)
       * Vz       = KF2 垂直速度 (m/s)，「!」= 已武裝
       * G        = 合加速度 (g)，「!」= imu_armed，「^」= g2_count 遞增中
       * Tc       = IMU 晶片溫度 (°C)
       * MOD      = 模組存活：BMP/IMU/LORA/SD（1=正常，0=死亡）
       * SD       = SD 累計成功寫入次數
       * LTX      = LoRa TX 序號 N（成功數 / 總發送數）
       * CA/CB    = cond_A / cond_B 原始值（0/1）
       * pk       = peak_rel_alt 最高點 (m) */
      char b[384]; int n;
      const char *g_flag = imu_armed ? "!" : (g2_count > 0 ? "^" : "");
      /* g_flag："!" 表示 imu_armed 已觸發，"^" 表示 g2_count 正在累積中 */

      /* 計算 cond_A_eff / cond_B_eff 供輸出顯示（與開傘決策同一組函式）*/
      int disp_Aeff = deploy_A_eff();
      int disp_Beff = deploy_B_eff();

      if (gd.valid)
        n = snprintf(b, sizeof(b),
          "T=%-7lu Ax=%+.3f Ay=%+.3f Az=%+.3f "
          "Gx=%+7.2f Gy=%+7.2f Gz=%+7.2f "
          "P=%7.2f RelH=%6.1f KfH=%6.1f Vz=%+6.2f%s "
          "G=%.2f%s Tc=%4.1f raw=0x%06lX%s "
          "lat=%+.5f lon=%+.5f altg=%.1f "
          "[%s] MOD=%d%d%d%d CA=%d/%d CB=%d/%d pk=%.1f "
          "SD=%lu LTX=N%lu(%lu/%lu)\r\n",
          (unsigned long)now, ax,ay,az,
          gx/0.017453293f,gy/0.017453293f,gz/0.017453293f,
          press,rel_alt,kf2_h,kf2_v,g_flag,
          total_g,imu_armed?"!":"",tc,(unsigned long)BMP585_GetLastRaw(),
          imu_ok?"":" !IMU",
          gd.latitude,gd.longitude,gd.altitude,
          state_str,
          mod.bmp585,mod.imu,mod.lora,mod.sdcard,
          cond_A,disp_Aeff, cond_B,disp_Beff, peak_rel_alt,
          (unsigned long)sd_write_cnt,
          (unsigned long)lora_seq,(unsigned long)lora_ok,
          (unsigned long)(lora_ok+lora_fail));
      else
        n = snprintf(b, sizeof(b),
          "T=%-7lu Ax=%+.3f Ay=%+.3f Az=%+.3f "
          "Gx=%+7.2f Gy=%+7.2f Gz=%+7.2f "
          "P=%7.2f RelH=%6.1f KfH=%6.1f Vz=%+6.2f%s "
          "G=%.2f%s Tc=%4.1f raw=0x%06lX%s "
          "GNSS=NO_FIX sv=%u/%u (b=%lu,l=%lu) "
          "[%s] MOD=%d%d%d%d CA=%d/%d CB=%d/%d pk=%.1f "
          "SD=%lu LTX=N%lu(%lu/%lu)\r\n",
          (unsigned long)now, ax,ay,az,
          gx/0.017453293f,gy/0.017453293f,gz/0.017453293f,
          press,rel_alt,kf2_h,kf2_v,g_flag,
          total_g,imu_armed?"!":"",tc,(unsigned long)BMP585_GetLastRaw(),
          imu_ok?"":" !IMU",
          (unsigned)gd.num_sats,(unsigned)gd.sats_in_view,
          (unsigned long)GNSS_GetByteCnt(),(unsigned long)GNSS_GetLineCnt(),
          state_str,
          mod.bmp585,mod.imu,mod.lora,mod.sdcard,
          cond_A,disp_Aeff, cond_B,disp_Beff, peak_rel_alt,
          (unsigned long)sd_write_cnt,
          (unsigned long)lora_seq,(unsigned long)lora_ok,
          (unsigned long)(lora_ok+lora_fail));

      /* USB CDC：人類可讀 key=value 格式（b）。
       * CSV 寫入已移到上方獨立節拍（狀態感知取樣率），與此 500ms 輸出脫鉤 */
      if (n > 0 && n < (int)sizeof(b)) {
        if (!cmd_is_typing()) { cdc_write(b); }
      }
    }

    /* ─── 互動功能與指示燈狀態更新 ─── */
    {
      // 1. 左側指示燈 B10（Active High：SET=亮，RESET=滅）＝LoRa 正在傳送
      //    ★PB7 禁用跳線已廢除（該腳改作 E22 M0），LED 純看實際 TX 活動：
      //      閃爍＝正在下行、恆滅＝模組沒起來或此刻沒在發。
      HAL_GPIO_WritePin(LED_B10_GPIO_Port, LED_B10_Pin,
                        (mod.lora && lora_tx_pending) ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // 3. 右側指示燈 B2（Active High：SET=亮，RESET=滅）系統綜合狀態
      static uint32_t t_led_b2 = 0;
      if (now - t_led_b2 >= 50) {
        t_led_b2 = now;
        uint8_t sen_ok = (mod.bmp585 && mod.imu);
        uint8_t sd_ok  = mod.sdcard;
        uint8_t gps_ok = GNSS_GetData().valid;

        if (!sen_ok || !sd_ok) {
          // 優先級 1：硬體故障 -> 快速閃爍 (5Hz, 每 200ms 週期：前 100ms 亮，後 100ms 滅)
          if ((now % 200) < 100) {
            HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_SET);   /* 亮 */
          } else {
            HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_RESET); /* 滅 */
          }
        } else if (!gps_ok) {
          // 優先級 2：GNSS 定位中 -> 慢速閃爍 (1Hz, 每 1000ms 週期：前 500ms 亮，後 500ms 滅)
          if ((now % 1000) < 500) {
            HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_SET);   /* 亮 */
          } else {
            HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_RESET); /* 滅 */
          }
        } else {
          // 優先級 3：一切正常 -> 恆亮
          HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_SET);     /* 亮 */
        }
      }

      // 4. 手動點火按鈕偵測與防呆 (PB6 / SIG_2_Pin 接地)
      static uint8_t  manual_fire_active = 0;
      static uint32_t manual_fire_start_t = 0;
      static GPIO_PinState stable_btn_state = GPIO_PIN_SET;
      static GPIO_PinState manual_fire_btn_last = GPIO_PIN_SET;
      static uint32_t last_state_change_t = 0;

      GPIO_PinState raw_btn_state = HAL_GPIO_ReadPin(GPIOB, SIG_2_Pin);
      
      // 軟體去彈跳 (Debounce)
      if (raw_btn_state != stable_btn_state) {
        if (now - last_state_change_t >= 50UL) { // 50ms 穩定時間
          stable_btn_state = raw_btn_state;
          last_state_change_t = now;
        }
      } else {
        last_state_change_t = now;
      }
      
      /* ★2026-07-31：加上閘門 ────────────────────────────────────────────
       * 原本這裡唯一的條件是「不是 FLIGHT_DEPLOYING」，也就是說在
       * IDLE / LAUNCHED / DEPLOYED / LANDED **全部**都能觸發，而且是
       * **同時點燃 PA0（降落傘）與 PA1（氣囊）兩路**。沒有檢查 manual_armed、
       * 沒有檢查飛行狀態、沒有時間閘門 —— PB6 拉低 50ms 就點火。
       *
       * PB6 有內部上拉（gpio.c: GPIO_PULLUP），所以不是浮空；但接線鬆脫碰地、
       * 連接器振動、焊橋都會滿足條件。人員可能就在箭體旁邊。
       *
       * 規範 4.6.3 要求「有人靠近時必須有兩個獨立事件」擋著儲能裝置，
       * 這條路徑一個都沒有。
       *
       * 閘門對齊遠端 dpl/abg 的桌測解鎖路徑：必須在 IDLE、必須先 ARM、
       * 而且必須在地面測試模式內。三者缺一不可。
       * 本次飛行不使用這個功能 —— 條件永遠不成立，等同關閉。*/
      if (stable_btn_state == GPIO_PIN_RESET && manual_fire_btn_last == GPIO_PIN_SET) {
        if (!manual_fire_active
            && flight_state == FLIGHT_IDLE
            && manual_armed
            && gnd_test_active()) {
          manual_fire_active = 1;
          manual_fire_start_t = now;
          /* ★2026-07-31：改硬體後傘迴路需要 PA0+PA1 同時驅動，只拉一路
           * 等於什麼都沒發生。這顆鈕的用途是地面驗證發火迴路，拉單路就
           * 失去意義，所以改走 deploy_fire_on()。
           * 三道閘門（IDLE + ARM + GNDTEST）不變 —— 那才是安全來源。*/
          deploy_fire_on();
          cdc_write("*** MANUAL FIRE ACTIVE (0.25s, PA0+PA1) ***\r\n");
        } else if (!manual_fire_active) {
          cdc_write("MANUAL FIRE REJECTED - need IDLE + ARM + GNDTEST\r\n");
        }
      }
      manual_fire_btn_last = stable_btn_state;

      // 0.25 秒定時結束，自動拉低
      if (manual_fire_active) {
        if (now - manual_fire_start_t >= 250UL) {
          manual_fire_active = 0;
          // 若目前並非飛行開傘中，則安全拉低點火腳
          if (flight_state != FLIGHT_DEPLOYING) {
            /* DEPLOYING 中不碰 —— 那是真的在開傘，脈衝由狀態機收尾。*/
            deploy_fire_off();
          }
          cdc_write("*** MANUAL FIRE ENDED ***\r\n");
        }
      }
    }
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
    /* ★HSE 起振失敗 → 降級 HSI 繼續飛，不再 Error_Handler 無限閃燈磚死。
     *   舊版註解宣稱有降級路徑，實際上那段程式碼只存在於 CSS callback（而 CSS
     *   從未啟用），開機失敗時走的是 Error_Handler，整塊板完全不動、連遙測都沒有。
     *   降級後 SYSCLK 一樣是 84MHz（HSI 16MHz /8 ×168 /4），但 HSI 是 RC 振盪器，
     *   未校準時精度約 ±1%：18 秒備援會有 ±180ms 誤差。相對於 apogee+1.34s 的
     *   餘裕仍可接受，而且遠勝於整塊板不動。開機報告會標 CLK=HSI-FB 讓你知道。*/
    RCC_OscInitTypeDef hsi = {0};
    clk_hsi_fallback = 1;
    hsi.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    hsi.HSIState            = RCC_HSI_ON;
    hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    hsi.PLL.PLLState        = RCC_PLL_ON;
    hsi.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    hsi.PLL.PLLM = 8;  hsi.PLL.PLLN = 168;
    hsi.PLL.PLLP = RCC_PLLP_DIV4;  hsi.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&hsi) != HAL_OK)
    {
      Error_Handler();   /* HSE 與 HSI 都起不來＝晶片層級故障，這才真的沒救 */
    }
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

  /* ★啟用時鐘安全系統（CSS）：HSE 在飛行中停振時觸發 NMI → HAL_RCC_CSSCallback
   *   自動把 PLL 切到 HSI 繼續跑。那個 callback 一直都寫在下面，但**從來沒有人
   *   呼叫 HAL_RCC_EnableCSS()**，所以整套保護是死碼——飛行中晶振一停，MCU 直接
   *   停擺，發火腳鎖在當下電位、狀態機停止、遙測也斷。
   *   已經在 HSI 上（HSE 起振就失敗）時不啟用：CSS 只監看 HSE。*/
  if (!clk_hsi_fallback) HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* ── CSS 回呼：飛行中 HSE 掛掉時由 NMI 呼叫 ─────────────────────────────
 * 進來時硬體已自動把 SYSCLK 切到 HSI 16MHz(裸速)。這裡把 PLL 重配回
 * 84MHz:APB 頻率復原 → UART/SPI 波特率全部不變,遙測與開傘照常。
 * USB 可能掉(HSI 精度不足),飛行中無所謂。就算重配失敗,也維持
 * HSI 16MHz 慢速活著——狀態機續跑,絕不磚死。 */
void HAL_RCC_CSSCallback(void)
{
  clk_hsi_fallback = 2;
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState  = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = 8;  osc.PLL.PLLN = 168;
  osc.PLL.PLLP = RCC_PLLP_DIV4;  osc.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&osc) == HAL_OK) {
    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 就地初始化 LED GPIO(不依賴 MX_GPIO_Init 是否已跑過)——
   * 之前 HSE 故障死在時鐘 init(GPIO 未設)= 全暗無從診斷;現在任何
   * 階段掛掉都保證看得到 5Hz 快閃。不呼叫 __disable_irq():USB 若已
   * 起來則保持存活,COM 不消失。 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  {
    GPIO_InitTypeDef gled = {0};
    gled.Pin   = LED_B10_Pin;
    gled.Mode  = GPIO_MODE_OUTPUT_PP;
    gled.Pull  = GPIO_NOPULL;
    gled.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_B10_GPIO_Port, &gled);
  }
  while (1)
  {
    HAL_GPIO_TogglePin(LED_B10_GPIO_Port, LED_B10_Pin);   /* 狀態燈 */
    HAL_Delay(100);   /* 快閃 5Hz → 表示 Error_Handler 被呼叫 */
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
