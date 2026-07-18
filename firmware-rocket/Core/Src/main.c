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
 * ★若降落傘電火頭改插 7V_OUT1（PA0 雙路冗餘），把這兩行改成 FIRE_7V_1_* 即可 */
#define DEPLOY_PORT     FIRE_7V_2_GPIO_Port
#define DEPLOY_PIN      FIRE_7V_2_Pin
#define LAUNCH_AZ_G  1.3f       /* 離架偵測合加速度閾值 (g) */
#define DEPLOY_PULSE_MS  2000UL   /* GPIO HIGH 持續時間 ms */
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
#define DEPLOY_TB_MS       20000UL  /* C: 備援強制觸發時間 (ms) */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- 飛行狀態機 ---- */
typedef enum {
    FLIGHT_IDLE,       /* 地面靜置，等待離架 */
    FLIGHT_LAUNCHED,   /* 離架後，監視高度與時間 */
    FLIGHT_DEPLOYING,  /* 開傘訊號輸出中（2 秒） */
    FLIGHT_DEPLOYED,   /* 開傘完成，等待落地 */
    FLIGHT_LANDED      /* 落地確認，進入低功耗記錄模式 */
} FlightState_t;

static FlightState_t flight_state   = FLIGHT_IDLE;
static uint32_t      launch_time_ms = 0;   /* HAL_GetTick() at launch */
static uint32_t      deploy_time_ms = 0;   /* HAL_GetTick() at deploy */

/* ---- 落地偵測參數 ---- */
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

  while (remaining > 0)
  {
    uint16_t chunk = (uint16_t)(remaining > 64 ? 64 : remaining);
    /* 重試 3 次，每次間隔 5ms */
    for (int r = 0; r < 3; r++)
    {
      if (CDC_Transmit_FS((uint8_t*)p, chunk) == USBD_OK) break;
      HAL_Delay(5);
    }
    p += chunk;
    remaining -= chunk;
    if (remaining > 0) HAL_Delay(5);
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
  lsm6_write_reg(0x18, 0x38); /* CTRL9_XL: Den_X/Y/Z=1，啟用加速度計三軸輸出 */
  lsm6_write_reg(0x19, 0x38); /* CTRL10_C: Den_X/Y/Z=1，啟用陀螺儀三軸輸出 */
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
  { char b[160];
    const char *rst =
      __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? "IWDG" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? "WWDG" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? "LPWR" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  ? "SOFT" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)  ? "POWER-ON" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)  ? "BROWNOUT" :
      __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  ? "NRST-PIN" : "?";
    __HAL_RCC_CLEAR_RESET_FLAGS();
    snprintf(b, sizeof(b),
      "MOD: BMP=%d IMU=%d LORA=%d  REF_PRESS=%.2f hPa  CLK=%s  RST=%s%s\r\n",
      mod.bmp585, mod.imu, mod.lora, ref_press,
      clk_hsi_fallback ? "HSI-FB(HSE FAIL!)" : "HSE", rst,
      bus_clamped ? "  BUS=CLAMPED!(SPI2 Hi-Z)" : "");
    cdc_write(b);
    if (mod.lora) LoRa_SendStr(b); }

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

    /* [A] DEPLOYING → DEPLOYED：脈衝 2 秒後關閉 GPIO */
    if (flight_state == FLIGHT_DEPLOYING &&
        (now - deploy_time_ms) >= DEPLOY_PULSE_MS) {
      HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_RESET);
      flight_state = FLIGHT_DEPLOYED;
      land_stable_start = 0;   /* 重置落地計時器 */
      cdc_write("*** DEPLOYED ***\r\n");
    }

    /* [A2] DEPLOYED → LANDED：落地偵測
     * 條件：total_g 接近 1g（靜止）且 rel_alt < 30m，持續 10 秒
     * 目的：進入低功耗模式，減少 SD 寫入，保留 LoRa beacon 供尋回 */
    if (flight_state == FLIGHT_DEPLOYED) {
      int land_cond = (fabsf(total_g - 1.0f) < LAND_G_DEV_THR) &&
                      (rel_alt < LAND_ALT_THR);
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

    /* [B] LAUNCHED → DEPLOYING
     *   主觸發：(A) 氣壓高度低於最高點 10m  AND  (B) 垂直速度持續向下 1.5s
     *   備  援：(C) 飛行時間 ≥ 20s，無條件強制開傘
     *
     *   A/B 同時成立 → 頂點後確認下降，最精確
     *   C 單獨成立   → 感測器異常時保底觸發
     */
    if (flight_state == FLIGHT_LAUNCHED) {
      uint32_t t_fly = now - launch_time_ms;

      /* 感測器故障時該條件自動成立，靠另一感測器單獨守門
       * 兩者皆死時不自動成立（避免立即觸發），只靠備援 C */
      int cond_A_eff = mod.bmp585 ? cond_A : (mod.imu ? 1 : 0);
      int cond_B_eff = mod.imu    ? cond_B : (mod.bmp585 ? 1 : 0);

      int deploy_main = (cond_A_eff && cond_B_eff);
      int deploy_bkup = (t_fly >= DEPLOY_TB_MS);   /* C: t > 20s */

      if (deploy_main || deploy_bkup) {
        deploy_time_ms = now;
        flight_state   = FLIGHT_DEPLOYING;
        HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_SET);
        if (deploy_main) {
          /* 觸發訊息含診斷數據，便於事後分析 */
          char msg[80];
          snprintf(msg, sizeof(msg),
            "*** DEPLOY(A+B) pk=%.1fm now=%.1fm vz=%.2fm/s ***\r\n",
            peak_rel_alt, rel_alt, kf2_v);
          cdc_write(msg);
        } else {
          cdc_write("*** DEPLOY(BACKUP T>20s)! ***\r\n");
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
         * │ 死亡     │ 也死亡   │ t>60s → 雙重故障備援           │
         * └──────────┴──────────┴────────────────────────────────┘
         */
        int baro_confirms  = (mod.bmp585 && rel_alt > 30.0f);
        /* 兩者皆死：純時間方案，60s 後無條件視為飛行
         * 地面/空中皆適用，接受 60s 後仍在地面時的誤觸風險 */
        int both_dead_bkup = (!mod.bmp585 && now > 60000UL);

        if (baro_confirms || both_dead_bkup) {
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
              BMP585_Init(&hspi2);
              cdc_write("BMP: FREEZE REINIT\r\n");
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
    if (now - t_lora >= 500) {
      t_lora = now;

      if (mod.lora) {
        /* TX 空閒時發送新封包（poll 已移至主迴圈頂部，每次迴圈執行）*/
        if (!LoRa_IsBusy()) {
          const char *st_s;
          switch (flight_state) {
            case FLIGHT_IDLE:      st_s = "IDLE"; break;
            case FLIGHT_LAUNCHED:  st_s = (mod.imu && is_boosting) ? "BOOST" : "LAUNCH"; break;
            case FLIGHT_DEPLOYING: st_s = "APOGEE"; break;
            case FLIGHT_DEPLOYED:  st_s = "DESCENT"; break;
            case FLIGHT_LANDED:    st_s = "LANDED"; break;
            default:               st_s = "IDLE"; break;
          }
          lora_seq++;   /* 序號遞增（接收端用來偵測掉包）*/
          /* ── LoRa 遙測封包格式 ───────────────────────────────────────────
           * ⚠ 此格式是「火箭端 ↔ 地面端」共用協定。
           * 改動下面任一欄位 → 必須同步更新地面解析端與對接協議。
           * ───────────────────────────────────────────────────────────────── */
          char lora_pkt[256];
          int lora_n;
          int ca_eff = mod.bmp585 ? cond_A : (mod.imu ? 1 : 0);
          int cb_eff = mod.imu ? cond_B : (mod.bmp585 ? 1 : 0);
          GNSS_Data gd = GNSS_GetData();

          if (gd.valid) {
            lora_n = snprintf(lora_pkt, sizeof(lora_pkt),
              "T%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f TC%.1f RAW0x%06lX ST:%s MOD%d%d%d%d GPS:%s SV:%d,%d BF:0,%lu CA:%d,%d CB:%d,%d PK%.1f SD%lu LR:%lu,%lu,%lu LAT%+0.5f LON%+0.5f\r\n",
              (unsigned long)now, ax, ay, az,
              gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
              press, rel_alt, kf2_h, kf2_v, total_g, tc,
              (unsigned long)BMP585_GetLastRaw(), st_s,
              mod.bmp585, mod.imu, mod.lora, mod.sdcard,
              "FIX_3D", (int)gd.sats_in_view, (int)gd.num_sats,
              (unsigned long)main_loop_cnt,
              (int)cond_A, ca_eff,
              (int)cond_B, cb_eff,
              peak_rel_alt, (unsigned long)sd_write_cnt,
              (unsigned long)lora_seq, (unsigned long)lora_ok, (unsigned long)(lora_ok + lora_fail),
              gd.latitude, gd.longitude);
          } else {
            lora_n = snprintf(lora_pkt, sizeof(lora_pkt),
              "T%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f TC%.1f RAW0x%06lX ST:%s MOD%d%d%d%d GPS:%s SV:%d,%d BF:0,%lu CA:%d,%d CB:%d,%d PK%.1f SD%lu LR:%lu,%lu,%lu\r\n",
              (unsigned long)now, ax, ay, az,
              gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
              press, rel_alt, kf2_h, kf2_v, total_g, tc,
              (unsigned long)BMP585_GetLastRaw(), st_s,
              mod.bmp585, mod.imu, mod.lora, mod.sdcard,
              "NO_FIX", (int)gd.sats_in_view, (int)gd.num_sats,
              (unsigned long)main_loop_cnt,
              (int)cond_A, ca_eff,
              (int)cond_B, cb_eff,
              peak_rel_alt, (unsigned long)sd_write_cnt,
              (unsigned long)lora_seq, (unsigned long)lora_ok, (unsigned long)(lora_ok + lora_fail));
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
        if (fw == FR_OK && bw_sd == (UINT)cn) {
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
           * 全部靜默丟失。節流 2s：重開＋seek 回原寫入位置。      */
          static uint32_t csv_reopen_t = 0;
          if (now - csv_reopen_t >= 2000UL) {
            csv_reopen_t = now;
            if (logger_reopen() == 0) cdc_write("SD: REOPEN OK\r\n");
          }
        }
      }
    }

    /* ─── UART/CDC 輸出：一般每 500ms；LANDED 後降頻到每 5s ───
     * （僅遙測輸出；SD 寫入節拍由上方 t_csv 區塊獨立控制）      */
    uint32_t out_interval = (flight_state == FLIGHT_LANDED) ? LAND_LOG_INTERVAL : 500UL;
    if (now - t_out >= out_interval) {
      t_out = now;
      HAL_GPIO_TogglePin(LED_B10_GPIO_Port, LED_B10_Pin);   /* 狀態燈 */

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

      /* 計算 cond_A_eff / cond_B_eff 供輸出顯示（與開傘邏輯相同公式）*/
      int disp_Aeff = mod.bmp585 ? cond_A : (mod.imu ? 1 : 0);
      int disp_Beff = mod.imu    ? cond_B : (mod.bmp585 ? 1 : 0);

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
  /* ── 主時鐘 HSE 25MHz(黑丸版晶振)；起振失敗自動降級 HSI ────────────────
   * 兩路皆達 SYSCLK=84MHz / USB 48MHz,差別只在精度(HSI ±1%)。
   * 實案(2026-07-09):一塊黑丸版 HSE 不起振 → 舊碼在 GPIO init 前就進
   * Error_Handler = 全暗+無 USB 磚死(DFU 卻能進,因 bootloader 走 HSI)。
   * 現在:降級續跑並設 clk_hsi_fallback,開機報告印 CLK=HSI-FB 可診斷。*/
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
    clk_hsi_fallback = 1;                    /* HSE 死 → HSI 16MHz 降級 */
    RCC_OscInitStruct = (RCC_OscInitTypeDef){0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;          /* 16/8=2MHz = ST 建議 VCO 輸入 */
    RCC_OscInitStruct.PLL.PLLN = 168;        /* ×168=336 → /4=84MHz、/7=48MHz */
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();                       /* HSI 也掛 = 晶片級故障,無解 */
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

  if (!clk_hsi_fallback) {
    /* CSS 時鐘安全系統：飛行中 HSE 失效(震動可殺機械晶振)→ 硬體自動切 HSI
     * 並觸發 NMI → HAL_RCC_CSSCallback 重配 PLL,飛控/開傘狀態機不死。 */
    HAL_RCC_EnableCSS();
  }
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
