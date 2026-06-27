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
#include "i2c.h"
#include "sdio.h"
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

#include "imu.h"
#include "lora.h"
#include "gnss.h"
#include "sdcard.h"
#include "logger.h"
#include "bmp585.h"
#include "cmd.h"
#include "imu_fast.h"

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
#define MAH_2KP         1.0f      /* Mahony 比例增益 × 2 */
#define MAH_2KI         0.01f     /* Mahony 積分增益 × 2 */

/* ---- 氣壓卡爾曼濾波器參數 ---- */
#define KF_Q  0.1f    /* 過程雜訊方差 (hPa²)：越大追蹤越快 */
#define KF_R  0.01f   /* 量測雜訊方差 (hPa²)：BMP585 RMS≈0.03hPa */

/* ---- 2D 卡爾曼濾波器參數（高度 + 速度，IMU+baro 融合）----
 * Q_H：高度過程雜訊，越小越信任 IMU 預測
 * Q_V：速度過程雜訊，越大越快用 baro 修正 IMU 漂移
 * R_H：氣壓計量測雜訊方差（BMP585 高度 RMS ≈ ±0.5m → 0.25）*/
#define KF2_Q_H   0.0001f  /* 高度過程雜訊 */
#define KF2_Q_V   0.002f   /* 速度過程雜訊（每 10ms 步）
                            * 0.002 ≈ 容忍 ~0.5m/s² 的姿態/偏移誤差，
                            *   靜態速度噪聲 σ≈0.25m/s，收斂 <1s —
                            *   符合 10~20m 投放、終端速度 3~6m/s 的量測需求。
                            * 歷史：曾因 TIM2 bug（IMU 無資料→預測步未執行→K1=0）
                            *   誤判為「Q 太小無法修正」而調到 0.5（→速度≈氣壓微分，
                            *   噪聲 ±0.5~1m/s）。TIM2 修復後不需要。            */
#define KF2_R_H   0.25f    /* 氣壓計高度量測雜訊方差 (±0.5m RMS) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;  /* USART1: TX=PA15 debug, RX=PA10 GPS @ 9600 */
/* I2C 和 SPI 句柄 */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi3;

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

static float         ref_press      = 1013.25f; /* 地面氣壓，啟動時初始化 */
static float         rel_alt        = 0.0f;     /* 相對高度（m） */

/* =========================
   Debug helpers
   ========================= */
void uart1_write(const char *s)
{
  if (!s) return;
  /* 9600 baud: 280 bytes ≈ 292ms；timeout 設 600ms 確保不截斷 */
  (void)HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 600);
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
   LSM6DSOTR via SPI3, CS = PB1 (active low)
   ========================= */
#define IMU_CS_PORT GPIOB
#define IMU_CS_PIN  GPIO_PIN_1
static inline void IMU_CS_LOW(void)  { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static inline void IMU_CS_HIGH(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);   }

static uint8_t lsm6_read_reg(uint8_t reg)
{
  uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
  uint8_t rx[2] = { 0 };
  IMU_CS_LOW();
  (void)HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 100);
  IMU_CS_HIGH();
  return rx[1];
}

static void lsm6_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
  IMU_CS_LOW();
  (void)HAL_SPI_Transmit(&hspi3, tx, 2, 100);
  IMU_CS_HIGH();
}

static int lsm6_read_burst(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx[32];
  uint8_t rx[32];
  if (len + 1 > sizeof(tx)) return -9;

  tx[0] = (uint8_t)(start_reg | 0x80); // READ only; auto-increment comes from CTRL3_C.IF_INC
  memset(&tx[1], 0x00, len);

  IMU_CS_LOW();
  HAL_StatusTypeDef ok = HAL_SPI_TransmitReceive(&hspi3, tx, rx, (uint16_t)(len + 1), 200);
  IMU_CS_HIGH();

  if (ok != HAL_OK) return -1;
  memcpy(buf, &rx[1], len);
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
  uart1_write(b);
  cdc_write(b);

  if (who != 0x6C) return -10;

  lsm6_write_reg(0x12, 0x01); // SW_RESET
  HAL_Delay(50);

  lsm6_write_reg(0x12, 0x44); // BDU + IF_INC
  lsm6_write_reg(0x10, 0x64); // XL 416Hz, +/-16g
  lsm6_write_reg(0x11, 0x6C); // G  416Hz, 2000dps
  lsm6_write_reg(0x18, 0x38); // CTRL9_XL: enable XL X/Y/Z
  lsm6_write_reg(0x19, 0x38); // CTRL10_C: enable G  X/Y/Z
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

static void mahony_update(float ax,float ay,float az,
                          float gx,float gy,float gz,float dt)
{
  float n=sqrtf(ax*ax+ay*ay+az*az);
  if(n<0.001f)return;
  n=1.f/n; ax*=n;ay*=n;az*=n;

  float vx=2.f*(q1*q3-q0*q2);
  float vy=2.f*(q0*q1+q2*q3);
  float vz=q0*q0-q1*q1-q2*q2+q3*q3;

  float ex=ay*vz-az*vy;
  float ey=az*vx-ax*vz;
  float ez=ax*vy-ay*vx;

  mah_ix+=ex*MAH_2KI*dt; mah_iy+=ey*MAH_2KI*dt; mah_iz+=ez*MAH_2KI*dt;
  gx+=MAH_2KP*ex+mah_ix; gy+=MAH_2KP*ey+mah_iy; gz+=MAH_2KP*ez+mah_iz;

  gx*=0.5f*dt; gy*=0.5f*dt; gz*=0.5f*dt;
  float qa=q0,qb=q1,qc=q2;
  q0+=(-qb*gx-qc*gy-q3*gz);
  q1+=(qa*gx+qc*gz-q3*gy);
  q2+=(qa*gy-qb*gz+q3*gx);
  q3+=(qa*gz+qb*gy-qc*gx);

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
static float  az_bias_g   = 0.0f;  /* 垂直加速度偏差 (g)，world frame     */
static float  ax_bias_g   = 0.0f;  /* 加速度計 X 軸偏移 (g)              */
static float  ay_bias_g   = 0.0f;  /* 加速度計 Y 軸偏移 (g)              */
static float  gbx_dps     = 0.0f;  /* 陀螺儀 X 軸零率偏移 (dps)          */
static float  gby_dps     = 0.0f;  /* 陀螺儀 Y 軸零率偏移 (dps)          */
static float  gbz_dps     = 0.0f;  /* 陀螺儀 Z 軸零率偏移 (dps)          */
static float  imu_vz      = 0.0f;  /* 垂直速度，含 baro 修正 (m/s)（互補濾波，保留備用）*/
static float  imu_hz      = 0.0f;  /* IMU+baro 互補高度（保留備用）                      */
/* ── 降落傘終端速度用：氣壓計微分速度 ──────────────────────────
 * 氣壓計每 20ms 更新一次高度，微分後 LP 濾波（τ=2s）得到垂直速度。
 * 優於 IMU 積分的原因：終端速度時 g≈1g，阻尼邏輯會把 IMU 速度歸零；
 * 氣壓計微分不受 g 值影響，能正確反映降落速度。
 * ──────────────────────────────────────────────────────────── */
static float  vz_baro_lp  = 0.0f;  /* 氣壓計微分垂直速度 LP 濾波 (m/s)   */
/* ── 2D 卡爾曼濾波器狀態（高度 + 速度，IMU+baro 融合）──────────
 * 預測：IMU 垂直加速度（10ms），修正：氣壓計高度（20ms）
 * kf2_h 取代 imu_hz，kf2_v 取代 vz_ms，漂移自動修正      */
static float  kf2_h   = 0.0f;   /* KF 估計高度 (m)               */
static float  kf2_v   = 0.0f;   /* KF 估計速度 (m/s)             */
static float  kf2_p00 = 1.0f;   /* 協方差 P[高度,高度]           */
static float  kf2_p01 = 0.0f;   /* 協方差 P[高度,速度]           */
static float  kf2_p11 = 1.0f;   /* 協方差 P[速度,速度]           */
static uint32_t sd_write_cnt = 0;  /* SD 成功寫入次數 */
static uint8_t  imu_ok    = 0;     /* IMU 初始化是否成功 */

/* ---- 模組存活狀態 ---- */
typedef struct {
    uint8_t bmp585  : 1;   /* 氣壓計 */
    uint8_t imu     : 1;   /* IMU */
    uint8_t lora    : 1;   /* LoRa */
    uint8_t sdcard  : 1;   /* SD 卡 */
} ModStatus_t;
static ModStatus_t mod      = {0};   /* 預設全部失效 */

/* ---- 落地偵測狀態機 ---- */
typedef enum {
    LAND_FLIGHT   = 0,  /* 飛行中 */
    LAND_IMPACT   = 1,  /* 偵測到高 g 碰撞，等待確認 */
    LAND_LANDED   = 2,  /* 已確認落地，vz 持續歸零 */
} LandState_t;
static uint8_t bmp_err_cnt  = 0;     /* BMP 連續失敗計數 */
static uint8_t imu_err_cnt  = 0;     /* IMU 連續失敗計數 */
#define MOD_ERR_MAX  5               /* 連續 N 次失敗 → 標記死亡 */

/* ======================================================
   氣壓計 Kalman 濾波器
   ====================================================== */
static float kf_p_est  = 1013.25f;  /* 估計氣壓 */
static float kf_p_err  = 1.0f;      /* 估計誤差 */

static float kf_update(float meas)
{
  kf_p_err += KF_Q;
  float K = kf_p_err / (kf_p_err + KF_R);
  kf_p_est += K * (meas - kf_p_est);
  kf_p_err *= (1.f - K);
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
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* 等 USB CDC 完整枚舉 */
  HAL_Delay(1000);

  /* ---- IMU 初始化（重試 3 次）---- */
  for (int _i = 0; _i < 3; _i++) {
    if (lsm6_init() == 0) { mod.imu = 1; imu_ok = 1; break; }
    HAL_Delay(200);
  }

  /* ---- LoRa 初始化（重試 3 次）---- */
  for (int _i = 0; _i < 3; _i++) {
    if (LoRa_Init() == 0) { mod.lora = 1; break; }
    HAL_Delay(200);
  }

  /* ---- BMP585 初始化（重試 5 次，間隔 300ms）---- */
  HAL_Delay(50);   /* 確保 SPI3 空閒後再存取 BMP CS */
  for (int _i = 0; _i < 5; _i++) {
    uint8_t bid = BMP585_Init(&hspi3);
    char bmp_msg[40];
    snprintf(bmp_msg, sizeof(bmp_msg), "BMP_INIT try%d id=0x%02X\r\n", _i, bid);
    uart1_write(bmp_msg); cdc_write(bmp_msg);
    if (bid == 0x50 || bid == 0x51) { mod.bmp585 = 1; break; }
    HAL_Delay(300);
  }
  /* ref_press 故意延後到 IMU 校準之後再取樣（見下方），
   * 讓 BMP585 多 ~2 秒暖機，消除冷啟熱漂移導致的 -4m 系統偏移 */

  /* ---- GNSS 初始化 ---- */
  GNSS_Init(&huart1);

  /* ---- 開機狀態整合報告 ---- */
  { char b[96];
    snprintf(b, sizeof(b),
      "MOD: BMP=%d IMU=%d LORA=%d  REF_PRESS=%.2f hPa\r\n",
      mod.bmp585, mod.imu, mod.lora, ref_press);
    uart1_write(b); cdc_write(b);
    if (mod.lora) LoRa_SendStr(b); }

  /* ---- IMU 校準（僅在 IMU 成功初始化時執行）---- */
  if (mod.imu) {
    /* 收斂階段（1 秒）：讓 Mahony 姿態估算收斂到靜止方向 */
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/2048.f, a_y=ray/2048.f, a_z=raz/2048.f;  /* ±16g */
        float g_x=rgx/16.384f*0.017453293f;
        float g_y=rgy/16.384f*0.017453293f;
        float g_z=rgz/16.384f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
      }
    }
    /* 校準階段（1 秒）：同時計算 az/ax/ay 加速度偏移 + 陀螺儀零率偏移 */
    float s_az=0,s_ax=0,s_ay=0,s_gx=0,s_gy=0,s_gz=0;
    int   n_cal = 0;
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/2048.f, a_y=ray/2048.f, a_z=raz/2048.f;  /* ±16g */
        float g_x=rgx/16.384f*0.017453293f;
        float g_y=rgy/16.384f*0.017453293f;
        float g_z=rgz/16.384f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
        s_az += world_az(a_x,a_y,a_z);
        s_ax += a_x;
        s_ay += a_y;
        s_gx += rgx/16.384f;   /* dps */
        s_gy += rgy/16.384f;
        s_gz += rgz/16.384f;
        n_cal++;
      }
    }
    if (n_cal > 0) {
      az_bias_g = (s_az / n_cal) - 1.0f; /* 垂直方向：去掉 1g 重力 */
      ax_bias_g = s_ax / n_cal;           /* 水平方向：整體偏移      */
      ay_bias_g = s_ay / n_cal;
      gbx_dps   = s_gx / n_cal;           /* 陀螺零率偏移 (dps)      */
      gby_dps   = s_gy / n_cal;
      gbz_dps   = s_gz / n_cal;
      char b[128];
      snprintf(b, sizeof(b),
        "BIAS: az=%.5fg ax=%.5fg ay=%.5fg | gx=%.3f gy=%.3f gz=%.3f dps (n=%d)\r\n",
        az_bias_g, ax_bias_g, ay_bias_g, gbx_dps, gby_dps, gbz_dps, n_cal);
      uart1_write(b); cdc_write(b);
    }
  }

  /* ---- 地面氣壓基準（IMU 校準完成後取樣，BMP585 已暖機 ~2s）──────
   * BMP585 冷啟動有 +0.5 hPa 熱漂移（≈ -4m），等感測器穩定後取樣
   * 可將系統偏移從 -4m 降至 ±0.5m 以內。
   * 取 20 筆平均（400ms）並剔除超範圍讀值。                     */
  if (mod.bmp585) {
    float sum = 0.f; int cnt = 0;
    for (int _bi = 0; _bi < 20; _bi++) {
      HAL_Delay(20);
      float p = BMP585_ReadPressure();
      if (p > 800.f && p < 1100.f) { sum += p; cnt++; }
    }
    if (cnt >= 5) {
      ref_press = sum / cnt;
      kf_p_est  = ref_press;
    }
    char b2[64];
    snprintf(b2, sizeof(b2), "REF_PRESS=%.2f hPa (n=%d)\r\n", ref_press, cnt);
    uart1_write(b2); cdc_write(b2);
  }

  /* ---- ImuFast 初始化（TIM2 500Hz 中斷）----
   * 必須在 IMU 初始化 + 校準完成後才啟動，
   * 否則 ISR 會在 sensor 未配置前讀到 gyro=0 的垃圾資料 */
  ImuFast_Init();
  /* 傳入校準後的初始四元數與感測器偏移，讓 ISR Mahony 從正確姿態開始 */
  ImuFast_SetAttitude(q0, q1, q2, q3);
  ImuFast_SetBias(gbx_dps * 0.017453293f,
                  gby_dps * 0.017453293f,
                  gbz_dps * 0.017453293f,
                  ax_bias_g, ay_bias_g);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_imu = 0, t_bmp = 0, t_out = 0;
  uint8_t  sd_init_done = 0;
  float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;
  float press = ref_press, total_g = 1;

  uint8_t cf_init = 0;

  /* 落地偵測狀態機 */
  LandState_t land_state = LAND_FLIGHT;
  uint32_t    land_t0    = 0;
  float       land_baro0 = 0.0f;

  /* IMU.CSV 高速記錄用的 FatFs 檔案 */
  FIL   imu_fil;
  uint8_t  imu_fil_open  = 0;   /* 1 = 已開檔 */
  uint16_t csv_err_cnt   = 0;   /* CSV 寫入錯誤次數（含已自動復原者）*/
  char     csv_name[16]  = "";  /* 本次開機使用的序號檔名，如 IMU_003.CSV */
  /* retry buffer：f_write 失敗的那批 64 筆先存起來，下次迴圈補寫 */
  static char csv_retry_buf[8192];
  static int  csv_retry_len = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();

    /* ─── 指令系統 ─── */
    cmd_execute_pending();
    cmd_flush_echo();

    /* ─── GNSS NMEA 解析（從 ISR 移到主迴圈：atof/strtok 不可在中斷內跑）─── */
    GNSS_Poll();

    /* ─── SD 卡延遲初始化（啟動後 3 秒）─── */
    if (!sd_init_done && now >= 2000UL) {
      sd_init_done = 1;
      /* 先只掛載，掃序號後再開檔，使 LOG 與 CSV 同號且都不覆蓋舊資料 */
      if (logger_mount()) {
        /* 找下一個未使用的序號：IMU/LOG 任一存在就跳號（兩檔保證同步）
         * IMU_001.CSV ↔ LOG_001.TXT … 099 */
        int seq = 1;
        for (; seq <= 99; seq++) {
          char in[16], ln[16]; FILINFO fi;
          snprintf(in, sizeof(in), "IMU_%03d.CSV", seq);
          snprintf(ln, sizeof(ln), "LOG_%03d.TXT", seq);
          if (f_stat(in, &fi) != FR_OK && f_stat(ln, &fi) != FR_OK) break; /* 兩者皆不存在 */
        }
        if (seq > 99) seq = 99;   /* 滿了蓋最後一組 */
        snprintf(csv_name, sizeof(csv_name), "IMU_%03d.CSV", seq);
        { char ln[16]; snprintf(ln, sizeof(ln), "LOG_%03d.TXT", seq);
          logger_set_name(ln); }
        /* 用序號檔名開 LOG（APPEND，恢復路徑沿用同名接續，不清空）*/
        logger_open();
      }
      if (logger_is_ready()) {
        mod.sdcard = 1;
        uart1_write("SD: OK\r\n"); cdc_write("SD: OK\r\n");
        /* 開啟新序號 CSV（建新檔，不覆蓋舊資料）*/
        if (f_open(&imu_fil, csv_name, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
          imu_fil_open  = 1;
          csv_retry_len = 0;
          /* 寫入 CSV 標頭 */
          UINT bw_hdr = 0;
          const char *hdr = "ts_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,g_tot,baro_m,vz_ms,imu_hz_m\n";
          f_write(&imu_fil, hdr, strlen(hdr), &bw_hdr);
          { char msg[32]; snprintf(msg,sizeof(msg),"%s: OK\r\n",csv_name);
            uart1_write(msg); cdc_write(msg); }
          /* SYNC 標記：寫入 log.txt，記錄同一瞬間的兩種時間基準，
           * 事後分析可用 T(ms) − ts_us(µs)/1000 對齊 LOG 與 CSV 時間軸 */
          { char sm[64]; UINT bw_s = 0;
            snprintf(sm, sizeof(sm), "SYNC T=%lu ts_us=%lu\r\n",
                     (unsigned long)now, (unsigned long)ImuFast_GetTs());
            f_write(&file, sm, strlen(sm), &bw_s); f_sync(&file);
            uart1_write(sm); cdc_write(sm); }
        } else {
          { char msg[32]; snprintf(msg,sizeof(msg),"%s: FAIL\r\n",csv_name);
            uart1_write(msg); cdc_write(msg); }
        }
      } else {
        extern FRESULT fres;
        char e[48];
        snprintf(e, sizeof(e), "SD: FAIL (fres=%d)\r\n", (int)fres);
        uart1_write(e); cdc_write(e);
      }
    }

    /* ─── IMU 每 10ms ─── */
    if (now - t_imu >= 10) {
      float dt = (float)(now - t_imu) * 0.001f;
      if (dt > 0.05f) dt = 0.05f;
      t_imu = now;

      if (mod.imu) {
        /* 從 ImuFast ring buffer peek 最新一筆，無需鎖 SPI3 */
        ImuSample_t latest;
        int imu_rd = ImuFast_GetLatest(&latest);
        if (imu_rd == 0) {
          imu_err_cnt = 0;
          /* 原始值轉換 */
          ax = (float)latest.ax / 2048.f;   /* ±16g: 1g = 2048 LSB */
          ay = (float)latest.ay / 2048.f;
          az = (float)latest.az / 2048.f;
          const float deg2rad = 0.017453293f;
          gx = (float)latest.gx / 16.384f * deg2rad;
          gy = (float)latest.gy / 16.384f * deg2rad;
          gz = (float)latest.gz / 16.384f * deg2rad;

          /* 套用 bias 校準值（陀螺偏移已由 ISR Mahony 內部處理）*/
          float ax_c = ax - ax_bias_g;
          float ay_c = ay - ay_bias_g;

          total_g = sqrtf(ax_c*ax_c + ay_c*ay_c + az*az);
          /* Mahony 已移至 500Hz ISR，主迴圈直接讀取 ISR 輸出 */

          /* 積分垂直速度/高度 */
          {
            /* imu_fast_world_az：ISR Mahony 輸出的世界系垂直加速度（g，含 1g 重力）
             * 減去 az_bias_g 修正世界系垂直偏移，再轉 m/s² 並扣除重力 */
            float az_w   = imu_fast_world_az - az_bias_g;
            float lin_az = (az_w - 1.0f) * 9.80665f;
            float g_dev  = fabsf(total_g - 1.0f);

            /* ── imu+baro 互補版本 ── */
            imu_vz += lin_az * dt;
            imu_hz += imu_vz * dt;
            /* 阻尼：必須同時滿足 (g≈1g) AND (氣壓計確認速度接近零)
             * 純靠 g 判斷靜止在降落傘場景下會誤判：
             * 終端速度時 g≈1g（阻力=重力），但實際仍在快速下降。
             * vz_baro_lp > 0.5m/s 表示氣壓計偵測到真實運動 → 不阻尼 */
            if      (g_dev < 0.08f && fabsf(vz_baro_lp) < 0.5f)
              imu_vz *= (1.0f - dt / 0.15f);
            else if (g_dev < 0.20f && fabsf(vz_baro_lp) < 1.0f)
              imu_vz *= (1.0f - dt / 1.0f);

            /* ── KF2 預測（10ms，用 IMU 垂直加速度）──────────────
             * 狀態轉移矩陣 F = [[1,dt],[0,1]]
             * h_pred = h + v*dt + 0.5*a*dt²
             * v_pred = v + a*dt
             * P_pred = F*P*F^T + Q
             * lin_az 限幅 ±3g（30 m/s²）：避免衝擊事件污染速度估算 */
            {
              float dt2 = dt * dt;
              float az_kf = lin_az;
              if (az_kf >  29.4f) az_kf =  29.4f;   /* +3g */
              if (az_kf < -29.4f) az_kf = -29.4f;   /* -3g */
              kf2_h += kf2_v * dt + 0.5f * az_kf * dt2;
              kf2_v += az_kf * dt;
              float p00 = kf2_p00 + dt*(kf2_p01 + kf2_p01) + dt2*kf2_p11 + KF2_Q_H;
              float p01 = kf2_p01 + dt * kf2_p11;
              float p11 = kf2_p11 + KF2_Q_V;
              kf2_p00 = p00;
              kf2_p01 = p01;
              kf2_p11 = p11;
            }
          }
          /* 更新 ImuFast 快照（100Hz）
           * vz_ms = 氣壓計微分速度（LP 濾波）：適合終端速度量測
           * imu_hz_m = IMU+baro 互補高度（不受阻尼影響）           */
          imu_fast_vz_ms  = kf2_v;        /* KF2 融合速度（IMU 預測 + baro 修正）*/
          imu_fast_hz_m   = kf2_h;        /* KF2 融合高度                        */
        } else {
          /* IMU 讀取連續失敗 → 標記死亡 */
          if (++imu_err_cnt >= MOD_ERR_MAX) {
            mod.imu = 0; imu_ok = 0;
            uart1_write("WARN: IMU DEAD\r\n");
            cdc_write("WARN: IMU DEAD\r\n");
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
        ImuFast_LockSpi();
        float p_raw = BMP585_ReadPressure();
        ImuFast_UnlockSpi();

        /* ── 凍結看門狗：raw 連續 50 次(1s)完全相同 = 感測器卡死 ──
         * 真實氣壓在 OSR×4 下必有 ±幾 LSB 雜訊，完全不變只可能是
         * 轉換停止（暫存器被 SPI 毛刺改寫 / 狀態機卡死）→ 重新初始化。
         * SPI 整條死掉時 ReadPressure 回 0、raw 也不更新 → 同樣觸發。*/
        {
          static uint32_t bmp_raw_prev = 0;
          static uint8_t  bmp_same_cnt = 0;
          uint32_t raw_now = BMP585_GetLastRaw();
          if (raw_now == bmp_raw_prev) {
            if (++bmp_same_cnt >= 50) {
              bmp_same_cnt = 0;
              ImuFast_LockSpi();
              BMP585_Init(&hspi3);
              ImuFast_UnlockSpi();
              cdc_write("BMP: FREEZE REINIT\r\n");
            }
          } else { bmp_same_cnt = 0; bmp_raw_prev = raw_now; }
        }

        if (p_raw > 800.f && p_raw < 1100.f) {

          /* ── 氣壓突變保護（plausibility gate，含防鎖死機制）─────
           * 候選高度與上一有效值差 >10m/20ms（=500m/s）→ 拒收，
           * 攔截 SPI 毛刺（實測毛刺步階 15~230m）。
           * 防鎖死：若毛刺先被接受、真實讀值反而被擋（IMU_016 實案：
           * 假值 -32.4m 進門後，真值 +1m 連續被拒 18.5s = 氣壓「凍結」），
           * 連續拒收 25 次（0.5s）即強制重新錨定 — 寧可接受一次跳變，
           * 不能永久失明。重新錨定後 KF2 由下方 >5m 重置邏輯對齊。  */
          float alt_cand = 44330.f * (1.f - powf(p_raw / ref_press, 0.1903f));
          float alt_jump = alt_cand - rel_alt;   /* 與上一有效值的差 */

          static uint8_t baro_rej_cnt = 0;
          int baro_valid = (now < 2000UL) || (fabsf(alt_jump) < 10.0f);
          if (!baro_valid && ++baro_rej_cnt >= 25) {
            baro_valid = 1;          /* 0.5s 連續拒收 → 重新錨定到候選值 */
          }

          if (baro_valid) {
            baro_rej_cnt = 0;
            bmp_err_cnt = 0;
            press   = kf_update(p_raw);   /* 只對有效讀值更新 KF */
            rel_alt = 44330.f * (1.f - powf(press / ref_press, 0.1903f));

            /* 更新 ImuFast 快照：ISR 取樣時使用最新值 */
            imu_fast_baro_m = rel_alt;
            if (!cf_init) { cf_init = 1; }

            /* ── KF2 大偏差重置：高度誤差 > 5m 時直接對齊氣壓計 ──
             * 原因：衝擊/翻滾導致 IMU 飽和，kf2_v 積分偏離真實值；
             * 此時 baro 是唯一可信來源，強制重置比慢速收斂更可靠。
             * 重置後以高協方差重新讓 KF 快速吸收後續量測。         */
            if (fabsf(rel_alt - kf2_h) > 5.0f) {
              kf2_h   = rel_alt;
              kf2_v   = vz_baro_lp;   /* 以氣壓微分速度作初始估計 */
              kf2_p00 = 4.0f;
              kf2_p01 = 0.0f;
              kf2_p11 = 4.0f;
            }

            /* ── IMU 死亡備援：等速模型預測 ──────────────────────
             * IMU 正常時預測步由 10ms 區塊執行；IMU 死亡後預測停跑
             * → p01 不再增長 → K1→0 → kf2_v 永久凍結。
             * 此處改用等速模型 + 大 Q 預測，讓氣壓計獨立修正速度
             * （精度降為 ~±0.5m/s，但不會凍結，投放測試仍可量測）。*/
            if (!mod.imu) {
              kf2_h   += kf2_v * dt_baro;
              kf2_p00 += dt_baro * 2.0f * kf2_p01
                       + dt_baro * dt_baro * kf2_p11 + KF2_Q_H;
              kf2_p01 += dt_baro * kf2_p11;
              kf2_p11 += 0.05f;
            }

            /* ── KF2 更新（20ms，用氣壓計高度）──────────────────
             * 量測矩陣 H = [1, 0]（只量測高度）
             * y = baro_h - h_pred  (innovation)
             * S = P[0][0] + R
             * K = P*H^T / S
             * x = x + K*y,  P = (I - K*H)*P                 */
            {
              float y  = rel_alt - kf2_h;
              float S  = kf2_p00 + KF2_R_H;
              float K0 = kf2_p00 / S;
              float K1 = kf2_p01 / S;
              kf2_h += K0 * y;
              kf2_v += K1 * y;
              float p00n = (1.0f - K0) * kf2_p00;
              float p01n = (1.0f - K0) * kf2_p01;
              float p11n = kf2_p11 - K1 * kf2_p01;
              kf2_p00 = p00n;
              kf2_p01 = p01n;
              kf2_p11 = (p11n > 1e-4f) ? p11n : 1e-4f; /* 防協方差負數 */
            }

            /* ── 氣壓計微分速度（降落傘終端速度用）─────────────────
             * 每次有效氣壓讀值更新時，對高度差微分並進行 LP 濾波。
             * τ = 1s：靜態 σ ≈ 0.10 m/s，步階響應 ~3s 收斂到終端速度。
             * （若需更快但更噪：τ=0.5s；若需更穩但更慢：τ=2s）         */
            {
              static float rel_alt_vz_prev = 0.0f;
              static uint8_t vz_baro_inited = 0;
              if (!vz_baro_inited) {
                vz_baro_inited  = 1;
                rel_alt_vz_prev = rel_alt;
              } else {
                float vz_raw = (rel_alt - rel_alt_vz_prev) / dt_baro;
                float alpha  = dt_baro / (dt_baro + 1.0f);  /* τ=1s LP */
                vz_baro_lp  += alpha * (vz_raw - vz_baro_lp);
              }
              rel_alt_vz_prev = rel_alt;
            }
          }
          /* ── 落地偵測狀態機 ────────────────────────────────────────
           * FLIGHT → IMPACT：total_g > 6g 且 KF2 速度 < -1 m/s
           * IMPACT → LANDED：300ms 後氣壓高度變化 < 0.5m（確認靜止）
           * IMPACT → FLIGHT：300ms 後高度仍在變（誤判，繼續飛）
           * LANDED：持續將 kf2_v / imu_vz 歸零，防止積分漂移
           * ────────────────────────────────────────────────────── */
          switch (land_state) {
          case LAND_FLIGHT:
              if (total_g > 6.0f && kf2_v < -1.0f) {
                  land_state = LAND_IMPACT;
                  land_t0    = now;
                  land_baro0 = rel_alt;
              }
              break;
          case LAND_IMPACT:
              if (now - land_t0 >= 300UL) {
                  if (fabsf(rel_alt - land_baro0) < 0.5f) {
                      land_state = LAND_LANDED;
                      land_baro0 = rel_alt;   /* 記錄落地高度，供離地偵測 */
                      kf2_v  = 0.0f;
                      kf2_h  = rel_alt;
                      imu_vz = 0.0f;
                  } else {
                      land_state = LAND_FLIGHT;   /* 誤判，繼續 */
                  }
              }
              break;
          case LAND_LANDED:
              kf2_v  = 0.0f;
              imu_vz = 0.0f;
              /* 離地偵測：氣壓離開落地高度 >2m → 回到 FLIGHT。
               * 原本 LANDED 是終點站 → 同一次開機的第二次投放
               * vz 全程被釘在 0（多次投放測試的隱形殺手）。      */
              if (fabsf(rel_alt - land_baro0) > 2.0f)
                  land_state = LAND_FLIGHT;
              break;
          }

          /* 誤差鉗制：超過 ±20m 時不修正（等待 baro 恢復穩定）*/
          if (cf_init) {
            float err = rel_alt - imu_hz;
            if (fabsf(err) <= 20.0f) {
              float kp  = (total_g > 2.0f) ? 0.01f : 0.10f;
              float ki  = (total_g > 2.0f) ? 0.001f : 0.02f;
              imu_hz += kp * err;
              imu_vz += ki * err;
            }
          }
        } else {
          /* BMP 讀值超範圍 → 連續失敗標記死亡 */
          if (++bmp_err_cnt >= MOD_ERR_MAX) {
            mod.bmp585 = 0;
            uart1_write("WARN: BMP DEAD\r\n");
            cdc_write("WARN: BMP DEAD\r\n");
          }
        }
      } else {
        /* ── BMP 死亡自動復活：每 5s 重新初始化（與 SD 同策略）──
         * 原本標記死亡後永遠不再嘗試 → 一次瞬斷賠掉整趟資料 */
        static uint32_t bmp_reinit_t = 0;
        if (now - bmp_reinit_t >= 5000UL) {
          bmp_reinit_t = now;
          ImuFast_LockSpi();
          uint8_t bid = BMP585_Init(&hspi3);
          ImuFast_UnlockSpi();
          if (bid == 0x50 || bid == 0x51) {
            mod.bmp585 = 1; bmp_err_cnt = 0;
            cdc_write("BMP: REINIT OK\r\n");
          }
        }
      }
    }

    /* LoRa 已移除（imu 專案不使用 LoRa，避免阻塞 IMU 記錄）*/

    /* ─── ImuFast buffer 排空 → 寫入序號 CSV（每 64 筆一次）─── */
    if (imu_fil_open) {
      /* ① 優先把上次失敗的那批（retry buffer）補寫回去
       *    節流 100ms：避免卡片半死時每圈主迴圈關檔+開檔捶打 SD */
      static uint32_t csv_retry_wr_t = 0;
      if (csv_retry_len > 0 && now - csv_retry_wr_t >= 100UL) {
        csv_retry_wr_t = now;
        UINT bw2 = 0;
        FRESULT fr2 = f_write(&imu_fil, csv_retry_buf, (UINT)csv_retry_len, &bw2);
        if (fr2 == FR_OK && bw2 == (UINT)csv_retry_len) {
          csv_retry_len = 0;   /* 補寫成功，清除 retry */
        } else {
          /* retry 還是失敗 → 重開檔，本次跳過排新資料（保留 ring buffer）*/
          csv_err_cnt++;
          f_close(&imu_fil);
          if (f_open(&imu_fil, csv_name, FA_WRITE | FA_OPEN_APPEND) != FR_OK)
            imu_fil_open = 0;
        }
      }
      /* ② 只有在 retry 清空後，才排新資料寫入 CSV */
      if (imu_fil_open && csv_retry_len == 0 && ImuFast_Available() >= 64) {
        static char csv_buf[8192];
        int  csv_len = 0;
        ImuSample_t s;
        for (int _i = 0; _i < 64; _i++) {
          ImuFast_Pop(&s);
          float a_x = s.ax / 2048.0f;
          float a_y = s.ay / 2048.0f;
          float a_z = s.az / 2048.0f;
          float g_tot = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z);
          if (csv_len > (int)sizeof(csv_buf) - 128) break;
          csv_len += snprintf(csv_buf + csv_len, (int)sizeof(csv_buf) - csv_len,
            "%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.2f,%.3f,%.3f\n",
            (unsigned long)s.ts_us,
            s.gx / 16.384f,  s.gy / 16.384f,  s.gz / 16.384f,
            a_x, a_y, a_z, g_tot,
            s.baro_m, s.vz_ms, s.hz_m);
        }
        UINT bw = 0;
        FRESULT fr_csv = f_write(&imu_fil, csv_buf, (UINT)csv_len, &bw);
        static uint16_t imu_sync_cnt = 0;
        if (fr_csv == FR_OK && bw == (UINT)csv_len) {
          if (++imu_sync_cnt >= 16) {
            imu_sync_cnt = 0;
            if (f_sync(&imu_fil) != FR_OK) fr_csv = FR_DISK_ERR;
          }
        } else if (fr_csv == FR_OK) {
          fr_csv = FR_DISK_ERR;
        }
        if (fr_csv != FR_OK) {
          /* 把這批存進 retry buffer，下次迴圈優先補寫，避免資料丟失 */
          memcpy(csv_retry_buf, csv_buf, (size_t)csv_len);
          csv_retry_len = csv_len;
          csv_err_cnt++;
          f_close(&imu_fil);
          if (f_open(&imu_fil, csv_name, FA_WRITE | FA_OPEN_APPEND) != FR_OK)
            imu_fil_open = 0;
        }
      }
    }

    /* ─── CSV 斷線自動重啟（每 5s 嘗試一次）─── */
    if (!imu_fil_open && sd_init_done) {
      /* 防滿排空已移至上方無條件區塊（涵蓋 imu_fil_open=1 的 retry 死局）*/
      static uint32_t csv_retry_t = 0;
      if (now - csv_retry_t >= 5000UL) {
        csv_retry_t = now;
        if (f_open(&imu_fil, csv_name, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
          imu_fil_open  = 1;
          csv_retry_len = 0;
        }
      }
    }

    /* ─── UART/CDC 輸出每 500ms ─── */
    if (now - t_out >= 500) {
      t_out = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

      static char b[256]; int n;
      n = snprintf(b, sizeof(b),
        "T=%-7lu Ax=%+.3f Ay=%+.3f Az=%+.3f "
        "Gx=%+7.2f Gy=%+7.2f Gz=%+7.2f "
        "P=%7.2f RelH=%6.1f KfH=%6.1f KfV=%+6.2f "
        "G=%.2f Waz=%+.3f qn=%.2f att=%d LD=%d%s MOD=%d%d%d SD=%lu IMU500=%lu(ms%lu,e%u)\r\n",
        (unsigned long)now, ax, ay, az,
        gx/0.017453293f, gy/0.017453293f, gz/0.017453293f,
        press, rel_alt, kf2_h, kf2_v,
        total_g, (float)imu_fast_world_az, ImuFast_GetQn2(),
        (int)imu_fast_att_ok, (int)land_state,
        imu_ok?"":" !IMU",
        mod.bmp585, mod.imu, mod.sdcard,
        (unsigned long)sd_write_cnt,
        (unsigned long)ImuFast_Available(),
        (unsigned long)ImuFast_GetMissed(),
        (unsigned)csv_err_cnt);

      if (n > 0 && n < (int)sizeof(b)) {
        /* SD 寫入 */
        if (mod.sdcard && logger_is_ready()) {
          UINT bw_sd = 0;
          FRESULT fw = f_write(&file, b, (UINT)n, &bw_sd);
          if (fw == FR_OK && bw_sd == (UINT)n) {
            /* f_sync 每 4 次（2s）一次：斷電最多丟 2s 資料。
             * 原為 60 次（30s）→ 短時間測試後直接斷電會整個 log.txt 空白 */
            static uint8_t log_sync_cnt = 0;
            if (++log_sync_cnt >= 4) { log_sync_cnt = 0; f_sync(&file); }
            sd_write_cnt++;
          } else {
            static uint8_t sd_err_cnt = 0;
            if (sd_err_cnt < 5) {
              /* 法醫輸出：fw=9 (FR_INVALID_OBJECT) 的判定要素全印出來
               *   fs : file.obj.fs 指標（0=被關閉；非 &SDFatFS=被改寫）
               *   fid/vid : FIL 記錄的掛載 id vs 卷的目前 id（不等=發生過重掛載）
               *   ty : SDFatFS.fs_type（0=FATFS 物件被清掉/損毀）
               *   err : FIL 內部鎖存的 I/O 錯誤碼                     */
              char e[112];
              snprintf(e,sizeof(e),
                "SD_ERR fw=%d bw=%u n=%d | fs=%08lX(&v=%08lX) fid=%u vid=%u ty=%d err=%d\r\n",
                (int)fw,(unsigned)bw_sd,n,
                (unsigned long)(uint32_t)file.obj.fs,
                (unsigned long)(uint32_t)&SDFatFS,
                (unsigned)file.obj.id,(unsigned)SDFatFS.id,
                (int)SDFatFS.fs_type,(int)file.err);
              cdc_write(e); sd_err_cnt++;
            }
            /* ── log.txt 自動復活（每 5s 重試一次，分級恢復）────────
             * 第 1 次：溫和 — logger_init 重新掛載+開檔（治 FIL 失效）。
             * 第 2 次起：核彈 — HAL_SD_DeInit 後再掛載。卡片若因電壓凹陷
             *   /震動瞬斷自行重置，MCU 端 hsd.State 仍是 READY，
             *   SD_initialize 的防 GC 守衛會跳過 BSP_SD_Init → 卡片
             *   永遠不會被重新初始化（實測症狀：三張卡都「無聲死亡」）。
             *   DeInit 讓 hsd.State 回 RESET，守衛放行完整重新初始化。
             * 注意：重掛載會使 imu_fil 失效 → CSV 下一批寫入失敗一次，
             *   由 CSV retry buffer + 5s 重開機制接手，資料不丟。      */
            static uint32_t log_retry_t = 0;
            static uint8_t  log_retry_fails = 0;
            if (now - log_retry_t >= 5000UL) {
              log_retry_t = now;
              if (log_retry_fails >= 1) {
                f_close(&imu_fil); imu_fil_open = 0;
                HAL_SD_DeInit(&hsd);   /* State→RESET：下次 disk_initialize 全重來 */
                cdc_write("SD: NUCLEAR RESET\r\n");
              }
              logger_init();
              if (logger_is_ready()) log_retry_fails = 0;
              else if (log_retry_fails < 200) log_retry_fails++;
            }
          }
        }
        /* UART/CDC */
        if (!cmd_is_typing()) { cdc_write(b); }  /* UART1=9600baud 太慢，imu專案只用 CDC */
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
  *
  * 診斷版：改用 HSI（內部 16 MHz RC）取代 HSE 外部晶振。
  *   HSI=16MHz / PLLM=16 → VCO_in=1MHz
  *   PLLN=336 → VCO=336MHz
  *   PLLP=DIV4 → SYSCLK=84MHz
  *   PLLQ=7   → USB=48MHz
  * 不需任何外部晶振，確認 HSE 是否為故障根因。
  * 若 USB CDC 出現 → HSE 晶振問題（缺件/頻率不符），改回 HSE 時需修正 PLLM。
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;   /* 16MHz / 16 = 1MHz VCO_in  */
  RCC_OscInitStruct.PLL.PLLN = 336;  /* VCO = 336MHz               */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* SYSCLK = 84MHz   */
  RCC_OscInitStruct.PLL.PLLQ = 7;   /* USB = 48MHz                 */
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
  /* 不呼叫 __disable_irq() — 保持 USB 中斷存活，COM4 才不會消失
   * 先確保 GPIOA 時鐘已開（SystemClock_Config 失敗時 MX_GPIO_Init 尚未執行）
   * LED 快閃示警（PA2 = LED，Low 點亮，High 熄滅）                           */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  {
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_2;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
  }
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
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
