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
#define DEPLOY_PORT     GPIOA
#define DEPLOY_PIN      GPIO_PIN_1
#define LAUNCH_AZ_G  1.3f       /* 離架偵測合加速度閾值 (g) */
#define DEPLOY_ALT_M    850.0f    /* 開傘高度（相對起飛點，m） */
#define DEPLOY_T1_MS    21000UL   /* 主窗口開始 ms */
#define DEPLOY_T2_MS    23000UL   /* 主窗口結束 ms */
#define DEPLOY_TB_MS    25000UL   /* 備援觸發 ms */
#define DEPLOY_PULSE_MS 2000UL    /* GPIO HIGH 持續時間 ms */
#define IMU_ARM_G       1.5f      /* IMU 積分啟動閾值 (g)，測試用可調低 */
#define MAH_2KP         1.0f      /* Mahony 比例增益 × 2 */
#define MAH_2KI         0.01f     /* Mahony 積分增益 × 2 */

/* ---- 氣壓卡爾曼濾波器參數 ---- */
#define KF_Q  0.5f    /* 過程雜訊方差 (hPa²)：越大追蹤越快 */
#define KF_R  0.01f   /* 量測雜訊方差 (hPa²)：BMP585 RMS≈0.03hPa */
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

/* ---- 飛行狀態機 ---- */
typedef enum {
    FLIGHT_IDLE,       /* 地面靜置，等待離架 */
    FLIGHT_LAUNCHED,   /* 離架後，監視高度與時間 */
    FLIGHT_DEPLOYING,  /* 開傘訊號輸出中（2 秒） */
    FLIGHT_DEPLOYED    /* 開傘完成 */
} FlightState_t;

static FlightState_t flight_state   = FLIGHT_IDLE;
static uint32_t      launch_time_ms = 0;   /* HAL_GetTick() at launch */
static uint32_t      deploy_time_ms = 0;   /* HAL_GetTick() at deploy */
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

  if (who != 0x6C) return -10;

  lsm6_write_reg(0x12, 0x01); // SW_RESET
  HAL_Delay(50);

  lsm6_write_reg(0x12, 0x44); // BDU + IF_INC
  lsm6_write_reg(0x10, 0x40); // XL 104Hz, +/-2g
  lsm6_write_reg(0x11, 0x4C); // G  104Hz, 2000dps
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
static float  az_bias_g   = 0.0f;  /* 靜止時垂直加速度偏差 (g) */
static float  imu_vz      = 0.0f;  /* 垂直速度 (m/s) */
static float  imu_hz      = 0.0f;  /* IMU 估算高度 (m) */
static uint8_t imu_armed  = 0;     /* g > 閾值後鎖定，持續積分 */
static uint32_t g2_count  = 0;     /* 連續 >LAUNCH_AZ_G 的 10ms 計數 */
static uint32_t sd_write_cnt = 0;  /* SD 成功寫入次數 */
static uint8_t  imu_ok    = 0;     /* IMU 初始化是否成功 */

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

  /* 開傘輸出腳預設低 */
  HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_RESET);

  /* 等 USB CDC 完整枚舉（1000ms 確保 COM port 出現後才開始用 CDC）*/
  HAL_Delay(1000);

  /* IMU 初始化（快速，~70ms）*/
  imu_ok = (lsm6_init() == 0) ? 1 : 0;

  /* BMP585 初始化 */
  { uint8_t bmp_id = BMP585_Init(&hspi3);
    char b[48];
    snprintf(b,sizeof(b),"BMP: %s chip_id=0x%02X\r\n",bmp_id?"OK":"FAIL",bmp_id);
    uart1_write(b); cdc_write(b); }

  /* 取地面氣壓基準（快速單次讀取，不做長時間平均）*/
  { float p = BMP585_ReadPressure();
    if(p>800.f&&p<1100.f){ ref_press=p; kf_p_est=p; }
    char b[48];
    snprintf(b,sizeof(b),"REF_PRESS=%.2f hPa\r\n",ref_press);
    uart1_write(b); cdc_write(b); }

  /* GNSS 初始化（PA10=USART1_RX @ 9600 baud）*/
  GNSS_Init(&huart1);

  /* ── 阻塞式 Az 偏差校準（正確公式）──────────────────────────────
   * 步驟 1：讓 Mahony 收斂 1 秒（100 次 × 10ms），四元數趨於穩定
   * 步驟 2：再取 100 次 world_az() 平均，計算偏差
   * 正確公式：az_bias_g = mean(world_az_stationary) - 1.0f
   *   → 靜止時 lin_az = (world_az - az_bias_g - 1.0) * g = 0
   * 【注意】舊公式錯誤地把 mean(raw_az)≈1.0 存入，
   *         導致 lin_az 永遠 ≈ -9.8 m/s²，imu_hz 直線下沉。 */
  if (imu_ok) {
    /* --- 收斂階段（1 秒）--- */
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/16384.f, a_y=ray/16384.f, a_z=raz/16384.f;
        float g_x=rgx/16.384f*0.017453293f;
        float g_y=rgy/16.384f*0.017453293f;
        float g_z=rgz/16.384f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
      }
    }
    /* --- 校準階段（1 秒，取 world_az 平均）--- */
    float bias_sum2 = 0.f; int bias_cnt2 = 0;
    for (int ci = 0; ci < 100; ci++) {
      HAL_Delay(10);
      int16_t rt,rgx,rgy,rgz,rax,ray,raz;
      if (lsm6_read_raw_LSM6DSOTR(&rt,&rgx,&rgy,&rgz,&rax,&ray,&raz)==0) {
        float a_x=rax/16384.f, a_y=ray/16384.f, a_z=raz/16384.f;
        float g_x=rgx/16.384f*0.017453293f;
        float g_y=rgy/16.384f*0.017453293f;
        float g_z=rgz/16.384f*0.017453293f;
        mahony_update(a_x,a_y,a_z,g_x,g_y,g_z,0.01f);
        bias_sum2 += world_az(a_x,a_y,a_z);
        bias_cnt2++;
      }
    }
    if (bias_cnt2 > 0) {
      az_bias_g = (bias_sum2 / (float)bias_cnt2) - 1.0f; /* 正確：偏差量 ≈ 0 */
      char b[64];
      snprintf(b,sizeof(b),"AZ_BIAS=%.5f g (n=%d)\r\n",az_bias_g,bias_cnt2);
      uart1_write(b); cdc_write(b);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_imu=0,t_bmp=0,t_out=0,t_sd_init=0;
  uint8_t  sd_init_done=0;
  float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;
  float press=ref_press, alt=0, total_g=1;
  float tc=25.f; (void)tc; /* 氣壓版：溫度不輸出，抑制 warning */

  uint8_t cf_init  = 0;   /* 互補濾波器初始化旗標 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 指令系統 */
    cmd_execute_pending();
    cmd_flush_echo();

    /* SD 卡延遲初始化：啟動 3 秒後 */
    if (!sd_init_done && (now - t_sd_init) >= 3000) {
      sd_init_done = 1;
      logger_init();
    }

    /* ---- IMU 每 10ms ---- */
    if (now - t_imu >= 10) {
      float dt = (float)(now - t_imu) * 0.001f;
      if (dt > 0.05f) dt = 0.05f;   /* 限制首次大 dt，防止跳躍 */
      t_imu = now;

      int16_t raw_t,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az;
      if (lsm6_read_raw_LSM6DSOTR(&raw_t,&raw_gx,&raw_gy,&raw_gz,
                                   &raw_ax,&raw_ay,&raw_az) == 0) {
        ax = (float)raw_ax / 16384.f;
        ay = (float)raw_ay / 16384.f;
        az = (float)raw_az / 16384.f;
        float deg2rad = 0.017453293f;
        gx = (float)raw_gx / 16.384f * deg2rad;
        gy = (float)raw_gy / 16.384f * deg2rad;
        gz = (float)raw_gz / 16.384f * deg2rad;
        tc = (float)raw_t / 256.f + 25.f;
        total_g = sqrtf(ax*ax+ay*ay+az*az);

        mahony_update(ax,ay,az,gx,gy,gz,dt);

        /* 發射偵測：total_g > 閾值持續 200ms */
        if (total_g >= LAUNCH_AZ_G) {
          g2_count++;
          if (!imu_armed && g2_count >= 20) {
            imu_armed      = 1;
            launch_time_ms = now;
            flight_state   = FLIGHT_LAUNCHED;
            imu_hz = 0.0f; imu_vz = 0.0f; /* 以起飛點為零點重置 */
          }
        } else {
          if (!imu_armed) g2_count = 0;
        }

        /* ── 積分垂直速度/高度 ──────────────────────────────────────
         * lin_az = (world_az - az_bias_g - 1.0) * g
         *   az_bias_g ≈ 0（校準後的小量感測器偏差）
         *   world_az stationary ≈ 1.0 → lin_az ≈ 0（正確）
         *
         * 靜止阻尼（分級）：
         *   |g-1| < 8%  → 強衰減 τ≈0.15s（幾乎靜止）
         *   |g-1| < 20% → 弱衰減 τ≈1.0s（微晃動）
         *   |g-1| ≥ 20% → 不衰減（飛行中/自由落體）
         * ──────────────────────────────────────────────────────── */
        if (imu_armed) {
          float az_w   = world_az(ax,ay,az) - az_bias_g;
          float lin_az = (az_w - 1.0f) * 9.80665f;
          imu_vz += lin_az * dt;
          imu_hz += imu_vz * dt;

          float g_dev = fabsf(total_g - 1.0f);
          if      (g_dev < 0.08f) imu_vz *= (1.0f - dt / 0.15f); /* 強衰減 */
          else if (g_dev < 0.20f) imu_vz *= (1.0f - dt / 1.0f);  /* 弱衰減 */
          /* else: 飛行中，不衰減 */
        }
      }
    }

    /* ---- BMP585 每 200ms + 互補濾波器修正 ---- */
    if (now - t_bmp >= 200) {
      float dt_baro = (float)(now - t_bmp) * 0.001f;
      if (dt_baro > 0.5f) dt_baro = 0.5f;
      t_bmp = now;
      float p_raw = BMP585_ReadPressure();
      if (p_raw > 800.f && p_raw < 1100.f) {
        press = kf_update(p_raw);
        alt   = 44330.f*(1.f-powf(press/1013.25f,0.1903f));
        rel_alt = 44330.f*(1.f-powf(press/ref_press,0.1903f));

        /* ── 互補濾波器（觀測器形式）──────────────────────────────
         * IMU：短期準、長期漂移（積分誤差累積）
         * 氣壓：長期準、短期雜訊；高速時受氣動壓影響
         *
         * 誤差 = baro_alt - imu_hz
         * 位置修正：imu_hz += Kp * err
         * 速度修正：imu_vz += Ki * err / dt
         *
         * 增益依 G 值切換：
         *   高G (>2g, 推進段) → 氣壓受動壓影響大，少修正
         *   低G (<2g, 滑翔/降落) → 氣壓可信，多修正
         * ──────────────────────────────────────────────────────── */
        if (!cf_init) { cf_init = 1; }

        if (!imu_armed) {
          /* 未發射：高度歸零，速度清除 */
          imu_hz = 0.0f; imu_vz = 0.0f;
        } else {
          float err  = rel_alt - imu_hz;
          float kp   = (total_g > 2.0f) ? 0.02f : 0.25f;  /* 位置增益 */
          float ki   = (total_g > 2.0f) ? 0.01f : 0.10f;  /* 速度增益 */
          imu_hz += kp * err;
          imu_vz += ki * err / dt_baro;
        }
      }
    }

    /* ---- 開傘邏輯 ---- */
    if (flight_state == FLIGHT_LAUNCHED) {
      uint32_t t_fly = now - launch_time_ms;
      if (rel_alt >= DEPLOY_ALT_M || t_fly >= DEPLOY_T1_MS) {
        flight_state = FLIGHT_DEPLOYING;
        deploy_time_ms = now;
        HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_SET);
      }
    }
    if (flight_state == FLIGHT_DEPLOYING && (now - deploy_time_ms) >= 2000) {
      HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_RESET);
      flight_state = FLIGHT_DEPLOYED;
    }

    /* ---- UART/CDC 輸出每 500ms ---- */
    if (now - t_out >= 500) {
      t_out = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

      (void)GNSS_GetData(); /* 氣壓版：不使用 GNSS 資料 */

      /* ── 氣壓 project 閹割版：只輸出 BMP585 高度 ── */
      char b[320]; int n;
      n=snprintf(b,sizeof(b),
        "T=%-7lu P=%7.2f H=%6.1f RelH=%6.1f\r\n",
        (unsigned long)now,
        press, alt, rel_alt);

      if(n>0&&n<(int)sizeof(b)){
        /* SD 寫入（不受打字暫停影響）*/
        if(logger_is_ready()){
          UINT bw_sd=0;
          FRESULT fw=f_write(&file,b,(UINT)n,&bw_sd);
          if(fw==FR_OK&&bw_sd==(UINT)n){ f_sync(&file); sd_write_cnt++; }
          else{
            static uint8_t sd_err_cnt=0;
            if(sd_err_cnt<5){
              char e[64];
              snprintf(e,sizeof(e),"SD_ERR fw=%d bw=%u n=%d\r\n",(int)fw,(unsigned)bw_sd,n);
              uart1_write(e); cdc_write(e); sd_err_cnt++;
            }
          }
        }
        /* UART/CDC（打字中暫停）*/
        if(!cmd_is_typing()){ uart1_write(b); cdc_write(b); }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * 先嘗試 HSE（8 MHz 晶振）→ PLL → 84 MHz / USB 48 MHz
  * 若 HSE 啟動失敗（晶振未裝或損壞），自動 fallback 到 HSI（16 MHz）
  * 相同 PLL 輸出：SYSCLK=84 MHz, PLLQ=48 MHz（USB 仍能正常運作）
  *
  * 重要：此函數刻意不使用 CubeMX USER CODE 區塊，
  * 以避免 CubeMX 重生成時抹去 HSI fallback。
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* ---- 直接使用 HSI 16 MHz（自製 PCB 無 HSE 晶振）----
   * HSI = 16 MHz → /16 → 1 MHz ref → ×336 = 336 MHz VCO
   * PLLP /4 = 84 MHz SYSCLK，PLLQ /7 = 48 MHz USB clock           */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 336;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ            = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- Bus clocks ---- */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
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
   * 改用 LED 快閃示警（PA2 = LED，Low 點亮，High 熄滅）          */
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
