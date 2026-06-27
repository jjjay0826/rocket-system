/*
 * imu_fast.c - 500Hz IMU 高速記錄模組（v2，Blocking SPI）
 *
 * 本版採用 Blocking SPI（與原始版相同），但新增：
 *   1. Mahony 互補濾波器移入 ISR（500Hz）
 *   2. 自適應 Kp/Ki（依 acc_norm 偏離 1g 程度）
 *   3. Ring buffer 1024 → 2048 筆
 *
 * 放棄 DMA 模式（HAL blocking + DMA 混用在中斷上下文有衝突風險）。
 * ISR 總時間：SPI ~50µs + Mahony ~2µs = ~52µs / 2000µs = 2.6%，安全。
 */

#include "imu_fast.h"
#include "main.h"
#include "spi.h"
#include <string.h>
#include <math.h>

extern SPI_HandleTypeDef hspi3;

#define IMU_CS_PORT   GPIOB
#define IMU_CS_PIN    GPIO_PIN_1
#define IMU_CS_LOW()  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET)
#define IMU_CS_HIGH() HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET)

#define LSM6_OUTX_L_G  0x22U
#define LSM6_READ_FLAG 0x80U

#define BUF_MASK   (IMUFAST_BUF_SIZE - 1U)
#define DEG2RAD    0.017453293f

/* ── Ring buffer ── */
static volatile ImuSample_t ring_buf[IMUFAST_BUF_SIZE];
static volatile uint16_t    head     = 0;
static volatile uint16_t    tail     = 0;
static volatile uint32_t    missed   = 0;
static volatile uint8_t     spi_lock = 0;
static volatile uint32_t    ts_us    = 0;

/* ── 共享快照 ── */
volatile float   imu_fast_baro_m   = 0.0f;
volatile float   imu_fast_vz_ms    = 0.0f;
volatile float   imu_fast_hz_m     = 0.0f;
volatile float   imu_fast_world_az = 0.0f;
volatile uint8_t imu_fast_att_ok   = 0;

/* ── Mahony 姿態狀態 ── */
static float mah_q0 = 1.0f, mah_q1 = 0.0f,
             mah_q2 = 0.0f, mah_q3 = 0.0f;
static float mah_ix = 0.0f, mah_iy = 0.0f, mah_iz = 0.0f;

/* ── 感測器偏移 ── */
static float mah_gbx  = 0.0f, mah_gby  = 0.0f, mah_gbz  = 0.0f;
static float mah_ax_b = 0.0f, mah_ay_b = 0.0f;

/* ══════════════════════════════════════════════════════
 * mahony_isr
 * ══════════════════════════════════════════════════════ */
static void mahony_isr(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float kp, float ki)
{
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n >= 0.001f) {
        n = 1.0f / n;
        ax *= n; ay *= n; az *= n;

        float vx = 2.0f*(mah_q1*mah_q3 - mah_q0*mah_q2);
        float vy = 2.0f*(mah_q0*mah_q1 + mah_q2*mah_q3);
        float vz = mah_q0*mah_q0 - mah_q1*mah_q1
                 - mah_q2*mah_q2 + mah_q3*mah_q3;

        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        mah_ix += ex * ki * 0.002f;
        mah_iy += ey * ki * 0.002f;
        mah_iz += ez * ki * 0.002f;

        gx += kp*ex + mah_ix;
        gy += kp*ey + mah_iy;
        gz += kp*ez + mah_iz;
    }

    gx *= 0.001f; gy *= 0.001f; gz *= 0.001f;   /* 0.5 * dt = 0.5 * 0.002 */
    {
        float qa = mah_q0, qb = mah_q1, qc = mah_q2;
        mah_q0 += (-qb*gx - qc*gy - mah_q3*gz);
        mah_q1 += ( qa*gx + qc*gz - mah_q3*gy);
        mah_q2 += ( qa*gy - qb*gz + mah_q3*gx);
        mah_q3 += ( qa*gz + qb*gy - qc*gx);
    }

    n = 1.0f / sqrtf(mah_q0*mah_q0 + mah_q1*mah_q1
                   + mah_q2*mah_q2 + mah_q3*mah_q3);
    mah_q0 *= n; mah_q1 *= n; mah_q2 *= n; mah_q3 *= n;
}

/* ══════════════════════════════════════════════════════
 * world_az_isr
 * ══════════════════════════════════════════════════════ */
static float world_az_isr(float ax, float ay, float az)
{
    return 2.0f*(mah_q1*mah_q3 - mah_q0*mah_q2)*ax
          +2.0f*(mah_q2*mah_q3 + mah_q0*mah_q1)*ay
          +(mah_q0*mah_q0 - mah_q1*mah_q1
           -mah_q2*mah_q2 + mah_q3*mah_q3)*az;
}

/* ══════════════════════════════════════════════════════
 * ImuFast_Init
 * ══════════════════════════════════════════════════════ */
void ImuFast_Init(void)
{
    head = 0; tail = 0; missed = 0; spi_lock = 0; ts_us = 0;
    imu_fast_baro_m   = 0.0f;
    imu_fast_vz_ms    = 0.0f;
    imu_fast_hz_m     = 0.0f;
    imu_fast_world_az = 0.0f;
    imu_fast_att_ok   = 0;
}

void ImuFast_SetAttitude(float q0, float q1, float q2, float q3)
{
    mah_q0 = q0; mah_q1 = q1;
    mah_q2 = q2; mah_q3 = q3;
    mah_ix = 0.0f; mah_iy = 0.0f; mah_iz = 0.0f;
}

void ImuFast_SetBias(float gbx_rps, float gby_rps, float gbz_rps,
                     float ax_b_g,  float ay_b_g)
{
    mah_gbx  = gbx_rps; mah_gby  = gby_rps; mah_gbz  = gbz_rps;
    mah_ax_b = ax_b_g;  mah_ay_b = ay_b_g;
}

/* ══════════════════════════════════════════════════════
 * ImuFast_Callback  —  TIM2 ISR，每 2ms
 * Blocking SPI：與原始版相同架構，加入 Mahony 計算
 * ══════════════════════════════════════════════════════ */
void ImuFast_Callback(void)
{
    ts_us += 2000U;

    if (spi_lock) { missed++; return; }

    uint16_t next = (uint16_t)((head + 1U) & BUF_MASK);
    if (next == tail) { missed++; return; }

    /* ── SPI Burst 讀取（Blocking，與原版相同）── */
    uint8_t tx[13] = { LSM6_OUTX_L_G | LSM6_READ_FLAG,
                       0,0,0,0,0,0,0,0,0,0,0,0 };
    uint8_t rx[13] = {0};
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 13, 2);
    IMU_CS_HIGH();

    /* ── 解碼 ── */
    ImuSample_t *s = (ImuSample_t *)&ring_buf[head];
    s->ts_us = ts_us;
    s->gx = (int16_t)(((uint16_t)rx[2]  << 8) | rx[1]);
    s->gy = (int16_t)(((uint16_t)rx[4]  << 8) | rx[3]);
    s->gz = (int16_t)(((uint16_t)rx[6]  << 8) | rx[5]);
    s->ax = (int16_t)(((uint16_t)rx[8]  << 8) | rx[7]);
    s->ay = (int16_t)(((uint16_t)rx[10] << 8) | rx[9]);
    s->az = (int16_t)(((uint16_t)rx[12] << 8) | rx[11]);
    s->baro_m = imu_fast_baro_m;
    s->vz_ms  = imu_fast_vz_ms;
    s->hz_m   = imu_fast_hz_m;

    /* ── 物理量換算（含偏移）── */
    float ax_g = (float)s->ax / 2048.0f - mah_ax_b;
    float ay_g = (float)s->ay / 2048.0f - mah_ay_b;
    float az_g = (float)s->az / 2048.0f;
    float gx_r = (float)s->gx / 16.384f * DEG2RAD - mah_gbx;
    float gy_r = (float)s->gy / 16.384f * DEG2RAD - mah_gby;
    float gz_r = (float)s->gz / 16.384f * DEG2RAD - mah_gbz;

    /* ── 自適應 Kp/Ki ── */
    float acc_n = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    float dev   = fabsf(acc_n - 1.0f);
    float kp, ki;
    if      (dev < 0.10f) { kp = 1.0f;  ki = 0.01f;  }
    else if (dev < 0.30f) { kp = 0.5f;  ki = 0.005f; }
    else                  { kp = 0.0f;  ki = 0.0f;   }

    /* ── Mahony（500Hz）── */
    mahony_isr(ax_g, ay_g, az_g, gx_r, gy_r, gz_r, kp, ki);

    /* ── 輸出 ── */
    imu_fast_world_az = world_az_isr(ax_g, ay_g, az_g);
    imu_fast_att_ok   = (dev < 0.30f) ? 1U : 0U;

    head = next;
}

/* ══════════════════════════════════════════════════════
 * 查詢 / 取出 API
 * ══════════════════════════════════════════════════════ */
uint16_t ImuFast_Available(void)
{
    return (uint16_t)((head - tail) & BUF_MASK);
}

int ImuFast_Pop(ImuSample_t *s)
{
    if (tail == head) return -1;
    *s   = (ImuSample_t)ring_buf[tail];
    tail = (uint16_t)((tail + 1U) & BUF_MASK);
    return 0;
}

int ImuFast_GetLatest(ImuSample_t *s)
{
    if (head == tail) return -1;
    uint16_t last = (uint16_t)((head - 1U) & BUF_MASK);
    *s = (ImuSample_t)ring_buf[last];
    return 0;
}

/* LockSpi：簡單設旗標即可，無 DMA 無需等待 */
void ImuFast_LockSpi(void)   { spi_lock = 1; }
void ImuFast_UnlockSpi(void) { spi_lock = 0; }

uint32_t ImuFast_GetMissed(void) { return missed; }

/* 目前 ISR 時間戳（µs，自 ImuFast_Init 起算）：供 log/CSV 時間軸對齊 */
uint32_t ImuFast_GetTs(void) { return ts_us; }

/* ISR 四元數範數平方（診斷用：正常應恆 ≈1.00，偏離代表姿態發散）*/
float ImuFast_GetQn2(void)
{
    return mah_q0*mah_q0 + mah_q1*mah_q1
         + mah_q2*mah_q2 + mah_q3*mah_q3;
}
