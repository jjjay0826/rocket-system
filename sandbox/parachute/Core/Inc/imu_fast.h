/*
 * imu_fast.h - 500Hz IMU 高速記錄模組（v2）
 *
 * 變更：
 *   - Mahony 互補濾波器移入 ISR（500Hz），主迴圈不再呼叫 mahony_update()
 *   - 自適應 Kp/Ki：依 acc_norm 偏離 1g 動態調整
 *   - 新增 imu_fast_world_az / imu_fast_att_ok 供主迴圈讀取
 *   - Ring buffer 1024 → 2048 筆（56 KB，~4 秒緩衝）
 */

#ifndef INC_IMU_FAST_H_
#define INC_IMU_FAST_H_

#include <stdint.h>

/* ──────────────────────────────────────────
 * 每筆樣本結構（28 bytes, packed）
 * ────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint32_t ts_us;        /* 時間戳記（µs，自 ImuFast_Init 起算）*/
    int16_t  gx, gy, gz;  /* 角速度原始值（LSB, ÷14.286 → dps；±2000dps=70mdps/LSB）*/
    int16_t  ax, ay, az;  /* 加速度原始值（LSB, ÷2048  → g）    */
    float    baro_m;       /* 最新氣壓相對高度（m）               */
    float    vz_ms;        /* KF2 垂直速度（m/s）                 */
    float    hz_m;         /* KF2 高度（m）                       */
} ImuSample_t;             /* 共 28 bytes */

/* ──────────────────────────────────────────
 * Ring buffer 大小（必須是 2 的冪次方）
 * 2048 × 28 B = 56 KB，約 4 s @ 500 Hz
 * ────────────────────────────────────────── */
#define IMUFAST_BUF_SIZE   2048U   /* 2048×28B=56KB ≈ 4s 緩衝（RAM 總用量 ~75KB/128KB）*/

/* ──────────────────────────────────────────
 * ★2026-08-07：Mahony 的 |a| 閘（單位 g）
 *
 * 加速度計量到的方向，只有在合加速度接近 1 g 時【才是】重力方向。
 * 舊碼只擋除以零（n >= 0.001f），等於把任何外力方向都當成「下」。
 *
 * 2026-08-01 火箭實飛的後果（firmware-rocket 同款問題，已修）：
 *   · 滑行段 |a|≈0.06 g（純阻力雜訊）被當成重力 → 頂點垂直速度估計
 *     ch1 −2.42、ch2 +11.65 m/s，真值是 0
 *   · 開傘衝擊 6 g 被當成重力 → 速度估計衝到 +30.86 m/s，
 *     而實際正在以 30 m/s 下降
 *
 * 對投放測試來說這更要緊：這支韌體的存在目的就是【量準終端速度】，
 * 而開傘衝擊正是它一定會遇到、也一定會污染姿態的事件。
 *
 * 兩處共用：imu_fast.c 的 mahony_isr()（500 Hz，投放中真正跑的）
 *          main.c 的 mahony_update()（開機收斂/校準，靜止 1 g）
 * ────────────────────────────────────────── */
#define MAH_ACC_GATE_LO   0.85f
#define MAH_ACC_GATE_HI   1.15f

/* ──────────────────────────────────────────
 * 共享快照變數
 *   主迴圈 → ISR：baro_m, vz_ms, hz_m
 *   ISR    → 主迴圈：world_az, att_ok
 * ────────────────────────────────────────── */
extern volatile float   imu_fast_baro_m;    /* 氣壓相對高度 (m)                        */
extern volatile float   imu_fast_vz_ms;     /* KF2 速度 (m/s)                          */
extern volatile float   imu_fast_hz_m;      /* KF2 高度 (m)                            */
extern volatile float   imu_fast_world_az;  /* Mahony 輸出垂直加速度 (g，含 1g 重力)   */
extern volatile uint8_t imu_fast_att_ok;    /* 姿態可靠旗標（1=可信，0=高動態/不可信）*/

/* ──────────────────────────────────────────
 * API
 * ────────────────────────────────────────── */

/* 初始化：在 MX_TIM2_Init 之後、HAL_TIM_Base_Start_IT 之前呼叫 */
void ImuFast_Init(void);

/* 傳入校準結束後的初始四元數（必須在 HAL_TIM_Base_Start_IT 之前）*/
void ImuFast_SetAttitude(float q0, float q1, float q2, float q3);

/* 傳入陀螺儀零率偏移（rad/s）和加速度水平偏移（g）             */
void ImuFast_SetBias(float gbx_rps, float gby_rps, float gbz_rps,
                     float ax_b_g,  float ay_b_g);

/* TIM2 中斷 callback */
void ImuFast_Callback(void);

/* Ring buffer 查詢 */
uint16_t ImuFast_Available(void);
int      ImuFast_Pop(ImuSample_t *s);
int      ImuFast_GetLatest(ImuSample_t *s);

/* SPI2 佔用控制（IMU ISR 與 BMP585 主迴圈共用 SPI2） */
void ImuFast_LockSpi(void);
void ImuFast_UnlockSpi(void);

/* 遺失樣本查詢 */
uint32_t ImuFast_GetMissed(void);

/* 目前 ISR 時間戳（µs）：供 log/CSV 時間軸對齊 */
uint32_t ImuFast_GetTs(void);

/* ISR 四元數範數平方（診斷用，正常 ≈1.00）*/
float ImuFast_GetQn2(void);

#endif /* INC_IMU_FAST_H_ */
