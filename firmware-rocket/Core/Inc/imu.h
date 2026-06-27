#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h" // 必須包含這個，編譯器才認識 I2C_HandleTypeDef

#define LSM6DSOX_ADDR  (0x6A << 1)

// 只放宣告
void imu_read(void);

#endif
