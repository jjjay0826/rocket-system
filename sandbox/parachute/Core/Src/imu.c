#include "imu.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

// 引用 main.c 裡的 I2C 句柄
extern I2C_HandleTypeDef hi2c1;

#ifndef I2C_MEMADD_SIZE_8BIT
#define I2C_MEMADD_SIZE_8BIT 0x01U
#endif

void imu_read(void) {
    uint8_t data[6];
    int16_t ax, ay, az;
    char msg[64];

    if (HAL_I2C_Mem_Read(&hi2c1, LSM6DSOX_ADDR, 0x28, I2C_MEMADD_SIZE_8BIT, data, 6, 100) == HAL_OK) {
        ax = (int16_t)((data[1] << 8) | data[0]);
        ay = (int16_t)((data[3] << 8) | data[2]);
        az = (int16_t)((data[5] << 8) | data[4]);

        int n = snprintf(msg, sizeof(msg), "ACC:%d,%d,%d\r\n", ax, ay, az);

        // 送 USB CDC（避免 BUSY）
        uint8_t r;
        do {
            r = CDC_Transmit_FS((uint8_t*)msg, (uint16_t)n);
            if (r != 0) HAL_Delay(1);   // 0 = OK，其它都等一下
        } while (r != 0);
    }
}
