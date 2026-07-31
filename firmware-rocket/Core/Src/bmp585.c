/*
 * bmp585.c — BMP5 family SPI driver (BMP585 / BMP581)
 *
 * 硬體接法（rocket_v2 黑丸版，與 LSM6DS3 共用 SPI2）：
 *   SPI2 SCK  = PB13
 *   SPI2 MISO = PB14
 *   SPI2 MOSI = PB15
 *   BMP585 CS = PB12  (= CubeMX label BARO_CS，見 bmp585.h)
 *   LSM6DS3 CS = PA8  （同一條匯流排上的另一顆）
 *   BMP585 CSB 必須接 GPIO（不能接 VDD，否則進 I2C 模式）
 *
 * ⚠ 2026-07-31 更正：此處原本寫「共用 SPI3 / SCK=PB12 / CS=PB10」，那是
 *   自製板時代的接法。程式碼一直是對的（CS 走 BMP_CS_PORT/PIN 巨集），
 *   錯的只有這段註解 —— 但它會在查 SPI2 焊點問題時把人帶去量錯的腳位。
 *
 * Register map (BMP5 series, Bosch BMP5-Sensor-API):
 *   0x01 : CHIP_ID        (BMP585=0x51, BMP581=0x50)
 *   0x36 : OSR_CONFIG     [6]=press_en  [5:3]=osr_t  [2:0]=osr_p
 *   0x37 : ODR_CONFIG     [4:2]=odr    [1:0]=pwr_mode (01=normal)
 *   0x1D-0x1F : TEMP_DATA (XLSB, LSB, MSB) little-endian 24-bit signed
 *   0x20-0x22 : PRESS_DATA(XLSB, LSB, MSB) little-endian 24-bit unsigned
 *
 * SPI 協定：
 *   Write: CS↓, [reg & 0x7F, val], CS↑
 *   Read : CS↓, [reg | 0x80, 0x00, ...], CS↑  (同一 transaction 讀回)
 *
 * Conversion:
 *   P_Pa  = raw_press / 64.0f
 *   P_hPa = raw_press / 6400.0f
 */

#include "bmp585.h"
#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"

static SPI_HandleTypeDef *bmp_spi;
static uint32_t bmp_last_raw = 0;

static inline void BMP_CS_LOW(void)
{
    HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);
}
static inline void BMP_CS_HIGH(void)
{
    HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);
}

/* SPI write: 送 [reg_addr, val] */
static HAL_StatusTypeDef bmp_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };
    BMP_CS_LOW();
    HAL_StatusTypeDef r = HAL_SPI_Transmit(bmp_spi, tx, 2, 100);
    BMP_CS_HIGH();
    return r;
}

/* SPI read: 送 [reg_addr | 0x80, dummy × len]，同步收回 len bytes */
static HAL_StatusTypeDef bmp_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 30) return HAL_ERROR;

    uint8_t tx[32] = { reg | 0x80 };   /* tx[1..len] = 0x00 (dummy clocks) */
    uint8_t rx[32];

    BMP_CS_LOW();
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(bmp_spi, tx, rx, (uint16_t)(len + 1), 100);
    BMP_CS_HIGH();

    if (r == HAL_OK)
        memcpy(data, &rx[1], len);  /* 跳過第一個 byte（送 addr 時的 dummy rx） */
    return r;
}

/* ---- 軟體重置（BMP5 家族：CMD 暫存器 0x7E 寫 0xB6）----------------------
 * 給「凍結看門狗」用。原本凍結時只呼叫 BMP585_Init()，而它只做了讀 CHIP_ID
 * ＋ 寫兩個設定暫存器 —— **卡住的狀態機不會因為寫暫存器就解開**，所以那條
 * 救援路徑很可能是空操作。宜蘭那次 MOD 掉 0 之後就沒再回來，符合這個描述。
 *
 * ★刻意不放進開機路徑。開機時晶片是乾淨的，加一道重置只是多一個失敗機會，
 *   而氣壓計是開傘條件 A 的唯一來源 —— 賽前不在關鍵路徑上多加動作。
 *   救援路徑則相反：那時晶片已經不正常了，重置沒有東西可以弄壞。
 * ------------------------------------------------------------------------- */
void BMP585_SoftReset(void)
{
    if (!bmp_spi) return;
    bmp_write_reg(0x7E, 0xB6);
    HAL_Delay(10);          /* 資料表 ~2ms，取 10ms 保守 */
}

/* ---- 初始化：回傳 chip_id（0=I2C/SPI 無回應） ---- */
uint8_t BMP585_Init(SPI_HandleTypeDef *hspi)
{
    bmp_spi = hspi;
    BMP_CS_HIGH();      /* 確保 CS 釋放 */
    HAL_Delay(10);      /* power-on stabilisation */

    /* 讀 CHIP_ID → 0x51=BMP585, 0x50=BMP581, 其他=異常 */
    uint8_t chip_id = 0;
    if (bmp_read_regs(0x01, &chip_id, 1) != HAL_OK) return 0;
    /* 驗證晶片身分：只有 0x51(BMP585)/0x50(BMP581) 才算真的在。沒接晶片時
     * SPI 常讀到 0xFF/0x00、bmp_read_regs 仍回 HAL_OK → 這裡擋掉，避免上層
     * mod.bmp585 誤判存活。*/
    if (chip_id != 0x51 && chip_id != 0x50) return 0;

    /* OSR_CONFIG (0x36): press_en=1(bit6), osr_t=x1, osr_p=x4
       0x40 | 0x02 = 0x42  ← press_en 必須設，否則氣壓輸出為 0 */
    bmp_write_reg(0x36, 0x42);

    /* ODR_CONFIG (0x37): pwr_mode=01 (Normal mode) */
    bmp_write_reg(0x37, 0x01);

    HAL_Delay(10);
    return chip_id;
}

/* ---- 讀氣壓：回傳 hPa，失敗回 0.0f ---- */
float BMP585_ReadPressure(void)
{
    uint8_t data[3] = {0};

    /* PRESS_DATA_XLSB = 0x20，連續讀 3 bytes（little-endian） */
    if (bmp_read_regs(0x20, data, 3) != HAL_OK) return 0.0f;

    uint32_t raw = (uint32_t)data[0]
                 | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16);

    bmp_last_raw = raw;
    /* 1 LSB = 1/64 Pa → 轉為 hPa */
    return (float)raw / 6400.0f;
}

uint32_t BMP585_GetLastRaw(void) { return bmp_last_raw; }

/* ---- 計算高度（國際標準大氣公式） ---- */
float BMP585_ReadAltitude(float seaLevel_hPa)
{
    float p = BMP585_ReadPressure();
    if (p <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(p / seaLevel_hPa, 0.1903f));
}
