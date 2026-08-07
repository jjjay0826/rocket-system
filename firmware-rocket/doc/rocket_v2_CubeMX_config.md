# rocket_v2 CubeMX 重配設定清單

依 rocket_v2 schematic（2026-03-30）逐項設定。在 CubeMX 開 `rocket sensor.ioc`，
照下面改完 → Project Manager → 確認 Toolchain=STM32CubeIDE → 「Generate Code」。

> 原則：先把舊外設關掉（SPI3/SDIO/I2C1/I2C3），再開新外設，最後設 GPIO 預設值。
> ⚠️ 產碼前確認「Keep User Code」打勾，否則 USER CODE 區塊以外的手改會遺失。

---

## 1. 時脈（不動，沿用舊板）
- HSI 16MHz → PLL（M=16, N=336, P=4, Q=7）→ SYSCLK 84MHz / USB 48MHz
- **無 HSE**（自製板無晶振）。SystemClock_Config 已手寫在 USER CODE 外，CubeMX 不會動它。

## 2. 要「關閉」的舊外設
| 外設 | 動作 |
|---|---|
| SPI3 | Disable |
| SDIO | Disable（FATFS 會跟著改）|
| I2C1 | Disable |
| I2C3 | Disable |

## 3. SPI1 — SD 卡（新增）
- Mode: **Full-Duplex Master**，NSS = Disable（軟體 CS）
- 腳位：PA5=SPI1_SCK, PA6=SPI1_MISO, PA7=SPI1_MOSI
- 參數：
  - Data Size = 8 bits, MSB First
  - CPOL = **Low**, CPHA = **1 Edge**（SD 卡用 SPI mode 0）
  - Baud Prescaler = 16（≈5.25MHz 起始；diskio 會自行降到 <400kHz 做 init 再拉高）
- ※ SPI1 在 APB2(84MHz)，故 prescaler 16 = 5.25MHz

## 4. SPI2 — 感測器 BMP585 + LSM6DSO（新增）
- Mode: **Full-Duplex Master**，NSS = Disable
- 腳位：PB13=SPI2_SCK, PB14=SPI2_MISO, PB15=SPI2_MOSI
- 參數：
  - Data Size = 8 bits, MSB First
  - CPOL = **High**, CPHA = **2 Edge**（= SPI mode 3，沿用舊板 working 設定）
  - Baud Prescaler = **8**（≈5.25MHz）
    - ⚠️ 舊板用 prescaler 2 = 21MHz，但 BMP585/LSM6DSO 規格上限都是 10MHz（超規）。
      新板建議降到 prescaler 8（合規、更可靠）。SPI2 在 APB1(42MHz)。

## 5. USART1 — LoRa E22（角色改變）
- Mode: Asynchronous
- 腳位：PA9=USART1_TX, PA10=USART1_RX
- Baud = **9600**, 8N1（E22 出廠預設）
- **NVIC：勾選 USART1 global interrupt**（lora_e22.c 用 IT 收發）
- ⚠️ 接線驗證：MCU PA9(TX)→E22 RXD、PA10(RX)→E22 TXD（不可 TX-TX）

## 6. USART2 — GPS（新增）
- Mode: Asynchronous
- 腳位：PA2=USART2_TX, PA3=USART2_RX
- Baud = 9600, 8N1
- **NVIC：勾選 USART2 global interrupt**（GPS 用）
- ⚠️ 接線驗證：MCU PA2(TX)→GPS RX、PA3(RX)→GPS TX

## 7. USB — CDC（沿用，但注意 VBUS）
- USB_OTG_FS = Device_Only，加入 USB_DEVICE 中介層 CDC
- 腳位：PA11=DM, PA12=DP
- ⚠️ **VBUS sensing = Disabled**：舊板 PA9 曾做 VBUS，新板 PA9 改 USART1_TX，
  必須關掉 VBUS sensing（本來就是板級修正之一）。

## 8. FATFS — 改成 SPI 模式
- 關掉 SDIO 後，FATFS → Mode = **User-defined**
- 產碼後會生成 `FATFS/Target/user_diskio.c`（USER_read/write/ioctl 空殼）
- 由我提供的 `sd_diskio_spi.c` 接上（整合步驟見該檔註解）

## 9. GPIO 輸出（產碼後設預設位準）
| 腳位 | 標籤 | 模式 | 預設 | 備註 |
|---|---|---|---|---|
| PA0 | 7V_CMD_1 | Output PP | **LOW** | 發火，安全鎖定 |
| PA1 | 7V_CMD_2 | Output PP | **LOW** | 發火 |
| PB9 | 5V_CMD_1 | Output PP | **LOW** | 發火 |
| PB8 | 5V_CMD_2 | Output PP | **LOW** | 發火 |
| PB5 | SIG_OUT_3 | Output PP | LOW | 訊號輸出 |
| PB6 | SIG_OUT_2 | Output PP | LOW | |
| PB7 | SIG_OUT_1 | Output PP | LOW | |
| PA4 | SD_CS_N | Output PP | **HIGH** | SD 片選 idle 高 |
| PB12 | BARO_CS_N | Output PP | **HIGH** | BMP585 片選 |
| PA8 | IMU_CS_N | Output PP | **HIGH** | LSM6DSO 片選 |
| PB10 | LED_B10 | Output PP | HIGH | 狀態燈（低電平亮，視接法）|
| PB2 | LED_B2 | Output PP | HIGH | 狀態燈 |

> ★ 4 支發火腳（PA0/PA1/PB8/PB9）務必設定 **預設 LOW + 上電即低**，
>   配合硬體 10kΩ 下拉，是上電不誤觸的雙重保險。

---

## 產碼後 handle 對照（給後續改 user code 用）
| 用途 | 舊 handle | 新 handle |
|---|---|---|
| 感測器 SPI | hspi3 | **hspi2** |
| SD SPI | (SDIO) | **hspi1** |
| LoRa | (SPI3) | **huart1** |
| GPS | huart1 | **huart2** |

## 產碼後待辦（我負責）
1. `lora.c/h`（SX1268）停用 → 改用 `lora_e22.c/h`
2. `user_diskio.c` 接 `sd_diskio_spi.c`
3. main.c：CS 腳改 PB12/PA8、感測器 hspi3→hspi2、GPS huart1→huart2、
   發火 4 通道、狀態 LED PA2→PB10/PB2
4. bmp585.c：傳入的 SPI handle hspi3→hspi2
