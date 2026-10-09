# hardware/ — 航電板的硬體設計檔

兩代板子的電路圖、PCB 與網表，全部用 EasyEDA 畫的。
日期以 **EasyEDA 標題欄**為準（檔案本身的修改時間只是下載時間，不可信）。

## 兩代板子

| 資料夾 | 板子 | 設計日期 | 板上有什麼 |
|---|---|---|---|
| [`avionics_v1_2025-05/`](avionics_v1_2025-05/) | **Avionics_v1**（第一代自製板） | 建立 2025-02-11、更新 2025-05-22 | STM32F411 直接焊在板上、BMP 氣壓計、LSM6 IMU、**433 MHz** LoRa、ATGM GPS、SD、USB、18650 電池、AP63205 降壓 |
| [`rocket_v7_2026-06/`](rocket_v7_2026-06/) | **rocket_v7** | 建立 2026-05-05、更新 2026-06-07 | STM32F411、BMP、LoRa（含 M0／M1／AUX 腳）、GPS、SD、USB |

### avionics_v1_2025-05/

| 檔案 | 原始檔名 | 內容 |
|---|---|---|
| `schematic_avionics_v1_2025-05-22.pdf` | `SCH_Schematic1_2025-05-22.pdf` | 電路圖，6 頁 |
| `pcb_avionics_v1_2025-05-22.pdf` | `PCB_PCB1_2025-05-22.pdf` | PCB 佈線圖，5 頁 |
| `pcb_avionics_v1_3d_2025-05-21.step` | `3D_PCB1_2025-05-21.step` | 3D 模型（30 MB，可用 FreeCAD／Fusion 開） |

### rocket_v7_2026-06/

| 檔案 | 原始檔名 | 內容 |
|---|---|---|
| `schematic_rocket_v7_2026-06-22.pdf` | `SCH_Schematic1_2026-06-22.pdf` | 電路圖，1 頁 |
| `netlist_rocket_v7_2026-06-26.tel` | `Netlist_Schematic1_2026-06-26.tel` | 網表（2026-06-26 匯出） |

**網表是韌體腳位的依據。**2026-07-20 韌體就是照這份網表確認
LoRa 的 AUX 腳有接到 MCU，才啟用 `LORA_USE_AUX`（修好「沒裝模組也回報發送成功」）。
改韌體腳位之前先對這份。

> 2026-08-01 實飛用的是 2026-06-30 起改用 WeAct 黑丸 STM32F411 的版本。
> 這份 rocket_v7 是目前找得到的**最新**電路圖，但不保證與實飛那塊逐線相同。

### 每個資料夾都有 `SHA256SUMS`

收進來時算的雜湊值，`sha256sum -c SHA256SUMS` 可確認檔案沒被改過。

---

## 資料表（沒有附檔）

原廠資料表的版權屬於原廠，**不放進公開 repo**。下表記下我們參考的元件與版本，
請到原廠網站以型號搜尋下載。

| 元件 | 用途 | 我們參考的文件 |
|---|---|---|
| EBYTE **E22-900T22D** | LoRa 遙測（900 MHz） | User Manual EN **v1.3** |
| **ATGM336H**（晶片 AT6558） | GPS | ATGM336H 規格書 ＋ **CASIC** 協定手冊（英文版） |
| ST **LSM6DS3** | 加速度計／陀螺儀 | ST datasheet |
| Bosch **BMP585** | 氣壓計 | Bosch datasheet |

B 板換 2.4 GHz **E28** 的接線與法規，見
[`../firmware-rocket/doc/e28_b_board_checklist.md`](../firmware-rocket/doc/e28_b_board_checklist.md)。

---

## 沒有進來的東西

| | 為什麼 |
|---|---|
| `SCH_Schematic1_2026-06-22_page-0001.jpg` | 就是上面那份 PDF 的截圖 |
| 「Battery 2S2P 18650」系列（6 個 PDF／PNG） | 名字不符內容：其實是 Canva 畫的系統方塊圖，內容還是舊的 433 MHz 設計，而且檔案屬性含個人姓名 |
