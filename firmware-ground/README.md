# firmware-ground — 地面接收端

## 角色

接收火箭端的 LoRa 遙測 → **原樣轉發**到電腦（USB CDC 虛擬 COM）。

**本端不解析封包。**收到一整行就照原位元組吐給電腦,真正的解析器是
`rocket_system_ground_side/src/core/models.py` 的 `SensorData`。這個分工是刻意的:
封包格式改動時只有 Python 端要跟,韌體不用重燒。

## 現況

已實作,是一支精簡的橋接韌體（`Core/Src/main.c` + `lora_bridge.c`）,
從火箭端複製後移除了感測器 / SD / GPS / 開傘等飛行邏輯。

> 本節之前寫著「此資料夾已清空,等待重寫」——那是 2026-06 的狀態,早已不成立。

## 硬體

- MCU：WeAct 黑丸版 STM32F411CEU6（HSE 25MHz），與火箭端同款
- USART1：PA9=TX→E22 RXD、PA10=RX←E22 TXD
- LoRa：E22-900T22D，UART 透傳模式（M0/M1 接 GND）
- USB CDC：對電腦的虛擬 COM

⚠ **頻道 / 速率 / 封包參數必須與火箭端一致**才收得到。火箭端可用
`#CMD:SETCH_nn#` 換頻,換完地面端也要跟著換,否則整條鏈路斷掉。

## 封包格式

火箭端每 500ms 送一行 ASCII，以 `\r\n` 結尾：

```
T28386 SQ42 AX+0.007 AY+0.026 AZ+0.978 GX+6.09 GY-1.05 GZ-2.80 P997.92
RH-0.1 KH-0.1 VZ+0.00 GA0.98 ST:0 MOD:F GPS:1,8 C:0 VF8.12 VA7.98
LAT+22.17483 LON+120.89272
```

完整欄位語意、狀態碼、模組旗標見 [`../shared/protocol.h`](../shared/protocol.h)。

> ⚠ 2026-07-30 修正：本節原本寫的是
> `N=<seq> T=<ms> P=.. RH=.. KH=.. G=.. S=<ID|LA|DP|DD> M=<bmp><imu><lora><sd>`,
> 那是 2026-06 之前的舊格式。實際封包**沒有 `=` 分隔、沒有 2 字元狀態碼**,
> 模組旗標是 hex 而非 4 個十進位數字,而且多了十幾個欄位。
>
> 同時移除了「用 `RKT_LORA_RX_FMT` 解析」的建議——那個巨集已經刪掉了。
> 現行格式**不能用單一個 `sscanf` 解**：GPS 定位與否是兩種長度、尾端可能被
> RF 切掉,而 `sscanf` 一個欄位不合就整串放棄。要寫 C 解析器請照 Python 端的
> 策略：逐欄位前綴搜尋，**缺 ST/MOD/GA 就整幀丟棄**（理由見 protocol.h 末段）。

## 如果要讓本端也解析

目前沒有這個需求（電腦端解析已經夠用，而且改起來快）。真要做的話：

1. UART 收 LoRa，逐位元組進 ring buffer，遇 `\n` 收完一包
2. 逐欄位前綴解析成 `rkt_telemetry_t`（見 protocol.h），**不要用 sscanf**
3. 截斷檢查：缺 `ST`/`MOD`/`GA` 即丟棄整幀
4. （選配）本地顯示（OLED）、記 SD、用 `SQ` 序號算掉包率
