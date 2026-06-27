# firmware-ground — 地面接收端

> ⚠ 此資料夾已清空,等待重寫。先前的 `lora_receiver` 是從 sensor 專案複製來的半成品
> (含一堆用不到的驅動),已移除。

## 角色

接收火箭端的 LoRa 遙測 → 解析 → 輸出到電腦(USB CDC / 序列埠 / 顯示)。

## 封包格式(與火箭端共用)

火箭端送的是 ASCII 文字封包,以 `\n` 結尾:

```
N=<seq> T=<ms> P=<氣壓> RH=<相對高m> KH=<Kalman高m> G=<總G> S=<ID|LA|DP|DD> M=<bmp><imu><lora><sd>
```

完整欄位語意、狀態碼、建議解析格式見 [`../shared/protocol.h`](../shared/protocol.h)。
重寫時建議直接 `#include "../shared/protocol.h"`,用 `RKT_LORA_RX_FMT` 解析,讓兩端協定同步。

## 硬體(待定)

- MCU:STM32F411CEU6(或地面站採用的板子)
- LoRa:E22-900T22D,UART 透傳模式,**頻道/速率/封包參數必須與火箭端一致**才收得到。

## 重寫建議(乾淨骨架)

只需要:
1. UART 收 LoRa(E22 透傳,逐位元組進 ring buffer,遇 `\n` 收完一包)
2. 用 `RKT_LORA_RX_FMT` 解析成 `rkt_telemetry_t`
3. 輸出:USB CDC 轉發到電腦 / 或本地顯示(OLED、序列繪圖)
4. (選配)記 SD、偵測掉包(看 `N` 序號是否連續)

不需要火箭端那些:BMP585、IMU、發火腳、KF2 估計器、SDIO sensor 邏輯。

## 參考

舊半成品備份(可翻來抄收 LoRa 的部分):
`C:\Users\Jay0826\Desktop\航電程式\lora_receiver`
