# rocket-system

STM32F411 火箭航電 **Monorepo**：火箭端（發送）、地面端（接收）與測試專案放在同一個倉庫，
以保證 LoRa 遙測協定兩端同步。

## 為什麼用 Monorepo

火箭端與地面端共用同一套封包格式。**改格式時若分倉，必須同步更新多個地方、很容易漏改**；
同一個 repo 可以「一次 commit 同時改兩端」，協定永遠一致。封包格式的單一真實來源放在
[`shared/protocol.h`](shared/protocol.h)。

## 結構

```
rocket-system/
├─ firmware-rocket/     火箭飛控（STM32F411CEU6, rocket_v7 板）— 降落傘自動投放，飛行/安全關鍵
├─ firmware-ground/     地面接收站（解析 LoRa 遙測、接電腦）
├─ sandbox/             實驗 / 測試專案（新功能先在這驗證，過了再進正式韌體）
│  ├─ parachute/        降落傘投放測試
│  └─ baro/             氣壓計測試
├─ shared/
│  └─ protocol.h        LoRa 遙測封包契約（兩端共用的單一真實來源）
├─ .gitignore           STM32CubeIDE 專用（排除 Debug/、.elf、workspace 暫存…）
└─ README.md
```

> 每個 `firmware-*` / `sandbox/*` 各自是獨立的 STM32CubeIDE 專案（自帶 `.project`/`.cproject`/`.ioc`）。

## LoRa 遙測格式

ASCII 文字封包，E22-900T22D UART 透傳，`\n` 結尾：

```
N=<seq> T=<ms> P=<氣壓> RH=<相對高m> KH=<Kalman高m> G=<總G> S=<ID|LA|DP|DD> M=<bmp><imu><lora><sd>
```

完整欄位語意、狀態碼、解析格式見 [`shared/protocol.h`](shared/protocol.h)。

> ⚠ 目前兩端尚未實際 `#include shared/protocol.h`（發送端格式寫死在 `main.c`、接收端有自己的解析碼）。
> 讓兩端改用 `protocol.h` 的巨集是建議的下一步，採用後才真正得到協定同步保證。

## 開發 / 編譯

1. STM32CubeIDE → File → Open Projects from File System → 選 `firmware-rocket`（或其他子專案）資料夾。
2. 目標 MCU：STM32F411CEU6（WeAct「黑藥丸」V2 模組）。
3. 燒錄：ST-Link SWD（PA13/PA14/GND）或 USB-C DFU。

## 工作流程

新功能先在 `sandbox/` 開發、實測；驗證通過再整併進 `firmware-rocket` / `firmware-ground`，
避免實驗性程式碼動搖飛行韌體。

火箭韌體的工作守則與已踩過的雷見 [`firmware-rocket/CLAUDE.md`](firmware-rocket/CLAUDE.md)。
