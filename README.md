# rocket-system

STM32F411 火箭航電 **Monorepo**：火箭端（發送）、地面端（接收）與測試專案放在同一個倉庫，
以保證 LoRa 遙測協定兩端同步。

---

## 👉 從這裡開始

| 你是誰 | 讀哪份 |
|---|---|
| **第一次接觸這個專案** | **[`doc/getting_started.md`](doc/getting_started.md)** — 從零開始，20 分鐘，只需要會看 C |
| **在找檔案** | **[`doc/where_is_everything.md`](doc/where_is_everything.md)** — 完整檔案地圖，以及「新東西該放哪」 |
| 已經熟悉，要找特定東西 | [`doc/README.md`](doc/README.md) — 依「你想做什麼」路由，並標明每份文件的狀態 |

> ⚠ **這個 repo 不是全部。整套系統要 clone 兩個：**
>
> ```bash
> git clone https://github.com/jjjay0826/rocket-system.git
> git clone https://github.com/jx06T/rocket_system_ground_side.git
> ```
>
> 第二個是筆電上跑的地面站 GUI。**認明 `jx06T` 那個帳號是本尊**，
> `jjjay0826` 底下同名的是 fork。

**這套系統已經實際飛過一次** —— 2026-08-01 旭海，第 161 隊，競賽第三名。
飛到 833.7 m。ch1 的開傘主路徑在頂點後 1.5 秒準時觸發，ch2 主路徑失效、由 18 秒備援接手；
最終失效發生在回收系統的機械端，
箭體未回收。完整結論：[`doc/flight_161_summary.md`](doc/flight_161_summary.md)。

那次的**原始遙測就在 `doc/flightdata/`**，火箭模型在 `sim/models/` —— clone 下來就有，
不必跟任何人要檔案。

## 為什麼用 Monorepo

火箭端與地面端共用同一套封包格式。**改格式時若分倉，必須同步更新多個地方、很容易漏改**；
同一個 repo 可以「一次 commit 同時改兩端」，協定永遠一致。封包格式的單一真實來源放在
[`shared/protocol.h`](shared/protocol.h)。

## 結構

```
rocket-system/
├─ doc/                 ★ 全部文件 —— 先看 doc/README.md（導覽＋狀態標示）
├─ tools/               ★ 分析工具 —— 遙測／影像／模擬三條管線，見 tools/README.md
├─ firmware-rocket/     火箭飛控（STM32F411CEU6, 黑丸版）— 降落傘自動投放，飛行/安全關鍵
├─ firmware-ground/     地面接收站（解析 LoRa 遙測、接電腦）
├─ sandbox/             實驗 / 測試專案（新功能先在這驗證，過了再進正式韌體）
│  ├─ parachute/        降落傘投放測試
│  └─ baro/             氣壓計測試
├─ sim/                 OpenRocket 模型與推力曲線
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

> **兩端都沒有實際 `#include shared/protocol.h`**（發送端格式寫死在 `main.c:1984`／`:1993`，
> 接收端有自己的解析碼），**但契約是自動驗證的** —— 地面站 repo 的
> `tests/test_crossrepo_protocol.py` 直接讀 `main.c`，把格式字串抓出來與
> `protocol.h` 的巨集逐字比對（2026-08-04 重跑 26/26 通過）。
>
> ⚠ 但**沒有 CI**，而且那支測試住在**另一個 repo**。
> **改封包格式或指令字串之後一定要跑：** `cd rocket_system_ground_side && python tests/run_all.py`
> 細節見 [`shared/README.md`](shared/README.md)。

## 開發 / 編譯

1. STM32CubeIDE → File → Open Projects from File System → 選 `firmware-rocket`（或其他子專案）資料夾。
2. 目標 MCU：STM32F411CEU6（WeAct「黑藥丸」V2 模組）。
3. 燒錄：ST-Link SWD（PA13/PA14/GND）或 USB-C DFU。

## 工作流程

新功能先在 `sandbox/` 開發、實測；驗證通過再整併進 `firmware-rocket` / `firmware-ground`，
避免實驗性程式碼動搖飛行韌體。

發射日的完整操作程序見 [`doc/launch_day_reference.md`](doc/launch_day_reference.md)。

**寫新文件時請一起更新 [`doc/README.md`](doc/README.md) 的檔案總覽與狀態標示。**
2026-08 就是因為沒有索引，repo 裡一度同時存在三份講同一件事而結論相反的文件。
