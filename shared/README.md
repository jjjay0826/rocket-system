# shared/ — 兩端共用的東西

放**火箭端與地面端都要用到**的檔案。放在這裡的東西改一次兩端同步，
這也是這個專案用 monorepo 的理由。

| 檔案 | 用途 |
|---|---|
| `protocol.h` | LoRa 遙測封包契約 |
| `lora_922_rocket.ini` | E22-900T22D 暫存器設定 — 火箭端 |
| `lora_922_ground.ini` | E22-900T22D 暫存器設定 — 地面端 |

---

## LoRa 模組設定檔

用原廠設定工具（RF_Setting）匯入。內容是 12 個位元組的暫存器傾印：

```
C0,00,09,00,00,00,64,20,48,03,00,00,
```

**兩個檔目前位元組完全相同**，這是對的 —— 一條鏈路的兩端必須共用
同樣的頻道、空中速率與封包長度，否則收不到。分成兩個檔只是為了在
設定工具裡不會弄錯是在燒哪一顆。

> ⚠ 改任何一個都要**同時改另一個**，否則鏈路會靜默斷掉
> （模組不會報錯，就只是收不到東西）。
>
> 頻率相關的法規依據見 `doc/` 下的競賽合規紀錄；
> 2026-07 曾發生協作者把頻率設成 868 MHz 的事故。

---

## `protocol.h` 沒有被 `#include`，但**它是同步的**

先講清楚，因為這件事很容易誤解：

| | |
|---|---|
| 有任何 `.c` `#include` 它嗎？ | **沒有** |
| 那它跟實際封包一致嗎？ | **一致，而且是自動驗證的** |

- 火箭端的封包格式**寫死在** `firmware-rocket/Core/Src/main.c:1984`（有 GPS）
  與 `:1993`（無 GPS）
- `firmware-ground` **不解析** —— `lora_bridge.c` 是純 UART 透傳
- 真正的解析器在**另一個 repo**：
  [`rocket_system_ground_side`](https://github.com/jjjay0826/rocket_system_ground_side)
  的 `src/core/models.py`

### ★ 同步是靠一支跨 repo 測試

`rocket_system_ground_side/tests/test_crossrepo_protocol.py`
**直接讀 `firmware-rocket/Core/Src/main.c`**，把 `snprintf` 的格式字串抓出來，
逐項比對。它驗的不是「我以為的格式」，是**韌體原始碼本身**：

```
✓ 從 main.c 抓到兩種封包格式（有/無 GPS）
✓ protocol.h 的 RKT_LORA_TX_FMT_GPS   與 main.c 逐字相同
✓ protocol.h 的 RKT_LORA_TX_FMT_NOGPS 與 main.c 逐字相同
✓ 韌體狀態機共 5 個狀態 / 狀態順序
✓ ★ARM / DPL / CAL / GND / GND_OFF / ABG token 與韌體逐字相同
✓ ★點傘一律走 deploy_fire_on()（PA0+PA1 同時）
✓ 自動開傘訊息不含 'successfully'（不誤觸下行確認）
                                        RESULT PASS  26/26
```

2026-08-04 對修復後的 `main.c` 重跑：**26/26 通過**，
`protocol.h` 含 `SQ`／`VF`／`VA`／`LAT`／`LON`／`C:` 全部欄位。

### ⚠ 這個機制的真正弱點

不是「沒同步」，而是**位置與執行方式**：

1. **沒有 CI。**測試要手動跑 —— 只改韌體、沒 clone 地面站 repo 的人，
   完全不會知道有這支測試存在。
2. **它住在另一個 repo。**從 `rocket-system` 這邊看不到。

**所以：改 `main.c` 的封包格式或指令字串之後，一定要跑它：**

```bash
cd rocket_system_ground_side
python tests/test_crossrepo_protocol.py     # 只跑跨 repo 協定
python tests/run_all.py                     # 全部 14 支
```

它會自動找 `rocket-system`，找不到就設環境變數 `ROCKET_FIRMWARE_REPO`。

> 權威順序（寫在 `rocket_system_ground_side/doc/telemetry_format.md`）：
> **三者不一致時，以 `main.c` 為準。**
