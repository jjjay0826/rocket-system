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

## 🔴 `protocol.h` 目前是裝飾品

**沒有任何 `.c` 檔 `#include` 它。**

- 火箭端的封包格式**寫死在** `firmware-rocket/Core/Src/main.c:1984`（有 GPS）
  與 `:1993`（無 GPS）
- 地面端韌體**根本不解析** —— `lora_bridge.c` 是純 UART 透傳，
  真正的解析在 PC 端軟體，而那份**不在這個 repo 裡**
- `firmware-ground/Core/Src/main.c:20` 只在**註解**裡提到它

而且它已經和實際封包脫節：缺 `SQ`、`VF`、`VA` 與 GPS 欄位（2026-07 之後才加的）。

**所以「monorepo 保證協定兩端同步」目前沒有任何機制在保證。**

### ★ 但權威來源其實已經定案了 —— 只是定在另一個 repo 裡

地面站軟體（[jjjay0826/rocket_system_ground_side](https://github.com/jjjay0826/rocket_system_ground_side)）
的 `doc/telemetry_format.md` 開頭寫著：

> 權威來源：發送端格式字串寫死在 `firmware-rocket/Core/Src/main.c` 的 `lora_pkt` snprintf；
> C 語言版的契約在 `rocket-system/shared/protocol.h`；解析器在 `src/core/models.py`。
> **三者不一致時，以 `main.c` 為準。**

所以規則是清楚的：**`main.c` 說了算。**剩下的問題只是 `protocol.h` 該怎麼處理：

| 選項 | |
|---|---|
| **補完並讓火箭端真的 `#include`** | 一勞永逸，但要動飛行韌體 |
| **刪掉**，在 README 明講真實來源是 `main.c:1984` | 零風險，誠實 |

**留著不同步是最糟的選項** —— 它看起來像真的。

> ⚠ 封包格式**橫跨兩個 repo**：改 `main.c` 的格式字串時，
> `rocket_system_ground_side/src/core/models.py` 的解析器必須同步改。
> 這是 monorepo 沒能涵蓋到的接縫。見 [../doc/where_is_everything.md](../doc/where_is_everything.md)。
