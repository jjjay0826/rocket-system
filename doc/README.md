# doc/ 導覽 —— 給下一屆

## 🆕 第一次接觸這個專案?先讀 **[getting_started.md](getting_started.md)**

**本頁是給「已經懂這個領域」的人用的路由表**,直接就出現開傘判據、
拖曳傘、卡爾曼、遙測封包這些詞。如果這些名詞你還不熟,
先花 20 分鐘讀 [getting_started.md](getting_started.md) —— 那份從
「這塊板子到底是幹嘛的」開始講,只需要會看 C 語言。

---

**懂了之後回到這頁。**下面每一份文件都標了狀態,
🔴 標記的**不要引用它的結論**,那是已經被推翻的舊版,留著只是為了追溯歷史。

---

## 五分鐘認識現況

**2026-08-01 在旭海實際飛過一次(第 161 隊,競賽第三名)。**

- 飛到 **833.7 m**,開傘在**頂點後 1.5 秒**準時觸發 —— **航電做對了它該做的事**
- 失效在回收系統的機械端:拖曳傘展開時把箭體帶進 **3.2 轉/秒**的自旋,
  主傘在自旋中充氣、傘繩扭轉、載荷集中到少數幾條線上、結構失效,
  以 32 m/s 落海
- **箭體未回收,只尋回鼻錐** → 兩塊板的 SD 卡沉海,**50 Hz 資料永久遺失**

完整版:**[flight_161_summary.md](flight_161_summary.md)** ← 沒時間的話只讀這份

---

## 依你要做的事找檔案

### 「我要準備下一次發射」

| 順序 | 檔案 | 為什麼 |
|---|---|---|
| 1 | **[launch_day_reference.md](launch_day_reference.md)** | 發射日操作手冊。編譯開關、開機檢查、地面站啟動、全部指令、按鈕、狀態燈 |
| 2 | **[open_defects_20260801.md](open_defects_20260801.md)** | 上次發射前的缺陷清單。**A 段是「會擋住發射」的項目**,先確認每一條的現況 |
| 3 | **[flight_161_summary.md](flight_161_summary.md)** 第七節 | 上次飛完得到的六條改進項,前三條是回收系統 |
| 4 | [../firmware-rocket/doc/e28_b_board_checklist.md](../firmware-rocket/doc/e28_b_board_checklist.md) | B 板換 2.4 GHz E28 的接線與法規 |

> ⚠ **最優先確認的一件事**:`firmware-rocket/Core/Src/main.c` 目前
> **在 git HEAD 上是編譯不過的**(字串字面值被換行切斷)。修正版在工作區但未 commit。
> 見本頁最後的「已知的倉庫問題」。

### 「我要知道上次為什麼失敗」

| 檔案 | 涵蓋 | 狀態 |
|---|---|---|
| **[flight_161_summary.md](flight_161_summary.md)** | 全部三項失效的總表 ＋ 子系統成績單 ＋ 無解清單 | 🟢 **從這裡開始** |
| **[parachute_failure_20260801.md](parachute_failure_20260801.md)** | 主傘失效的完整因果鏈、證據、判據 | 🟢 開傘議題的權威版 |
| [failure_analysis_20260801.md](failure_analysis_20260801.md) | **§1 GNSS 失效**、**§2 卡爾曼速度失效** | 🟡 這兩節仍有效 |
| ↳ 同檔 §3 主傘 | | 🔴 **已被推翻,勿引用** |

### 「我要分析我自己的飛行資料」

**[../tools/README.md](../tools/README.md)** —— 兩條管線的完整使用說明,指令可以直接複製貼上:

- **管線 A 遙測**:`raw_chN.log` → 解析 → 雙板儀表板 / 對照表 / 速度反解
- **管線 B 影像**:YouTube 直播存檔 → 抽幀 → 追蹤裁切 → 天空扣除
- **管線 C 模擬**:OpenRocket `.ork` → 81 組矩陣 → 韌體開傘邏輯重跑 → 地面站回放

### 「我要改韌體」

| 檔案 | 內容 |
|---|---|
| [../firmware-rocket/doc/interactive_features.md](../firmware-rocket/doc/interactive_features.md) | 互動功能與 LED 指示(PB6/PB7 開關、兩顆 LED 的語意) |
| [../firmware-rocket/doc/rocket_v2_CubeMX_config.md](../firmware-rocket/doc/rocket_v2_CubeMX_config.md) | CubeMX 逐項設定清單(重新產碼時照這份) |
| [../shared/README.md](../shared/README.md) | 兩端共用的東西:封包契約、LoRa 模組設定檔 |
| [failure_analysis_20260801.md](failure_analysis_20260801.md) §2 | 卡爾曼速度為什麼不可信 —— **改開傘判據前必讀** |

### 「我要做地面站」

[../firmware-ground/README.md](../firmware-ground/README.md) ＋ [../shared/protocol.h](../shared/protocol.h)。
沒有真飛行資料時用 `tools/replay309.py` 灌模擬封包進去。

### 「我要準備簡報 / 報告」

[final_presentation_plan.md](final_presentation_plan.md) —— 2026 年那次的簡報規劃。
比賽已經結束(第三名),但**評分表逐項拆解**與**論述框架**對下一屆仍然適用。

---

## 檔案總覽

### doc/

| 檔案 | 狀態 | 內容 |
|---|---|---|
| `getting_started.md` | 🟢 **新手第一站** | 從零開始:這塊板子是什麼、怎麼判斷開傘、板上有什麼、名詞表、第一個小時動手做什麼 |
| `where_is_everything.md` | 🟢 **找檔案看這份** | 東西分散在**三處**,兩個 GitHub repo ＋ 一處無版控。含 clone 指令與搶救清單 |
| `README.md` | 🟢 | 本頁(路由表) |
| `flight_161_summary.md` | 🟢 **入口** | 2026-08-01 飛行總結。總表／失效鏈／子系統成績單／無解清單／改進項 |
| `parachute_failure_20260801.md` | 🟢 | 主傘失效完整分析。**開傘議題以這份為準** |
| `launch_day_reference.md` | 🟢 | 發射日操作手冊(764 行,最厚的一份) |
| `failure_analysis_20260801.md` | 🟡 部分 | §1 GNSS 🟢／§2 卡爾曼 🟢／**§3 主傘 🔴 已被推翻** |
| `open_defects_20260801.md` | 🟡 快照 | 2026-08-01 發射前的缺陷清單。部分項目已被飛行結果回答 |
| `final_presentation_plan.md` | 🟡 歷史 | 2026 決賽簡報規劃。評分表拆解仍可參考 |
| `sim309_81cases.csv` | 🟢 資料 | 81 組 OpenRocket 模擬(3 推力 × 3 仰角 × 9 風況) |
| `sim309_analysis.txt` | 🟢 資料 | 上者的分析輸出 |

### 其他位置的文件

| 檔案 | 內容 |
|---|---|
| [`../README.md`](../README.md) | Monorepo 總覽、為什麼用 monorepo、編譯方式 |
| [`../tools/README.md`](../tools/README.md) | 全部分析工具的使用說明 |
| [`../firmware-ground/README.md`](../firmware-ground/README.md) | 地面接收端 |
| [`../firmware-rocket/doc/`](../firmware-rocket/doc/) | 韌體互動功能、B 板 E28 換裝 |

---

## 狀態標示的意思

| | 意思 |
|---|---|
| 🟢 | **現行有效。**可以直接引用 |
| 🟡 | **歷史紀錄或部分過時。**內容仍有價值,但引用前看該檔的頂部說明 |
| 🔴 | **結論已被推翻。**留著只為追溯,**不要引用** |

**寫新文件時請一起維護這一頁。**這次就是因為沒有索引,
repo 裡一度同時存在三份講同一件事而結論相反的文件。

---

## 🔴 已知的倉庫問題(下一屆接手時要先處理)

### 1. `main.c` 在 HEAD 上編譯不過

```bash
git diff HEAD -- firmware-rocket/Core/Src/main.c
```

commit `b0ad361` 裡有 4 處 C 字串字面值被換行切斷(寫檔時 shell 把 `\r\n`
當成真的換行處理掉了):

```c
reply("MSG WARN Deploy pulse already active
"); return; }
```

修正版在工作區,**尚未 commit**。接手第一件事就是確認它編得過再 commit。

### 2. 🔴 原始飛行資料不在 repo 裡(而且只存在一台電腦上)

2026-08-01 的原始 log 在 `D:\Downloads\`,**沒有版控、沒有備份**。

**不可再生的部分總共只有 4 MB**(其餘 714 MB 是可以從 YouTube 重抽的影像幀)。
其中 `_utc.log` 那兩個最關鍵 —— **只有 UTC 時戳能把兩塊板放到同一條時間軸上**,
整個失效分析都依賴它。

完整清單與搬進版控的指令見 [where_is_everything.md](where_is_everything.md)。
**沒有這 4 MB,`doc/` 裡所有分析都無法重跑、無法查核。**

### 3. 🟠 協定同步機制沒有 CI,而且住在另一個 repo

`shared/protocol.h` 沒有被任何 `.c` `#include`,但**它是同步的** ——
地面站 repo 有一支 `tests/test_crossrepo_protocol.py`,**直接讀 `main.c`**
把格式字串抓出來逐字比對(2026-08-04 重跑 26/26 通過)。

問題不是「沒同步」,是:

1. **沒有 CI**,要手動跑
2. **它在另一個 repo** —— 只改韌體的人根本不知道有這支測試

**改 `main.c` 的封包格式或指令字串後,一定要跑:**

```bash
cd rocket_system_ground_side && python tests/run_all.py
```

完整說明見 [../shared/README.md](../shared/README.md)。

---

## 2026-08-04 做過的結構整理

給接手的人一個交代,以免以為東西不見了:

| 動作 | 原因 |
|---|---|
| 刪除 `firmware-rocket/` 與 `firmware-ground/` 的 `Application/User/` | 2026-06-26 CubeIDE UnderRoot 遷移的殘骸。這兩個是**根結構**專案,`Core/Src` 才是真實 source folder,`Application/` **從未被編譯**(產物只有 `Debug/Core/Src/*.o`),`.project`／`.cproject` 也完全沒有引用它 |
| 刪除 `firmware-rocket/Core/Inc/imu.h`、`lora.h` | 0 個 include,對應的 `.c` 早已改名成 `.bak`(而 `.bak` 被 gitignore,所以 repo 裡根本沒有) |
| `firmware-ground` 的 `.ioc` 改名 `rocket sensor` → `rocket ground` | **兩個專案的 `.ioc` 都自稱 rocket sensor。**ground 的 `.cproject` 還有 5 處 `${workspace_loc:/rocket sensor}` —— 同一個 workspace 下建置,輸出可能寫進火箭專案的目錄 |
| `lora_922_*.ini` 搬到 `shared/` | 地面端的設定檔原本住在火箭專案裡 |
| `rocket_v2_CubeMX_config.md` 搬進 `firmware-rocket/doc/` | 與其他兩份韌體文件放一起 |
| `.gitignore` 移除 `*.launch` 規則 | 與它自己上面三行的註解「刻意保留入庫:`*.launch`」自相矛盾。既有的兩個沒事(早就被追蹤),但**新增的會被靜默忽略** |

### ⚠ `sandbox/baro` 刻意沒有比照辦理 —— 它的結構和另外兩個不一樣

整理過程中差點把 `sandbox/baro` 也一起清掉,**那會讓它連結失敗**。原因:

| | `firmware-rocket` / `firmware-ground` | `sandbox/baro` |
|---|---|---|
| 專案根 | 資料夾本身 | **`STM32CubeIDE/` 子資料夾** |
| 結構 | 根結構(2026-06-26 已遷移) | **舊 linked-resource** |
| `Core/Src/*.c` | 真實 source folder | 以 `<link>` 連進 `Application/User/Core/` 這個虛擬名稱 |
| `Application/User/` 裡的 startup／syscalls／sysmem | 沒被引用的重複品 | **實體檔案,而且是建置唯一的來源** |

`sandbox/baro/Core/` 底下雖然也有一份 startup／syscalls,但那在**專案根之外**,
`.project` 沒有連它,所以不在建置裡。**動 baro 的檔案結構之前先看 `STM32CubeIDE/.project`。**

**沒有動的**(已知取捨,不是疏漏):

- `main.c` 2468 行的 monolith。IMU 是唯一沒有獨立驅動的感測器
  (bmp585／gnss／lora_e22／logger／sd 都有),`imu.c` 是最自然的第一刀 ——
  **但那是飛安關鍵路徑,不該在沒有台架驗證的情況下拆。**
- `sdcard.c` 是死碼但不能刪:`logger.c:18` 的 `extern FIL file;` 靠它定義。

---

## 給下一屆的三個忠告

**① 兩塊完全獨立的板,是這個專案做過最值得的決定。**
2026-08-01 那次,**兩次開傘衝擊各只有一塊板錄到** —— 單板就永遠不會知道
發生過兩次衝擊,整條失效鏈也就推不出來。電源、感測器、天線全部獨立。

**② 純量測比濾波後的量可靠一個數量級。**實測:

| 量 | 雙板差 |
|---|---|
| 頂點**純氣壓**高度 | **0.3 m** |
| 頂點**卡爾曼**高度 | 3.5 m |
| 最大上升速度(卡爾曼) | 10.3% |
| `cond_B` 觸發時刻(吃卡爾曼速度) | **4.5 秒** |

開傘判據能用純氣壓就別用濾波後的速度。

**③ 事後才想量的東西,加進封包幾乎不要錢。**
2026-08-01 的關鍵證據(1166 °/s 自旋)是**事後**從封包裡湊出來的;
如果當初直接送 `|w|`,地面在飛行當下就看得到。同理加速規的峰值保持暫存器 ——
2 Hz 取樣抓不到開傘衝擊真峰值,那個暫存器成本近乎為零。
