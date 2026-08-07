# 所有檔案在哪裡

**整套系統是兩個 GitHub repo,兩個都要 clone。**
少任何一個都跑不起來 —— 一個是火箭上的韌體,另一個是筆電上的地面站。

```bash
git clone https://github.com/jjjay0826/rocket-system.git
git clone https://github.com/jx06T/rocket_system_ground_side.git
```

**其他什麼都不需要。**2026-08-01 的原始飛行資料已經收在第一個 repo 的
`doc/flightdata/`,火箭模型在 `sim/models/` —— 不必跟任何人要檔案。

---

## 總覽

| # | Repo | 是什麼 | 語言 |
|---|---|---|---|
| 1 | **`jjjay0826/rocket-system`** | 韌體 monorepo：火箭端＋地面端 STM32、分析工具、文件、飛行資料 | C ＋ Python |
| 2 | **`jx06T/rocket_system_ground_side`** | 地面站軟體，跑在筆電上的 PyQt GUI | Python |

---

## 1. 韌體 monorepo

```
https://github.com/jjjay0826/rocket-system
```

| 資料夾 | 內容 |
|---|---|
| `firmware-rocket/` | 火箭上那塊板的 C 程式(STM32CubeIDE 專案) |
| `firmware-ground/` | 地面那塊板的 C 程式 —— **只做 LoRa→USB 透傳,233 行** |
| `sandbox/` | 桌上實驗用的專案(`parachute` 投放測試、`baro` 氣壓計) |
| `shared/` | 兩端共用:封包契約、LoRa 模組設定檔 |
| `sim/` | OpenRocket 模型(`models/`)與引擎推力曲線(`motors/`) |
| `tools/` | Python 分析工具(遙測／影像／模擬三條管線) |
| `doc/` | 全部文件 —— 從 [README.md](README.md) 進去 |

**注意**:這裡的 `firmware-ground` 是那塊**STM32 板子**的韌體,
不是筆電上的地面站軟體。地面站軟體在第 2 處。

---

## 2. 地面站軟體 —— **另一個 repo**

```
https://github.com/jx06T/rocket_system_ground_side
```

> **★ 認明 `jx06T` 這個帳號 —— 那是本尊。**
> `jjjay0826/rocket_system_ground_side` 是它的 fork,不要拿 fork 當開發基準,
> 除非你確定它比較新。

目前版本 **v3.4**。PyQt GUI,跑在筆電上:讀序列埠、即時繪圖、OpenGL 顯示火箭姿態、
Folium 畫經緯度、顯示任務階段。

```bash
git clone https://github.com/jx06T/rocket_system_ground_side.git
cd rocket_system_ground_side
pip install -r requirements.txt
cp settings.example.json settings.json     # 改成你的 COM 埠
python main.py
```

| 檔案 | 用途 |
|---|---|
| `main.py` | GUI 進入點 |
| `run_backend_ch1.bat` / `run_backend_ch2.bat` | **兩塊板各跑一個後端** |
| `settings.json` | COM 埠等設定(`settings.example.json` 是範本) |
| `src/core/models.py` | **封包解析器**(`SensorData.from_new_format()`) |
| `doc/telemetry_format.md` | 封包格式規範 |
| `doc/architecture.md`、`walkthrough.md` | 架構與操作導覽 |
| `logs/`、`data/` | 執行時產生的記錄 |

### ★★ 這個 repo 裡有一支測試會去讀韌體的 `main.c`

`tests/test_crossrepo_protocol.py` **直接讀 `firmware-rocket/Core/Src/main.c`**,
把 `snprintf` 的格式字串抓出來,與 `shared/protocol.h` 的巨集、與地面站自己的
解析器逐字比對。它驗的不是「我以為的格式」,是韌體原始碼本身。

2026-08-04 對修復後的韌體重跑:**26/26 通過**,全套 14 支測試也全過。

```bash
cd rocket_system_ground_side
python tests/test_crossrepo_protocol.py     # 只跑跨 repo 協定
python tests/run_all.py                     # 全部 14 支
```

它會自動找 `rocket-system`,找不到就設環境變數 `ROCKET_FIRMWARE_REPO`。

> ⚠ **沒有 CI**,而且測試住在**地面站 repo**。
> 只改韌體、沒 clone 這邊的人不會知道它存在 —— 這是這個機制唯一的弱點。
> **改 `main.c` 的封包格式或指令字串之後,一定要跑它。**

權威順序寫在 `doc/telemetry_format.md`:三者不一致時,**以 `main.c` 為準**。

---

## 3. 飛行資料 —— **已經在 repo 裡了**

`rocket-system/doc/flightdata/`。clone 下來就有,不用找任何人要。

| | |
|---|---|
| `20260801/` | 2026-08-01 旭海實飛的**四份原始 log**(3.7 MB) |
| `20260720_ground_test/` | 2026-07 的三次測試(437 KB) |

> **`_utc.log` 那兩個是關鍵。**兩塊板各自以「自己判定離架」為時間原點,
> ch1 比 ch2 晚 0.49 秒。**只有 UTC 時戳能把兩塊板放到同一條時間軸上**,
> 而 2026-08-01 的整個失效分析(兩次開傘衝擊、自旋時序)完全依賴這件事。

逐檔說明見 [flightdata/README.md](flightdata/README.md)。

### 沒收進來的東西,以及怎麼自己生

**都可以再生,所以不用跟任何人要:**

| | 怎麼生 |
|---|---|
| 解析後的 CSV、事件檔 | `tools/parse_raw_lora.py` |
| 飛行儀表板 | `tools/flight_report.py` |
| 影片幀(原本 668 MB) | `tools/grab_yt_frames.py`,來源 `youtube.com/watch?v=ogR7rgce_ps` 的 **5:54:53 起 47 秒** |
| 81 組模擬矩陣 | `tools/gen_sim_matrix_309.py`(模型已在 `sim/models/`) |

**唯一永久遺失的是 SD 卡的 50 Hz 記錄** —— 箭體未回收,兩塊板隨之沉海。
那不是誰忘了備份,是物理上不存在了。

---

## 兩個 repo 的關係

```
        ┌─────────────────────────┐         ┌──────────────────────────────┐
        │  rocket-system          │         │ rocket_system_ground_side    │
        │  （C / STM32 韌體）      │         │ （Python / PyQt，跑在筆電）    │
        ├─────────────────────────┤         ├──────────────────────────────┤
        │ firmware-rocket  ───────┼─LoRa──▶ │                              │
        │   封包格式的權威來源      │         │                              │
        │   main.c:1984           │         │  src/core/models.py           │
        │                         │         │    ← 解析器要跟著 main.c 改    │
        │ firmware-ground  ───────┼─USB───▶ │                              │
        │   （純透傳，不解析）      │         │  main.py  GUI                 │
        └─────────────────────────┘         └──────────────────────────────┘

  ⚠ 改封包格式時，兩個 repo 都要動。這是 monorepo 沒能涵蓋到的接縫。
```

---

## 完整檔案地圖

只列**人寫的**檔案。CubeMX 產生的(`gpio.c`／`spi.c`／`tim.c`／`usart.c`／
`stm32f4xx_*`／`system_stm32f4xx.c`／`syscalls.c`／`sysmem.c`)和
`Drivers/`／`Middlewares/`／`USB_DEVICE/` 不列 —— 那些不要手改,
改了下次重新產碼會被蓋掉。

### rocket-system

```
firmware-rocket/Core/
├─ Src/
│  ├─ main.c              2468  ★ 全部飛行邏輯：Mahony、KF2、IMU 驅動、
│  │                            開傘判據、狀態機、封包組裝。封包格式在 :1984/:1993
│  ├─ cmd.c                435  USB CDC 命令列（CLEAR / READ / 校準…）
│  ├─ sd_diskio_spi.c      420  SD 卡 SPI 模式的 FatFS diskio 層
│  ├─ logger.c             331  CSV 寫入、預分配、批次 sync、錯誤復原
│  ├─ lora_e22.c           320  E22-900T22D 透傳驅動（非阻塞收發）
│  ├─ gnss.c               167  GPS NMEA 解析。★ 完全不做濾波，直接賦值
│  ├─ bmp585.c             143  氣壓計 SPI 驅動
│  └─ sdcard.c              20  死碼，但 logger.c 的 extern FIL file 靠它，不能刪
└─ Inc/                          對應標頭 ＋ main.h（腳位定義都在這）

firmware-ground/Core/Src/
├─ main.c                  233  初始化與主迴圈
└─ lora_bridge.c           186  LoRa UART → USB CDC 純透傳。★ 不解析封包

sandbox/
├─ parachute/    投放測試韌體（23 個原始檔）。500 Hz CSV、KF2、無開傘、無 LoRa
└─ baro/         氣壓計測試（31 個原始檔）。★ 舊 linked-resource 結構，見下方警告

shared/
├─ protocol.h          封包契約（沒被 include，但由跨 repo 測試逐字驗證）
├─ lora_922_rocket.ini E22 暫存器設定 — 火箭端
└─ lora_922_ground.ini 同上 — 地面端（與火箭端位元組相同）

sim/
├─ models/3.0.9.ork    ★ 定出 DEPLOY_TB=18s 的那份模型
├─ models/3.0.8.ork    前一版
└─ motors/Pioneer5K{,_m10,_p10}.eng   引擎推力曲線（標準／−10%／+10%）

tools/                20 支分析腳本，用途總表見 tools/README.md
doc/                  全部文件，路由見 doc/README.md
doc/flightdata/       ★ 原始遙測，唯一不可再生的東西
```

### rocket_system_ground_side

```
main.py                       GUI 進入點
run_backend_ch1.bat / _ch2    ★ 兩塊板各跑一個後端
settings.json                 COM 埠等設定（gitignore，用 settings.example.json 複製）

src/
├─ backend_daemon.py     266  串口讀取常駐程式，經 ZMQ 送給 GUI
├─ core/
│  ├─ models.py          430  ★ 封包解析器 SensorData.from_new_format()
│  ├─ communicator.py    296  串口收發
│  ├─ lora_protocol.py   120  ★ 上行指令 token（#CMD:ARM_SYSTEM_SALT7763# 等）
│  └─ observer.py         13  觀察者介面
├─ gui/
│  ├─ main_window.py    2172  ★ 主視窗
│  ├─ ui_main.py         375  版面
│  └─ visualizers/            姿態(OpenGL)／折線圖／地圖／log／任務階段
├─ storage/csv_storage.py 127 落地存檔
└─ utils/settings.py      153 設定讀寫

tests/                   ★ 14 支測試，python tests/run_all.py 全跑
└─ test_crossrepo_protocol.py  ★★ 直接讀韌體 main.c 逐字比對

doc/
├─ telemetry_format.md   178  ★ 封包格式規範（權威順序寫在這）
├─ architecture.md       252  地面站架構
├─ rocket_side_requirements.md  107  對火箭端的要求
├─ attitude_config.md     84  姿態顯示的軸向設定
├─ walkthrough.md         42  操作導覽
└─ implementation_plan_two.md / health_check_report.md   歷史紀錄
```

---

## 檔案規劃:新東西該放哪

| 我寫了… | 放這裡 | 注意 |
|---|---|---|
| 新的**感測器驅動** | `firmware-rocket/Core/Src/<晶片名>.c` ＋ `Inc/<晶片名>.h` | 跟 `bmp585.c` 一樣的形狀。**不要塞進 `main.c`** |
| 新的**飛行邏輯**(判據、濾波) | `main.c` | 已經 2468 行了,但那是飛安關鍵路徑,**沒有台架驗證前不要拆** |
| **實驗性**的東西 | `sandbox/<新專案>/` | 驗證過再整併進 `firmware-rocket`。這是專案的既定工作流程 |
| 新的**分析腳本** | `tools/*.py` | **同時更新 `tools/README.md` 的用途總表**,否則沒人知道它存在 |
| 新的**文件** | `doc/*.md` | **同時更新 `doc/README.md` 的檔案總覽與狀態標示** |
| 新的**飛行資料** | `doc/flightdata/<YYYYMMDD>/` | 只放原始 log。CSV／圖表／影像幀都可再生,不要進版控 |
| 新的**火箭模型**、引擎曲線 | `sim/models/`、`sim/motors/` | **不要在腳本裡寫絕對路徑**(2026-08 踩過) |
| 兩端都要用的東西 | `shared/` | |
| 地面站的**新畫面元件** | `ground_side/src/gui/visualizers/` | |
| 地面站的**新解析欄位** | `ground_side/src/core/models.py` | **改完跑 `python tests/run_all.py`** |

### 三條規矩

**① 改 `main.c` 的封包格式或指令字串 → 一定要跑跨 repo 測試**

```bash
cd rocket_system_ground_side && python tests/run_all.py
```

沒有 CI,不跑就不會有人發現兩端對不上。

**② 新增分析腳本或文件 → 同時更新對應的 README 總表**

2026-08 整理時發現 `tools/` 有 20 支腳本,**其中 9 支從沒被 README 提過** ——
等於不存在。文件也一度出現三份講同一件事而結論相反。

**③ 不要在腳本裡寫絕對路徑**

`gen_sim_matrix_309.py` 曾經寫死 `D:\Downloads\3.0.9.ork`,換一台電腦就跑不動。
模型現在在 `sim/models/`,路徑用 `Path(__file__).parent.parent` 推導。

### ⚠ `sandbox/baro` 的結構和其他專案不一樣

它的**專案根在 `STM32CubeIDE/` 子資料夾**,用舊的 linked-resource 結構,
`Application/User/` 裡的 startup 與 syscalls 是**建置唯一的來源**。
根目錄的 `Core/` 那份在專案根之外,不在建置裡。
**動它的檔案結構之前先看 `STM32CubeIDE/.project`。**

---

## 相關

- 專案入門 → [getting_started.md](getting_started.md)
- 文件路由 → [README.md](README.md)
- 共用檔案與協定同步機制 → [../shared/README.md](../shared/README.md)
- 工具用途總表 → [../tools/README.md](../tools/README.md)
