# 所有檔案在哪裡

**東西不在同一個地方。**一共三處,兩個是 GitHub repo,一個完全沒有版控。
接手的人第一件事就是把這三處都拿到手。

---

## 三處總覽

| # | 位置 | 是什麼 | 版控 | 大小 |
|---|---|---|---|---|
| 1 | `Desktop/rocket-system/` | **韌體 monorepo**(火箭端＋地面端 STM32＋分析工具＋文件) | ✅ GitHub | 187 MB |
| 2 | `Desktop/rocket_system_ground_side/` | **地面站軟體**(筆電上跑的 PyQt GUI) | ✅ GitHub | 15 MB |
| 3 | `D:\Downloads\` ＋ `D:\Downloads\flight_161_analysis\` | **飛行原始資料與分析產出** | 🔴 **無** | 718 MB |

> `C:\stm32_project` 只是終端機的預設工作目錄,**裡面沒有東西**,不要去那裡找。

---

## 1. 韌體 monorepo

```
https://github.com/jjjay0826/rocket-system
```

```bash
git clone https://github.com/jjjay0826/rocket-system.git
```

| 資料夾 | 內容 |
|---|---|
| `firmware-rocket/` | 火箭上那塊板的 C 程式(STM32CubeIDE 專案) |
| `firmware-ground/` | 地面那塊板的 C 程式 —— **只做 LoRa→USB 透傳,233 行** |
| `sandbox/` | 桌上實驗用的專案(`parachute` 投放測試、`baro` 氣壓計) |
| `shared/` | 兩端共用:封包契約、LoRa 模組設定檔 |
| `sim/` | OpenRocket 模型與引擎推力曲線 |
| `tools/` | Python 分析工具(遙測／影像／模擬三條管線) |
| `doc/` | 全部文件 —— 從 [README.md](README.md) 進去 |

**注意**:這裡的 `firmware-ground` 是那塊**STM32 板子**的韌體,
不是筆電上的地面站軟體。地面站軟體在第 2 處。

---

## 2. 地面站軟體 —— **另一個 repo**

```
https://github.com/jjjay0826/rocket_system_ground_side
```

目前版本 **v3.4**。PyQt GUI,跑在筆電上:讀序列埠、即時繪圖、OpenGL 顯示火箭姿態、
Folium 畫經緯度、顯示任務階段。

```bash
git clone https://github.com/jjjay0826/rocket_system_ground_side.git
cd rocket_system_ground_side
pip install -r requirements.txt
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

### ★ 封包格式的權威來源已經定案了

`rocket_system_ground_side/doc/telemetry_format.md` 開頭寫著:

> 權威來源:發送端格式字串寫死在 `firmware-rocket/Core/Src/main.c` 的 `lora_pkt` snprintf;
> C 語言版的契約在 `rocket-system/shared/protocol.h`;解析器在 `src/core/models.py`。
> **三者不一致時,以 `main.c` 為準。**

也就是說封包格式**同時存在三個地方**,而規則是 `main.c` 說了算。
`shared/protocol.h` 目前沒有任何 `.c` include 它,詳見 [../shared/README.md](../shared/README.md)。

---

## 3. 🔴 飛行資料 —— 完全沒有版控

`D:\Downloads\` 與 `D:\Downloads\flight_161_analysis\`。
**這一處只存在於這台電腦。硬碟壞了就全沒了。**

### 不可再生的部分只有 4 MB

| 檔案 | 大小 | |
|---|---|---|
| `raw_ch1_20260801_152419_5e03e520.log` | 1.75 MB | 完整原始 log(含發射前 93 分鐘待機) |
| `raw_ch2_20260801_152419_c212c94f.log` | 1.94 MB | 同上,ch2 |
| `raw_ch1_..._utc.log` | 15 KB | **★ 帶 UTC 時戳的版本** |
| `raw_ch2_..._utc.log` | 18 KB | **★ 同上** |
| `raw_ch1_20260720_*.log`、`20260721_*.log` | 437 KB | 2026-07 的三次地面／飛行測試 |
| **合計** | **4.0 MB** | |

> **`_utc.log` 那兩個特別重要。**兩塊板各自以「自己判定離架」為時間原點,
> ch1 比 ch2 晚 0.49 秒。**只有 UTC 時戳能把兩塊板放到同一條時間軸上**,
> 而 2026-08-01 的整個失效分析(兩次開傘衝擊、自旋時序)完全依賴這件事。

### 可再生的部分(714 MB,不用備份)

| | 怎麼再生 |
|---|---|
| `chute_video/`(668 MB) | 影片幀。`tools/grab_yt_frames.py` 從 YouTube 存檔重抽:<br>`youtube.com/watch?v=ogR7rgce_ps` 的 **5:54:53 起 47 秒** |
| `viewer/`(36 MB)、`chute_frames/`(7 MB) | `tools/build_viewer.py`、`track_crop.py` 重跑 |
| `*_parsed.csv`、`*_events.txt` | `tools/parse_raw_lora.py` 從原始 log 重產 |
| `flight_161_20260801_report.*` | `tools/flight_report.py` 重產 |

**所以真正要搶救的只有那 4 MB。**

---

## 🔴 給接手者:最該先做的一件事

**把那 4 MB 的原始 log 放進版控。**它是整個專案唯一無法重製的東西,
而且小到沒有理由不收。建議:

```bash
cd rocket-system
mkdir -p doc/flightdata/20260801
cp "D:/Downloads/raw_ch1_20260801_152419_5e03e520_utc.log" doc/flightdata/20260801/
cp "D:/Downloads/raw_ch2_20260801_152419_5e03e520_utc.log" doc/flightdata/20260801/
cp "D:/Downloads/raw_ch1_20260801_152419_5e03e520.log"     doc/flightdata/20260801/
cp "D:/Downloads/raw_ch2_20260801_152419_c212c94f.log"     doc/flightdata/20260801/
```

沒有這些檔案,`doc/` 裡所有的分析都**無法重跑、無法查核**。

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

## 相關

- 專案入門 → [getting_started.md](getting_started.md)
- 文件路由 → [README.md](README.md)
- 共用檔案與 `protocol.h` 的現況 → [../shared/README.md](../shared/README.md)
