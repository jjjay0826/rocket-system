# tools/ 使用說明

三條獨立的管線：**遙測分析**、**影像分析**、**模擬與回放**。
下面的指令都可以直接複製貼上，路徑用 2026-08-01 那次飛行的實際檔名。

> 專案總導覽在 [`../doc/README.md`](../doc/README.md)。

## 這裡有什麼

| 我想做… | 用哪支 |
|---|---|
| 把地面站的原始 log 變成可分析的 CSV | `parse_raw_lora.py` |
| 做一張雙板飛行儀表板（給報告用） | `flight_report.py` |
| 逐項比對兩塊板 | `compare_boards.py` |
| GPS 不能用時反解速度向量 | `fit_ballistic.py` |
| 向主辦方證明觸水速度合規 | `descent_rate.py` |
| 從沒走到 LANDED 的 SD 卡救資料 | `recover_log.py` |
| 從 YouTube 直播存檔抽出飛行畫面 | `grab_yt_frames.py` |
| 追蹤飛行體、疊幀降噪 | `track_crop.py` |
| 把天空扣掉讓傘衣／傘繩浮出來 | `separate_objects.py` |
| 判斷傘有沒有撕裂 | `tear_analysis.py`、`fragment_track.py` |
| 拖時間軸一次比對多組影像 | `build_viewer.py` |
| 產生 OpenRocket 模擬矩陣 | `gen_sim_matrix_309.py` |
| 分析模擬結果的飛安邊際 | `analyse_309.py` |
| **用真的韌體邏輯重跑模擬**（驗開傘判據） | `fw_logic.py` |
| 把模擬灌進地面站當假飛行 | `replay309.py` |
| 對原始碼做結構斷言（驗證審查修復） | `verify_audit_20260731.py` |

---

## 管線 A：遙測分析

```
raw_chN_*.log  ──parse_raw_lora──▶  *_parsed.csv  ──flight_report──▶  儀表板 PNG/PDF
                                          │
                                          ├──compare_boards──▶  雙板對照表
                                          └──fit_ballistic──▶   速度向量反解
```

### A1　解析原始 log

```bash
python tools/parse_raw_lora.py <raw_log> --out-dir <輸出目錄>
```

實例：

原始 log 就在 repo 裡（`doc/flightdata/`），在 repo 根目錄執行：

```bash
python tools/parse_raw_lora.py doc/flightdata/20260801/raw_ch1_20260801_152419_5e03e520_utc.log --out-dir out
python tools/parse_raw_lora.py doc/flightdata/20260801/raw_ch2_20260801_152419_5e03e520_utc.log --out-dir out
```

產出兩個檔：

| 檔案 | 內容 |
|---|---|
| `*_parsed.csv` | 每包一列，位元欄位（MOD／C）已展開成獨立欄 |
| `*_events.txt` | 狀態機轉換、cond 旗標、韌體訊息、掉包統計、GPS 逐點速度 |

**★ 地面站的 log 若有 UTC 時戳前綴，一定要用那個版本。**有 UTC 就有兩塊板的
共同時鐘，下游不必再靠擬合對齊。CSV 會多一個 `utc` 欄（當日秒數）。

### A2　產生儀表板

```bash
python tools/flight_report.py <先失聯那塊的csv> <涵蓋全程那塊的csv> --out <輸出.png> [--dpi 300]
```

實例：

```bash
python tools/flight_report.py \
    out/raw_ch1_20260801_152419_5e03e520_utc_parsed.csv \
    out/raw_ch2_20260801_152419_5e03e520_utc_parsed.csv \
    --out flight_161_20260801_report.png --dpi 300
```

- **第一個參數放先失聯的板**（ch1），第二個放涵蓋全程的（ch2）。工具用第二塊
  當主時間軸。
- `--dpi`：螢幕看 150 夠、列印 300、海報 400。預設 200。
- 同時輸出**向量 PDF**（列印／投影用這個，檔案小又不糊）。`--no-pdf` 可關。
- 自動處理**空中重開機**：偵測到 uptime 歸零就把那段救回來，並由重開機前的
  (壓力, 高度) 反解原始 ref_press，把高度換算回同一基準。

### A3　雙板逐項對照（純文字）

```bash
python tools/compare_boards.py <csv1> <csv2>
```

### A4　速度向量反解（含水平分量）

```bash
python tools/fit_ballistic.py <csv1> [csv2]
```

GPS 上升段不能用時，用「頂點垂直速度=0 → 加速度計量到的 GA 完全來自水平
速度」反解。會輸出敏感度區間 —— **看區間不要看點值**。

### A5　SD 卡資料救援

```bash
python tools/recover_log.py E:\ --keep-tail
```

飛行沒走到 LANDED 時 `f_truncate` 不會執行，SD 上的檔案是 16 MB 加垃圾尾巴，
直接開會看到亂碼。這支用四道判據切掉尾巴。指到卡的根目錄會把每個 `logN.csv`
都跑一遍，並印出各檔出現過哪些 state（藉此判斷哪個檔才是飛行）。

### A6　下降速率合規證明

```bash
python tools/descent_rate.py <地面站的 telemetry CSV>
```

用**裸氣壓高度的線性回歸**算下降率，不是直接讀 `vz` 欄
（`vz` 是卡爾曼輸出，2026-08-01 實測雙板差 10%）。
產出可以直接附在給主辦方的文件裡，證明觸水速度符合規範 4.2.3 的 12 m/s。

---

## 管線 B：影像分析

```
YouTube ──grab_yt_frames──▶ frames/*.png ──track_crop──▶ 追蹤裁切＋疊幀
                                                  │
                                                  └──separate_objects──▶ 天空扣除、多通道
```

### B1　從 YouTube 抓片段並抽幀

```bash
python tools/grab_yt_frames.py "<網址>" --start 5:54:53 --dur 47 --out-dir chute_video
```

- 只下載指定區間，不會抓整部影片
- 自動取 **video-only 最高畫質**串流（YouTube 的混流版通常上限只有 720p）
- 抽出無損 PNG。`--fps N` 可降抽幀率，`--no-frames` 只下載不抽

**怎麼找時間戳**：遙測有 UTC，用點火閃光那一幀對齊即可。本次飛行：

| 事件 | UTC（台灣時間） | 相對離架 |
|---|---|---|
| 離架 | 16:57:28.050 | T+0 |
| 開傘點火 | 16:57:44.088 | T+16.0 |
| 開傘衝擊 6.14 g | 16:57:46.097 | T+18.0 |
| 遙測最後一包（20 m） | 16:58:11.607 | T+43.6 |

### B2　追蹤裁切 ＋ 疊幀降噪

```bash
python tools/track_crop.py "chute_video/frames/f_*.png" --out-dir tracked \
       --every 4 --stack 7 --upscale 3 --win-w 460 --win-h 620
```

| 參數 | 意義 |
|---|---|
| `--stack N` | **疊 N 幀降噪**。這是唯一真正提升畫質的手法（雜訊降 √N 倍） |
| `--win-w/h` | 裁切窗大小。傘放在上方 25%，下面 75% 留給拖曳的箭身 |
| `--upscale` | 放大倍率（純視覺，不增加資訊） |
| `--every N` | 每 N 幀輸出一張 |
| `--max-shift` | 疊幀對齊容許的位移；相機搖鏡時要放寬 |

會印出**裁切邊界檢查**（該邊有物體 = 被切掉），不要用猜的：

```
裁切邊界檢查（該邊有物體 = 被切掉）： 下6/30  上3/30  左2/30  右2/30
```

### B3　天空扣除 ＋ 多通道分離

```bash
python tools/separate_objects.py "tracked/f_*.png" --out-dir separated --up 1
```

對非物體像素做**二次曲面最小平方擬合**得到天空，扣掉之後剩下的才是訊號。
輸出五個面板：

| 面板 | 看什麼 |
|---|---|
| 原圖 | 對照 |
| 天空擬合扣除＋對比 | 綜合判讀 |
| **色度通道** | 暖色**傘衣**的位置與形狀 |
| **亮度殘差** | 白色的**傘繩、箭身** —— 色度通道完全看不到這些 |
| 合成輪廓 | 只看形狀 |

`--chroma-gain` / `--lum-gain` 分別調兩個通道的增益。想看細繩就把
`--lum-gain` 開大。

> `enhance_frames.py` 是這支的早期簡化版（只有色度單通道），已被取代。

### B4　拖時間軸、一次比對多組影像

```bash
python tools/build_viewer.py "separated/*_sep.png" --out-dir viewer
```

產生一個**單一 HTML 的本機檢視器**：可拖時間軸、可釘選多組並排比對。
影像用相對路徑引用而不是 base64 內嵌 —— 150 張 base64 進 HTML 會超過 10 MB，
開起來很卡。

### B5　判斷傘有沒有撕裂

```bash
python tools/tear_analysis.py --frames "frames60/g_*.png" --telemetry <parsed.csv>
python tools/fragment_track.py --frames "frames60/g_*.png" --t-start 18.2 --t-end 19.5
```

| 工具 | 判據 |
|---|---|
| `tear_analysis.py` | ① 下降率有沒有**階梯**（阻力面積突減）② 傘衣投影面積（**要乘距離平方**，否則火箭接近相機會讓像素數自然變大） |
| `fragment_track.py` | 追蹤**每一個**碎塊而非最大那團，看相對距離是**持續拉大**（真脫離）還是**固定**（附著／被繩子連著） |

> **🔴 2026-08-01 的教訓：這兩支在那次的影片上都得不到結論。**
> 傘衣只有約 80 px、候選碎塊 10~30 px，H.264 在該位元率下把它們吃掉了，
> 最近鄰追蹤 2~3 幀就失效。**不要基於那種解析度的影片宣稱「看到布片飛走」。**
> 詳見 [`../doc/parachute_failure_20260801.md`](../doc/parachute_failure_20260801.md) §3.4。

---

## 管線 C：模擬與回放

```
3.0.9.ork ──gen_sim_matrix_309──▶ 81 組 ──(OpenRocket 手動 Run all)──▶ 結果 CSV
                                                    │
                                     ┌──────────────┼──────────────┐
                             analyse_309      fw_logic        replay309
                            （飛安邊際）   （真韌體邏輯）   （灌進地面站）
```

### C1　產生模擬矩陣

```bash
python tools/gen_sim_matrix_309.py
```

3 推力 × 3 仰角 × 9 風況 = 81 組。**OpenRocket 24.12 沒有 headless 模式**
（試過 `--help`，它直接開 GUI），所以中間要手動在 GUI 按「Run all simulations」再存檔。

### C2　飛安邊際分析

```bash
python tools/analyse_309.py
```

開傘時序用**彈道解析式**算，不從 `.ork` 讀 —— 模擬設 `deployevent=apogee`，
頂點之後那段是傘已開的等速下降，拿它算「掉 10 m 要多久」會高估
（6 m/s 掉 10 m 要 1.7 s，彈道只要 1.43 s）。

### C3　★ 用真的韌體邏輯重跑

```bash
python tools/fw_logic.py
```

**這支和 C2 不一樣，而且不一樣的地方很重要。**`analyse_309.py` 用解析式，
`replay309.py` 直接查 OpenRocket 的開傘事件（那只驗地面站顯示層）。
這支是**把韌體的 `cond_A` / `cond_B` / 備援計時器用 Python 重寫**，
餵模擬資料下去，看它到底什麼時候開傘。

> 2026-08-01 發射前就是靠這支發現：解析式說 C 備援 0/81 組會先觸發，
> **真跑韌體邏輯是 7/81**。A∧B 實際要頂點後 1.95 s 而非解析式的 1.55 s。

### C4　灌進地面站當假飛行

```bash
python tools/replay309.py
```

沒有真飛行資料時，用第二顆 E22（USB-TTL）把模擬封包發出去，
地面站收到的流與真火箭無異（含狀態機演進與開傘 MSG 事件）。

> `sim_replay.py` 是舊版，**已脫節而且是安靜地脫節** —— 它跑得起來，
> 只是封包缺 `SQ`/`VF`/`VA`（2026-07 之後才加的欄位），
> 地面站的鏈路品質欄會永遠是空的。**用 `replay309.py`。**

---

## 其他

### 驗證審查修復

```bash
python tools/verify_audit_20260731.py
```

對原始碼做**結構斷言**（韌體無法在 PC 上執行，所以只能驗結構）。
改完一批 findings 之後跑這支確認沒有漏改或改錯地方。

---

## 相依套件

```bash
pip install numpy matplotlib pillow yt-dlp imageio-ffmpeg
```

`imageio-ffmpeg` 會附帶一份 ffmpeg 執行檔，不必另外安裝。
沒有用到 scipy（環境裡沒有），所有數值方法都是手寫的。

---

## 這批工具的共同原則

1. **量到的 vs 算出來的分開標**。圖表與輸出都會註明每個數字的來源。
2. **兩塊板對等**。不預設誰是真值；兩者的差就是量測不確定度。
3. **不要用猜的**。裁切有沒有切到、對齊準不準、GPS 可不可信，都用可以
   重跑的量測去判，並把判據印出來。
