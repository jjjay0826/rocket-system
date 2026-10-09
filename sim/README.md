# sim/ — OpenRocket 模型、引擎曲線與模擬輸出

```
sim/
├─ models/
│  ├─ 3.0.9.ork          ★ 現行模型：定出 DEPLOY_TB = 18 s 的那份（81 組模擬）
│  ├─ 3.0.8.ork          前一版
│  └─ archive/           更早的版本，只為追溯設計演進
├─ motors/
│  └─ Pioneer5K{,_m10,_p10}.eng    引擎推力曲線（標準／−10%／+10%）
└─ runs/
   └─ 20260720_deploy_tb/          把 DEPLOY_TB 從 20 s 改成 18 s 時用的模擬輸出
```

81 組模擬矩陣與分析在 [`../doc/sim309_81cases.csv`](../doc/sim309_81cases.csv)、
[`../doc/sim309_analysis.txt`](../doc/sim309_analysis.txt)，產生方式見
[`../tools/README.md`](../tools/README.md) 的管線 C。

---

## models/archive/

日期是檔案時間（存檔或下載時間）。「OpenRocket 註記」是模型檔裡 `comment` 欄位的原文。

| 檔案 | 原始檔名 | 日期 | OpenRocket 註記 |
|---|---|---|---|
| `rocket_ver2.ork` | `rocket ver.2.ork` | 2025-11-01 | Motor changed to correct version … |
| `rocket_2.5.3_with_chute.ork` | `Rocket 2.5.3 with chute.ork` | 2025-11-08 | Rail button measurements to be specefied |
| `rocket_2.6.1_a.ork` | `Rocket 2.6.1.ork` | 2026-02-07 | fin was redesigned AGAIN / proper length（4 KB） |
| `rocket_2.6.1_b.ork` | `Rocket 2.6.1 (1).ork` | 2026-04-03 | 同上（170 KB，與 `_a` 內容不同） |
| `3.0.1.ork` | 同 | 2026-07-20 | everything is set |
| `3.0.4.ork` | 同 | 2026-07-20 | everything is set |
| `3.0.5.ork` | 同 | 2026-07-28 | everything is set |
| `3.0.5_short.ork` | `3.0.5(short).ork` | 2026-07-28 | everything is set |

**分析請用 `models/3.0.9.ork`。**這裡的舊版只用來回答「設計是怎麼一路改過來的」。

---

## runs/20260720_deploy_tb/

2026-07-20 把 `DEPLOY_TB_MS`（備援計時器）從 20 s 改成 18 s，
依據的就是這批模擬（commit `fe55491`）。推力 ±10% 包絡下頂點落在 16.1～17.95 s。

| 檔案 | 推力 | OpenRocket 模擬名稱（檔頭原文） | 檔頭警告 |
|---|---|---|---|
| `5k_0.csv` | 標準 | SIM | 37 m/s 高速開傘 |
| `5k_0_h.csv` | 標準 | WORST CONDITION | 37.1 m/s 高速開傘 |
| `5k_0_r.csv` | 標準 | DISTANCE | 42.7 m/s 高速開傘 |
| `5k_m10.csv` | −10% | tailwind | — |
| `5k_m10_h.csv` | −10% | headwind | — |
| `5k_p10.csv` | +10% | tail wind | 40.1 m/s 高速開傘 |
| `5k_p10_h.csv` | +10% | headwind | 39.9 m/s 高速開傘 |
| `worst_case_simdata.csv` | 標準 | WORST CONDITION（原檔名 `simdata.csv`） | 40 m/s 高速開傘 |

- 檔頭的「高速開傘」警告是 **OpenRocket 自己設定的開傘延遲**，不是航電的行為
- ⚠ **這批是從哪一版 `.ork` 跑出來的沒有留下紀錄。**從檔案時間看：
  `worst_case_simdata.csv`（16:08）緊接在 `3.0.1.ork` 存檔（16:04）之後；
  `5k_*.csv`（20:29～20:56）和 `3.0.4.ork` 存檔（20:47）在同一個時段。
  所以照 repo 的慣例它們本該是「可再生、不入庫」，但因為一個飛安參數是依它們定的、
  又無法確定能原樣重跑，**原檔收進來當證據**
- [`../tools/sim_replay.py`](../tools/sim_replay.py) 的使用說明拿 `5k_p10_h.csv` 當範例：

```bash
python tools/sim_replay.py sim/runs/20260720_deploy_tb/5k_p10_h.csv --dry
```

`SHA256SUMS` 是收進來時算的雜湊值。
