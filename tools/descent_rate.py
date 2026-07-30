# -*- coding: utf-8 -*-
"""從地面站 CSV 算下降速率 —— 用來向主辦方證明「觸水速度 < 12 m/s」（規範 4.2.3）。

  python tools/descent_rate.py <地面站的 telemetry CSV>

## 為什麼用「裸氣壓高度的線性回歸」而不是直接看 vz 欄

傘下是**等速**下降 → 高度對時間是一條直線 → 斜率就是下降速率。
對整段做回歸，用的是 `rel_height`（裸氣壓，欄位 RH），這條路徑：

  · 不經 Kalman、不經 IMU 積分、不經 Mahony 姿態
  · 不受加速度計 ±16g 飽和影響
  · 隨機雜訊被幾百個樣本平均掉

`vz`（KF 速度）當**獨立交叉驗證**用 —— 它走的是另一條路（IMU 積分 + 氣壓修正），
兩者對得上才可信。對不上就代表其中一條有問題，要先查清楚再交報告。

## 事件靠 total_accel 抓，不靠 stage

stage 是火箭端的判斷；用 g 值獨立定事件，報告才不是「自己說自己對」。
"""
import csv, sys, math


def load(path):
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                t = float(r["timestamp_ms"]) / 1000.0
                h = float(r["rel_height"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isnan(t) or math.isnan(h):
                continue
            def g(k, d=float("nan")):
                try: return float(r[k])
                except (KeyError, TypeError, ValueError): return d
            rows.append(dict(t=t, h=h, vz=g("vz"), ga=g("total_accel"),
                             st=g("flight_state", -1), kh=g("kfh_height")))
    rows.sort(key=lambda r: r["t"])
    return rows


def fit(seg):
    """對 (t, h) 做最小平方；回傳 (斜率 m/s, 斜率標準誤, R²)"""
    n = len(seg)
    if n < 3:
        return None
    mt = sum(r["t"] for r in seg) / n
    mh = sum(r["h"] for r in seg) / n
    sxx = sum((r["t"] - mt) ** 2 for r in seg)
    if sxx == 0:
        return None
    sxy = sum((r["t"] - mt) * (r["h"] - mh) for r in seg)
    slope = sxy / sxx
    inter = mh - slope * mt
    resid = [r["h"] - (slope * r["t"] + inter) for r in seg]
    sse = sum(e * e for e in resid)
    sst = sum((r["h"] - mh) ** 2 for r in seg)
    se = math.sqrt(sse / (n - 2) / sxx) if n > 2 else float("nan")
    r2 = 1 - sse / sst if sst else float("nan")
    return slope, se, r2, math.sqrt(sse / (n - 2))


def main(path):
    rows = load(path)
    if len(rows) < 10:
        print("資料太少，無法分析"); return 1
    print(f"讀入 {len(rows)} 筆，t = {rows[0]['t']:.1f} ~ {rows[-1]['t']:.1f} s "
          f"（平均取樣間隔 {(rows[-1]['t']-rows[0]['t'])/(len(rows)-1)*1000:.0f} ms）\n")

    apo = max(rows, key=lambda r: r["h"])
    print(f"最高點：{apo['h']:.1f} m @ t={apo['t']:.2f} s")

    # 事件：開傘 = 頂點之後第一個 g 尖峰；觸水 = 最後一個 g 尖峰
    after = [r for r in rows if r["t"] > apo["t"] and not math.isnan(r["ga"])]
    spikes = [r for r in after if r["ga"] > 3.0]
    t_dep = spikes[0]["t"] if spikes else None
    t_hit = spikes[-1]["t"] if len(spikes) > 1 else None
    print(f"開傘衝擊（g>3 首次）：{f'{t_dep:.2f} s' if t_dep else '未偵測到'}")
    print(f"觸水/觸地（g>3 末次）：{f'{t_hit:.2f} s' if t_hit else '未偵測到'}")
    if spikes:
        print(f"  ⚠ 峰值 {max(r['ga'] for r in spikes):.1f} g"
              f"（加速度計 ±16g，total_g 上限 27.7g —— 超過即被削頂，"
              f"這個數字是下限不是真值）")

    # 穩態段：開傘後 3 秒（等傘張滿、擺盪衰減）到觸水前 2 秒
    lo = (t_dep + 3.0) if t_dep else (apo["t"] + 5.0)
    hi = (t_hit - 2.0) if t_hit else rows[-1]["t"]
    seg = [r for r in rows if lo <= r["t"] <= hi]
    print(f"\n穩態下降段：{lo:.1f} ~ {hi:.1f} s，共 {len(seg)} 筆")

    res = fit(seg)
    if not res:
        print("穩態段樣本不足"); return 1
    slope, se, r2, rms = res
    v = abs(slope)
    print("\n" + "=" * 62)
    print("【主要證據】裸氣壓高度線性回歸（不經 KF / IMU / 姿態）")
    print("=" * 62)
    print(f"  下降速率 = {v:.3f} ± {se:.3f} m/s   (95% 信賴區間 ±{1.96*se:.3f})")
    print(f"  R² = {r2:.5f}    殘差 RMS = {rms:.2f} m")
    ok = v + 1.96 * se < 12.0
    print(f"\n  規範 4.2.3（<12 m/s）：{'✅ 通過' if ok else '❌ 不通過'}"
          f"    餘裕 {12.0 - v:.2f} m/s")
    if r2 < 0.99:
        print("  ⚠ R² 偏低 → 不是乾淨的等速段（傘擺盪？分段開傘？）需人工看曲線")

    # 交叉驗證：KF 的 vz 欄
    vzs = [r["vz"] for r in seg if not math.isnan(r["vz"])]
    if vzs:
        mv = sum(vzs) / len(vzs)
        sd = math.sqrt(sum((x - mv) ** 2 for x in vzs) / len(vzs))
        print("\n" + "-" * 62)
        print("【交叉驗證】KF 速度欄 vz（IMU 積分 + 氣壓修正，另一條路徑）")
        print("-" * 62)
        print(f"  平均 {abs(mv):.3f} m/s   標準差 {sd:.3f}")
        d = abs(abs(mv) - v)
        print(f"  與回歸值相差 {d:.3f} m/s "
              + ("✅ 兩條獨立路徑一致" if d < 0.5 else
                 "⚠ 差異偏大 —— 交報告前先查清楚哪一條有問題"))

    # 分段檢查：速率有沒有隨高度變化（空氣密度、傘沒張滿）
    if len(seg) >= 60:
        print("\n" + "-" * 62)
        print("【分段】把穩態段切三份，確認速率穩定")
        print("-" * 62)
        k = len(seg) // 3
        for i, name in enumerate(("前段", "中段", "後段")):
            sub = seg[i*k:(i+1)*k] if i < 2 else seg[2*k:]
            r = fit(sub)
            if r:
                print(f"  {name}  {sub[0]['h']:>7.0f} → {sub[-1]['h']:>6.0f} m"
                      f"   {abs(r[0]):.3f} m/s")
        print("  （高空空氣稀薄，速率通常略高於低空；差異應 <15%）")

    print(f"\n交給主辦方時建議附上：本輸出 + 高度-時間散佈圖與回歸線 + 原始 CSV")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    sys.exit(main(sys.argv[1]))
