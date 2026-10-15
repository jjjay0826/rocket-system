"""verify_timeline_0801.py — 從原始 log 重算 2026-08-01 事件時間軸，逐列對照文件

用法:
  python tools/verify_timeline_0801.py

只讀 doc/flightdata/20260801/ 的兩個 _utc.log，不依賴任何中間 CSV。
每一列印出「文件寫的值」與「log 算出來的值」。

時間軸的定義（和 parachute_failure_20260801.md 相同）:
  T+ = ch2 板子時鐘 − ch2 第一個 ST:1 封包的時鐘（T7464588）
  ch1 的時鐘用地面站 UTC 換算到 ch2 時鐘（兩板各取 utc−tick 的中位數相減）

這個 T+0 **不是離架瞬間**，見下方【1】。
"""
import os
import re
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "..", "doc", "flightdata", "20260801")
F1 = os.path.join(D, "raw_ch1_20260801_152419_5e03e520_utc.log")
F2 = os.path.join(D, "raw_ch2_20260801_152419_5e03e520_utc.log")
# _utc 版的 ch2 沒有收錄 MSG 行 → MSG 的前後順序一律從完整版讀
R1 = os.path.join(D, "raw_ch1_20260801_152419_5e03e520.log")
R2 = os.path.join(D, "raw_ch2_20260801_152419_c212c94f.log")

PKT = re.compile(r"^(\d\d):(\d\d):(\d\d\.\d+)\s+(?:\S+\s+)?T(\d+) SQ(\d+)")
FIELD = {k: re.compile(rf"\b{k}([+-]?\d+\.?\d*)") for k in ("RH", "GX", "GA", "VZ")}
ST = re.compile(r"ST:(\d)")


def utc_s(h, m, s):
    return int(h) * 3600 + int(m) * 60 + float(s)


def load(fn):
    pkts, msgs = [], []
    for line in open(fn, encoding="utf-8", errors="replace"):
        m = PKT.match(line)
        if m:
            p = {"utc": utc_s(*m.group(1, 2, 3)), "t": int(m.group(4)), "sq": int(m.group(5))}
            for k, rx in FIELD.items():
                f = rx.search(line)
                p[k] = float(f.group(1)) if f else None
            s = ST.search(line)
            p["st"] = int(s.group(1)) if s else None
            pkts.append(p)
        elif " MSG " in line:
            msgs.append((len(pkts), line.split(" MSG ", 1)[1].strip()))
    # 只取重開機前（ch1 在空中斷電後 T 從頭數）
    cut = next((i for i in range(1, len(pkts)) if pkts[i]["t"] < pkts[i - 1]["t"]), len(pkts))
    return pkts[:cut], [(i, s) for i, s in msgs if i <= cut]


def msgs_after_sq(raw, pkts):
    """完整版 log 裡每則 MSG 緊接在哪一包之後 → [(該包在 pkts 的下一個索引, 內容)]"""
    idx = {p["sq"]: i for i, p in enumerate(pkts)}
    out, last = [], None
    for line in open(raw, encoding="utf-8", errors="replace"):
        m = re.search(r"(?:^|\s)T(\d+) SQ(\d+)", line)
        if m:
            last = int(m.group(2))
        elif line.startswith("MSG ") and last in idx:
            out.append((idx[last] + 1, line[4:].strip()))
    return out


def first_st1(pkts):
    return next(i for i in range(1, len(pkts)) if pkts[i]["st"] >= 1 and pkts[i - 1]["st"] == 0)


def apex(pkts, to_t):
    """RH 最高點附近 5 點拋物線擬合 → (T+, 高度)"""
    i = max(range(len(pkts)), key=lambda k: pkts[k]["RH"] or -1e9)
    pts = pkts[max(0, i - 2): i + 3]
    xs = [to_t(p["t"]) for p in pts]
    ys = [p["RH"] for p in pts]
    n = len(xs)
    # 最小平方 y = a x² + b x + c
    sx = [sum(x ** k for x in xs) for k in range(5)]
    sy = [sum(y * x ** k for x, y in zip(xs, ys)) for k in range(3)]
    A = [[sx[4], sx[3], sx[2]], [sx[3], sx[2], sx[1]], [sx[2], sx[1], n]]
    import numpy as np
    a, b, c = np.linalg.solve(np.array(A), np.array(sy[::-1]))
    x0 = -b / (2 * a)
    return x0, a * x0 * x0 + b * x0 + c


def main():
    p1, _ = load(F1)
    p2, _ = load(F2)
    m1 = msgs_after_sq(R1, p1)
    m2 = msgs_after_sq(R2, p2)
    off1 = statistics.median(p["utc"] - p["t"] / 1000 for p in p1)
    off2 = statistics.median(p["utc"] - p["t"] / 1000 for p in p2)
    k2 = first_st1(p2)
    T0 = p2[k2]["t"]                              # ch2 時鐘的 T+0
    t2 = lambda t: (t - T0) / 1000                # ch2 tick → T+
    t1 = lambda t: (t + (off1 - off2) * 1000 - T0) / 1000   # ch1 tick → T+

    def row(name, doc, got, note=""):
        print(f"  {name:<34}{doc:>14}{got:>18}   {note}")

    print("=" * 96)
    print("【0】兩板時鐘換算")
    print("=" * 96)
    d = [round((a["utc"] - a["t"] / 1000) - off1, 3) for a in p1]
    print(f"  ch1 → ch2 時鐘偏移 {(off1 - off2):+.3f} s")
    print(f"  ch1 每包 (utc − tick) 對中位數的散佈：{min(d):+.3f} ~ {max(d):+.3f} s（地面站接收延遲的抖動）")

    print()
    print("=" * 96)
    print("【1】T+0 是什麼 —— 「ch1 晚 0.49 s 判定離架」是不是真的")
    print("=" * 96)
    k1 = first_st1(p1)
    for name, ps, k, conv in (("ch1", p1, k1, t1), ("ch2", p2, k2, t2)):
        a, b = ps[k - 1], ps[k]
        print(f"  {name}: 最後 ST:0 = SQ{a['sq']} @T+{conv(a['t']):+.3f}   "
              f"第一個 ST:1 = SQ{b['sq']} @T+{conv(b['t']):+.3f}   "
              f"中間掉了 {b['sq'] - a['sq'] - 1} 包")
    print(f"  → ch1 第一個 ST:1 在 ch2 軸上是 T+{t1(p1[k1]['t']):+.3f}，"
          f"差值 = 掉包 + 2 Hz 量化，不是判定延遲")
    print(f"  → 兩板的「離架判定」都只能框在上面那兩包之間；T+0 比判定晚 0～0.5 s，"
          f"判定又比起動晚約 0.2 s（2.5 g 連續 20 筆＠100 Hz）")

    print()
    print("=" * 96)
    print(f"  {'事件':<34}{'文件':>14}{'log 重算':>18}")
    print("=" * 96)
    for name, ps, conv in (("頂點 ch1", p1, t1), ("頂點 ch2", p2, t2)):
        x, h = apex(ps, conv)
        row(name, "14.50 / 833.7", f"{x:.2f} / {h:.1f}", "RH 5 點拋物線")

    # ch1 開傘訊息夾在哪兩包之間
    for i, s in m1:
        if "auto A+B" in s:
            a, b = p1[i - 1], p1[i]
            row("ch1 A∧B 點火", "16.0～16.5",
                f"{t1(a['t']):.2f}～{t1(b['t']):.2f}",
                f"MSG 在 SQ{a['sq']}(ST:{a['st']}) 與 SQ{b['sq']}(ST:{b['st']}) 之間")
    # ch2 備援：韌體條件是 launch_time + 18 s。
    #   下界一：MSG 出現在 ST:1 那包之後
    #   上界：launch_time ≤ T0（T0 那包已是 ST:1）→ 點火 ≤ T+18.00
    for i, s in m2:
        if "backup timer" in s:
            a, b = p2[i - 1], p2[i]
            row("ch2 備援 C 點火", "18.00", f"{t2(a['t']):.2f}～18.00",
                f"MSG 在 SQ{a['sq']}(ST:{a['st']}) 之後；判定離架 ≤ T0 → 點火 ≤ 18.00")
            # 送 MSG 時 LoRa_Send 阻塞主迴圈（UART 9600 baud，10 bit/byte）。
            # 阻塞結束後那一圈把 t_lora 設成當下 → 下一包 = 阻塞結束 + 500 ms
            blk = len(s.encode()) + len("MSG ") + 2          # 加回 "MSG " 與換行
            end = b["t"] - 500
            row("  └ 依韌體阻塞行為推算", "", f"≈{t2(end - blk * 10000 / 9600):.2f}",
                f"下一包 SQ{b['sq']} 偏離格點 {(b['t'] - T0) % 500} ms；MSG {blk} B ≈ {blk*10/9.6:.0f} ms 阻塞")
    # 事件段的跳拍
    for i in range(1, len(p2)):
        gap = p2[i]["t"] - p2[i - 1]["t"]
        if gap > 600 and 15 < t2(p2[i]["t"]) < 20:
            row("ch2 跳拍", "1010 ms", f"{gap} ms", f"SQ{p2[i-1]['sq']}→SQ{p2[i]['sq']}")
            break
    # 衝擊
    g1 = max((p for p in p1 if 15 < t1(p["t"]) < 20), key=lambda p: p["GA"] or 0)
    g2 = max((p for p in p2 if 15 < t2(p["t"]) < 20), key=lambda p: p["GA"] or 0)
    row("ch1 第一次衝擊", "18.04 / 6.14 g", f"{t1(g1['t']):.2f} / {g1['GA']:.2f} g",
        f"GX {g1['GX']:+.0f} °/s")
    row("ch2 第二次衝擊", "18.52 / 5.62 g", f"{t2(g2['t']):.2f} / {g2['GA']:.2f} g",
        f"GX {g2['GX']:+.0f} °/s")
    row("兩次衝擊間隔", "482 ms", f"{(t2(g2['t']) - t1(g1['t'])) * 1000:.0f} ms",
        "兩板取樣時刻都只到 2 Hz，各自 ±0.25 s")
    # 滾轉
    for want in (16.5, 17.0, 17.5):
        a = min(p1, key=lambda p: abs(t1(p["t"]) - want))
        b = min(p2, key=lambda p: abs(t2(p["t"]) - want))
        row(f"滾轉 @{want}", "", f"{a['GX']:+.0f} / {b['GX']:+.0f} °/s",
            f"ch1 取 T+{t1(a['t']):.2f}、ch2 取 T+{t2(b['t']):.2f}")
    # 穩定下降率
    sel = [p for p in p2 if 22 <= t2(p["t"]) <= 43.6 and p["RH"] is not None]
    if len(sel) > 3:
        xs = [t2(p["t"]) for p in sel]
        ys = [p["RH"] for p in sel]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        row("穩定下降 22→43.6", "32.1 m/s", f"{-slope:.1f} m/s", f"ch2 RH 線性擬合，{len(sel)} 點")

    # 影像對齊（2026-10-15 在直播存檔上實測，數值見 parachute_failure 第 0.1 節）
    #   直播 = youtube.com/watch?v=ogR7rgce_ps 的播放秒數
    V_MOVE = 21293.13                     # 箭開始動
    # +過 2.5 g（0.03~0.07）+20 筆判定（0.19~0.20）+等到 T+0 那包（0.07~0.15）
    V_T0 = 21293.48
    print()
    print("=" * 96)
    print(f"【2】影像對齊：T+0 ＝ 直播 {V_T0:.2f} s（± 0.07），箭開始動在 T{V_MOVE - V_T0:+.2f}")
    print("=" * 96)
    for name, v in (("主傘未張開（細長條）", (21311.47, 21311.54)),
                    ("主傘縮成一團、開始鼓起", (21311.64, 21311.79)),
                    ("主傘攤到最大", (21311.84, 21312.04)),
                    ("已成長條", (21312.49, 21312.54))):
        row(name, "", f"{v[0] - V_T0:.2f}～{v[1] - V_T0:.2f}", f"直播 {v[0]:.2f}～{v[1]:.2f}")
    row("ch1 那筆對應直播", "", f"{t1(g1['t']) + V_T0:.2f}", "畫面：主傘還沒張開、呈細長條")
    row("ch2 那筆對應直播", "", f"{t2(g2['t']) + V_T0:.2f}", "畫面：主傘攤到最大的區間內")


if __name__ == "__main__":
    main()
