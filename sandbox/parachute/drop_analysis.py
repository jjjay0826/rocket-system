# -*- coding: utf-8 -*-
"""
drop_analysis.py — 降落傘投放 CSV 分析（rocket parachute 韌體專用）

用法：  python drop_analysis.py F:\IMU_002.CSV

原理：終端速度是等速下降 → 對「傘下穩態段」的氣壓高度做線性回歸，
      斜率＝下降速度（不經 KF/姿態/加速度計，最乾淨的量測路徑）。
事件偵測全用 g_tot（釋放=失重、開傘=衝擊峰、落地=最大峰）。
相容新舊欄名（kf_vz_ms/vz_ms、kf_h_m/imu_hz_m）。
"""
import csv, sys, math

def main(path):
    rows = []
    with open(path, newline='') as f:
        rdr = csv.reader(f)
        hdr = next(rdr)
        for ln in rdr:
            try:
                rows.append([float(x) for x in ln])
            except ValueError:
                pass  # 過濾偶發損毀列（寫入恢復邊界）
    ix = {n: i for i, n in enumerate(hdr)}
    t = [r[0] / 1e6 for r in rows]
    g = [r[ix['g_tot']] for r in rows]
    b = [r[ix['baro_m']] for r in rows]
    n = len(rows)
    print(f"檔案: {path}")
    print(f"樣本: {n} 筆, 時長 {t[-1]-t[0]:.1f}s")

    # ── 釋放：法一＝連續 0.2s（100 筆）g_tot < 0.3（失重，慢開傘）──
    rel = None
    how = "失重偵測"
    for i in range(n - 100):
        if all(x < 0.3 for x in g[i:i+100]):
            rel = i
            break
    if rel is None:
        # 法二（快開傘備援）：傘立刻張開時 g 不會掉到 0.3 以下，
        # 改用氣壓：高度跌破「歷史滾動最高點 − 2m」＝已在下降，
        # 釋放點＝該滾動最高點出現的位置
        how = "氣壓下降偵測（快開傘，無失重段）"
        # 用 0.2s 平滑氣壓判斷（原始噪聲 σ≈0.15m 會誤觸發；平滑後 ≈0.015m）
        cb = [0.0]
        for x in b:
            cb.append(cb[-1] + x)
        def b_sm(i, W=100):
            lo, hi = max(0, i - W // 2), min(n, i + W // 2)
            return (cb[hi] - cb[lo]) / (hi - lo)
        bmax, bmax_i = b_sm(0), 0
        for i in range(n):
            v = b_sm(i)
            if v > bmax:
                bmax, bmax_i = v, i
            elif bmax - v > 2.0:
                # 觸發後從高原點往前找「實際開始下降」處（跌破 0.3m）
                rel = bmax_i
                for j in range(bmax_i, i):
                    if bmax - b_sm(j) > 0.3:
                        rel = j
                        break
                break
    if rel is None:
        print("!! 找不到釋放點（無失重段、也無 >2m 下降）— 這個檔可能不是投放紀錄")
        return

    # ── 落地：釋放後 1~30s 內的最大 g 峰
    #   （限 30s：排除落地很久之後撿板子/磕碰的假峰）──
    imp = max(range(rel + 500, min(n, rel + 15000)), key=lambda i: g[i])
    # ── 開傘衝擊：釋放與落地之間（扣掉落地前 0.2s）的最大 g 峰 ──
    op = max(range(rel, imp - 100), key=lambda i: g[i])

    print(f"\n── 事件 ──")
    print(f"釋放   t={t[rel]:.2f}s  高度 {b[rel]:+.1f}m（{how}）")
    print(f"開傘   t={t[op]:.2f}s（釋放後 {t[op]-t[rel]:.2f}s）"
          f" 高度 {b[op]:+.1f}m  衝擊 {g[op]:.1f}g"
          f"  ← 距落地點還有 {b[op]-b[imp]:.1f}m")
    print(f"落地   t={t[imp]:.2f}s（釋放後 {t[imp]-t[rel]:.2f}s）"
          f" 高度 {b[imp]:+.1f}m  撞擊 {g[imp]:.1f}g")
    print(f"總墜落 {b[rel]-b[imp]:.1f}m / {t[imp]-t[rel]:.2f}s")

    # ── 穩態段：從「落地往回」取（越接近地面越收斂），
    #    範圍 = 開傘衝擊後 0.5s ～ 落地前 0.3s，且 g_tot 在 0.7~1.4
    #   （傘下平衡＝阻力≈重力）。錨定尾端而非頭端：頭端緊貼開傘，
    #    常含減速暫態，會把終端速度灌高。──
    s1 = imp - 150                     # 落地前 0.3s（避開觸地）
    s_min = op + 250                   # 最早不超過開傘後 0.5s
    # g 用 0.1s 滑動平均再判帶：500Hz 原始值在鐘擺擺盪下瞬時超帶，
    # 會讓擴張立刻中斷（IMU_002 實案）
    W = 50
    cg = [0.0]
    for x in g:
        cg.append(cg[-1] + x)
    def g_smooth(i):
        lo, hi = max(0, i - W // 2), min(n, i + W // 2)
        return (cg[hi] - cg[lo]) / (hi - lo)
    # 帶寬 0.5~2.0：只為排除自由落體(≈0.1)/開傘衝擊(≥2.5)/觸地，
    # 傘下鐘擺會讓平滑 g 在 ~0.8~1.5 擺盪（IMU_002 實測），屬正常物理
    s0 = s1
    while s0 > s_min and (0.5 < g_smooth(s0 - 1) < 2.0):
        s0 -= 1                        # 從尾端往回擴張，平滑 g 出帶即停
    # 抽稀到 100Hz（每 5 筆取 1）：氣壓實際 100Hz 更新、CSV 500Hz 記錄
    # → 每值重複 5 筆；不抽稀會把重複樣本當獨立資料，標準誤低估 √5 倍
    seg_t = [t[i] for i in range(s0, s1, 5)]
    seg_b = [b[i] for i in range(s0, s1, 5)]
    dur = seg_t[-1] - seg_t[0] if len(seg_t) > 20 else 0.0

    print(f"\n── 終端速度（氣壓法）──")
    if dur < 0.5:
        print("!! 穩態段不足 0.5s — 傘開太晚，這筆量不出終端速度。")
        print("   （提高投放點或加快開傘，穩態 ≥3s 才能給 ±0.1~0.2 m/s）")
        return

    def fit(ts, bs):
        """最小二乘回歸 → (斜率, 斜率標準誤)"""
        m = len(ts)
        tm = sum(ts) / m
        bm = sum(bs) / m
        sxx = sum((x - tm) ** 2 for x in ts)
        sxy = sum((x - tm) * (y - bm) for x, y in zip(ts, bs))
        slope = sxy / sxx
        resid = [y - (bm + slope * (x - tm)) for x, y in zip(ts, bs)]
        se = math.sqrt(sum(r * r for r in resid) / max(m - 2, 1) / sxx)
        return slope, se

    slope, se = fit(seg_t, seg_b)
    # ── 收斂檢查：前半 vs 後半斜率。仍在減速時前半會明顯更快，
    #    此時「後半」才是較接近終端速度的估計。──
    h = len(seg_t) // 2
    v1, _ = fit(seg_t[:h], seg_b[:h])
    v2, se2 = fit(seg_t[h:], seg_b[h:])

    print(f"穩態段 {seg_t[0]:.2f}~{seg_t[-1]:.2f}s（{dur:.2f}s, {len(seg_t)} 筆）")
    if abs(v1) - abs(v2) > 0.3:        # 前半比後半快 >0.3 m/s ＝ 還在減速
        print(f"⚠ 段內仍在減速（前半 {abs(v1):.2f} → 後半 {abs(v2):.2f} m/s），未達穩態。")
        print(f"★ 終端速度 ≲ {abs(v2):.2f} ± {2*se2:.2f} m/s（取後半，仍屬上界）")
        print(f"  （整段平均 {abs(slope):.2f} 含暫態，勿採用）")
    else:
        print(f"★ 終端速度 = {abs(slope):.2f} ± {2*se:.2f} m/s（95% 信賴）")
        if dur < 3.0:
            print(f"⚠ 穩態只有 {dur:.1f}s（<3s）：擺盪未完全平均掉，")
            print(f"  建議下次留 ≥3s 穩態再定案。")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(r"用法: python drop_analysis.py F:\IMU_00X.CSV")
        sys.exit(1)
    main(sys.argv[1])
