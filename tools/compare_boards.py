#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兩塊航電板的交叉比對。

雙板熱備援的價值在這裡具體兌現：兩塊板獨立量同一趟飛行，凡是兩邊一致的
就是物理事實，凡是兩邊不一致的就是某一塊板的問題 —— 單板永遠分不出來。

時鐘對齊：兩塊板各自以「自己偵測到離架」為 T+0，起算點會差幾百毫秒。
用氣壓頂點（同一個物理瞬間、兩邊都量得很準）做對齊基準。
"""
import argparse
import csv
import math

import numpy as np


def load(p):
    r = list(csv.DictReader(open(p, encoding="utf-8")))
    d = {}
    for k in "t_rel rh kh vz ga ax press lat lon".split():
        d[k] = np.array([float(x[k]) if x[k] else np.nan for x in r])
    for k in "st cond sats mod".split():
        d[k] = np.array([int(x[k]) if x[k] else -1 for x in r])
    m = d["t_rel"] >= -2.0
    return {k: v[m] for k, v in d.items()}


def apogee(d, span=3):
    """拋物線擬合氣壓高度峰值 → (t, h)。2Hz 下真峰值不會落在取樣點上。"""
    i = int(np.nanargmax(d["rh"]))
    lo, hi = max(0, i - span), min(len(d["rh"]), i + span + 1)
    a, b, c = np.polyfit(d["t_rel"][lo:hi], d["rh"][lo:hi], 2)
    tp = -b / (2 * a)
    return tp, a * tp * tp + b * tp + c


def slope(t, y, half=1.0):
    o = np.full(len(t), np.nan)
    for i in range(len(t)):
        m = np.abs(t - t[i]) <= half
        if m.sum() >= 3 and not np.isnan(y[m]).any():
            o[i] = np.polyfit(t[m], y[m], 1)[0]
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv1"); ap.add_argument("csv2")
    ap.add_argument("--n1", default="ch1（板 1）"); ap.add_argument("--n2", default="ch2（板 2）")
    a = ap.parse_args()

    A, B = load(a.csv1), load(a.csv2)
    tA, hA = apogee(A); tB, hB = apogee(B)
    off = tB - tA          # B 的時鐘要減掉 off 才對齊到 A

    L = []
    W = L.append
    W("=" * 78)
    W("雙板交叉比對")
    W("=" * 78)
    W(f"\n【時鐘對齊】以氣壓頂點為基準（同一物理瞬間）")
    W(f"  {a.n1} 頂點  T+{tA:.3f}s   {hA:.1f} m")
    W(f"  {a.n2} 頂點  T+{tB:.3f}s   {hB:.1f} m")
    W(f"  → 高度差 {abs(hA-hB):.1f} m（兩顆獨立氣壓計，這是量測可信度的直接證據）")
    W(f"  → 時鐘偏移 {off:+.3f} s（{a.n2} 的 T+ 減去這個值 = {a.n1} 的 T+）")
    W(f"     偏移來源：兩塊板各自偵測離架（2.5g×200ms）的時刻不同 ＋ 各自 2Hz 取樣相位")

    W(f"\n【上升段 —— 兩邊該一致的量】")
    for nm, D in ((a.n1, A), (a.n2, B)):
        i = int(np.nanargmax(D["ga"][D["t_rel"] < 10]))
        vb = slope(D["t_rel"], D["rh"], 1.0)
        W(f"  {nm}")
        W(f"    最大 GA（推力段）  {D['ga'][i]:.2f} g @ T+{D['t_rel'][i]:.2f}s")
        W(f"    頂點氣壓高度       {apogee(D)[1]:.1f} m")
        W(f"    最低壓力           {np.nanmin(D['press']):.2f} hPa")
        W(f"    氣壓導出最大速度   {np.nanmax(vb):.1f} m/s")
        W(f"    卡爾曼最大 VZ      {np.nanmax(D['vz']):.1f} m/s")

    W(f"\n【★ 卡爾曼速度品質 —— 兩邊不該不一致，但它們不一致】")
    W("  判準：頂點時真實垂直速度 = 0。KF 說多少？")
    for nm, D in ((a.n1, A), (a.n2, B)):
        ta, _ = apogee(D)
        vz_ap = float(np.interp(ta, D["t_rel"], D["vz"]))
        vb = slope(D["t_rel"], D["rh"], 1.0)
        vb_ap = float(np.interp(ta, D["t_rel"], vb))
        # KF 的 VZ 何時過零 → 與真頂點的時間差 = 延遲
        m = (D["t_rel"] > ta - 3) & (D["t_rel"] < ta + 6)
        tt, vv = D["t_rel"][m], D["vz"][m]
        tz = np.nan
        for i in range(1, len(tt)):
            if vv[i - 1] > 0 >= vv[i]:
                tz = tt[i - 1] + (tt[i] - tt[i - 1]) * vv[i - 1] / (vv[i - 1] - vv[i])
                break
        W(f"  {nm}")
        W(f"    頂點時 KF 的 VZ      {vz_ap:+7.2f} m/s   （氣壓斜率同時刻 {vb_ap:+.2f}）")
        W(f"    KF 的 VZ 過零時刻    T+{tz:.2f}s   → 落後真頂點 {tz-ta:+.2f} s")

    W(f"\n【開傘序列（全部換算到 {a.n1} 的時間軸）】")
    ev = []
    for nm, D, o in ((a.n1, A, 0.0), (a.n2, B, off)):
        for s, lab in ((2, "ST->2 DEPLOYING"), (3, "ST->3 DEPLOYED")):
            idx = np.where(D["st"] == s)[0]
            if len(idx):
                ev.append((D["t_rel"][idx[0]] - o, f"{nm}  {lab}",
                           f"RH={D['rh'][idx[0]]:.1f}m VZ={D['vz'][idx[0]]:+.2f} GA={D['ga'][idx[0]]:.2f}g"))
        for bit, lab in ((3, "cond_A"), (1, "cond_B")):
            idx = np.where((D["cond"] >> bit) & 1 == 1)[0]
            if len(idx):
                ev.append((D["t_rel"][idx[0]] - o, f"{nm}  {lab} 成立",
                           f"RH={D['rh'][idx[0]]:.1f}m VZ={D['vz'][idx[0]]:+.2f}"))
        i = int(np.nanargmax(D["ga"][D["t_rel"] > 10])) + int((D["t_rel"] <= 10).sum())
        ev.append((D["t_rel"][i] - o, f"{nm}  開傘後最大 GA {D['ga'][i]:.2f} g",
                   f"RH={D['rh'][i]:.1f}m"))
    ev.append((tA, "── 氣壓頂點（兩板一致）──", f"{hA:.1f} m"))
    for t, lab, extra in sorted(ev):
        W(f"  T+{t:6.2f}s   {lab:34s} {extra}")

    W(f"\n【遙測涵蓋範圍】")
    for nm, D, o in ((a.n1, A, 0.0), (a.n2, B, off)):
        W(f"  {nm}  最後一包 T+{D['t_rel'][-1]-o:.2f}s（{a.n1} 時間軸）"
          f"  高度 {D['rh'][-1]:.1f} m")
    W(f"  模組健康變化：")
    for nm, D, o in ((a.n1, A, 0.0), (a.n2, B, off)):
        prev = None
        for i in range(len(D["mod"])):
            if D["mod"][i] != prev:
                m = D["mod"][i]
                W(f"    {nm}  T+{D['t_rel'][i]-o:8.2f}s  MOD:{m:X}"
                  f"  bmp={(m>>3)&1} imu={(m>>2)&1} lora={(m>>1)&1} sd={m&1}")
                prev = m

    W(f"\n【GPS 交叉比對 —— 兩顆獨立接收機】")
    for nm, D, o in ((a.n1, A, 0.0), (a.n2, B, off)):
        g = ~np.isnan(D["lat"]) & (D["t_rel"] >= 0)
        lat, lon, tt = D["lat"][g], D["lon"][g], D["t_rel"][g] - o
        R = 6371000.0
        e = np.radians(lon - lon[0]) * R * math.cos(math.radians(lat[0]))
        n = np.radians(lat - lat[0]) * R
        W(f"  {nm}")
        prev = 0
        for i in range(1, len(tt)):
            if lat[i] == lat[prev] and lon[i] == lon[prev]:
                continue
            d = math.hypot(e[i] - e[prev], n[i] - n[prev]); dt = tt[i] - tt[prev]
            v = d / dt if dt > 0 else 0
            if v > 80 or i == len(tt) - 1:
                tag = "🔴 跳點" if v > 80 else "最後一包"
                W(f"    T+{tt[i]:6.2f}s  E{e[i]:+7.1f} N{n[i]:+7.1f}  sats={D['sats'][g][i]:2d}"
                  f"  {v:6.1f} m/s  {tag}")
            prev = i

    print("\n".join(L))
    return "\n".join(L)


if __name__ == "__main__":
    main()
