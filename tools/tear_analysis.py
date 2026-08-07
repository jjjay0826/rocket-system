#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判斷主傘是否在降落過程中撕裂。

## 撕裂會留下兩種可量的痕跡

**① 下降率階梯式變快。**阻力面積一旦減少，終端速度立刻上升
（v ∝ 1/√(CdA)）。撕裂是突發事件，所以會是階梯不是斜坡。
把降落段切成小窗口逐段擬合斜率，看有沒有階梯。

**② 傘衣投影面積下降。**但**不能直接比像素數** —— 火箭在接近相機，
距離縮短會讓像素數自然變大。要乘上距離平方才是真實面積：

    A_true ∝ A_pixel × R²

距離 R 由遙測高度 ＋ 彈道重建的順航向位移求得（GPS 上升段不可用）。

## 判讀

| 觀察 | 結論 |
|---|---|
| 下降率有階梯 ＋ 面積在同一時刻下降 | **撕裂**，且時間點可定 |
| 下降率從頭到尾平坦 ＋ 面積不變 | **從未充氣**（streamer），不是撕裂 |
| 下降率平坦但面積本來就小 | 同上 |
| 下降率先慢後快 | 先充氣、後失效 |
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
from PIL import Image, ImageFilter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

BANNER_TOP, BANNER_BOT = 0.16, 0.88


def canopy_pixels(im, chroma=-8.0, grid=32, min_px=25):
    """回傳 (像素數, 寬, 高, 質心)。只算彩色傘衣，不含白色箭身。"""
    a = np.asarray(im.filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)
    s = a[:, :, 2] - np.maximum(a[:, :, 0], a[:, :, 1])
    m = s < chroma
    h, w = m.shape
    m[:int(h * BANNER_TOP), :] = False
    m[int(h * BANNER_BOT):, :] = False
    if m.sum() < min_px:
        return None
    cell = np.zeros((grid, grid), int)
    for r in range(grid):
        for c in range(grid):
            cell[r, c] = m[r*h//grid:(r+1)*h//grid, c*w//grid:(c+1)*w//grid].sum()
    pr, pc = np.unravel_index(int(np.argmax(cell)), cell.shape)
    win = np.zeros_like(m)
    win[max(0, pr-3)*h//grid:min(grid, pr+4)*h//grid,
        max(0, pc-3)*w//grid:min(grid, pc+4)*w//grid] = True
    m &= win
    if m.sum() < min_px:
        return None
    ys, xs = np.where(m)
    return (int(m.sum()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1),
            (float(xs.mean()), float(ys.mean())))


def descent_steps(csv_path, t_start, win=2.0, step=0.5):
    """滑動窗口擬合下降率。回傳 [(t_中心, 下降率 m/s, 窗內點數)]。"""
    import csv as _csv
    rows = [r for r in _csv.DictReader(open(csv_path, encoding="utf-8"))
            if r["t_rel"] and float(r["t_rel"]) >= t_start]
    t = np.array([float(r["t_rel"]) for r in rows])
    h = np.array([float(r["rh"]) for r in rows])
    out = []
    tc = t.min() + win / 2
    while tc + win / 2 <= t.max():
        m = np.abs(t - tc) <= win / 2
        if m.sum() >= 4:
            out.append((tc, -np.polyfit(t[m], h[m], 1)[0], int(m.sum())))
        tc += step
    return out


def main():
    ap = argparse.ArgumentParser(description="判斷主傘是否撕裂")
    ap.add_argument("--frames", required=True, help="60fps 幀的 glob")
    ap.add_argument("--telemetry", required=True)
    ap.add_argument("--fps", type=float, default=59.94)
    ap.add_argument("--t0", type=float, default=15.5)
    ap.add_argument("--every", type=int, default=6)
    ap.add_argument("--deploy", type=float, default=18.04, help="開傘衝擊時刻")
    a = ap.parse_args()

    print("=" * 72)
    print("【一】下降率有沒有階梯 —— 撕裂會讓阻力面積突然減少")
    print("=" * 72)
    ds = descent_steps(a.telemetry, a.deploy + 1.0)
    print(f"  {'時間':>8}{'下降率':>10}{'點數':>6}   （2 秒滑動窗，每 0.5 秒一格）")
    for tc, v, n in ds:
        bar = "█" * int(v / 1.2)
        print(f"  T+{tc:6.2f}{v:9.1f} m/s{n:6d}   {bar}")
    vs = np.array([v for _, v, _ in ds])
    print(f"\n  平均 {vs.mean():.1f} m/s   標準差 {vs.std():.1f}   "
          f"範圍 {vs.min():.1f}~{vs.max():.1f}")
    # 最大單步變化
    d = np.abs(np.diff(vs))
    i = int(np.argmax(d))
    print(f"  最大單步變化 {d[i]:.1f} m/s（T+{ds[i][0]:.2f}→{ds[i+1][0]:.2f}）")
    print(f"  線性趨勢 {np.polyfit([t for t,_,_ in ds], vs, 1)[0]:+.2f} m/s per s")

    print()
    print("=" * 72)
    print("【二】傘衣投影面積 —— 要扣掉距離變化")
    print("=" * 72)
    import csv as _csv
    rows = [r for r in _csv.DictReader(open(a.telemetry, encoding="utf-8")) if r["t_rel"]]
    tt = np.array([float(r["t_rel"]) for r in rows])
    hh = np.array([float(r["rh"]) for r in rows])

    files = sorted(glob.glob(a.frames))[::a.every]
    print(f"  掃描 {len(files)} 幀…")
    res = []
    for k, f in enumerate(files):
        t = a.t0 + k * a.every / a.fps
        r = canopy_pixels(Image.open(f).convert("RGB"))
        if r is None:
            continue
        alt = float(np.interp(t, tt, hh))
        # 順航向位移：開傘後水平速度很快被阻力吃掉，取固定 800 m
        rng = math.hypot(alt, 800.0)
        res.append((t, r[0], r[1], r[2], alt, rng))

    print(f"  偵測到 {len(res)} 幀\n")
    print(f"  {'時間':>8}{'像素數':>8}{'寬':>5}{'高':>5}{'長寬比':>7}"
          f"{'高度':>8}{'斜距':>8}{'距離校正面積':>13}")
    base = None
    for t, n, w, h, alt, rng in res:
        corr = n * (rng / 1000.0) ** 2
        if base is None:
            base = corr
        print(f"  T+{t:6.2f}{n:8d}{w:5d}{h:5d}{h/max(w,1):7.2f}"
              f"{alt:8.0f}{rng:8.0f}{corr:11.0f} ({corr/base*100:3.0f}%)")

    if len(res) >= 6:
        ts = np.array([r[0] for r in res])
        ca = np.array([r[1] * (r[5] / 1000.0) ** 2 for r in res])
        sl = np.polyfit(ts, ca, 1)[0]
        print(f"\n  距離校正後面積：平均 {ca.mean():.0f}，標準差 {ca.std():.0f}"
              f"（{ca.std()/ca.mean()*100:.0f}%）")
        print(f"  線性趨勢 {sl:+.1f} per s（{sl/ca.mean()*100:+.1f}% per s）")
        d = np.abs(np.diff(ca)) / ca[:-1]
        i = int(np.argmax(d))
        print(f"  最大單步變化 {d[i]*100:.0f}%（T+{ts[i]:.2f}→{ts[i+1]:.2f}）")

    print()
    print("=" * 72)
    print("判讀指引")
    print("=" * 72)
    print("  撕裂 → 下降率有階梯 ＋ 同一時刻面積下降")
    print("  從未充氣 → 下降率平坦 ＋ 面積小且不變")
    print("  先充氣後失效 → 下降率先慢後快")
    print("  ※ 面積的散布本來就大（傘在擺盪、投影角一直變），")
    print("    要看的是有沒有【單向的階梯】，不是散布本身。")


if __name__ == "__main__":
    main()
