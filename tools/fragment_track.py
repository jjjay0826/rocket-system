#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""追蹤傘衣的【每一個】碎塊，判斷有沒有東西真的脫離。

## 為什麼要多物體，不能用單一 blob

先前的 canopy_pixels() 只取最大的一團，所以「有沒有碎塊分離」這個問題
它結構上就答不出來 —— 碎塊永遠不是最大的那團，會被丟掉。

## 決定性判據

撕下來的布片與傘衣的**彈道係數差好幾個數量級**（一塊布 vs 25 kg 的箭）。
所以：

| 觀察 | 結論 |
|---|---|
| 相對位移**單調增大**，且增速與空氣阻力一致 | **真的脫離**（撕裂） |
| 相對位移在固定範圍內來回 | 附著的塌陷褶皺（扭轉） |

位移要用**傘衣質心當原點**，不能用畫面座標 —— 相機在搖。

## 尺度標定

不靠相機參數。用遙測已知的下降率（32.1 m/s）與斜距，
由傘衣質心的逐幀像素位移直接反解 px/m。
"""
import argparse
import glob
import math
import os
from collections import deque

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

FPS, T0 = 59.94, 15.5


def warm_mask(a, thr=-8.0):
    """暖色物體遮罩。s = B - max(R,G)，天空 s 很大，傘衣 s 為負。"""
    return (a[:, :, 2] - np.maximum(a[:, :, 0], a[:, :, 1])) < thr


def components(m, min_px=6):
    """連通元件（4-鄰接，BFS）。環境沒有 scipy，手寫。"""
    H, W = m.shape
    lab = np.zeros((H, W), np.int32)
    out, nxt = [], 0
    ys, xs = np.where(m)
    for y0, x0 in zip(ys, xs):
        if lab[y0, x0]:
            continue
        nxt += 1
        q = deque([(y0, x0)])
        lab[y0, x0] = nxt
        pix = []
        while q:
            y, x = q.popleft()
            pix.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                v, u = y + dy, x + dx
                if 0 <= v < H and 0 <= u < W and m[v, u] and not lab[v, u]:
                    lab[v, u] = nxt
                    q.append((v, u))
        if len(pix) < min_px:
            continue
        P = np.array(pix)
        out.append(dict(n=len(pix), cy=P[:, 0].mean(), cx=P[:, 1].mean(),
                        y0=P[:, 0].min(), y1=P[:, 0].max(),
                        x0=P[:, 1].min(), x1=P[:, 1].max()))
    return sorted(out, key=lambda d: -d["n"])


def locate(path, box=None, thr=-8.0):
    """在整幀（或給定框內）找暖色物體。回傳 (元件列表, 偏移)。"""
    im = Image.open(path).convert("RGB")
    ox, oy = 0, 0
    if box:
        im = im.crop(box)
        ox, oy = box[0], box[1]
    a = np.asarray(im.filter(ImageFilter.GaussianBlur(0.8))).astype(np.float32)
    m = warm_mask(a, thr)
    m[:int(m.shape[0] * 0.14), :] = False          # 上方記分帶
    m[int(m.shape[0] * 0.82):, :] = False          # 下方贊助商橫幅
    return components(m), (ox, oy)


def main():
    ap = argparse.ArgumentParser(description="追蹤傘衣碎塊，判斷是否真的脫離")
    ap.add_argument("--frames", required=True)
    ap.add_argument("--t-start", type=float, default=18.30)
    ap.add_argument("--t-end", type=float, default=19.40)
    ap.add_argument("--step", type=int, default=2, help="每 N 幀取一格")
    ap.add_argument("--win", type=int, default=260, help="裁切窗邊長")
    ap.add_argument("--up", type=int, default=4)
    ap.add_argument("--out", default="fragment_sheet.png")
    ap.add_argument("--thr", type=float, default=-8.0)
    a = ap.parse_args()

    files = sorted(glob.glob(a.frames))
    i0 = round((a.t_start - T0) * FPS)
    i1 = round((a.t_end - T0) * FPS)

    # ── 先在全幀找一次，定出追蹤窗 ──
    comp, _ = locate(files[i0], thr=a.thr)
    if not comp:
        raise SystemExit("起始幀找不到暖色物體，調 --thr 或 --t-start")
    cx, cy = comp[0]["cx"], comp[0]["cy"]
    print(f"起始 T+{a.t_start:.2f}  主體中心 ({cx:.0f},{cy:.0f})  {comp[0]['n']} px")

    print()
    print("=" * 96)
    print("每幀所有暖色元件（以主體質心為原點的相對位置）")
    print("=" * 96)
    print(f"{'T+':>7}{'#':>3}{'像素':>7}{'寬':>5}{'高':>5}{'長寬比':>7}"
          f"{'相對dx':>8}{'相對dy':>8}{'距離':>7}   說明")

    rows, tiles = [], []
    for i in range(i0, min(i1 + 1, len(files)), a.step):
        t = T0 + i / FPS
        # win<=0 或物體跑太快時一律全幀搜尋 —— 追蹤窗跟丟會直接讓判據失效
        comp, (ox, oy) = locate(files[i], None, a.thr)
        box = (int(cx - a.win // 2), int(cy - a.win // 2),
               int(cx + a.win // 2), int(cy + a.win // 2))
        if not comp:
            print(f"{t:7.2f}  —— 窗內無物體 ——")
            continue
        main_c = comp[0]
        mx, my = main_c["cx"] + ox, main_c["cy"] + oy
        for k, c in enumerate(comp[:5]):
            gx_, gy_ = c["cx"] + ox, c["cy"] + oy
            dx, dy = gx_ - mx, gy_ - my
            w, h = c["x1"] - c["x0"] + 1, c["y1"] - c["y0"] + 1
            tag = "主體" if k == 0 else ""
            print(f"{t:7.2f}{k:3d}{c['n']:7d}{w:5d}{h:5d}{h/max(w,1):7.2f}"
                  f"{dx:8.1f}{dy:8.1f}{math.hypot(dx,dy):7.1f}   {tag}")
            rows.append((t, k, c["n"], w, h, dx, dy, math.hypot(dx, dy)))
        # 追蹤窗跟著主體走（緩慢，避免被碎塊帶跑）
        cx += (mx - cx) * 0.6
        cy += (my - cy) * 0.6

        im = Image.open(files[i]).convert("RGB").crop(box)
        im = im.resize((a.win * a.up, a.win * a.up), Image.LANCZOS)
        tiles.append((t, im, comp, ox, oy))

    # ── 接觸表 ──
    if tiles:
        cols = 6
        rowsn = (len(tiles) + cols - 1) // cols
        tw, th = tiles[0][1].size
        bar = 34
        sheet = Image.new("RGB", (cols * tw, rowsn * (th + bar)), (250, 248, 243))
        d = ImageDraw.Draw(sheet)
        try:
            font = ImageFont.truetype("msjh.ttc", 26)
        except Exception:
            font = ImageFont.load_default()
        for j, (t, im, comp, ox, oy) in enumerate(tiles):
            X, Y = (j % cols) * tw, (j // cols) * (th + bar)
            sheet.paste(im, (X, Y + bar))
            d.text((X + 6, Y + 4), f"T+{t:.3f}", fill=(27, 42, 65), font=font)
        sheet.save(a.out)
        print(f"\n→ 接觸表 {a.out}  {sheet.size[0]}x{sheet.size[1]}  共 {len(tiles)} 格")

    # ── 分離判據 ──
    print()
    print("=" * 96)
    print("分離判據：非主體元件的相對距離隨時間變化")
    print("=" * 96)
    sec = {}
    for t, k, n, w, h, dx, dy, dist in rows:
        if k == 0:
            continue
        sec.setdefault(round(t, 3), []).append((n, dist))
    print(f"{'T+':>8}{'次要元件數':>12}{'最大次要px':>12}{'最遠距離px':>12}")
    for t in sorted(sec):
        v = sec[t]
        print(f"{t:8.3f}{len(v):12d}{max(x[0] for x in v):12d}{max(x[1] for x in v):12.1f}")


if __name__ == "__main__":
    main()
