#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在整幅畫面裡找到傘／火箭，裁切、疊幀降噪、放大。

## 為什麼要疊幀

單張放大**不會增加資訊**。但影片有一件事單張沒有：同一個物體被拍了很多次，
而 JPEG／H.264 的壓縮雜訊在每一幀是獨立的。把 N 張對齊後平均，雜訊降
√N 倍，而訊號不變 —— **這是唯一真正提升畫質的手法**。

60 fps 下相鄰 5 幀只差 83 ms，傘幾乎沒移動，對齊後疊起來很有效。

## 裁切範圍

★ 只框住傘是不夠的：分離的箭身在傘的**下方**拖很長。所以偵測到傘之後，
  框要往下延伸到畫面可用區的底部（避開台標橫幅），不要只加固定 margin。

## 判據

    skyness = B - max(R, G)
    藍天 大正值（約 +40）／白雲、箭身 約 0／暖色傘衣 負值（約 -30）

直播畫面上下有台標與贊助商橫幅，它們也是強彩色而且像素數是火箭的 10 倍，
所以先遮掉帶狀區，再用粗網格取最密集的一格。
"""
import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageFilter, ImageEnhance

BANNER_TOP = 0.23      # 上方台標帶（實測延伸到 y約240，留安全邊）
BANNER_BOT = 0.82      # 下方贊助商帶起點（實測 y>=900 就有橫幅，留安全邊）


def canopy_box(im, chroma=-8.0, min_px=25, grid=32):
    """找傘衣（彩色像素團）的包圍盒。回傳 (x0,y0,x1,y1) 或 None。"""
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
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def centroid(im, chroma=-8.0):
    b = canopy_box(im, chroma)
    if b is None:
        return None
    return ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)


def suppress_sky(arr, soft=14.0, dead=0.18):
    """天空壓白、物體拉對比。arr: HxWx3 float。"""
    im = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    blur = np.asarray(im.filter(ImageFilter.GaussianBlur(1.2))).astype(np.float32)
    s = blur[:, :, 2] - np.maximum(blur[:, :, 0], blur[:, :, 1])
    m = np.clip((np.median(s) - s) / soft, 0, 1)
    m = np.clip((m - dead) / (1 - dead), 0, 1)
    sel = m > 0.15
    out = arr.copy()
    if sel.sum() > 20:
        for c in range(3):
            ch = arr[:, :, c]
            lo, hi = np.percentile(ch[sel], 2), np.percentile(ch[sel], 98)
            if hi - lo > 1e-6:
                out[:, :, c] = np.clip((ch - lo) / (hi - lo), 0, 1) * 255.0
    return out * m[:, :, None] + 252.0 * (1 - m[:, :, None])


def main():
    ap = argparse.ArgumentParser(description="追蹤裁切 ＋ 疊幀降噪 ＋ 放大")
    ap.add_argument("pattern")
    ap.add_argument("--out-dir", default="tracked")
    ap.add_argument("--pad-x", type=int, default=120, help="左右外擴像素")
    ap.add_argument("--pad-up", type=int, default=90, help="向上外擴像素")
    ap.add_argument("--win-w", type=int, default=420, help="裁切窗寬")
    ap.add_argument("--win-h", type=int, default=560, help="裁切窗高（要夠高才裝得下拖曳的箭身）")
    ap.add_argument("--upscale", type=float, default=3.0, help="放大倍率")
    ap.add_argument("--stack", type=int, default=5,
                    help="疊幀張數（奇數；1 = 不疊）。這是唯一真正降噪的手法")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--sat", type=float, default=1.45)
    ap.add_argument("--max-shift", type=int, default=220,
                    help="疊幀時容許的最大對齊位移；相機搖鏡時要放寬")
    ap.add_argument("--sky", action="store_true", help="額外輸出天空抑制版")
    a = ap.parse_args()

    files = sorted(glob.glob(a.pattern))
    if not files:
        raise SystemExit(f"找不到 {a.pattern}")
    os.makedirs(a.out_dir, exist_ok=True)

    print("【1/3】掃描傘衣位置")
    cents, boxes = {}, {}
    for i, f in enumerate(files):
        im = Image.open(f).convert("RGB")
        b = canopy_box(im)
        if b:
            boxes[i] = b
            cents[i] = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
    if not boxes:
        raise SystemExit("整段都沒偵測到傘衣")
    W, H = Image.open(files[0]).size
    bs = np.array(list(boxes.values()))
    print(f"  偵測到 {len(boxes)}/{len(files)} 幀")
    print(f"  傘衣尺寸：寬 {bs[:,2].min()-bs[:,0].min():.0f}~"
          f"{(bs[:,2]-bs[:,0]).max():.0f} px，高 {(bs[:,3]-bs[:,1]).min():.0f}~"
          f"{(bs[:,3]-bs[:,1]).max():.0f} px")

    # ★ 逐幀跟隨裁切：物體橫跨整幅（相機在搖），用單一固定框會鬆到沒有放大效果。
    #   窗口把傘放在上方 1/4 處，下面 3/4 留給拖曳的箭身 —— 那些在傘下方，
    #   只框傘就會把它們切掉（這是前一版的錯）。
    CW, CH = a.win_w, a.win_h
    def frame_box(i):
        """把傘放在窗口上方 25%，下面 75% 留給拖曳的箭身。
        y0 夾在台標帶之下、贊助商帶之上 —— 否則橫幅會進到裁切裡，
        疊幀時還會因為對齊位移而變成疊影。"""
        if i not in boxes:
            return None
        b = boxes[i]
        cx = (b[0] + b[2]) // 2
        y0 = int(np.clip(b[1] - 0.25 * CH,
                         int(H * BANNER_TOP), max(int(H * BANNER_TOP), int(H * BANNER_BOT) - CH)))
        x0 = int(np.clip(cx - CW // 2, 0, W - CW))
        return x0, y0, x0 + CW, y0 + CH
    print(f"  裁切窗口 {CW}x{CH} px（逐幀跟隨；傘在上方 1/4，下方留給箭身）")

    print(f"\n【2/3】疊幀降噪（每張疊 {a.stack} 幀，對齊後平均）")
    half = a.stack // 2
    idxs = list(range(0, len(files), a.every))
    out_imgs = []
    for i in idxs:
        base = np.asarray(Image.open(files[i]).convert("RGB")).astype(np.float32)
        acc, n = base.copy(), 1
        if a.stack > 1 and i in cents:
            cx, cy = cents[i]
            for j in range(i - half, i + half + 1):
                if j == i or j < 0 or j >= len(files) or j not in cents:
                    continue
                dx, dy = cx - cents[j][0], cy - cents[j][1]
                if abs(dx) > a.max_shift or abs(dy) > a.max_shift:   # 太遠不疊，免得糊掉
                    continue
                other = Image.open(files[j]).convert("RGB").transform(
                    (W, H), Image.AFFINE, (1, 0, -dx, 0, 1, -dy), Image.BICUBIC)
                acc += np.asarray(other).astype(np.float32)
                n += 1
        stacked = acc / n
        out_imgs.append((i, stacked, n))

    print(f"\n【3/3】裁切、放大 {a.upscale}x、銳化")
    for i, arr, n in out_imgs:
        fb = frame_box(i)
        if fb is None:
            continue                       # 沒偵測到就不輸出，別用過期的框硬裁
        bx0, by0, bx1, by1 = fb
        crop = arr[by0:by1, bx0:bx1]
        for tag, data in (("", crop),) + ((("_sky", suppress_sky(crop)),) if a.sky else ()):
            p = Image.fromarray(np.clip(data, 0, 255).astype(np.uint8))
            p = p.resize((int(p.width * a.upscale), int(p.height * a.upscale)),
                         Image.LANCZOS)
            if not tag:
                p = ImageEnhance.Color(p).enhance(a.sat)
                p = ImageEnhance.Contrast(p).enhance(1.12)
            p = p.filter(ImageFilter.UnsharpMask(radius=2.5, percent=150, threshold=2))
            p.save(os.path.join(a.out_dir,
                                os.path.splitext(os.path.basename(files[i]))[0] + tag + ".png"))
    # ★ 檢查底部（與四邊）有沒有把物體切掉 —— 用猜的不行，要量
    def edge_hits(arr, bx0, by0, bx1, by1, band=6):
        c = arr[by0:by1, bx0:bx1]
        sk = c[:, :, 2] - np.maximum(c[:, :, 0], c[:, :, 1])
        obj = sk < np.median(sk) - 40          # 只算明顯的物體，薄雲不算
        return dict(下=bool(obj[-band:, :].any()), 上=bool(obj[:band, :].any()),
                    左=bool(obj[:, :band].any()), 右=bool(obj[:, -band:].any()))
    cut = {k: 0 for k in "下上左右"}
    nvalid = 0
    for i, arr, _ in out_imgs:
        fb = frame_box(i)
        if fb is None:
            continue
        nvalid += 1
        for k, v in edge_hits(arr, *fb).items():
            cut[k] += int(v)
    print("  裁切邊界檢查（該邊有物體 = 被切掉）：",
          "  ".join(f"{k}{v}/{nvalid}" for k, v in cut.items()))
    if cut["下"] > nvalid * 0.2:
        print("  ⚠ 下緣經常有物體 → 加大 --win-h 或 --pad-up 調小")

    nn = [n for _, _, n in out_imgs]
    print(f"  實際疊幀數：中位 {int(np.median(nn))}，範圍 {min(nn)}~{max(nn)}")
    print(f"  輸出尺寸 {int(a.win_w*a.upscale)}x{int(a.win_h*a.upscale)}")
    print(f"\n→ {len(out_imgs)} 張在 {a.out_dir}/")


if __name__ == "__main__":
    main()
