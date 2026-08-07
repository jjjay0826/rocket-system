#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把天空擬合掉，讓傘衣、傘繩、箭身全部浮出來。

## 為什麼要擬合天空而不是「調對比」

單純拉對比會把天空的雜訊一起放大。天空其實是一個**平滑的二次曲面**
（越靠近太陽越亮、越靠地平線越淡）。把它擬合掉之後，剩下的殘差就
只有物體 —— 這時再拉對比，放大的才是訊號。

## 為什麼要兩個通道

|         | 色度 R−B | 亮度殘差 |
|---------|---------|---------|
| 暖色傘衣 | **很強** | 中等 |
| 白色箭身 | 幾乎為零 | **很強** |
| 傘繩（細白線）| 零 | **唯一看得到的通道** |

前一版只用色度，所以**傘繩與箭身根本沒被抓到**。兩個通道要一起用。

## 輸出

  1 原圖
  2 天空扣除（彩色，強對比）—— 綜合判讀用
  3 色度通道（假色）—— 傘衣位置與形狀
  4 亮度殘差 —— 傘繩、箭身
  5 合成輪廓 —— 只看形狀
"""
import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageFilter


def fit_sky(a, mask_obj, order=2):
    """對非物體像素做二次曲面最小平方擬合，逐通道。回傳擬合出的天空。"""
    H, W, _ = a.shape
    yy, xx = np.mgrid[0:H, 0:W]
    x = (xx / W - 0.5).ravel()
    y = (yy / H - 0.5).ravel()
    cols = [np.ones_like(x), x, y]
    if order >= 2:
        cols += [x * x, x * y, y * y]
    A = np.stack(cols, axis=1)
    keep = ~mask_obj.ravel()
    if keep.sum() < A.shape[1] * 20:          # 物體太多就退回全體
        keep = np.ones_like(keep)
    out = np.empty_like(a)
    for c in range(3):
        b = a[:, :, c].ravel()
        coef, *_ = np.linalg.lstsq(A[keep], b[keep], rcond=None)
        out[:, :, c] = (A @ coef).reshape(H, W)
    return out


def rough_mask(a):
    """粗略物體遮罩，只用來把物體排除在天空擬合之外。"""
    s = a[:, :, 2] - np.maximum(a[:, :, 0], a[:, :, 1])
    lum = a.mean(axis=2)
    return (s < np.median(s) - 12) | (np.abs(lum - np.median(lum)) > 14)


def norm(x, lo=0.5, hi=99.7):
    a_, b_ = np.percentile(x, lo), np.percentile(x, hi)
    return np.clip((x - a_) / max(b_ - a_, 1e-6), 0, 1)


def analyse(im, chroma_gain=1.0, lum_gain=1.0):
    a = np.asarray(im.filter(ImageFilter.GaussianBlur(0.6))).astype(np.float32)
    sky = fit_sky(a, rough_mask(a))
    resid = a - sky                                  # 扣掉天空

    # ── 色度：傘衣是暖色，(R−B) 的殘差為正 ──
    chroma = resid[:, :, 0] - resid[:, :, 2]
    chroma = np.clip(chroma, 0, None) * chroma_gain

    # ── 亮度：白色箭身、傘繩比天空亮 ──
    lum = np.clip(resid.mean(axis=2), 0, None) * lum_gain

    # ── 合成：兩通道各自正規化後取大者 ──
    comb = np.maximum(norm(chroma), norm(lum))
    return a, sky, resid, chroma, lum, comb


def to_img(x, cmap=None):
    """x: HxW 0~1 或 HxWx3。cmap: None=灰階, 'warm'/'cool'=假色。"""
    if x.ndim == 3:
        return Image.fromarray(np.clip(x, 0, 255).astype(np.uint8))
    v = np.clip(x, 0, 1)
    if cmap == "warm":                                # 黑→紅→黃→白
        r = np.clip(v * 3, 0, 1); g = np.clip(v * 3 - 1, 0, 1); b = np.clip(v * 3 - 2, 0, 1)
    elif cmap == "cool":                              # 白→藍→黑（反相，看細線）
        r = g = 1 - v; b = np.clip(1 - v * 0.6, 0, 1)
    else:
        r = g = b = 1 - v                             # 反相灰階：物體黑、背景白
    return Image.fromarray((np.stack([r, g, b], -1) * 255).astype(np.uint8))


def panel(im, up=3, chroma_gain=1.0, lum_gain=1.0):
    a, sky, resid, chroma, lum, comb = analyse(im, chroma_gain, lum_gain)

    # 天空扣除後的彩色圖：把殘差加回一個中性底，再拉對比
    vis = np.zeros_like(a)
    for c in range(3):
        vis[:, :, c] = norm(resid[:, :, c], 1, 99.5) * 255.0
    # 物體區域保留原色調、天空壓白
    m = comb[:, :, None]
    colour = (np.stack([norm(a[:, :, c], 2, 99.8) for c in range(3)], -1) * 255.0)
    clean = colour * m + 250.0 * (1 - m)

    outs = [(im, "原圖"),
            (to_img(clean), "天空擬合扣除＋對比"),
            (to_img(norm(chroma), "warm"), "色度通道（傘衣）"),
            (to_img(norm(lum), "cool"), "亮度殘差（傘繩／箭身）"),
            (to_img(comb), "合成輪廓")]
    W, H = im.size
    big = (int(W * up), int(H * up))
    return [(p.resize(big, Image.LANCZOS).filter(
             ImageFilter.UnsharpMask(radius=2, percent=120, threshold=2)), lab)
            for p, lab in outs]


def main():
    ap = argparse.ArgumentParser(description="天空擬合扣除，多通道分離物體")
    ap.add_argument("pattern")
    ap.add_argument("--out-dir", default="separated")
    ap.add_argument("--up", type=float, default=3.0)
    ap.add_argument("--chroma-gain", type=float, default=1.0)
    ap.add_argument("--lum-gain", type=float, default=1.0)
    ap.add_argument("--every", type=int, default=1)
    a = ap.parse_args()

    files = sorted(glob.glob(a.pattern))[::a.every]
    if not files:
        raise SystemExit(f"找不到 {a.pattern}")
    os.makedirs(a.out_dir, exist_ok=True)

    from PIL import ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("msjh.ttc", 26)
    except Exception:
        font = ImageFont.load_default()

    for f in files:
        im = Image.open(f).convert("RGB")
        ps = panel(im, a.up, a.chroma_gain, a.lum_gain)
        pad, bar = 12, 40
        w = sum(p.width for p, _ in ps) + pad * (len(ps) + 1)
        h = max(p.height for p, _ in ps) + pad * 2 + bar
        cv = Image.new("RGB", (w, h), (250, 248, 243))
        d = ImageDraw.Draw(cv)
        x = pad
        for p, lab in ps:
            cv.paste(p, (x, pad + bar))
            d.text((x + 4, pad + 8), lab, fill=(27, 42, 65), font=font)
            x += p.width + pad
        name = os.path.splitext(os.path.basename(f))[0]
        cv.save(os.path.join(a.out_dir, name + "_sep.png"))
        ps[1][0].save(os.path.join(a.out_dir, name + "_clean.png"))
        print(f"  {name}  →  {cv.size[0]}x{cv.size[1]}")
    print(f"\n→ {len(files)} 組在 {a.out_dir}/")


if __name__ == "__main__":
    main()
