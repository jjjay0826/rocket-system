#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把開傘影像裡「已經存在但看不清楚」的資訊分離出來。

★ 先講清楚：**放大不會產生新資訊**。這支做的是對比與色度分離，
  讓原本就記錄在像素裡、但被藍天壓過去的細節浮出來。
  真要提升畫質，唯一的路是回到更高解析度的原始影片。

## 手法

1. **天空抑制**：天空是幾乎均勻的藍（B 遠大於 R、G），傘衣是暖色
   （R > B），箭身是白（R≈G≈B）。定義

       skyness = B - max(R, G)

   天空 skyness 大正值、白色物體約 0、暖色傘衣為負值。
   用整幅的中位數當天空基準，減掉之後就得到「物體圖」。

2. **局部對比拉伸**：只在物體區域做直方圖拉伸，不動天空。

3. **反銳化遮罩**：把傘繩這種一兩個像素寬的細線拉出來。

4. **Lanczos 放大**：純粹為了看得舒服，沒有增加資訊。
"""
import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageFilter, ImageEnhance


def load_rgb(path):
    im = Image.open(path)
    if im.mode != "RGB":
        im = im.convert("RGB")
    return im


def skyness(a):
    """a: HxWx3 float。回傳天空程度（天空大、物體小/負）。"""
    return a[:, :, 2] - np.maximum(a[:, :, 0], a[:, :, 1])


def object_mask(im, soft=14.0, dead=0.18):
    """0~1 的物體遮罩。用天空的色度基準去減。

    ★ 來源是 YouTube 壓縮過的畫面，天空裡有 8×8 的區塊雜訊。直接對原圖
      算遮罩會把區塊邊界也算成「物體」，後續對比拉伸再把它放大成一格一格
      的假結構。所以：
        ① 遮罩用「輕微模糊過」的影像算 —— 模糊只影響遮罩，不影響顯示
        ② 加一個死區 dead：低於它的一律歸零，把殘餘區塊雜訊壓平
    """
    blur = np.asarray(im.filter(ImageFilter.GaussianBlur(1.2))).astype(np.float32)
    s = skyness(blur)
    ref = np.median(s)
    m = (ref - s) / max(soft, 1e-6)
    m = np.clip(m, 0.0, 1.0)
    m = np.clip((m - dead) / (1.0 - dead), 0.0, 1.0)      # 死區
    return m


def stretch(x, lo_pct=1.0, hi_pct=99.5):
    lo, hi = np.percentile(x, lo_pct), np.percentile(x, hi_pct)
    if hi - lo < 1e-6:
        return np.zeros_like(x)
    return np.clip((x - lo) / (hi - lo), 0, 1)


def enhance(im, scale=4, sat=1.6, sharp_r=2.0, sharp_pct=180):
    """回傳 (放大原圖, 天空抑制圖, 物體遮罩圖) 三張，皆已放大。"""
    a = np.asarray(im).astype(np.float32)
    mask = object_mask(im)
    sel = mask > 0.15                       # 只在物體像素上算拉伸範圍

    # ── ① 天空抑制：天空壓成純白底，物體保留顏色並拉伸對比 ──
    obj = a.copy()
    if sel.sum() > 20:
        for c in range(3):
            ch = a[:, :, c]
            lo, hi = np.percentile(ch[sel], 2), np.percentile(ch[sel], 98)
            if hi - lo > 1e-6:
                obj[:, :, c] = np.clip((ch - lo) / (hi - lo), 0, 1) * 255.0
    blend = mask[:, :, None]
    sky_flat = np.full_like(a, 252.0)       # 乾淨白底，區塊雜訊被遮罩死區壓掉
    supp = obj * blend + sky_flat * (1.0 - blend)

    # ── ② 純遮罩圖：只看形狀，不受顏色干擾 ──
    mk = (1.0 - mask) * 255.0
    mk3 = np.repeat(mk[:, :, None], 3, axis=2)

    W, H = im.size
    big = (W * scale, H * scale)
    out = []
    for arr, do_sat in ((a, True), (supp, True), (mk3, False)):
        p = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
        p = p.resize(big, Image.LANCZOS)
        if do_sat:
            p = ImageEnhance.Color(p).enhance(sat)
            p = ImageEnhance.Contrast(p).enhance(1.15)
        p = p.filter(ImageFilter.UnsharpMask(radius=sharp_r, percent=sharp_pct, threshold=2))
        out.append(p)
    return out


def side_by_side(panels, labels, pad=10, bg=(250, 248, 243)):
    from PIL import ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("msjh.ttc", 22)
    except Exception:
        font = ImageFont.load_default()
    bar = 34
    w = sum(p.width for p in panels) + pad * (len(panels) + 1)
    h = max(p.height for p in panels) + pad * 2 + bar
    canvas = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(canvas)
    x = pad
    for p, lab in zip(panels, labels):
        canvas.paste(p, (x, pad + bar))
        d.text((x + 4, pad + 6), lab, fill=(27, 42, 65), font=font)
        x += p.width + pad
    return canvas


def main():
    ap = argparse.ArgumentParser(description="開傘影像對比／色度分離")
    ap.add_argument("pattern", help="檔案 glob，例如 '螢幕擷取畫面*.png'")
    ap.add_argument("--out-dir", default="enhanced")
    ap.add_argument("--scale", type=int, default=4)
    a = ap.parse_args()

    files = sorted(glob.glob(a.pattern))
    if not files:
        raise SystemExit(f"找不到符合 {a.pattern} 的檔案")
    os.makedirs(a.out_dir, exist_ok=True)

    sheets = []
    for f in files:
        im = load_rgb(f)
        panels = enhance(im, scale=a.scale)
        sbs = side_by_side(panels, ["原圖（放大）", "天空抑制＋對比", "物體遮罩"])
        name = os.path.splitext(os.path.basename(f))[0]
        outp = os.path.join(a.out_dir, name + "_enh.png")
        sbs.save(outp)
        panels[1].save(os.path.join(a.out_dir, name + "_clean.png"))
        sheets.append(panels[1])
        print(f"  {os.path.basename(f):45} {im.size[0]}x{im.size[1]} → "
              f"{panels[1].size[0]}x{panels[1].size[1]}")

    # 接觸表：全部「天空抑制」版並排，方便看時序
    H = max(p.height for p in sheets)
    W = sum(p.width for p in sheets) + 12 * (len(sheets) + 1)
    sheet = Image.new("RGB", (W, H + 24), (250, 248, 243))
    x = 12
    for p in sheets:
        sheet.paste(p, (x, 12))
        x += p.width + 12
    sheet.save(os.path.join(a.out_dir, "contact_sheet.png"))
    print(f"\n→ {a.out_dir}/ （每張三聯 ＋ contact_sheet.png）")


if __name__ == "__main__":
    main()
