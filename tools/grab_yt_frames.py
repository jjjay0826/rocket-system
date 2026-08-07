#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""從 YouTube 直播存檔抓一小段，並抽出無損 PNG 幀。

用途：把開傘過程的畫面拿到最高解析度，用來判讀傘衣狀態。
螢幕截圖經過「YouTube 壓縮 → 螢幕顯示 → 截圖 → 裁切」四層失真，
直接抽原始幀可以少掉後面三層。

## 用法

    python grab_yt_frames.py <URL> --start 1:23:45 --dur 12

    # 只下載片段不抽幀
    python grab_yt_frames.py <URL> --start 1:23:45 --dur 12 --no-frames

    # 指定抽幀率（預設抽出每一幀）
    python grab_yt_frames.py <URL> --start 1:23:45 --dur 12 --fps 10

## 怎麼找到正確的時間戳

遙測有 UTC 時戳，可以直接換算（本次飛行，台灣時間 UTC+8）：

    16:57:28.050   離架（ch2 ST:0→1）
    16:57:44.088   開傘點火（ch1 韌體訊息 "Parachute deployed auto A+B"）
    16:57:46.097   開傘衝擊 6.14 g ／ ch1 斷電重開
    16:58:11.607   遙測最後一包（高度 20 m）

在影片裡找到**點火閃光**那一幀，記下它的影片時間 T_video，
則任何事件的影片時間 = T_video + (事件 UTC − 16:57:28.05)。

## 注意

- 只下載你有權使用的內容。這支工具用 `--download-sections` 只取指定區間，
  不會下載整部影片。
- 需要 yt-dlp 與 ffmpeg：`pip install yt-dlp imageio-ffmpeg`
  （imageio-ffmpeg 會附帶一個 ffmpeg 執行檔，不必另外安裝）
"""
import argparse
import os
import subprocess
import sys


def ffmpeg_path():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        from shutil import which
        p = which("ffmpeg")
        if not p:
            sys.exit("找不到 ffmpeg。請執行：pip install imageio-ffmpeg")
        return p


def run(cmd):
    print("  $ " + " ".join(f'"{c}"' if " " in str(c) else str(c) for c in cmd))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit(f"指令失敗（exit {r.returncode}）")


def to_hhmmss(s):
    """接受 90 / 1:30 / 0:01:30 三種寫法，統一成 HH:MM:SS。"""
    parts = str(s).split(":")
    if len(parts) == 1:
        sec = float(parts[0])
    elif len(parts) == 2:
        sec = int(parts[0]) * 60 + float(parts[1])
    else:
        sec = int(parts[0]) * 3600 + int(parts[1]) * 60 + float(parts[2])
    h, rem = divmod(sec, 3600)
    m, s2 = divmod(rem, 60)
    return f"{int(h):02d}:{int(m):02d}:{s2:06.3f}"


def main():
    ap = argparse.ArgumentParser(description="從 YouTube 抓片段並抽幀")
    ap.add_argument("url")
    ap.add_argument("--start", required=True, help="起點，例如 1:23:45")
    ap.add_argument("--dur", type=float, default=15.0, help="長度（秒），預設 15")
    ap.add_argument("--out-dir", default="yt_clip")
    ap.add_argument("--fps", type=float, default=0,
                    help="抽幀率；0 = 抽出每一幀（預設）")
    ap.add_argument("--max-height", type=int, default=0,
                    help="限制最高畫質，例如 1080；0 = 不限（取最高）")
    ap.add_argument("--no-frames", action="store_true", help="只下載片段，不抽幀")
    a = ap.parse_args()

    os.makedirs(a.out_dir, exist_ok=True)
    ff = ffmpeg_path()
    start = to_hhmmss(a.start)
    end_s = (int(start[:2]) * 3600 + int(start[3:5]) * 60 + float(start[6:])) + a.dur
    end = to_hhmmss(end_s)
    clip = os.path.join(a.out_dir, "clip.mp4")

    # ── 下載：只取影像串流（判讀畫面不需要聲音），只取指定區間 ──
    # bv* = best video-only。YouTube 的最高畫質一定在 video-only 串流裡，
    # 混流(muxed)版本通常上限只有 720p。
    fmt = "bv*" if not a.max_height else f"bv*[height<=?{a.max_height}]"
    print(f"\n【1/2】下載 {start} ~ {end}（{a.dur:.0f} 秒）")
    run([sys.executable, "-m", "yt_dlp",
         "-f", fmt,
         "--download-sections", f"*{start}-{end}",
         "--force-keyframes-at-cuts",      # 精確切點（會多下載一點點）
         "--ffmpeg-location", ff,
         "-o", clip,
         "--no-playlist",
         a.url])

    if not os.path.exists(clip):
        cand = [f for f in os.listdir(a.out_dir) if f.startswith("clip")]
        if cand:
            clip = os.path.join(a.out_dir, cand[0])
        else:
            sys.exit("下載後找不到檔案")

    # 印出實際規格
    print("\n【片段規格】")
    subprocess.run([ff, "-hide_banner", "-i", clip], stderr=subprocess.STDOUT)

    if a.no_frames:
        print(f"\n→ {clip}")
        return

    # ── 抽幀：PNG 無損 ──
    fdir = os.path.join(a.out_dir, "frames")
    os.makedirs(fdir, exist_ok=True)
    print(f"\n【2/2】抽幀 → {fdir}")
    cmd = [ff, "-y", "-i", clip]
    if a.fps > 0:
        cmd += ["-vf", f"fps={a.fps}"]
    else:
        cmd += ["-vsync", "0"]            # 每一幀都抽，不做時間軸重取樣
    cmd += ["-compression_level", "1", os.path.join(fdir, "f_%05d.png")]
    run(cmd)

    n = len([f for f in os.listdir(fdir) if f.endswith(".png")])
    print(f"\n→ {n} 張 PNG 在 {fdir}")
    print(f"→ 接著可以跑：")
    print(f"   python tools/enhance_frames.py \"{fdir}/f_*.png\" --out-dir {a.out_dir}/enhanced")


if __name__ == "__main__":
    main()
