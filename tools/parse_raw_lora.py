#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把地面站的 raw_chN_*.log 解析成乾淨的 CSV ＋ 事件表。

封包格式（firmware-rocket/Core/Src/main.c:1984）：
  T<ms> SQ<n> AX%+.3f AY AZ GX%+.2f GY GZ P%.2f RH%.1f KH%.1f VZ%+.2f
  GA%.2f ST:%d MOD:%X GPS:1,%u C:%X VF%.2f VA%.2f LAT%+.5f LON%+.5f

位元欄位（main.c:1977-1978）：
  MOD = (bmp585<<3)|(imu<<2)|(lora<<1)|(sdcard<<0)
  C   = (cond_A<<3)|(ca_eff<<2)|(cond_B<<1)|(cb_eff<<0)
        cond_A = 氣壓高度跌破峰值 10m（且峰值 ≥20m）
        cond_B = kf2_v < -0.5 m/s 持續 1.5s
        *_eff  = 故障容錯後實際採用的值（感測器死掉時會被改寫）

用法：
    python parse_raw_lora.py raw_ch2_20260801_152419.log
        → raw_ch2_..._parsed.csv   全部封包，欄位展開
        → raw_ch2_..._events.txt   事件時序表
"""
import argparse
import math
import os
import re
import sys

# 地面站可能在每行前面加 UTC 時戳（有的版本會重複兩次）。有它就有共同時鐘，
# 兩塊板之間就不必再靠擬合對齊 —— 這是最可靠的時間基準。
PKT = re.compile(
    r"^(?:(?P<utc>\d\d:\d\d:\d\d\.\d\d\d)\s+(?:\d\d:\d\d:\d\d\.\d\d\d\s+)?)?"
    r"T(?P<t>\d+)\s+SQ(?P<sq>\d+)\s+"
    r"AX(?P<ax>[-+][\d.]+)\s+AY(?P<ay>[-+][\d.]+)\s+AZ(?P<az>[-+][\d.]+)\s+"
    r"GX(?P<gx>[-+][\d.]+)\s+GY(?P<gy>[-+][\d.]+)\s+GZ(?P<gz>[-+][\d.]+)\s+"
    r"P(?P<p>[\d.]+)\s+RH(?P<rh>[-+]?[\d.]+)\s+KH(?P<kh>[-+]?[\d.]+)\s+"
    r"VZ(?P<vz>[-+][\d.]+)\s+GA(?P<ga>[\d.]+)\s+"
    r"ST:(?P<st>\d+)\s+MOD:(?P<mod>[0-9A-F])\s+"
    r"GPS:(?P<fix>\d),(?P<sats>\d+)\s+C:(?P<cond>[0-9A-F])\s+"
    r"VF(?P<vf>[-+]?[\d.]+)\s+VA(?P<va>[-+]?[\d.]+)"
    r"(?:\s+LAT(?P<lat>[-+][\d.]+)\s+LON(?P<lon>[-+][\d.]+))?\s*$")

def _utc_secs(s):
    """hh:mm:ss.mmm -> 當日秒數。沒有時戳就回 None。"""
    if not s:
        return None
    h, m, x = s.split(":")
    return int(h) * 3600 + int(m) * 60 + float(x)


ST_NAME = {0: "IDLE", 1: "LAUNCHED", 2: "DEPLOYING", 3: "DEPLOYED", 4: "LANDED"}
COLS = ("utc t_ms sq ax ay az gx gy gz press rh kh vz ga st mod fix sats "
        "cond vf va lat lon cond_A ca_eff cond_B cb_eff "
        "m_bmp m_imu m_lora m_sd").split()


def parse(path):
    """回傳 (packets, messages, malformed)。

    packets  : dict 的 list，欄位見 COLS
    messages : (行號, 前一封包的 t_ms 或 None, 文字)
    malformed: (行號, 原始文字) —— LoRa 封包碎裂造成的殘行
    """
    pkts, msgs, bad = [], [], []
    last_t = None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln, line in enumerate(f, 1):
            line = line.rstrip("\r\n")
            if not line:
                continue
            m = PKT.match(line)
            if m:
                g = m.groupdict()
                mod, cond = int(g["mod"], 16), int(g["cond"], 16)
                r = {
                    "t_ms": int(g["t"]), "sq": int(g["sq"]),
                    "ax": float(g["ax"]), "ay": float(g["ay"]), "az": float(g["az"]),
                    "gx": float(g["gx"]), "gy": float(g["gy"]), "gz": float(g["gz"]),
                    "press": float(g["p"]), "rh": float(g["rh"]), "kh": float(g["kh"]),
                    "vz": float(g["vz"]), "ga": float(g["ga"]),
                    "st": int(g["st"]), "mod": mod,
                    "fix": int(g["fix"]), "sats": int(g["sats"]),
                    "cond": cond, "vf": float(g["vf"]), "va": float(g["va"]),
                    "lat": float(g["lat"]) if g["lat"] else None,
                    "lon": float(g["lon"]) if g["lon"] else None,
                    "cond_A": (cond >> 3) & 1, "ca_eff": (cond >> 2) & 1,
                    "cond_B": (cond >> 1) & 1, "cb_eff": cond & 1,
                    "m_bmp": (mod >> 3) & 1, "m_imu": (mod >> 2) & 1,
                    "m_lora": (mod >> 1) & 1, "m_sd": mod & 1,
                    "utc": _utc_secs(g["utc"]),
                    "_line": ln,
                }
                pkts.append(r)
                last_t = r["t_ms"]
            elif "MSG " in line[:30]:
                msgs.append((ln, last_t, line[line.index("MSG ") + 4:]))
            elif "MOD: BMP" in line:
                msgs.append((ln, last_t, "BOOT " + line[line.index("MOD: BMP"):]))
            else:
                bad.append((ln, line))
    return pkts, msgs, bad


def find_launch(pkts):
    """第一個 ST 由 0 變 1 的封包索引；沒有就回 None。"""
    for i in range(1, len(pkts)):
        if pkts[i]["st"] >= 1 and pkts[i - 1]["st"] == 0:
            return i
    return None


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def enu(lat, lon, lat0, lon0):
    """相對發射點的東/北位移（公尺）。小範圍平面近似，誤差 <0.01%。"""
    R = 6371000.0
    e = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * R
    return e, n


def main():
    ap = argparse.ArgumentParser(description="解析地面站 raw LoRa log")
    ap.add_argument("path")
    ap.add_argument("--out-dir", default=None)
    a = ap.parse_args()

    pkts, msgs, bad = parse(a.path)
    if not pkts:
        sys.exit("沒有解析到任何封包")

    out_dir = a.out_dir or os.path.dirname(os.path.abspath(a.path))
    stem = os.path.join(out_dir, os.path.splitext(os.path.basename(a.path))[0])

    li = find_launch(pkts)
    t0 = pkts[li]["t_ms"] if li is not None else pkts[0]["t_ms"]
    for p in pkts:
        p["t_rel"] = (p["t_ms"] - t0) / 1000.0

    # ── CSV ──
    csv_path = stem + "_parsed.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        f.write("t_rel," + ",".join(COLS) + "\n")
        for p in pkts:
            f.write(f"{p['t_rel']:.3f}," +
                    ",".join("" if p[c] is None else str(p[c]) for c in COLS) + "\n")

    # ── 事件 ──
    L = []
    W = L.append
    W("=" * 78)
    W(f"檔案      {os.path.basename(a.path)}")
    W(f"封包      {len(pkts)}   訊息 {len(msgs)}   殘缺行 {len(bad)}")
    W(f"T0 基準   T{t0}  (ST 0→1)" if li is not None else "T0 基準   第一包（沒偵測到離架）")
    W("=" * 78)

    # 掉包：SQ 應該連號
    lost = gaps = 0
    for i in range(1, len(pkts)):
        d = pkts[i]["sq"] - pkts[i - 1]["sq"]
        if d > 1:
            lost += d - 1
            gaps += 1
    span = pkts[-1]["sq"] - pkts[0]["sq"] + 1
    W(f"\n【鏈路】SQ {pkts[0]['sq']}–{pkts[-1]['sq']}（應收 {span}）"
      f" 實收 {len(pkts)}  遺失 {lost}（{100.0*lost/span:.2f}%）分 {gaps} 段")
    if bad:
        W(f"        殘缺行 {len(bad)} 行（LoRa 封包碎裂）")

    # 飛行段掉包單獨算
    if li is not None:
        fl = [p for p in pkts if p["t_rel"] >= 0]
        flost = sum(max(0, fl[i]["sq"] - fl[i-1]["sq"] - 1) for i in range(1, len(fl)))
        fspan = fl[-1]["sq"] - fl[0]["sq"] + 1
        W(f"        飛行段：應收 {fspan} 實收 {len(fl)} 遺失 {flost}"
          f"（{100.0*flost/fspan:.2f}%）")

    # 狀態轉換
    W("\n【狀態機】")
    prev = None
    for p in pkts:
        if p["st"] != prev:
            W(f"  T{p['t_rel']:+8.3f}s  ST:{p['st']}={ST_NAME.get(p['st'],'?')}"
              f"   KH={p['kh']:.1f}m VZ={p['vz']:+.2f}m/s GA={p['ga']:.2f}g")
            prev = p["st"]

    # cond 旗標轉換
    W("\n【開傘條件旗標 C:】")
    prev = None
    for p in pkts:
        if p["cond"] != prev:
            W(f"  T{p['t_rel']:+8.3f}s  C:{p['cond']:X}  "
              f"cond_A={p['cond_A']} ca_eff={p['ca_eff']} "
              f"cond_B={p['cond_B']} cb_eff={p['cb_eff']}"
              f"   KH={p['kh']:.1f}m VZ={p['vz']:+.2f}")
            prev = p["cond"]

    # 模組健康
    W("\n【模組健康 MOD:】")
    prev = None
    for p in pkts:
        if p["mod"] != prev:
            W(f"  T{p['t_rel']:+8.3f}s  MOD:{p['mod']:X}  "
              f"bmp={p['m_bmp']} imu={p['m_imu']} lora={p['m_lora']} sd={p['m_sd']}")
            prev = p["mod"]

    # 韌體訊息
    W("\n【韌體訊息】")
    for ln, t, txt in msgs:
        rel = f"T{(t - t0)/1000.0:+8.3f}s" if t is not None else "  (無時戳)"
        W(f"  {rel}  {txt}")

    # 極值
    if li is not None:
        fl = [p for p in pkts if p["t_rel"] >= -1.0]
        pk_kh = max(fl, key=lambda p: p["kh"])
        pk_rh = max(fl, key=lambda p: p["rh"])
        pk_ga = max(fl, key=lambda p: p["ga"])
        pk_vz = max(fl, key=lambda p: p["vz"])
        mn_vz = min(fl, key=lambda p: p["vz"])
        W("\n【極值（封包取樣 2Hz，真值可能落在取樣點之間）】")
        W(f"  最大 KH（卡爾曼高度）  {pk_kh['kh']:8.1f} m  @ T{pk_kh['t_rel']:+.3f}s")
        W(f"  最大 RH（純氣壓高度）  {pk_rh['rh']:8.1f} m  @ T{pk_rh['t_rel']:+.3f}s")
        W(f"  最大 GA（合加速度）    {pk_ga['ga']:8.2f} g  @ T{pk_ga['t_rel']:+.3f}s")
        W(f"  最大 VZ（上升）        {pk_vz['vz']:+8.2f} m/s @ T{pk_vz['t_rel']:+.3f}s")
        W(f"  最小 VZ（下降）        {mn_vz['vz']:+8.2f} m/s @ T{mn_vz['t_rel']:+.3f}s")
        W(f"  最低壓力               {min(p['press'] for p in fl):8.2f} hPa")

    # 封包節拍異常
    W("\n【封包節拍異常（>750ms 或 <250ms）】")
    n_odd = 0
    for i in range(1, len(pkts)):
        dt = pkts[i]["t_ms"] - pkts[i - 1]["t_ms"]
        dsq = pkts[i]["sq"] - pkts[i - 1]["sq"]
        if pkts[i]["t_rel"] < -2:
            continue
        if dt > 750 or dt < 250:
            n_odd += 1
            note = "掉包" if dsq > 1 else "★韌體自己跳拍（SQ 連號）"
            W(f"  T{pkts[i]['t_rel']:+8.3f}s  Δt={dt:5d}ms ΔSQ={dsq}  {note}")
    if not n_odd:
        W("  無")

    # GPS
    fixes = [p for p in pkts if p["lat"] is not None and p["t_rel"] >= 0]
    if fixes:
        lat0, lon0 = fixes[0]["lat"], fixes[0]["lon"]
        W(f"\n【GPS】發射點 {lat0:+.5f}, {lon0:+.5f}   落點（最後一包）"
          f" {fixes[-1]['lat']:+.5f}, {fixes[-1]['lon']:+.5f}")
        W(f"      直線距離 {haversine(lat0, lon0, fixes[-1]['lat'], fixes[-1]['lon']):.0f} m")
        W("      逐點速度（相鄰不同座標之間；>100 m/s 幾乎確定是跳點）")
        prev = None
        for p in fixes:
            if prev is None or (p["lat"], p["lon"]) != (prev["lat"], prev["lon"]):
                if prev is not None:
                    d = haversine(prev["lat"], prev["lon"], p["lat"], p["lon"])
                    dt = p["t_rel"] - prev["t_rel"]
                    v = d / dt if dt > 0 else 0
                    e, n = enu(p["lat"], p["lon"], lat0, lon0)
                    flag = "  ← 🔴 跳點" if v > 100 else ("  ← ⚠" if v > 50 else "")
                    W(f"        T{p['t_rel']:+7.2f}s E{e:+7.1f} N{n:+7.1f} "
                      f"sats={p['sats']:2d}  Δ{d:6.1f}m/{dt:.2f}s = {v:6.1f} m/s{flag}")
                prev = p

    txt_path = stem + "_events.txt"
    open(txt_path, "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"\n→ {csv_path}")
    print(f"→ {txt_path}")


if __name__ == "__main__":
    main()
