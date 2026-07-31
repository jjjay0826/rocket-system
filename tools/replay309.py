#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay309.py — 把 OpenRocket 的 3.0.9 模擬直接放進地面站畫面

為什麼不用 tools/sim_replay.py
  那支已經脫節，而且是【安靜地】脫節 —— 它跑得起來，只是餵的東西不對：
    · 封包缺 SQ / VF / VA（2026-07 之後的欄位）→ 鏈路品質那一欄永遠空的
    · 還在送「Airbag inflation started」→ 氣囊 2026-07-31 已移除
    · --st12 那套 12 狀態碼韌體從未實作，餵下去 ST 會超出 0~4
    · 加速度放在 AZ；但航電板現在是【豎放、感測器 −X 朝上】，
      真火箭靜止時 AX≈−1、AZ≈0。餵 AZ=1 的話畫面會顯示水平，
      正好把我們要驗的那件事驗成假的。

  留著那支不動（歷史紀錄），新的走這裡。

兩種餵法
  --zmq   （預設）直接扮演 backend 的 PUB socket，GUI 訂閱什麼就收什麼。
          【不需要序列埠、不需要 com0com、不需要跑 backend_daemon】
          用途：看畫面。圖表、地圖、階段列、事件標記全部會動。
  --port  走真實序列埠送 ASCII 封包（需要 com0com 虛擬埠對或第二顆 E22）。
          用途：連解析器一起驗，是最接近真飛行的路徑。

用法
  python tools/replay309.py --list
  python tools/replay309.py --sim T100_e80_E2               # 預設 --zmq
  python tools/replay309.py --sim T100_e80_E2 --speed 4     # 4 倍速
  python tools/replay309.py --sim T110_e81_E4 --both        # 雙板同時
  python tools/replay309.py --sim T100_e80_E2 --port COM20  # 走序列埠
  python tools/replay309.py --sim T100_e80_E2 --dry | head  # 只印封包
"""
import argparse
import glob
import json
import math
import os
import random
import sys
import time
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime

SIM_DIR = r"D:\Downloads\sim_309"
G = 9.80665

# 韌體 FlightState_t（firmware-rocket/Core/Src/main.c）
ST_IDLE, ST_LAUNCHED, ST_DEPLOYING, ST_DEPLOYED, ST_LANDED = 0, 1, 2, 3, 4
DEPLOY_PULSE_S = 1.0        # DEPLOY_PULSE_MS
LAND_STABLE_S = 10.0        # LAND_STABLE_MS

# 頻道 → ZMQ PUB 埠（與 src/utils/settings.py 的 DEFAULT_CHANNELS 一致）
ZMQ_PORT = {"ch1": 15555, "ch2": 15556}

WANT = ["Time", "Altitude", "Vertical velocity", "Vertical acceleration",
        "Total acceleration", "Roll rate", "Latitude", "Longitude",
        "Air pressure", "Air temperature"]


def load(sim_name):
    """從 sim_309 的三個 .ork 裡找出指定的那一組"""
    for p in sorted(glob.glob(os.path.join(SIM_DIR, "m309_T*.ork"))):
        root = ET.fromstring(
            zipfile.ZipFile(p).read("rocket.ork").decode("utf-8", "ignore"))
        for s in root.findall(".//simulation"):
            if s.find("name").text != sim_name:
                continue
            br = s.find(".//flightdata/databranch")
            if br is None:
                sys.exit(f"{sim_name} 沒有模擬結果 —— 在 OpenRocket 裡跑過並存檔了嗎？")
            ix = {c: i for i, c in enumerate(br.get("types").split(","))}
            rows = []
            for dp in br.findall("datapoint"):
                v = [float(x) if x.strip() not in ("NaN", "") else float("nan")
                     for x in dp.text.split(",")]
                rows.append({k: v[ix[k]] for k in WANT})
            ev = {e.get("type"): float(e.get("time")) for e in br.findall("event")}
            return rows, ev
    sys.exit(f"找不到 {sim_name}（用 --list 看有哪些）")


def listing():
    for p in sorted(glob.glob(os.path.join(SIM_DIR, "m309_T*.ork"))):
        root = ET.fromstring(
            zipfile.ZipFile(p).read("rocket.ork").decode("utf-8", "ignore"))
        names = [s.find("name").text for s in root.findall(".//simulation")]
        print(f"{os.path.basename(p)}  ({len(names)} 組)")
        for i in range(0, len(names), 3):
            print("   " + "  ".join(f"{n:<18}" for n in names[i:i + 3]))


def interp(rows, t):
    if t <= rows[0]["Time"]:
        return rows[0]
    if t >= rows[-1]["Time"]:
        return rows[-1]
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid]["Time"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = rows[lo], rows[hi]
    k = (t - a["Time"]) / (b["Time"] - a["Time"]) if b["Time"] > a["Time"] else 0.0
    return {key: a[key] + k * (b[key] - a[key]) for key in a}


def flight_state(t, ev):
    """由模擬事件推韌體的 ST。與 main.c 的狀態機同一套時序。"""
    t_lo = ev.get("liftoff", 1.1)
    t_dep = ev.get("recoverydevicedeployment", ev.get("apogee", 16.2))
    t_gnd = ev.get("groundhit", 1e9)
    if t < t_lo:
        return ST_IDLE
    if t < t_dep:
        return ST_LAUNCHED
    if t < t_dep + DEPLOY_PULSE_S:
        return ST_DEPLOYING
    if t < t_gnd + LAND_STABLE_S:
        return ST_DEPLOYED
    return ST_LANDED


class Board:
    """一塊航電板。兩塊板看同一條軌跡，但各自有獨立的噪聲、掉包與序號。"""

    def __init__(self, ch, loss, seed):
        self.ch = ch
        self.loss = loss                 # 封包掉落率
        self.rnd = random.Random(seed)
        self.seq = 0
        self.walk = [0.0, 0.0]           # GPS 慢漂
        self.ref_press = None
        self.deploy_sent = False

    def packet(self, t_sim, d, st, t_ms):
        n = self.rnd.gauss
        # OpenRocket 的 Air pressure 是 Pa，韌體遙測送的是 hPa（P1013.25）。
        # 不換算的話地面站會收到 P101300.00 —— 解析得動，但高度圖與
        # 氣壓漂移告警的門檻全部錯三個數量級。
        hpa = d["Air pressure"] / 100.0
        if self.ref_press is None:
            self.ref_press = hpa
        self.seq += 1

        # ── 加速度：感測器座標，−X 朝上（2026-08-01 實測的安裝方向）──
        # 加速度計量的是【重力反作用力】，靜止時朝上那支軸讀 +1g。
        # −X 朝上 ⇒ AX = −(1 + a_vert/g)；另兩軸只有橫向擾動。
        g_axis = (d["Vertical acceleration"] + G) / G
        ax = -g_axis + n(0, 0.012)
        ay = n(0, 0.018)
        az = n(0, 0.018)
        ga = math.sqrt(ax * ax + ay * ay + az * az)

        # ── 陀螺儀：自旋在縱軸上，也就是 GX ──
        roll = math.degrees(d["Roll rate"]) if not math.isnan(d["Roll rate"]) else 0.0
        gx, gy, gz = roll + n(0, 2.0), n(0, 2.5), n(0, 2.5)

        # ── GPS：慢漂 ±5m + 每包白噪 1.2m ──
        for i in range(2):
            self.walk[i] = max(-5.0, min(5.0, self.walk[i] + n(0, 0.6)))
        mpd = 111320.0
        lat = d["Latitude"] + (self.walk[1] + n(0, 1.2)) / mpd
        lon = d["Longitude"] + (self.walk[0] + n(0, 1.2)) / (
            mpd * math.cos(math.radians(d["Latitude"])))
        sats = self.rnd.randint(9, 16)

        rel_h = d["Altitude"] + n(0, 0.12)
        # cond_A / cond_B（C: 欄的 bit0/bit1），與韌體的意義一致
        cond = 0
        if st >= ST_DEPLOYING:
            cond = 3

        # VF/VA：分壓沒焊，真板讀到的是浮接雜訊。照實模擬 ——
        # 地面站的電量告警已停用，這裡餵真實的爛值才驗得到「它真的不吵」。
        vf, va = abs(n(0.12, 0.08)), abs(n(0.10, 0.07))

        return (
            "T{t} SQ{sq} AX{ax:+.3f} AY{ay:+.3f} AZ{az:+.3f} "
            "GX{gx:+.2f} GY{gy:+.2f} GZ{gz:+.2f} "
            "P{p:.2f} RH{rh:.1f} KH{kh:.1f} VZ{vz:+.2f} GA{ga:.2f} "
            "ST:{st} MOD:F GPS:1,{sats} C:{c:X} VF{vf:.2f} VA{va:.2f} "
            "LAT{lat:+.5f} LON{lon:+.5f}\r\n"
        ).format(t=t_ms, sq=self.seq, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz,
                 p=hpa + n(0, 0.02), rh=rel_h,
                 kh=d["Altitude"] + n(0, 0.15), vz=d["Vertical velocity"] + n(0, 0.05),
                 ga=ga, st=st, sats=sats, c=cond, vf=vf, va=va, lat=lat, lon=lon)


def main():
    ap = argparse.ArgumentParser(description="3.0.9 模擬 → 地面站")
    ap.add_argument("--sim", default="T100_e80_E2", help="模擬名稱，例：T110_e81_E4")
    ap.add_argument("--list", action="store_true", help="列出全部 81 組")
    ap.add_argument("--speed", type=float, default=1.0, help="播放倍速")
    ap.add_argument("--rate", type=float, default=2.0, help="封包率 Hz（韌體是 2）")
    ap.add_argument("--pre", type=float, default=8.0, help="離架前的地面靜置秒數")
    ap.add_argument("--both", action="store_true", help="同時餵 ch1 與 ch2")
    ap.add_argument("--loss", type=float, default=0.06, help="封包掉落率（0~1）")
    ap.add_argument("--port", help="序列埠（需 com0com 或第二顆 E22）")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--zmq", action="store_true", help="直接發給 GUI（預設）")
    ap.add_argument("--dry", action="store_true", help="只把封包印到螢幕")
    args = ap.parse_args()

    if args.list:
        listing()
        return

    rows, ev = load(args.sim)
    # ★要跑過「觸地 + LAND_STABLE_S」才看得到 LANDED。
    # 用 min(模擬結束, …) 的話會停在 157.9s，而 LANDED 要 163.7s 才成立 ——
    # 畫面永遠停在 DEPLOYED，剛好把最後一個狀態轉換驗不到。
    # 模擬資料用完之後 interp 會夾在最後一筆，等同「火箭躺在地上繼續發」，
    # 那正是真實情況。
    t_end = ev.get("groundhit", rows[-1]["Time"]) + LAND_STABLE_S + 15
    print(f"{args.sim}：{len(rows)} 筆資料點")
    print("  事件 " + "  ".join(f"{k}@{v:.2f}s" for k, v in sorted(ev.items(), key=lambda x: x[1])
                                if k in ("liftoff", "burnout", "apogee",
                                         "recoverydevicedeployment", "groundhit")))
    apo = max(rows, key=lambda r: r["Altitude"])
    print(f"  頂點 {apo['Altitude']:.0f} m @ {apo['Time']:.2f}s"
          f"　落地 {abs(rows[-1]['Vertical velocity']):.2f} m/s"
          f"　全長 {t_end + args.pre:.0f}s（{args.speed}倍速 → "
          f"{(t_end + args.pre) / args.speed:.0f}s）")

    boards = [Board("ch1", args.loss, 1)]
    if args.both:
        boards.append(Board("ch2", args.loss * 1.6, 2))   # ch2 天線較差

    mode = "port" if args.port else ("dry" if args.dry else "zmq")
    ser = pubs = None
    if mode == "port":
        import serial
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"  序列埠 {args.port} @ {args.baud}")
    elif mode == "zmq":
        import zmq
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                        "rocket_system_ground_side"))
        ctx = zmq.Context()
        pubs = {}
        for b in boards:
            s = ctx.socket(zmq.PUB)
            s.setsockopt(zmq.LINGER, 0)
            s.bind(f"tcp://127.0.0.1:{ZMQ_PORT[b.ch]}")
            pubs[b.ch] = s
            print(f"  ZMQ PUB  {b.ch} → tcp://127.0.0.1:{ZMQ_PORT[b.ch]}")
        print("  ⚠ 這會佔用 backend daemon 的埠 —— 先把 backend 關掉")
        print("  等 GUI 訂閱上來…（PUB 在訂閱者連上之前送的會被丟掉）")
        time.sleep(1.5)
    print()

    from importlib import import_module
    SensorData = None
    if mode == "zmq":
        SensorData = import_module("src.core.models").SensorData

    def emit(board, line):
        if board.rnd.random() < board.loss:
            return                       # 掉包：鏈路品質那一欄要看得到才真實
        if mode == "port":
            ser.write(line.encode("ascii", "replace"))
        elif mode == "dry":
            sys.stdout.write(f"[{board.ch}] {line}")
        else:
            d = SensorData.from_new_format(line.strip(), datetime.now())
            dd = d.to_dict()
            dd["timestamp"] = d.timestamp.isoformat()
            dd["gs_timestamp"] = time.time()
            pubs[board.ch].send_multipart(
                [board.ch.encode(), json.dumps(dd).encode()])

    def emit_msg(board, text):
        if mode == "port":
            ser.write(text.encode("ascii", "replace"))
        elif mode == "dry":
            sys.stdout.write(f"[{board.ch}] {text}")
        else:
            pubs[board.ch].send_multipart(
                [f"{board.ch}_log".encode(),
                 json.dumps({"level": "INFO", "message": f"🚀 [ROCKET MSG] [SUCCESS] {text.strip()}",
                             "logger": "replay"}).encode()])

    period = 1.0 / args.rate
    t_sim = -args.pre
    boot_ms = 3890
    t_wall = time.time()
    last_st = -1
    while t_sim < t_end:
        d = interp(rows, max(t_sim, 0.0))
        st = ST_IDLE if t_sim < 0 else flight_state(t_sim, ev)
        t_ms = boot_ms + int((t_sim + args.pre) * 1000)
        for b in boards:
            emit(b, b.packet(t_sim, d, st, t_ms))
            if st in (ST_DEPLOYING, ST_DEPLOYED) and not b.deploy_sent:
                b.deploy_sent = True
                peak = max(r["Altitude"] for r in rows)
                emit_msg(b, "MSG SUCCESS Parachute deployed (auto A+B "
                            f"pk={peak:.1f}m now={d['Altitude']:.1f}m "
                            f"vz={d['Vertical velocity']:.2f}m/s)\r\n")
        if st != last_st:
            name = ["IDLE", "LAUNCHED", "DEPLOYING", "DEPLOYED", "LANDED"][st]
            print(f"  t={t_sim:7.2f}s  ST:{st} {name:<10} "
                  f"alt={d['Altitude']:7.1f}m  vz={d['Vertical velocity']:+6.2f}m/s")
            last_st = st
        t_wall += period / args.speed
        time.sleep(max(0.0, t_wall - time.time()))
        t_sim += period

    print(f"\n完成。ch1 送出 {boards[0].seq} 包"
          + (f"，ch2 {boards[1].seq} 包" if args.both else ""))
    if ser:
        ser.close()
    if pubs:
        for s in pubs.values():
            s.close()


if __name__ == "__main__":
    main()
