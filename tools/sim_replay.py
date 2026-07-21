#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_replay.py — OpenRocket 模擬 CSV → 火箭遙測封包 LoRa 回放器
================================================================
用途:地面站開發/測試沒有真飛行資料時,把 OpenRocket 匯出的 CSV 轉成
firmware-rocket 的下行封包格式,經第二顆 E22(USB-TTL)發射,地面站
收到的流與真火箭無異(含飛行狀態機 ST 演進與開傘 MSG 事件)。

硬體:USB-TTL(3.3V)+ 一顆 E22-900T22D,參數與地面站相同
      (922.125MHz/CH72、9600 8N1、air 9.6k、透傳)。
      接線:TTL TX→E22 RXD、TTL RX→E22 TXD、3V3/5V→VCC、GND—GND、
      M0/M1→GND(透傳)。

用法:
  python sim_replay.py 5k_p10_h.csv --port COM7
  python sim_replay.py 5k_p10_h.csv --port COM7 --speed 4    # 4 倍速
  python sim_replay.py 5k_p10_h.csv --port COM7 --loop       # 循環播放
  python sim_replay.py 5k_p10_h.csv --dry                    # 不開串口,印到螢幕

零硬體備案(沒有第二顆 E22 時):裝 com0com 建虛擬串口對 COM20↔COM21,
本腳本 --port COM20、地面站 backend 設 COM21,即可跳過 RF 直餵。
"""
import argparse
import math
import random
import sys
import time

# ── OpenRocket 匯出欄位 index(與本專案 sim CSV 一致)──────────────
COL_T, COL_ALT, COL_VV, COL_VACC = 0, 1, 3, 5
COL_LAT, COL_LON = 13, 14
COL_PRESS = 48  # Air pressure (mbar = hPa)

# 韌體狀態碼(與 firmware-rocket main.c FlightState_t 一致)
ST_IDLE, ST_LAUNCHED, ST_DEPLOYING, ST_DEPLOYED, ST_LANDED = 0, 1, 2, 3, 4


def load_sim(path):
    rows, events = [], {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("# Event"):
                # "# Event APOGEE occurred at t=16.733 seconds"
                try:
                    name = line.split("Event ")[1].split(" occurred")[0]
                    t = float(line.split("t=")[1].split(" ")[0])
                    events.setdefault(name, t)
                except (IndexError, ValueError):
                    pass
                continue
            if line.startswith("#") or not line.strip():
                continue
            c = line.split(",")
            try:
                rows.append({
                    "t": float(c[COL_T]),
                    "alt": float(c[COL_ALT]),
                    "vv": float(c[COL_VV]),
                    "vacc": float(c[COL_VACC]),
                    "lat": float(c[COL_LAT]),
                    "lon": float(c[COL_LON]),
                    "press": float(c[COL_PRESS]),
                })
            except (IndexError, ValueError):
                continue
    return rows, events


def interp(rows, t):
    """線性內插取 t 時刻的模擬量(rows 已按時間排序)"""
    if t <= rows[0]["t"]:
        return rows[0]
    if t >= rows[-1]["t"]:
        return rows[-1]
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = rows[lo], rows[hi]
    k = (t - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    return {key: a[key] + k * (b[key] - a[key]) for key in a}


def flight_state(t, ev):
    """由模擬事件推韌體狀態機的 ST 值(現行 5 狀態)"""
    t_liftoff = ev.get("LIFTOFF", 1.1)
    t_deploy = ev.get("RECOVERY_DEVICE_DEPLOYMENT",
                      ev.get("APOGEE", 16.6) + 1.5)   # 傘壞的模擬檔:用 apogee+1.5s
    t_ground = ev.get("GROUND_HIT", 1e9)
    if t < t_liftoff:
        return ST_IDLE
    if t < t_deploy:
        return ST_LAUNCHED
    if t < t_deploy + 1.0:                  # DEPLOY_PULSE_MS = 1s
        return ST_DEPLOYING
    if t < t_ground + 10.0:                 # LANDED 需靜止 10s
        return ST_DEPLOYED
    return ST_LANDED


def flight_state12(t, ev, rate_hz):
    """st.md 12 狀態版:事件(2/4/6/7/9/10)接管 ST 連發 4 幀後回主狀態。
    地面站以第一幀時間戳去重、鎖 T0/T+。"""
    t_ign = ev.get("IGNITION", 0.0)
    t_burn = ev.get("BURNOUT", 5.9)
    t_apo = ev.get("APOGEE", 16.6)
    t_dep = ev.get("RECOVERY_DEVICE_DEPLOYMENT", t_apo + 1.5)
    t_gnd = ev.get("GROUND_HIT", 1e9)
    win = 4.0 / rate_hz                     # 事件連發 4 幀的時間窗(2Hz=2s)
    # 事件優先(最近開始者勝,涵蓋 TOUCHDOWN/AIRBAG 相鄰重疊)
    events = [(t_ign, 2), (t_burn, 4), (t_apo, 6), (t_dep, 7),
              (t_gnd, 9), (t_gnd + win, 10)]
    active = [(ts, code) for ts, code in events if ts <= t < ts + win]
    if active:
        return max(active)[1]               # 最近開始的事件接管
    # 底層持續狀態
    if t < -2.0:
        return 0                            # IDLE(上電自檢)
    if t < t_ign:
        return 1                            # ARMED(上架待發)
    if t < t_burn:
        return 3                            # POWERED_FLIGHT
    if t < t_apo:
        return 5                            # COASTING
    if t < t_gnd:
        return 8                            # DESCENT
    return 11                               # LANDED(落海待救)


def make_packet(t_ms, d, st):
    """組火箭端 2Hz 遙測封包(格式與 main.c LoRa 打包一字不差)"""
    n = random.gauss  # 感測噪聲讓資料「像活的」
    az_g = (d["vacc"] + 9.80665) / 9.80665 + n(0, 0.01)   # 比力近似(垂直向)
    kh = d["alt"] + n(0, 0.15)
    return (
        "T{tms} AX{ax:+.3f} AY{ay:+.3f} AZ{az:+.3f} "
        "GX{gx:+.2f} GY{gy:+.2f} GZ{gz:+.2f} "
        "P{p:.2f} RH{rh:.1f} KH{kh:.1f} VZ{vz:+.2f} GA{ga:.2f} "
        "ST:{st} MOD:F GPS:1,{sats} C:0 LAT{lat:+.5f} LON{lon:+.5f}\r\n"
    ).format(
        tms=t_ms,
        ax=n(0, 0.02), ay=n(0, 0.02), az=az_g,
        gx=n(0, 3), gy=n(0, 3), gz=n(0, 3),
        p=d["press"] + n(0, 0.02),
        rh=d["alt"] + n(0, 0.1), kh=kh, vz=d["vv"] + n(0, 0.05),
        ga=abs(az_g),
        st=st, sats=random.randint(14, 20),
        lat=d["lat"], lon=d["lon"],
    )


def main():
    ap = argparse.ArgumentParser(description="OpenRocket CSV -> LoRa telemetry replay")
    ap.add_argument("csv", help="OpenRocket exported CSV (e.g. 5k_p10_h.csv)")
    ap.add_argument("--port", help="COM port of USB-TTL + E22 (e.g. COM7)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    ap.add_argument("--rate", type=float, default=2.0, help="packet rate Hz (firmware: 2)")
    ap.add_argument("--pre", type=float, default=5.0, help="seconds of IDLE before T0")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--dry", action="store_true", help="print packets instead of serial")
    ap.add_argument("--st12", action="store_true",
                    help="use st.md 12-stage codes with 4-frame event bursts "
                         "(for the new ground-station timeline); default = "
                         "current firmware 5-state codes")
    args = ap.parse_args()

    rows, ev = load_sim(args.csv)
    if not rows:
        sys.exit("no data rows parsed - is this an OpenRocket export?")
    t_end = min(rows[-1]["t"], ev.get("GROUND_HIT", rows[-1]["t"]) + 30.0)
    print(f"loaded {len(rows)} rows, events: "
          + ", ".join(f"{k}@{v:.2f}s" for k, v in sorted(ev.items(), key=lambda x: x[1])))

    ser = None
    if not args.dry:
        if not args.port:
            sys.exit("--port required (or use --dry)")
        import serial  # pyserial
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"serial open: {args.port} @ {args.baud}")

    def emit(s):
        if ser:
            ser.write(s.encode("ascii", errors="replace"))
        else:
            print(s, end="")

    period = 1.0 / args.rate
    while True:
        # 開機報告(讓地面站看到 session 起點)
        emit("MOD: BMP=1 IMU=1 LORA=1  REF_PRESS={:.2f} hPa  CLK=HSE  RST=SIM-REPLAY\r\n"
             .format(rows[0]["press"]))
        deploy_sent = False
        t_sim = -args.pre                    # 負值=發射前 IDLE 墊場
        boot_ms = 3890                        # 模仿真韌體第一包 uptime
        while t_sim < t_end:
            d = interp(rows, max(t_sim, 0.0))
            if args.st12:
                st = flight_state12(t_sim, ev, args.rate)
            else:
                st = ST_IDLE if t_sim < 0 else flight_state(t_sim, ev)
            t_ms = boot_ms + int((t_sim + args.pre) * 1000)
            emit(make_packet(t_ms, d, st))
            # 開傘事件 MSG(與韌體自動開傘的下傳行為一致)
            if not deploy_sent and st in (ST_DEPLOYING, ST_DEPLOYED):
                apo = ev.get("APOGEE", 0)
                peak = interp(rows, apo)["alt"] if apo else d["alt"]
                emit("MSG SUCCESS Parachute deployed (auto A+B pk={:.1f}m now={:.1f}m "
                     "vz={:.2f}m/s)\r\n".format(peak, d["alt"], d["vv"]))
                deploy_sent = True
            time.sleep(period / args.speed)
            t_sim += period
        print("\n[replay] flight complete "
              f"({'looping' if args.loop else 'done'})")
        if not args.loop:
            break
        time.sleep(2.0 / args.speed)

    if ser:
        ser.close()


if __name__ == "__main__":
    main()
