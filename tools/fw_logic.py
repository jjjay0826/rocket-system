#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fw_logic.py — 用 Python 重跑韌體的開傘決策，餵 OpenRocket 的模擬資料

為什麼需要這支
  replay309.py 的 ST 是【查表】—— 它直接讀 OpenRocket 的
  recoverydevicedeployment 事件當開傘時刻。那驗的是【地面站顯示層】，
  完全沒有碰到韌體真正的判斷 `(A ∧ B) ∨ C`。

  而 an309.py 的 81 組分析是用【彈道解析式】估 A∧B 的時間
  （掉 10m 要 √(2·10/g)=1.43s、B 要 1.55s），沒有真的跑濾波器，
  也沒有感測器雜訊。

  這支把 main.c 的規則逐條實作出來，用模擬的高度/加速度當輸入，
  加上真實的感測器雜訊，看韌體【會在什麼時候開傘】。

  這是不飛就能驗開傘邏輯的唯一方法。

規則來源（firmware-rocket/Core/Src/main.c，逐一對照行號）
  73   LAUNCH_AZ_G      2.5     離架：total_g ≥ 2.5g 持續 200ms（g2_count ≥ 20 @100Hz）
  85   DEPLOY_DROP_M    10.0    A：跌破峰值 10m
  86   DEPLOY_PEAK_MIN_M 20.0   A：峰值需 ≥ 20m（防地面誤觸）
  87   DEPLOY_VZ_NEG_THR -0.5   B：kf2_v 低於此值
  88   DEPLOY_VZ_NEG_MS 1500    B：持續 1.5s
  89   DEPLOY_TB_MS     18000   C：離架後 18s 強制
  135~ KF2_*                    2 態卡爾曼 [高度, 速度]
  354~ LAND_*                   落地：|g−1|<0.15 且 rel_alt<30m 持續 10s

用法
  python tools/fw_logic.py                      # 全 81 組
  python tools/fw_logic.py --sim T110_e81_W4    # 單組，印逐步細節
  python tools/fw_logic.py --noise 0            # 關掉感測器雜訊
"""
import argparse
import glob
import math
import os
import random
import zipfile
import xml.etree.ElementTree as ET

SIM_DIR = r"D:\Downloads\sim_309"
G = 9.80665

# ── main.c 的常數，一個都不要改成「差不多的值」 ──────────────────────
LAUNCH_AZ_G = 2.5
LAUNCH_SUSTAIN_S = 0.20         # g2_count >= 20 @ 100Hz
DEPLOY_DROP_M = 10.0
DEPLOY_PEAK_MIN_M = 20.0
DEPLOY_VZ_NEG_THR = -0.5
DEPLOY_VZ_NEG_S = 1.5
DEPLOY_TB_S = 18.0
KF2_Q_H, KF2_Q_V = 0.0001, 0.001
KF2_R_H = 0.25
KF2_R_HIGHG_MULT = 25.0
KF2_RESET_THR_M = 5.0
KF2_RESET_GMAX = 1.5
KF2_AZ_CLAMP = 160.0

IMU_HZ, BARO_HZ = 100.0, 50.0   # 韌體的實際取樣率


def load(path):
    out = []
    root = ET.fromstring(zipfile.ZipFile(path).read("rocket.ork").decode("utf-8", "ignore"))
    for s in root.findall(".//simulation"):
        br = s.find(".//flightdata/databranch")
        if br is None:
            continue
        ix = {c: i for i, c in enumerate(br.get("types").split(","))}
        rows = []
        for dp in br.findall("datapoint"):
            v = [float(x) if x.strip() not in ("NaN", "") else float("nan")
                 for x in dp.text.split(",")]
            rows.append((v[ix["Time"]], v[ix["Altitude"]],
                         v[ix["Vertical acceleration"]], v[ix["Total acceleration"]]))
        ev = {e.get("type"): float(e.get("time")) for e in br.findall("event")}
        out.append((s.find("name").text, rows, ev))
    return out


def interp(rows, t):
    if t <= rows[0][0]:
        return rows[0]
    if t >= rows[-1][0]:
        return rows[-1]
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if rows[m][0] <= t:
            lo = m
        else:
            hi = m
    a, b = rows[lo], rows[hi]
    k = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    return tuple(a[i] + k * (b[i] - a[i]) for i in range(4))


def run_one(rows, ev, noise=1.0, seed=0, trace=False):
    """跑一遍韌體邏輯。回傳 dict。"""
    rnd = random.Random(seed)
    dt = 1.0 / IMU_HZ
    baro_every = int(IMU_HZ / BARO_HZ)

    imu_armed = False
    g2_count = 0
    t_launch = None
    kf2_h = kf2_v = 0.0
    p00, p01, p11 = 1.0, 0.0, 1.0
    peak = 0.0
    vz_neg_start = None
    cond_A = cond_B = 0
    t_A = t_B = t_C = None
    t_deploy = None
    why = None

    t_end = ev.get("groundhit", rows[-1][0])
    n = int(t_end / dt)
    for i in range(n):
        t = i * dt
        _, alt_true, vacc, tacc = interp(rows, t)

        # 感測器：加速度計量的是比力（含重力反作用），氣壓計有 ±0.5m RMS
        total_g = abs(vacc + G) / G + rnd.gauss(0, 0.012) * noise
        rel_alt = alt_true + rnd.gauss(0, 0.5) * noise

        # ── 離架偵測：total_g ≥ 2.5g 持續 200ms ──
        if not imu_armed:
            if total_g >= LAUNCH_AZ_G:
                g2_count += 1
                if g2_count >= int(LAUNCH_SUSTAIN_S * IMU_HZ):
                    imu_armed = True
                    t_launch = t
                    kf2_h = kf2_v = 0.0
                    p00, p01, p11 = 1.0, 0.0, 1.0
                    peak = 0.0
                    vz_neg_start = None
                    cond_A = cond_B = 0
            else:
                g2_count = 0
            continue

        # ── KF2 預測（100Hz，用世界系垂直加速度）──
        lin_az = max(-KF2_AZ_CLAMP, min(KF2_AZ_CLAMP, (total_g - 1.0) * G))
        kf2_h += kf2_v * dt + 0.5 * lin_az * dt * dt
        kf2_v += lin_az * dt
        p00 = p00 + dt * (2 * p01) + dt * dt * p11 + KF2_Q_H
        p01 = p01 + dt * p11
        p11 = p11 + KF2_Q_V

        # ── KF2 更新（50Hz，用氣壓高度）──
        if i % baro_every == 0:
            r_h = KF2_R_H * (KF2_R_HIGHG_MULT if total_g > 2.0 else 1.0)
            innov = rel_alt - kf2_h
            if abs(innov) > KF2_RESET_THR_M and total_g < KF2_RESET_GMAX:
                kf2_h, kf2_v = rel_alt, 0.0
                p00, p01, p11 = 1.0, 0.0, 1.0
            else:
                s = p00 + r_h
                k0, k1 = p00 / s, p01 / s
                kf2_h += k0 * innov
                kf2_v += k1 * innov
                p00 -= k0 * p00
                p01 -= k0 * p01
                p11 -= k1 * p01

            # ── 條件 A：裸氣壓高度跌破峰值 10m（峰值需 ≥ 20m）──
            if rel_alt > peak:
                peak = rel_alt
            cond_A = 1 if (peak >= DEPLOY_PEAK_MIN_M
                           and rel_alt < peak - DEPLOY_DROP_M) else 0
            if cond_A and t_A is None:
                t_A = t

        # ── 條件 B：kf2_v 持續向下 1.5s ──
        if kf2_v < DEPLOY_VZ_NEG_THR:
            if vz_neg_start is None:
                vz_neg_start = t
            cond_B = 1 if (t - vz_neg_start) >= DEPLOY_VZ_NEG_S else 0
        else:
            vz_neg_start = None
            cond_B = 0
        if cond_B and t_B is None:
            t_B = t

        # ── 條件 C：離架後 18s ──
        if (t - t_launch) >= DEPLOY_TB_S and t_C is None:
            t_C = t

        if t_deploy is None:
            if cond_A and cond_B:
                t_deploy, why = t, "A∧B"
            elif (t - t_launch) >= DEPLOY_TB_S:
                t_deploy, why = t, "C"
            if t_deploy is not None and trace:
                print(f"    開傘 t={t:.2f}s  由 {why}  "
                      f"kf2_h={kf2_h:.1f} kf2_v={kf2_v:+.2f} peak={peak:.1f} alt={rel_alt:.1f}")

    apo_t = max(rows, key=lambda r: r[1])[0]
    apo_h = max(r[1] for r in rows)
    return dict(t_launch=t_launch, t_A=t_A, t_B=t_B, t_C=t_C,
                t_deploy=t_deploy, why=why, apo_t=apo_t, apo_h=apo_h,
                ork_dep=ev.get("recoverydevicedeployment"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", help="只跑這一組，並印細節")
    ap.add_argument("--noise", type=float, default=1.0, help="感測器雜訊倍率（0=關）")
    ap.add_argument("--trials", type=int, default=1, help="每組跑幾次（雜訊不同種子）")
    args = ap.parse_args()

    sims = []
    for p in sorted(glob.glob(os.path.join(SIM_DIR, "m309_T*.ork"))):
        sims += load(p)
    if args.sim:
        sims = [s for s in sims if s[0] == args.sim]
        if not sims:
            raise SystemExit(f"找不到 {args.sim}")

    print("=" * 78)
    print("用 Python 重跑韌體的 (A∧B)∨C —— 輸入是 OpenRocket 的模擬 + 感測器雜訊")
    print("=" * 78)
    print(f"{'組合':<18}{'離架':>7}{'頂點':>8}{'A':>7}{'B':>7}{'開傘':>7}"
          f"{'由':>5}{'頂點後':>8}{'C餘裕':>8}")

    bad, cfirst, rows_out = [], [], []
    for name, rows, ev in sims:
        for k in range(args.trials):
            r = run_one(rows, ev, args.noise, seed=hash((name, k)) & 0xFFFF,
                        trace=bool(args.sim))
            if r["t_deploy"] is None or r["t_launch"] is None:
                print(f"{name:<18}  ✗ 沒有開傘")
                bad.append(name)
                continue
            after_apo = r["t_deploy"] - r["apo_t"]
            margin = (r["t_launch"] + DEPLOY_TB_S) - r["apo_t"]
            if r["why"] == "C":
                cfirst.append(name)
            if after_apo < 0:
                bad.append(f"{name}(上升段開傘)")
            rows_out.append((name, after_apo, margin, r))
            print(f"{name:<18}{r['t_launch']:>7.2f}{r['apo_t']:>8.2f}"
                  f"{(r['t_A'] or float('nan')):>7.2f}{(r['t_B'] or float('nan')):>7.2f}"
                  f"{r['t_deploy']:>7.2f}{r['why']:>5}{after_apo:>+8.2f}{margin:>+8.2f}")

    if len(sims) > 1:
        print()
        print("=" * 78)
        aa = [r[1] for r in rows_out]
        mm = [r[2] for r in rows_out]
        print(f"  跑了 {len(rows_out)} 組")
        print(f"  ★上升段開傘：{len(bad)} 組  {'✓ 沒有' if not bad else '✗ ' + str(bad)}")
        print(f"  ★由 C 觸發：{len(cfirst)}/{len(rows_out)}"
              f"  {'✓ A∧B 全部先到' if not cfirst else str(sorted(set(cfirst)))}")
        print(f"  開傘落在頂點後 {min(aa):+.2f} ~ {max(aa):+.2f}s（平均 {sum(aa)/len(aa):+.2f}）")
        print(f"  C 備援餘裕     {min(mm):+.2f} ~ {max(mm):+.2f}s（最小 {min(mm):.2f}）")
        worst = min(rows_out, key=lambda r: r[2])
        print(f"  最緊的一組     {worst[0]}  餘裕 {worst[2]:.2f}s")


if __name__ == "__main__":
    main()
