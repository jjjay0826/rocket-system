#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""從遙測反解完整速度向量（含水平分量），求開傘瞬間的總速度。

## 為什麼需要這支

氣壓計只給垂直速度。GPS 在上升段解算凍結（17 秒只移動 49 m），不能用。
但加速度計給了第三條路：

  自由飛行段（燃燒結束後、開傘前）唯一的非重力外力是阻力，
  所以加速度計量到的比力大小 GA·g 就是「阻力加速度」：

      GA(t)·g = k · |v(t)|²          k = ½ρCdA/m

  這裡有兩個未知數（k 與水平速度），但有兩組獨立觀測：
  ① 氣壓高度剖面 → 垂直速度
  ② GA 歷程       → 總速度大小

  ★ 關鍵：在**頂點**垂直速度 = 0，所以那一刻 GA 完全由水平速度造成。
     這讓水平分量變成可觀測量，不必依賴 GPS。

## 方法

以燃燒結束為初始條件，用二次阻力做 2D 彈道積分：

    dv/dt = -g·ẑ - k·|v|·v

對 (vz0, vx0, k) 做網格搜尋，同時擬合氣壓高度剖面與 GA 歷程。

## 假設與限制（★會影響結論，必讀）

- **等向阻力**：假設 Cd 與攻角無關。會風標的火箭攻角小，這個近似還可以，
  但開傘前速度低、姿態擺盪時失準。
- **k 為常數**：忽略空氣密度隨高度變化（0→840 m 差約 8%）與馬赫數效應。
- **GA 在頂點只有 0.06 g**，接近加速度計雜訊底（靜置散布約 ±0.015 g）。
  水平速度的相對誤差因此不小 —— 本程式會輸出敏感度區間，**看區間不要看點值**。
- 遙測 2 Hz，取樣點之間的變化看不到。
"""
import argparse
import csv
import math

import numpy as np

G = 9.80665


def load(p):
    r = list(csv.DictReader(open(p, encoding="utf-8")))
    d = {}
    for k in "t_rel rh ga vz".split():
        d[k] = np.array([float(x[k]) if x[k] else np.nan for x in r])
    d["st"] = np.array([int(x["st"]) if x["st"] else -1 for x in r])
    m = d["t_rel"] >= -1.0
    return {k: v[m] for k, v in d.items()}


def integrate(vz0, vx0, k, z0, t0, t_end, dt=0.01):
    """二次阻力 2D 彈道積分（RK4）。回傳 (t, z, x, vz, vx, a_drag)。"""
    n = int((t_end - t0) / dt) + 1
    T = np.empty(n); Z = np.empty(n); X = np.empty(n)
    VZ = np.empty(n); VX = np.empty(n); AD = np.empty(n)
    z, x, vz, vx = z0, 0.0, vz0, vx0

    def deriv(z, x, vz, vx):
        v = math.hypot(vz, vx)
        return vz, vx, -G - k * v * vz, -k * v * vx

    for i in range(n):
        T[i] = t0 + i * dt; Z[i] = z; X[i] = x; VZ[i] = vz; VX[i] = vx
        AD[i] = k * (vz * vz + vx * vx)
        k1 = deriv(z, x, vz, vx)
        k2 = deriv(z + .5*dt*k1[0], x + .5*dt*k1[1], vz + .5*dt*k1[2], vx + .5*dt*k1[3])
        k3 = deriv(z + .5*dt*k2[0], x + .5*dt*k2[1], vz + .5*dt*k2[2], vx + .5*dt*k2[3])
        k4 = deriv(z + dt*k3[0], x + dt*k3[1], vz + dt*k3[2], vx + dt*k3[3])
        z  += dt/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0])
        x  += dt/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1])
        vz += dt/6*(k1[2]+2*k2[2]+2*k3[2]+k4[2])
        vx += dt/6*(k1[3]+2*k2[3]+2*k3[3]+k4[3])
    return T, Z, X, VZ, VX, AD


def cost(p, obs_t, obs_h, obs_ga, z0, t0, t_end, w_ga):
    vz0, vx0, k = p
    if k <= 0 or vz0 <= 0 or vx0 < 0:
        return 1e18
    T, Z, X, VZ, VX, AD = integrate(vz0, vx0, k, z0, t0, t_end)
    h = np.interp(obs_t, T, Z)
    a = np.interp(obs_t, T, AD) / G          # 轉成 g，與 GA 同單位
    # 高度殘差以 m 計、GA 殘差以 g 計 —— 用 w_ga 把兩者放到可比的尺度
    return float(np.mean((h - obs_h) ** 2) + w_ga * np.mean((a - obs_ga) ** 2))


def fit(obs_t, obs_h, obs_ga, z0, t0, t_end, w_ga=3e5):
    """由粗到細的網格搜尋。沒有 scipy，但三個參數的網格搜尋綽綽有餘。"""
    best = (None, 1e18)
    rng = [(80., 160., 12), (0., 90., 12), (1e-4, 8e-4, 12)]
    for _ in range(7):
        grids = [np.linspace(lo, hi, n) for lo, hi, n in rng]
        for vz0 in grids[0]:
            for vx0 in grids[1]:
                for k in grids[2]:
                    c = cost((vz0, vx0, k), obs_t, obs_h, obs_ga, z0, t0, t_end, w_ga)
                    if c < best[1]:
                        best = ((vz0, vx0, k), c)
        vz0, vx0, k = best[0]
        rng = [(vz0 - (rng[0][1]-rng[0][0])/6, vz0 + (rng[0][1]-rng[0][0])/6, 9),
               (max(0., vx0 - (rng[1][1]-rng[1][0])/6), vx0 + (rng[1][1]-rng[1][0])/6, 9),
               (max(1e-5, k - (rng[2][1]-rng[2][0])/6), k + (rng[2][1]-rng[2][0])/6, 9)]
    return best


def analyse(path, name, t_burnout, t_deploy_cmd, t_shock, mass):
    d = load(path)
    t, rh, ga = d["t_rel"], d["rh"], d["ga"]

    # 擬合區間：燃燒結束後 0.5s（等推力尾流過去）到開傘指令前 0.5s
    lo, hi = t_burnout + 0.5, t_deploy_cmd - 0.5
    m = (t >= lo) & (t <= hi)
    obs_t, obs_h, obs_ga = t[m], rh[m], ga[m]
    z0 = float(np.interp(lo, t, rh))

    (vz0, vx0, k), c = fit(obs_t, obs_h, obs_ga, z0, lo, t_shock + 1.0)
    T, Z, X, VZ, VX, AD = integrate(vz0, vx0, k, z0, lo, t_shock + 1.0)

    L = []
    W = L.append
    W("=" * 76)
    W(f"{name}  ——  二次阻力 2D 彈道反解")
    W("=" * 76)
    W(f"  擬合區間  T+{lo:.2f} ~ {hi:.2f}s（{m.sum()} 個取樣點），起點高度 {z0:.1f} m")
    W(f"  RMS 殘差  高度 {math.sqrt(np.mean((np.interp(obs_t,T,Z)-obs_h)**2)):.2f} m"
      f"   GA {math.sqrt(np.mean((np.interp(obs_t,T,AD)/G-obs_ga)**2)):.4f} g")
    W("")
    W(f"  解出的參數")
    W(f"    起點垂直速度  {vz0:7.1f} m/s")
    W(f"    起點水平速度  {vx0:7.1f} m/s")
    W(f"    阻力係數 k    {k:.3e} 1/m   （a_drag = k·v²）")
    ang = math.degrees(math.atan2(vx0, vz0))
    W(f"    起點飛行路徑角 偏離鉛直 {ang:.1f}°")

    i_ap = int(np.argmax(Z))
    W("")
    W(f"  【頂點】T+{T[i_ap]:.2f}s   高度 {Z[i_ap]:.1f} m")
    W(f"    垂直速度 {VZ[i_ap]:+.2f} m/s（定義上 = 0）")
    W(f"    ★水平速度 {VX[i_ap]:.1f} m/s   ← 這一刻總速度全是水平的")
    W(f"    水平位移 {X[i_ap]:.0f} m")

    W("")
    W("  【開傘】")
    for lab, tq in (("指令發出（板1 A∧B）", t_deploy_cmd), ("開傘衝擊（GA 峰值）", t_shock)):
        vzq = float(np.interp(tq, T, VZ)); vxq = float(np.interp(tq, T, VX))
        vq = math.hypot(vzq, vxq); zq = float(np.interp(tq, T, Z))
        ang = math.degrees(math.atan2(abs(vxq), abs(vzq)))
        q = 0.5 * 1.15 * vq * vq
        W(f"    {lab}  T+{tq:.2f}s   高度 {zq:.0f} m")
        W(f"      垂直 {vzq:+7.2f} m/s   水平 {vxq:6.2f} m/s")
        W(f"      ★總速度 {vq:6.2f} m/s   （偏離鉛直 {ang:.0f}°）")
        W(f"      動壓 q = {q:.0f} Pa  →  每 m² 傘面 {q/9.80665:.0f} kgf")
    W("")
    W(f"  【水平位移對照】模型算到開傘時 {float(np.interp(t_shock,T,X)):.0f} m")
    W(f"    GPS 在開傘後跳到東方 507~601 m —— 若模型對，GPS 跳的是「追上真值」")

    return L, dict(vz0=vz0, vx0=vx0, k=k, T=T, Z=Z, X=X, VZ=VZ, VX=VX,
                   ap_t=T[i_ap], ap_h=Z[i_ap], ap_vx=VX[i_ap])


def sensitivity(path, t_burnout, t_deploy_cmd, t_shock):
    """GA 在頂點只有 0.06 g，接近雜訊底。把 GA 整體加減一個偏差重跑，看水平速度怎麼變。"""
    L = ["", "=" * 76, "敏感度：加速度計偏差對「水平速度」的影響", "=" * 76,
         "  頂點 GA 只有 0.06 g，靜置散布約 ±0.015 g。把整條 GA 平移後重解：", ""]
    d = load(path)
    t, rh, ga0 = d["t_rel"], d["rh"], d["ga"]
    lo, hi = t_burnout + 0.5, t_deploy_cmd - 0.5
    m = (t >= lo) & (t <= hi)
    z0 = float(np.interp(lo, t, rh))
    L.append(f"  {'GA 偏差':>10} {'頂點水平速度':>14} {'開傘總速度':>12} {'k':>12}")
    for bias in (-0.015, -0.008, 0.0, +0.008, +0.015):
        gab = np.clip(ga0[m] + bias, 1e-4, None)
        (vz0, vx0, k), _ = fit(t[m], rh[m], gab, z0, lo, t_shock + 1.0)
        T, Z, X, VZ, VX, AD = integrate(vz0, vx0, k, z0, lo, t_shock + 1)
        i = int(np.argmax(Z))
        vq = math.hypot(float(np.interp(t_shock, T, VZ)), float(np.interp(t_shock, T, VX)))
        L.append(f"  {bias:+10.3f} g {VX[i]:12.1f} m/s {vq:10.1f} m/s {k:12.3e}")
    return L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv1"); ap.add_argument("csv2", nargs="?")
    ap.add_argument("--mass", type=float, default=25.2)
    a = ap.parse_args()

    out = []
    # 板 2（ch2）：燃燒結束 T+3.0~3.5、板1 開傘換算到板2 時間軸 = T+16.49、衝擊 T+18.0
    L, r2 = analyse(a.csv1, "板 2 (ch2)", 3.5, 16.49, 18.04, a.mass)
    out += L
    if a.csv2:
        # 板 1（ch1）：自己的時間軸 —— 燃燒結束 T+3.0、開傘指令 T+16.02、衝擊 T+17.52
        L, r1 = analyse(a.csv2, "板 1 (ch1)", 3.0, 16.02, 17.52, a.mass)
        out += [""] + L
    out += sensitivity(a.csv1, 3.5, 16.49, 18.04)
    print("\n".join(out))


if __name__ == "__main__":
    main()
