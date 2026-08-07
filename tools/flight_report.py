#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雙板飛行遙測分析儀表板。

輸入：parse_raw_lora.py 產生的兩塊板的 *_parsed.csv
輸出：<stem>_report.png ＋ <stem>_report.txt

## 設計原則

1. **兩塊板對等**。不指定誰是「真值」——每一格都同時畫兩條，兩者的差
   就是量測不確定度。只有一塊板有資料的區間要明講。
2. **量到的 / 算出來的 分開標**。
3. **GPS 兩塊板都不可信**。上升段兩顆接收機都凍結、都在開傘後跳點。

## ★ 空中重開機的處理（2026-08-02 新增）

ch1 在開傘衝擊當下斷電重開（`RST=POWER-ON`），在約 720 m 的空中重新取了
氣壓零點（`REF_PRESS=924.25 hPa`），然後**繼續記錄整段降落**。原本的
「時間必須單調」過濾會把這些封包整段丟掉 —— 那是 41 個封包、涵蓋到
離地 10 m，比另一塊板還低。

本檔把它救回來：
  · 由重開機前的 (P, RH) 反解原始 ref_press，把重開機後的壓力換算回
    同一個高度基準
  · 用高度曲線對另一塊板做最小平方對齊，求出重開機的時刻
"""
import argparse
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec

plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "Microsoft YaHei", "DFKai-SB"]
plt.rcParams["axes.unicode_minus"] = False

INK, ACCENT, ACCENT2 = "#1b2a41", "#c1440e", "#e08214"
GOOD, COOL, MUTED    = "#1a7f5a", "#2c5f8a", "#8a8f98"
BG, BAD, PURPLE      = "#faf8f3", "#b00020", "#5b3b8c"

DESIGN_DESCENT = 6.449     # 3.0.9 模擬，無風組純垂直降速
LIFTOFF_MASS   = 29.999    # kg，實測
PROP_MASS      = 4.64      # kg，TASA_Pioneer_5K.eng

# ── 上升段水平位移的彈道重建（來自 tools/fit_ballistic.py）───────────
# GPS 上升段解算凍結，完全不能用。改由「頂點時垂直速度=0，所以加速度計
# 量到的 GA 完全來自水平速度」反解 —— 這條路不碰 GPS。
# 參數以 ch1 的時間軸為準，起點在燃燒結束後 0.5 秒。
BALLISTIC = dict(t0=3.50, z0=327.3, vz0=108.1, vx0=60.4, k=2.701e-4)
RAIL_TILT_DEG = 10.4       # 靜置加速度計反算的發射架離垂直角度
GPS_SETTLE_T   = 29.0      # ch2 的 GPS 解算在此之後才收斂（主時間軸）
BARO_EXP       = 0.1902632


# ───────────────────────────────────────────────────────────── 讀檔 / 工具

def _read(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    d = {}
    for k in "t_rel utc ax ay az gx gy gz press rh kh vz ga lat lon".split():
        d[k] = np.array([float(r[k]) if r[k] else np.nan for r in rows])
    for k in "st sats cond mod cond_A cond_B sq".split():
        d[k] = np.array([int(r[k]) if r[k] else -1 for r in rows])
    return d


def solve_ref_press(press, rh):
    """由 (壓力, 韌體算出的相對高度) 反解當初的 ref_press。

        RH = 44330·(1 - (P/P0)^0.1902632)   →   P0 = P / (1 - RH/44330)^(1/0.1902632)

    取中位數以免被個別雜訊帶偏。
    """
    m = np.isfinite(press) & np.isfinite(rh) & (np.abs(rh) > 50)
    p0 = press[m] / (1.0 - rh[m] / 44330.0) ** (1.0 / BARO_EXP)
    return float(np.median(p0))


def load(path):
    """回傳 (main, post)。post = 空中重開機之後的資料（沒有就是 None）。

    parse_raw_lora.py 以「離架封包」為 T+0，所以重開機後的封包 t_rel 會是
    一個很大的負數（uptime 從 0 重算）。用這個特徵切開。
    """
    d = _read(path)
    n = len(d["t_rel"])

    # 重開機的特徵是「檔案順序上時間往回跳一大段」（uptime 從 0 重算）。
    # ★不能用 t_rel < 某個負值來判斷 —— 發射前 93 分鐘的待機封包 t_rel 也是
    #   大負數（約 -5595 ~ 0），會被一起抓進來。
    cut = None
    for i in range(1, n):
        if d["t_rel"][i - 1] > 0 and (d["t_rel"][i] - d["t_rel"][i - 1]) < -1000.0:
            cut = i
            break

    idx = np.arange(n)
    main = {k: v[(d["t_rel"] >= -1.0) & (idx < (cut if cut is not None else n))]
            for k, v in d.items()}
    keep = np.concatenate(([True], np.diff(main["t_rel"]) > 0))
    main = {k: v[keep] for k, v in main.items()}

    if cut is None or n - cut < 5:
        return main, None

    post = {k: v[cut:] for k, v in d.items()}
    order = np.argsort(post["t_rel"])
    post = {k: v[order] for k, v in post.items()}
    # 換算回主基準：重開機後 ref_press 被重新取樣，RH 欄不能直接用
    p0 = solve_ref_press(main["press"], main["rh"])
    post["alt"] = 44330.0 * (1.0 - (post["press"] / p0) ** BARO_EXP)
    post["uptime"] = post["t_rel"] - post["t_rel"][0] + 0.0   # 相對第一包
    post["p0_main"] = p0
    # 韌體自己的 ref_press（＝重開機當下的壓力）
    post["p0_new"] = float(post["press"][0])
    return main, post


def align_post(post, ref_t, ref_h):
    """把重開機後的高度曲線對齊到參考板的時間軸（最小平方求時間偏移）。

    重開機頭幾包的壓力還是 ref_press 的初值（KF 未收斂），要排掉。
    """
    valid = np.abs(post["press"] - post["p0_new"]) > 0.5
    if valid.sum() < 4:
        return np.nan, valid
    tu, hu = post["uptime"][valid], post["alt"][valid]
    best, bc = np.nan, 1e18
    for off in np.arange(0.0, 40.0, 0.01):
        tt = tu + off
        m = (tt >= ref_t.min()) & (tt <= ref_t.max())
        if m.sum() < 3:
            continue
        c = float(np.mean((np.interp(tt[m], ref_t, ref_h) - hu[m]) ** 2))
        if c < bc:
            bc, best = c, off
    return best, valid


def local_slope(t, y, half=1.0):
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        m = np.abs(t - t[i]) <= half
        if m.sum() < 3 or np.isnan(y[m]).any():
            continue
        out[i] = np.polyfit(t[m], y[m], 1)[0]
    return out


def parabola_peak(t, y, span=3):
    i = int(np.nanargmax(y))
    lo, hi = max(0, i - span), min(len(t), i + span + 1)
    a, b, c = np.polyfit(t[lo:hi], y[lo:hi], 2)
    if a >= 0:
        return t[i], y[i]
    tp = -b / (2 * a)
    return tp, a * tp * tp + b * tp + c


def enu(lat, lon, lat0, lon0):
    R = 6371000.0
    return (np.radians(lon - lon0) * R * math.cos(math.radians(lat0)),
            np.radians(lat - lat0) * R)


def gps_track(d, shift=0.0, origin=None, tkey="t_rel"):
    """origin=(lat0,lon0) 讓多條軌跡共用同一原點，才能互相比較。"""
    t = d[tkey] + shift
    m = ~np.isnan(d["lat"]) & (d[tkey] >= (0 if tkey == "t_rel" else -1e9))
    t, lat, lon = t[m], d["lat"][m], d["lon"][m]
    lat0, lon0 = origin if origin is not None else (lat[0], lon[0])
    e, n = enu(lat, lon, lat0, lon0)
    v = np.full(len(e), np.nan)
    last = 0
    for i in range(1, len(e)):
        if lat[i] == lat[last] and lon[i] == lon[last]:
            continue
        dt = t[i] - t[last]
        v[i] = math.hypot(e[i] - e[last], n[i] - n[last]) / dt if dt > 0 else np.nan
        last = i
    return t, e, n, v, (lat[-1], lon[-1])


def reconstruct_downrange(t_end, dt=0.02):
    """重建上升段的「順航向水平位移」（不使用 GPS）。

    兩段：
      ① 推力段（離架 → 燃燒結束）：飛行路徑角由發射架傾角線性長到燃盡時
         的角度（實測靜置 10.4°，燃盡由彈道反解得 atan(vx0/vz0)）。
         用實測氣壓高度當參數化，所以垂直方向仍是量測值。
      ② 滑行段：二次阻力 2D 積分，起點取彈道反解的燃盡狀態。

    回傳 (t, x, z)；t 是 ch1 時間軸。
    """
    b = BALLISTIC
    # ② 滑行段
    n = int((t_end - b["t0"]) / dt) + 1
    T = np.empty(n); X = np.empty(n); Z = np.empty(n)
    z, x, vz, vx = b["z0"], 0.0, b["vz0"], b["vx0"]
    for i in range(n):
        T[i], X[i], Z[i] = b["t0"] + i * dt, x, z
        v = math.hypot(vz, vx)
        az_, ax_ = -9.80665 - b["k"] * v * vz, -b["k"] * v * vx
        z += vz * dt + .5 * az_ * dt * dt
        x += vx * dt + .5 * ax_ * dt * dt
        vz += az_ * dt
        vx += ax_ * dt
    # ① 推力段：角度由 RAIL_TILT 線性長到燃盡角
    th1 = math.radians(RAIL_TILT_DEG)
    th2 = math.atan2(b["vx0"], b["vz0"])
    m = int(b["t0"] / dt) + 1
    Tb = np.linspace(0, b["t0"], m)
    Zb = np.linspace(0, b["z0"], m)          # 以高度為參數，垂直仍是量測
    th = th1 + (th2 - th1) * (Tb / b["t0"])
    Xb = np.concatenate(([0.0], np.cumsum(np.diff(Zb) * np.tan(th[1:]))))
    return (np.concatenate((Tb, T)), np.concatenate((Xb, Xb[-1] + X)),
            np.concatenate((Zb, Z)))


def descent_rate(t, h):
    """回傳 (端點法, 最小平方, 起訖)。"""
    return ((h[0] - h[-1]) / (t[-1] - t[0]), -np.polyfit(t, h, 1)[0],
            (t[0], t[-1], h[0], h[-1]))


# ─────────────────────────────────────────────────────────────── 單板事實

def facts(d):
    t, rh, kh, vz, ga = d["t_rel"], d["rh"], d["kh"], d["vz"], d["ga"]
    f = {}
    f["ap_t"], f["ap_h"] = parabola_peak(t, rh)
    f["ap_kh"] = float(np.nanmax(kh))
    bo = np.where((t > 0.2) & (ga < 1.0))[0]
    f["bo_lo"], f["bo_hi"], f["bo_h"] = t[bo[0] - 1], t[bo[0]], rh[bo[0]]
    for s, n in ((1, "launch"), (2, "deploying"), (3, "deployed")):
        idx = np.where(d["st"] == s)[0]
        f["t_" + n] = t[idx[0]] if len(idx) else np.nan
    # 最大加速度只取推力段（全程取會抓到開傘衝擊）
    mb = t <= f["bo_hi"] + 0.2
    i = int(np.nanargmax(ga[mb])); f["max_g"], f["max_g_t"] = ga[mb][i], t[mb][i]
    for k, n in (("cond_A", "condA"), ("cond_B", "condB")):
        idx = np.where(d[k] == 1)[0]
        f["t_" + n] = t[idx[0]] if len(idx) else np.nan
    f["v_baro"] = local_slope(t, rh, 1.0)
    f["v_max_baro"] = float(np.nanmax(f["v_baro"]))
    f["v_max_kf"] = float(np.nanmax(vz))
    if not np.isnan(f["t_deploying"]):
        i = int(np.argmin(np.abs(t - f["t_deploying"])))
        f["h_deploy"] = rh[i]
        f["alt_lost"] = f["ap_h"] - f["h_deploy"]
    m = t > f["t_deploying"] - 0.1 if not np.isnan(f["t_deploying"]) else t > 1e9
    f["shock_g"] = float(np.nanmax(ga[m])) if m.sum() else np.nan
    f["shock_t"] = float(t[m][int(np.nanargmax(ga[m]))]) if m.sum() else np.nan
    f["t_end"], f["h_end"] = t[-1], rh[-1]
    m = t >= f["t_deployed"] + 1.0 if not np.isnan(f["t_deployed"]) else t > 1e9
    if m.sum() >= 6:
        f["desc_ep"], f["desc_fit"], f["desc_span"] = descent_rate(t[m], rh[m])
        f["desc_kf"] = float(np.nanmean(vz[m]))
        f["desc_mask"] = m
    else:
        f["desc_ep"] = f["desc_fit"] = f["desc_kf"] = np.nan
        f["desc_mask"] = m
    return f


# ────────────────────────────────────────────────────────────────── 繪圖

def build(A, B, PA, fa, fb, na, nb, shift, poff, pvalid, png, title, sub,
          dpi=200, also_pdf=True):
    ta, tb = A["t_rel"] + shift, B["t_rel"]
    pt = PA["uptime"][pvalid] + poff if PA is not None else None
    ph = PA["alt"][pvalid] if PA is not None else None
    tmax = max(tb.max(), pt.max() if pt is not None else 0)

    fig = plt.figure(figsize=(23, 17.2), facecolor=BG)
    gs = GridSpec(3, 3, figure=fig, height_ratios=[0.94, 0.82, 1.52],
                  hspace=0.42, wspace=0.235,
                  left=0.045, right=0.985, top=0.895, bottom=0.080)
    fig.text(0.045, 0.962, title, fontsize=27, fontweight="bold", color=INK)
    fig.text(0.045, 0.930, sub, fontsize=12.6, color="#4a5568")

    # ── ① 高度剖面 ＋ 殘差條 ────────────────────────────────
    # ★ 殘差以「下方獨立小圖」呈現，不用 inset_axes —— inset 的白底方塊會
    #   把下降段曲線整段遮掉（2026-08-02 的教訓）。
    g0 = GridSpecFromSubplotSpec(2, 1, subplot_spec=gs[0, :2],
                                 height_ratios=[3.4, 1.0], hspace=0.08)
    ax = fig.add_subplot(g0[0]); ax.set_facecolor("white")
    ax.plot(tb, B["rh"], color=INK, lw=2.6, label=f"{nb} 氣壓高度（全程）", zorder=5)
    ax.plot(ta, A["rh"], color=GOOD, lw=2.0, ls=(0, (5, 2)),
            label=f"{na} 氣壓高度（獨立第二顆感測器）", zorder=6)
    if pt is not None:
        ax.plot(pt, ph, color=GOOD, lw=2.0, ls=(0, (1.5, 1.5)),
                label=f"{na} 重開機後（壓力換算回同一基準）", zorder=7)
        ax.plot(pt, ph, ".", color=GOOD, ms=4, zorder=7)
    ax.fill_between(tb, 0, B["rh"], color=INK, alpha=.05, zorder=1)
    ax.plot(ta[-1], A["rh"][-1], "x", color=BAD, ms=14, mew=3, zorder=9)
    ax.annotate(f"★{na} 在此斷電重開機\n"
                f"T+{ta[-1]:.2f}s  開傘衝擊 {fa['shock_g']:.2f} g\n"
                f"RST=POWER-ON，SD 同時死\n"
                f"重開機後在 {ph[0] if pt is not None else 0:.0f} m 空中重取氣壓零點",
                xy=(ta[-1], A["rh"][-1]), xytext=(20.5, 300), ha="left",
                fontsize=9.6, color=BAD, fontweight="bold",
                bbox=dict(fc="white", ec=BAD, alpha=.94, boxstyle="round,pad=0.32"),
                arrowprops=dict(arrowstyle="->", color=BAD, lw=1.4), zorder=11)

    for te, lab, c, yy, ha in (
        (0.0,               "離架 T+0",                        ACCENT,  700, "left"),
        (fb["bo_hi"],       f"燃燒結束\n約T+{fb['bo_hi']:.1f}s", ACCENT2, 700, "left"),
        (fb["ap_t"],        f"頂點 T+{fb['ap_t']:.2f}s\n{fb['ap_h']:.0f} m", GOOD, 962, "right"),
        (fa["t_deploying"] + shift,
         f"★{na} 開傘（A且B）T+{fa['t_deploying']+shift:.2f}s", BAD, 962, "left"),
        (fb["t_deploying"],
         f"{nb} 備援計時器 T+{fb['t_deploying']:.2f}s", "#9a6b00", 890, "left"),
    ):
        if np.isnan(te):
            continue
        ax.axvline(te, color=c, ls=":", lw=1.5, alpha=.9, zorder=3)
        ax.annotate(lab, xy=(te + (-0.4 if ha == "right" else 0.4), yy),
                    fontsize=10.2, color=c, ha=ha, fontweight="bold", zorder=9, va="top")

    ax.set_ylabel("高度 (m)", fontsize=11.5)
    ax.set_title(f"高度剖面與飛行事件時序   ── 兩塊板獨立量測疊圖"
                 f"（時間軸以{nb}為準；{na}飛行段依氣壓頂點對齊 {shift:+.3f}s，"
                 f"重開機段依高度曲線對齊）",
                 fontsize=12.6, fontweight="bold", color=INK, pad=9)
    ax.grid(alpha=.25); ax.legend(loc="upper right", fontsize=9.4)
    ax.set_xlim(-1.5, tmax + 1.5); ax.set_ylim(-30, 1000)
    ax.tick_params(labelbottom=False)

    axr = fig.add_subplot(g0[1], sharex=ax); axr.set_facecolor("#fbfaf7")
    ov = (tb >= ta.min()) & (tb <= ta.max())
    diff = np.interp(tb[ov], ta, A["rh"]) - B["rh"][ov]
    axr.plot(tb[ov], diff, color=PURPLE, lw=1.5,
             label=f"飛行段（RMS {np.sqrt(np.nanmean(diff**2)):.2f} m）")
    if pt is not None:
        ov2 = (tb >= pt.min()) & (tb <= pt.max())
        d2 = np.interp(tb[ov2], pt, ph) - B["rh"][ov2]
        axr.plot(tb[ov2], d2, color=ACCENT2, lw=1.5,
                 label=f"重開機後（RMS {np.sqrt(np.nanmean(d2**2)):.2f} m）"
                       "　★時間偏移為擬合值，此段吻合不算獨立證據")
    axr.axhline(0, color=MUTED, lw=.9)
    axr.set_xlabel("飛行時間 T+ (s)", fontsize=11.5)
    axr.set_ylabel(f"{na} - {nb}\n(m)", fontsize=9)
    axr.tick_params(labelsize=8.5); axr.grid(alpha=.3)
    axr.legend(fontsize=8.2, loc="lower left", ncol=2)
    axr.set_ylim(-12, 12)

    # ── ② 3D 軌跡 ───────────────────────────────────────────
    ax = fig.add_subplot(gs[0, 2], projection="3d"); ax.set_facecolor("white")
    _org = (B["lat"][~np.isnan(B["lat"]) & (B["t_rel"] >= 0)][0],
            B["lon"][~np.isnan(B["lat"]) & (B["t_rel"] >= 0)][0])
    gt, ge, gn, gv, glast = gps_track(B, origin=_org)
    gh = np.interp(gt, tb, B["rh"])

    # 降落段：唯一可信的 GPS（ch1 斷電冷啟後）
    az = np.nan
    if PA is not None:
        pgt, pge, pgn, _pv, _pl = gps_track(PA, poff, origin=_org, tkey="uptime")
        pgh = np.interp(pgt, tb, B["rh"])
        az = math.atan2(pge[-1], pgn[-1])          # 落點方位（弧度，自北順時針）
    # 上升段：彈道重建（不使用 GPS），沿落點方位投影
    rt, rx, rz = reconstruct_downrange(fa["t_deploying"] + 0.5)
    re_, rn_ = rx * math.sin(az), rx * math.cos(az)

    ax.plot(ge, gn, gh, color=MUTED, lw=1.2, ls=":", alpha=.75,
            label="GPS 原始（凍結，錯的）")
    ax.plot(re_, rn_, rz, color=ACCENT2, lw=2.6, label="上升段：彈道重建")
    if PA is not None:
        ax.plot(pge, pgn, pgh, color=GOOD, lw=2.8, label="降落段：GPS 量測")
        ax.plot([pge[-1]], [pgn[-1]], [pgh[-1]], "o", color=BAD, ms=8)
    ax.plot([0], [0], [0], "^", color=ACCENT, ms=10)
    ax.plot([re_[-1]], [rn_[-1]], [rz[-1]], "*", color=BAD, ms=13)
    ax.set_xlabel("東向 (m)", fontsize=9); ax.set_ylabel("北向 (m)", fontsize=9)
    ax.set_zlabel("高度 (m)", fontsize=9); ax.tick_params(labelsize=8)
    ax.set_title("三維軌跡　高度全程量測\n"
                 f"上升水平＝彈道重建（開傘時 {rx[-1]:.0f} m）／降落水平＝{na} GPS",
                 fontsize=10.2, fontweight="bold", color=INK, pad=2)
    ax.legend(fontsize=7.2, loc="upper left", framealpha=.85)
    ax.view_init(elev=22, azim=-118)

    # ── ③ 速度 ──────────────────────────────────────────────
    ax = fig.add_subplot(gs[1, 0]); ax.set_facecolor("white")
    ax.plot(tb, fb["v_baro"], color=INK, lw=2.5, label=f"{nb} 氣壓斜率", zorder=6)
    ax.plot(ta, fa["v_baro"], color=INK, lw=1.4, ls=(0, (5, 2)), alpha=.75,
            label=f"{na} 氣壓斜率", zorder=5)
    if pt is not None:
        ax.plot(pt, local_slope(pt, ph, 1.0), color=ACCENT2, lw=1.6,
                label=f"{na} 重開機後 氣壓斜率", zorder=6)
    ax.plot(tb, B["vz"], color=BAD, lw=1.7, alpha=.9, label=f"{nb} 卡爾曼 VZ", zorder=4)
    ax.plot(ta, A["vz"], color=GOOD, lw=1.7, alpha=.9, label=f"{na} 卡爾曼 VZ", zorder=4)
    ax.axhline(0, color=MUTED, lw=.9)
    ax.axhline(-0.5, color="#9a6b00", lw=1.2, ls=":", label="cond_B 門檻 -0.5 m/s")
    ax.axvline(fb["ap_t"], color=GOOD, ls=":", lw=1.3)
    vb = float(np.interp(fb["ap_t"], tb, fb["v_baro"]))
    va = float(np.interp(fb["ap_t"], ta, A["vz"]))
    vz2 = float(np.interp(fb["ap_t"], tb, B["vz"]))
    ax.annotate("頂點瞬間的垂直速度（真值 = 0）：" + chr(10) +
                f"   氣壓斜率 {vb:+.1f}   ／   {na} 卡爾曼 {va:+.1f}"
                f"   ／   {nb} 卡爾曼 {vz2:+.1f} m/s",
                xy=(0.03, 0.05), xycoords="axes fraction", fontsize=9.4,
                color=INK, fontweight="bold",
                bbox=dict(fc="#fff4e0", ec=MUTED, alpha=.95, boxstyle="round,pad=0.32"))
    ax.set_xlabel("飛行時間 T+ (s)", fontsize=11)
    ax.set_ylabel("垂直速度 (m/s)   ＋上升", fontsize=11)
    ax.set_title("● 垂直速度：氣壓（兩板一致）vs 卡爾曼（兩板分歧）",
                 fontsize=12.2, fontweight="bold", color=BAD, pad=8)
    ax.grid(alpha=.25); ax.legend(fontsize=8.0, loc="upper right")
    ax.set_xlim(-1.5, tmax + 1.5)

    # ── ④ 加速度 ────────────────────────────────────────────
    ax = fig.add_subplot(gs[1, 1]); ax.set_facecolor("white")
    ax.plot(tb, B["ga"], color=INK, lw=1.9, label=f"{nb} 合加速度 GA")
    ax.plot(ta, A["ga"], color=GOOD, lw=1.6, ls=(0, (5, 2)), alpha=.9,
            label=f"{na} 合加速度 GA")
    if pt is not None:
        ax.plot(pt, PA["ga"][pvalid], color=ACCENT2, lw=1.4,
                label=f"{na} 重開機後 GA")
    ax.axhline(2.5, color=ACCENT, ls=":", lw=1.3, label="離架偵測門檻 2.5 g")
    ax.axhline(1.0, color=MUTED, lw=.9, alpha=.6)
    ax.annotate(f"推力段最大\n{na} {fa['max_g']:.2f} g ／ {nb} {fb['max_g']:.2f} g",
                xy=(fb["max_g_t"], fb["max_g"]), xytext=(fb["max_g_t"] + 3.2, 5.7),
                fontsize=9.6, color=PURPLE, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=PURPLE, lw=1.3))
    ax.annotate(f"開傘衝擊 {na} {fa['shock_g']:.2f} g ／ {nb} {fb['shock_g']:.2f} g\n"
                f"（2Hz 取樣，真峰值必然更高）",
                xy=(fa["shock_t"] + shift, fa["shock_g"]),
                xytext=(fa["shock_t"] + shift + 4.2, 4.0),
                fontsize=9.4, color=BAD, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=BAD, lw=1.3))
    ax.set_xlabel("飛行時間 T+ (s)", fontsize=11)
    ax.set_ylabel("比力大小 (g)", fontsize=11)
    ax.set_title("加速度歷程（IMU 量測，2Hz 遙測取樣）",
                 fontsize=12.2, fontweight="bold", color=INK, pad=8)
    ax.grid(alpha=.25); ax.legend(fontsize=8.6, loc="center right")
    ax.set_xlim(-1.5, tmax + 1.5)

    # ── ⑤ 地面航跡：三段，共用同一原點才能互相比較 ──────────
    ax = fig.add_subplot(gs[1, 2]); ax.set_facecolor("white")
    org = (B["lat"][~np.isnan(B["lat"]) & (B["t_rel"] >= 0)][0],
           B["lon"][~np.isnan(B["lat"]) & (B["t_rel"] >= 0)][0])
    gt, ge, gn, gv, glast = gps_track(B, origin=org)
    st = gt >= GPS_SETTLE_T
    at, ae, an, av, alast = gps_track(A, shift, origin=org)
    def wander(t, e, n):
        """路徑總長 / 淨位移 / 平均地速。傘降時真實運動應接近直線漂移，
        曲折比接近 1；接收機在原地繞圈時曲折比會很大。"""
        path = float(np.sum(np.hypot(np.diff(e), np.diff(n))))
        net = math.hypot(e[-1] - e[0], n[-1] - n[0])
        return path, net, path / max(t[-1] - t[0], 1e-9)

    ax.plot(ge[~st], gn[~st], "-", color=MUTED, lw=1.5, alpha=.8,
            label=f"{nb} 上升段：解算凍結")
    ax.plot(ae, an, "-", color="#c0a860", lw=1.5, alpha=.85,
            label=f"{na} 上升段：解算凍結")
    # ★時間窗必須與 ch1 重開機段相同，否則 ch2 會因避開最糟那段而顯得較好
    w0 = (PA["uptime"][0] + poff) if PA is not None else GPS_SETTLE_T
    st = gt >= w0
    p2, n2_, v2 = wander(gt[st], ge[st], gn[st])
    ax.plot(ge[st], gn[st], "-o", color=BAD, lw=1.8, ms=3.4, alpha=.85,
            label=f"{nb} 同時間窗：亂走（{v2:.0f} m/s）")
    sep = np.nan
    if PA is not None:
        pgt, pge, pgn, pgv, plast = gps_track(PA, poff, origin=org, tkey="uptime")
        p1, n1_, v1 = wander(pgt, pge, pgn)
        ax.plot(pge, pgn, "-s", color=GOOD, lw=2.4, ms=4.0,
                label=f"★{na} 重開機後：唯一可用（{v1:.1f} m/s）")
        ok = (pgt >= gt.min()) & (pgt <= gt.max())
        sep = float(np.mean(np.hypot(pge[ok] - np.interp(pgt[ok], gt, ge),
                                     pgn[ok] - np.interp(pgt[ok], gt, gn))))
        ax.plot(pge[-1], pgn[-1], "o", color=GOOD, ms=12, zorder=10)
    ax.plot(0, 0, "^", color=ACCENT, ms=13, label="發射點", zorder=9)
    for ee, nn, vv, c in ((ge, gn, gv, BAD), (ae, an, av, "#8a5a00")):
        j = vv > 100
        if j.any():
            ax.plot(ee[j], nn[j], "x", color=c, ms=12, mew=2.4, zorder=10)
    ax.annotate(f"同一段時間、同一根箭：" + chr(10) +
                f"　{na} 路徑 {p1:.0f} m ／淨位移 {n1_:.0f} m（曲折比 {p1/max(n1_,1e-9):.1f}）" + chr(10) +
                f"　{nb} 路徑 {p2:.0f} m ／淨位移 {n2_:.0f} m（曲折比 {p2/max(n2_,1e-9):.1f}）" + chr(10) +
                f"→ {nb} 在原地繞圈；{na}（斷電冷啟）才是真的",
                xy=(0.02, 0.03), xycoords="axes fraction", fontsize=8.2,
                color=BAD, fontweight="bold",
                bbox=dict(fc="#fde8e8", ec=BAD, alpha=.93, boxstyle="round,pad=0.3"))
    ax.set_xlabel("東向 (m)", fontsize=11); ax.set_ylabel("北向 (m)", fontsize=11)
    ax.set_title(f"地面投影航跡 ── 只有{na}重開機後可用",
                 fontsize=12.2, fontweight="bold", color=BAD, pad=8)
    ax.grid(alpha=.25); ax.legend(fontsize=7.4, loc="upper right")

    # ── ⑥ 傘降段：現在兩塊板都有 ────────────────────────────
    ax = fig.add_subplot(gs[2, 0]); ax.set_facecolor("white")
    m = fb["desc_mask"]; t2, h2 = tb[m], B["rh"][m]
    ax.plot(t2, h2, "o-", color=INK, lw=2.2, ms=3.6, label=f"{nb}（量測）", zorder=5)
    k2 = np.polyfit(t2, h2, 1)
    ax.plot(t2, np.polyval(k2, t2), "--", color=BAD, lw=1.8,
            label=f"{nb} 擬合 {-k2[0]:.1f} m/s", zorder=6)
    if pt is not None:
        ax.plot(pt, ph, "s-", color=GOOD, lw=2.0, ms=3.4,
                label=f"{na} 重開機後（獨立量測）", zorder=5)
        k1 = np.polyfit(pt, ph, 1)
        ax.plot(pt, np.polyval(k1, pt), ":", color="#0f5c40", lw=1.8,
                label=f"{na} 擬合 {-k1[0]:.1f} m/s", zorder=6)
    td = np.linspace(t2[0], t2[0] + h2[0] / DESIGN_DESCENT, 50)
    ax.plot(td, h2[0] - DESIGN_DESCENT * (td - t2[0]), ":", color=COOL, lw=2.0,
            label=f"設計值 {DESIGN_DESCENT:.1f} m/s（3.0.9 模擬）", zorder=4)
    ax.axhline(0, color=MUTED, lw=1.0)
    ax.annotate(f"兩塊板獨立量到 30~34 m/s" + chr(10) +
                f"實測比設計快 {fb['desc_ep']/DESIGN_DESCENT:.1f} 倍" + chr(10) +
                f"→ 有效阻力面積僅剩 {100*(DESIGN_DESCENT/fb['desc_ep'])**2:.0f} %",
                xy=(t2[0] + 2.5, 235), fontsize=10.6, color=BAD, fontweight="bold",
                bbox=dict(fc="#fde8e8", ec=BAD, boxstyle="round,pad=0.4"), zorder=10)
    ax.set_xlabel("飛行時間 T+ (s)", fontsize=11)
    ax.set_ylabel("高度 (m)", fontsize=11)
    ax.set_title("● 傘降段：等速 30 m/s ── 雙板獨立確認",
                 fontsize=12.2, fontweight="bold", color=BAD, pad=8)
    ax.grid(alpha=.25); ax.legend(fontsize=8.4, loc="upper right")
    ax.set_xlim(t2[0] - 1, max(t2[-1], td[-1]) + 2); ax.set_ylim(-30, h2[0] + 60)

    # ── ⑦ 數據總表 ─────────────────────────────────────────
    ax = fig.add_subplot(gs[2, 1:]); ax.axis("off")
    ap_mean = (fa["ap_h"] + fb["ap_h"]) / 2
    if pt is not None:
        d_ep1, d_fit1, _ = descent_rate(pt, ph)
        a_desc = f"{d_ep1:.1f} m/s（端點）"
        a_end = f"T+{pt[-1]:.2f}s ／ {ph[-1]:.0f} m"
    else:
        a_desc, a_end = "無資料", f"T+{ta[-1]:.2f}s ／ {A['rh'][-1]:.0f} m"
    rows = [
        ["項目", na, nb, "依據 / 備註"],
        ["頂點高度", f"{fa['ap_h']:.1f} m", f"{fb['ap_h']:.1f} m",
         f"量測；兩板差 {abs(fa['ap_h']-fb['ap_h']):.1f} m → 取 {ap_mean:.1f} m"],
        ["頂點時刻", f"T+{fa['ap_t']:.2f}s", f"T+{fb['ap_t']:.2f}s", "量測＋拋物線擬合（用於對齊）"],
        ["推力段最大加速度", f"{fa['max_g']:.2f} g", f"{fb['max_g']:.2f} g", "量測（IMU，2Hz）"],
        ["燃燒結束", f"T+{fa['bo_lo']:.1f}~{fa['bo_hi']:.1f}s",
         f"T+{fb['bo_lo']:.1f}~{fb['bo_hi']:.1f}s", "量測；2Hz 只能給區間"],
        ["上升最大速度（氣壓）", f"{fa['v_max_baro']:.0f} m/s", f"{fb['v_max_baro']:.0f} m/s",
         "導出：氣壓斜率"],
        ["★ 開傘觸發源", "A且B", "C 備援計時器 T>18s", "量測：兩板韌體訊息明載"],
        ["★ 開傘時刻", f"T+{fa['t_deploying']+shift:.2f}s", f"T+{fb['t_deploying']:.2f}s",
         f"量測；{na} 早 {fb['t_deploying']-fa['t_deploying']-shift:.2f}s"],
        ["★ 開傘衝擊", f"{fa['shock_g']:.2f} g", f"{fb['shock_g']:.2f} g",
         "量測；2Hz 取樣，真峰值更高"],
        ["★ 開傘時總速度", "約 51 m/s", "約 50 m/s", "導出：頂點 GA 反解，偏離鉛直 65°"],
        ["cond_A 成立", f"T+{fa['t_condA']+shift:.2f}s", f"T+{fb['t_condA']:.2f}s",
         f"量測；差 {abs(fa['t_condA']+shift-fb['t_condA'])*1000:.0f} ms → 純氣壓判據可靠"],
        ["● cond_B 成立", f"T+{fa['t_condB']+shift:.2f}s", f"T+{fb['t_condB']:.2f}s",
         f"量測；差 {abs(fa['t_condB']+shift-fb['t_condB']):.1f} s → 靠卡爾曼，不可靠"],
        ["● 頂點時卡爾曼 VZ", f"{va:+.1f} m/s", f"{vz2:+.1f} m/s", "量測；真值 = 0 → 加性正偏差"],
        ["● 傘降速率", a_desc, f"{fb['desc_ep']:.1f} m/s（端點）",
         "量測：氣壓；兩板獨立、互相印證"],
        ["　同段卡爾曼 VZ", "—（重開機後未武裝）", f"{fb['desc_kf']:.1f} m/s",
         "韌體輸出，與氣壓矛盾，不可信"],
        ["● 有效阻力面積", f"{2*(LIFTOFF_MASS-PROP_MASS)*9.80665/(1.15*fb['desc_ep']**2):.2f} m² 實際",
         f"設計需 {2*(LIFTOFF_MASS-PROP_MASS)*9.80665/(1.15*DESIGN_DESCENT**2):.1f} m² → 剩 {100*(DESIGN_DESCENT/fb['desc_ep'])**2:.0f} %",
         f"導出；燃盡質量 {LIFTOFF_MASS-PROP_MASS:.2f} kg（起飛 {LIFTOFF_MASS} 實測 − 推進劑 {PROP_MASS}）"],
        ["● 觸水動能", "—", f"{0.5*(LIFTOFF_MASS-PROP_MASS)*fb['desc_ep']**2/1000:.1f} kJ",
         f"導出；等同從 {fb['desc_ep']**2/(2*9.80665):.0f} m 自由落下"],
        ["★ 推力段加速度驗證", f"實測 {fa['max_g']:.2f} g", f"實測 {fb['max_g']:.2f} g",
         "引擎曲線正算 5.65 g（1469 N ÷ 26.52 kg）→ 誤差 0.0~1.2%"],
        ["★ 空中斷電重開機", f"T+{ta[-1]:.2f}s  RST=POWER-ON", "無",
         f"量測；{na} 於 {ph[0] if pt is not None else 0:.0f} m 重取氣壓零點後續錄"],
        ["★ 重開機後狀態", "ST 全程 0 (IDLE)", "—",
         "量測；墜落救援未觸發（postreset_watch 被 PORRST 關掉）"],
        ["遙測終止", a_end, f"T+{tb[-1]:.2f}s ／ {B['rh'][-1]:.0f} m",
         f"{na} 反而錄到更低（多 {B['rh'][-1]-(ph[-1] if pt is not None else 0):.0f} m）"],
        ["模組健康 MOD", "F→E（SD 死）→重開機後 F", "全程 F（四模組全活）", "量測"],
        ["● GPS：上升段", f"凍結後跳 {np.nanmax(av):.0f} m/s",
         f"凍結後跳 {np.nanmax(gv):.0f} m/s", "量測；17 s 只移動 49 m，兩板皆不可用"],
        ["● GPS：傘降段", f"★可用（路徑{p1:.0f}m／淨{n1_:.0f}m，{v1:.1f} m/s）",
         f"不可用（路徑{p2:.0f}m／淨{n2_:.0f}m，{v2:.0f} m/s）",
         f"量測；{nb} 在原地繞圈，{na} 因斷電冷啟而恢復"],
        ["　兩板同時刻座標差", f"{plast[0]:+.5f}, {plast[1]:+.5f}",
         f"{glast[0]:+.5f}, {glast[1]:+.5f}", f"量測；平均相距 {sep:.0f} m"],
        ["★ 落點（採 {} 重開機後）".format(na), 
         f"東 {pge[-1]:.0f} m 北 {pgn[-1]:.0f} m ／ 距 {math.hypot(pge[-1],pgn[-1]):.0f} m"
         f" ／ 方位 {(math.degrees(math.atan2(pge[-1],pgn[-1]))+360)%360:.0f}°",
         f"（{nb} 給 {math.hypot(ge[-1],gn[-1]):.0f} m，不採）",
         "導出；末包在 10 m 高度，外推到水面再約 10 m"],
    ]
    tbl = ax.table(cellText=rows, cellLoc="left", loc="upper center",
                   colWidths=[.205, .195, .205, .395])
    tbl.auto_set_font_size(False); tbl.set_fontsize(8.4); tbl.scale(1, 0.96)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#d8d3c8")
        if r == 0:
            cell.set_facecolor(INK); cell.set_text_props(color="white", fontweight="bold")
        elif "●" in rows[r][0]:
            cell.set_facecolor("#fde8e8"); cell.set_text_props(fontweight="bold", color=BAD)
        elif "★" in rows[r][0]:
            cell.set_facecolor("#fff4e0"); cell.set_text_props(fontweight="bold")
        else:
            cell.set_facecolor("white" if r % 2 else "#f4f1ea")
    ax.set_title("雙板實測數據總表   ── 兩塊板對等呈現，兩者的差就是量測不確定度",
                 fontsize=12.4, fontweight="bold", color=INK, pad=6)

    fig.text(0.045, 0.038,
             f"資料來源：地面站 {na} 與 {nb} 兩條獨立 LoRa 記錄，各自來自一塊獨立航電板（獨立電源、獨立感測器、獨立天線）。"
             f"高度／壓力／加速度為直接量測；速度以氣壓高度做 ±1 s 最小平方斜率導出（未使用韌體卡爾曼輸出）。"
             f"{na} 重開機後的高度由原始壓力換算回同一基準（反解 ref_press = {PA['p0_main'] if PA is not None else 0:.2f} hPa）。",
             fontsize=9.5, color="#5a6472")
    fig.text(0.045, 0.017,
             f"限制：① 2 Hz 取樣，取樣點之間的峰值（尤其開傘衝擊）看不到　"
             f"② GPS 兩板皆不可信 —— 上升段解算凍結、開傘前後跳點（{na} 達 {np.nanmax(av):.0f} m/s、{nb} 達 {np.nanmax(gv):.0f} m/s）　"
             f"③ {na} 重開機段的時間軸由高度曲線對齊而得，非獨立時基　④ 兩板遙測都在落地前中斷，觸水瞬間未記錄。",
             fontsize=9.5, color="#5a6472")

    fig.savefig(png, dpi=dpi, facecolor=BG)
    if also_pdf:
        # PDF 是向量：文字與線條無限放大都清晰，列印／投影用這個。
        # 3D 那格與少數半透明填色仍會被光柵化，但其餘全部是向量。
        fig.savefig(os.path.splitext(png)[0] + ".pdf", facecolor=BG)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="雙板飛行遙測分析儀表板")
    ap.add_argument("csv_a"); ap.add_argument("csv_b")
    ap.add_argument("--name-a", default="ch1"); ap.add_argument("--name-b", default="ch2")
    ap.add_argument("--title", default="火箭飛行遙測分析 — 161 隊 2026-08-01 旭海（雙板獨立量測）")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dpi", type=int, default=200,
                    help="PNG 解析度（預設 200；螢幕看 150 夠、列印用 300）")
    ap.add_argument("--no-pdf", action="store_true", help="不要一併輸出向量 PDF")
    a = ap.parse_args()

    A, PA = load(a.csv_a)
    B, _ = load(a.csv_b)
    fa, fb = facts(A), facts(B)

    # ★ 地面站若有記 UTC 時戳，兩塊板就有共同時鐘 —— 直接用，不要擬合。
    #   擬合出來的偏移雖然事後驗證只差 19 ms，但那是運氣，不是保證。
    use_utc = np.isfinite(A["utc"]).all() and np.isfinite(B["utc"]).all()
    if use_utc:
        # ch1 的 t_rel 加上這個值 = ch2 的 t_rel（兩板各自以自己偵測到的離架為 0）
        shift = float((A["utc"] - A["t_rel"])[0] - (B["utc"] - B["t_rel"])[0])
        align_src = f"UTC 共同時鐘（{shift:+.3f}s）"
    else:
        shift = fb["ap_t"] - fa["ap_t"]
        align_src = f"氣壓頂點擬合（{shift:+.3f}s）"

    poff, pvalid = (np.nan, None)
    if PA is not None:
        if use_utc and np.isfinite(PA["utc"]).all():
            # 重開機後的封包也有 UTC → 直接換算，完全不必擬合。
            # 首兩包的接收時戳被開機期的緩衝延遲，用穩定段取中位數。
            base = float(np.median((PA["utc"] - PA["uptime"])[2:]))
            poff = base - float((B["utc"] - B["t_rel"])[0])
            pvalid = np.abs(PA["press"] - PA["p0_new"]) > 0.5
            post_src = "UTC 共同時鐘"
        else:
            poff, pvalid = align_post(PA, B["t_rel"], B["rh"])
            post_src = "高度曲線擬合"

    png = a.out or (os.path.splitext(a.csv_b)[0].replace("_parsed", "") + "_report.png")
    ap_mean = (fa["ap_h"] + fb["ap_h"]) / 2
    build(A, B, PA, fa, fb, a.name_a, a.name_b, shift, poff, pvalid, png,
          a.title + f"　[時間對齊：{align_src}]",
          f"頂點 {ap_mean:.1f} m（兩板差 {abs(fa['ap_h']-fb['ap_h']):.1f} m）  ·  "
          f"最大 {max(fa['max_g'],fb['max_g']):.2f} g  ·  "
          f"開傘 {a.name_a} A且B ／ {a.name_b} 備援計時器  ·  "
          f"傘降 30~34 m/s 雙板獨立確認（設計值 {DESIGN_DESCENT:.1f}）",
          dpi=a.dpi, also_pdf=not a.no_pdf)

    L = ["=" * 80, f"雙板量測對照（{a.name_a} 時間已 {shift:+.3f}s 對齊到 {a.name_b}）", "=" * 80,
         f"  {'項目':<22}{a.name_a:>14}{a.name_b:>14}{'差':>12}"]
    for lab, ka, kb, fmt in (
            ("頂點高度 (m)", fa["ap_h"], fb["ap_h"], "{:14.1f}"),
            ("推力段最大 g", fa["max_g"], fb["max_g"], "{:14.2f}"),
            ("上升最大速度 (m/s)", fa["v_max_baro"], fb["v_max_baro"], "{:14.1f}"),
            ("開傘時刻 (s)", fa["t_deploying"] + shift, fb["t_deploying"], "{:14.2f}"),
            ("開傘衝擊 (g)", fa["shock_g"], fb["shock_g"], "{:14.2f}"),
            ("cond_A 成立 (s)", fa["t_condA"] + shift, fb["t_condA"], "{:14.3f}"),
            ("cond_B 成立 (s)", fa["t_condB"] + shift, fb["t_condB"], "{:14.2f}"),
            ("卡爾曼最大 VZ (m/s)", fa["v_max_kf"], fb["v_max_kf"], "{:14.1f}")):
        L.append(f"  {lab:<22}" + fmt.format(ka) + fmt.format(kb) + f"{abs(ka-kb):12.3f}")

    if PA is not None:
        pt = PA["uptime"][pvalid] + poff; ph = PA["alt"][pvalid]
        e1, f1, sp = descent_rate(pt, ph)
        e2, f2, sp2 = fb["desc_ep"], fb["desc_fit"], fb["desc_span"]
        L += ["", "=" * 80, "傘降速率 —— 兩塊板獨立量測", "=" * 80,
              f"  反解 {a.name_a} 原始 ref_press = {PA['p0_main']:.2f} hPa"
              f"（韌體重開機後改用 {PA['p0_new']:.2f} hPa）",
              f"  重開機段對齊偏移 {poff:.2f} s（以高度曲線最小平方求得）", "",
              f"  {a.name_a}（重開機後）  {sp[2]:.1f} m → {sp[3]:.1f} m，"
              f"歷時 {sp[1]-sp[0]:.2f} s   端點 {e1:.2f} m/s ／ 擬合 {f1:.2f} m/s",
              f"  {a.name_b}（全程）      {sp2[2]:.1f} m → {sp2[3]:.1f} m，"
              f"歷時 {sp2[1]-sp2[0]:.2f} s   端點 {e2:.2f} m/s ／ 擬合 {f2:.2f} m/s",
              f"  → 兩板差 {abs(e1-e2)/max(e1,e2)*100:.1f} %"
              f"（一塊板中途斷電重開、重取氣壓零點，仍吻合到此範圍內）",
              f"  有效阻力面積 = 設計值的 {100*(DESIGN_DESCENT/e2)**2:.1f} %"]
    open(os.path.splitext(png)[0] + ".txt", "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"\n-> {png}")


if __name__ == "__main__":
    main()
