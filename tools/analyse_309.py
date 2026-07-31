# -*- coding: utf-8 -*-
"""3.0.9 的 81 組飛安分析。與 3.0.5(short) 的舊結論逐項對照。

★ 開傘時序用彈道解析式，不從 .ork 讀
   模擬設 deployevent=apogee，所以頂點之後那段是傘已開的等速下降。
   拿它去算「掉 10m 要多久」會高估（6 m/s 掉 10m 要 1.7s，彈道只要 1.43s）。
   彈道：A(掉10m)=√(2·10/g)=1.43s；B(vz<−0.5 撐 1.5s)=0.05+1.5=1.55s
   → A∧B = 頂點 +1.55s（B 是瓶頸）
"""
import zipfile, glob, math, csv, collections, pathlib
import xml.etree.ElementTree as ET

OUT = str(pathlib.Path(__file__).resolve().parent.parent / "doc" / "sim309_analysis.txt")
f = open(OUT, "w", encoding="utf-8", buffering=1)
def out(*a): f.write(" ".join(map(str, a)) + "\n")

G = 9.80665
LAUNCH_AZ_G, SUSTAIN, TB = 2.5, 0.20, 18.0
T_AB = max(math.sqrt(2*10.0/G), 0.5/G + 1.5)      # 1.55 s

# V3 4.3.3 回收區（相對發射點，東/北為正）
ZONE = {"A": (2039, -1895), "B": (922, -1895), "C": (-52, -136),
        "D": (-52, 142), "E": (922, 1840), "F": (2039, 1840), "G": (2812, 19)}
ORDER = ["C", "B", "A", "G", "F", "E", "D"]


def inside(e, n):
    pts = [ZONE[k] for k in ORDER]
    sign = 0
    for i in range(len(pts)):
        x1, y1 = pts[i]; x2, y2 = pts[(i+1) % len(pts)]
        cr = (x2-x1)*(n-y1) - (y2-y1)*(e-x1)
        s = (cr > 0) - (cr < 0)
        if s == 0: continue
        if sign == 0: sign = s
        elif s != sign: return False
    return True


rows = []
for p in sorted(glob.glob(r"D:\Downloads\sim_309\m309_T*.ork")):
    root = ET.fromstring(zipfile.ZipFile(p).read("rocket.ork").decode("utf-8", "ignore"))
    for s in root.findall(".//simulation"):
        fd = s.find("flightdata")
        if fd is None or fd.get("maxaltitude") is None: continue
        br = fd.find(".//databranch")
        ix = {c: i for i, c in enumerate(br.get("types").split(","))}
        data = [[float(v) if v.strip() not in ("NaN", "") else float("nan")
                 for v in dp.text.split(",")] for dp in br.findall("datapoint")]
        T, ALT, TA, VV = ix["Time"], ix["Altitude"], ix["Total acceleration"], ix["Vertical velocity"]
        PE, PN = ix["Position East of launch"], ix["Position North of launch"]

        t_det, cross = None, None
        for r in data:
            if r[TA]/G + 1.0 >= LAUNCH_AZ_G:
                if cross is None: cross = r[T]
                if r[T] - cross >= SUSTAIN: t_det = r[T]; break
            else: cross = None
        apo = max(data, key=lambda r: r[ALT])
        last = data[-1]
        rows.append(dict(
            name=s.find("name").text,
            thrust=s.find("name").text.split("_")[0],
            ang=s.find("name").text.split("_")[1],
            wind=s.find("name").text.split("_")[2],
            apo=apo[ALT], t_apo=apo[T], t_det=t_det,
            coast=(apo[T]-t_det) if t_det else None,
            margin=(t_det + TB - apo[T]) if t_det else None,
            east=last[PE], north=last[PN],
            gv=abs(float(fd.get("groundhitvelocity") or "nan")),
            rodv=float(fd.get("launchrodvelocity")),
            ft=float(fd.get("flighttime"))))

out(f"讀入 {len(rows)} 組（3.0.9）\n")

# ═══ 1. 備援餘裕 ═══
out("=" * 76)
out("【1】C 備援(離架+18s)必須在頂點之後")
out("=" * 76)
bad = [r for r in rows if r["margin"] is None or r["margin"] <= 0]
out(f"餘裕 ≤0：{len(bad)} 組   {'✓ 全部安全' if not bad else '✗ 有上升段開傘風險'}")
rs = sorted(rows, key=lambda r: r["margin"])
out(f"\n{'組合':<18}{'頂點(m)':>9}{'滑行(s)':>9}{'餘裕(s)':>9}")
for r in rs[:5]:
    out(f"{r['name']:<18}{r['apo']:>9.0f}{r['coast']:>9.2f}{r['margin']:>9.2f}")
out(f"   ...最寬鬆 {rs[-1]['name']} {rs[-1]['margin']:.2f}s")
out(f"\n★ 最小餘裕 {rs[0]['margin']:.2f}s   （3.0.5short 是 0.58s）")
nC = sum(1 for r in rows if r["coast"] > TB - T_AB)
out(f"★ C 比 A∧B 早觸發：{nC}/81   （3.0.5short 是 25/81）")
for t in ("T090", "T100", "T110"):
    sub = [r for r in rows if r["thrust"] == t]
    out(f"   {t}: {sum(1 for r in sub if r['coast'] > TB-T_AB)}/27"
        f"   滑行 {min(r['coast'] for r in sub):.2f}~{max(r['coast'] for r in sub):.2f}s"
        f"   頂點 {min(r['apo'] for r in sub):.0f}~{max(r['apo'] for r in sub):.0f}m")

# ═══ 2. 落點 ═══
out("")
out("=" * 76)
out("【2】落點 vs 回收區（西界 −52m / 東界 2812m / 南北 ±1890m）")
out("=" * 76)
o = [r for r in rows if not inside(r["east"], r["north"])]
out(f"出界：{len(o)}/81   {'✓' if not o else '✗'}   （3.0.5short 是 1/81）")
for r in sorted(o, key=lambda r: r["east"]):
    out(f"   ✗ {r['name']:<18} 東{r['east']:>8.0f} 北{r['north']:>8.0f}")
w = sorted(rows, key=lambda r: r["east"])
out(f"\n最偏西 5 組：")
for r in w[:5]:
    out(f"   {r['name']:<18} 東{r['east']:>8.0f}m  距西界 {r['east']-(-52):>7.0f}m")
out(f"最偏東：{w[-1]['name']} 東{w[-1]['east']:.0f}m（東界 2812m）")

# 東風敏感度
out("")
out("每 1 m/s 東風把落點往西推多少 / 臨界東風：")
out(f"{'推力':<6}{'仰角':<6}{'無風':>8}{'東風4':>8}{'每1m/s':>9}{'臨界':>8}")
crit = []
for t in ("T090", "T100", "T110"):
    for a in ("e79", "e80", "e81"):
        c0 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_CALM")
        c4 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_E4")
        sl = (c0 - c4) / 4.0
        vc = (c0 + 52) / sl
        crit.append((vc, t, a))
        out(f"{t:<6}{a:<6}{c0:>8.0f}{c4:>8.0f}{sl:>9.0f}{vc:>8.1f}")
crit.sort()
out(f"\n★ 最容易回陸地：{crit[0][1]}_{crit[0][2]} 東風 {crit[0][0]:.1f} m/s"
    f"   （3.0.5short 是 3.8 m/s）")

# 8/1 實際風況
out("")
out("=" * 76)
out("【3】8/1 13:00 實際預報（東北東偏東 2 m/s → 東風分量 1.96 m/s）")
out("=" * 76)
EC = 2.0 * math.cos(math.radians(90 - 78.75))
out(f"地面東風分量 {EC:.2f} m/s → 傘下等效 {EC*1.6:.2f} ~ {EC*1.8:.2f} m/s\n")
out(f"{'推力':<6}{'仰角':<6}{'預測落點':>10}{'距西界':>9}  判定")
worst = None
for t in ("T090", "T100", "T110"):
    for a in ("e79", "e80", "e81"):
        c0 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_CALM")
        c4 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_E4")
        sl = (c0 - c4) / 4.0
        lo = c0 - sl*EC*1.8
        m = lo + 52
        if worst is None or lo < worst[0]: worst = (lo, f"{t}_{a}")
        out(f"{t:<6}{a:<6}{lo:>10.0f}{m:>9.0f}  " +
            ("✅" if m > 100 else ("🟡" if m > 0 else "🔴 出界")))
out(f"\n★ 最壞 {worst[1]} 落在東 {worst[0]:.0f}m，距西界 {worst[0]+52:.0f}m")
for a in ("e79", "e80", "e81"):
    ms = []
    for t in ("T090", "T100", "T110"):
        c0 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_CALM")
        c4 = next(r["east"] for r in rows if r["name"] == f"{t}_{a}_E4")
        ms.append(c0 - (c0-c4)/4.0*EC*1.8 + 52)
    out(f"   仰角 {a[1:]}° → 最小餘裕 {min(ms):.0f} m")

# ═══ 4. 規範 ═══
out("")
out("=" * 76)
out("【4】規範檢查")
out("=" * 76)
gv = [r for r in rows if not math.isnan(r["gv"])]
over = [r for r in gv if r["gv"] >= 12.0]
out(f"4.2.3 觸水 <12 m/s：{'✓ 全數合規' if not over else '✗ '+str(len(over))+' 組超標'}"
    f"   最快 {max(r['gv'] for r in gv):.2f} m/s")
rv = sorted(rows, key=lambda r: r["rodv"])
out(f"離架速度：{rv[0]['rodv']:.1f} ~ {rv[-1]['rodv']:.1f} m/s"
    f"   （最低 {rv[0]['name']}）")
out(f"頂點高度：{min(r['apo'] for r in rows):.0f} ~ {max(r['apo'] for r in rows):.0f} m"
    f"   （1K 組目標 1000m）")
out(f"飛行時間：{min(r['ft'] for r in rows):.0f} ~ {max(r['ft'] for r in rows):.0f} s")

# CSV
cp = str(pathlib.Path(__file__).resolve().parent.parent / "doc" / "sim309_81cases.csv")
with open(cp, "w", newline="", encoding="utf-8-sig") as fh:
    wr = csv.writer(fh)
    wr.writerow(["組合", "頂點m", "到頂點s", "離架偵測s", "滑行s", "備援餘裕s",
                 "落點東m", "落點北m", "區內", "觸水m/s", "離架m/s", "飛行時間s"])
    for r in sorted(rows, key=lambda r: r["name"]):
        wr.writerow([r["name"], f"{r['apo']:.1f}", f"{r['t_apo']:.2f}",
                     f"{r['t_det']:.2f}", f"{r['coast']:.2f}", f"{r['margin']:.2f}",
                     f"{r['east']:.0f}", f"{r['north']:.0f}",
                     "Y" if inside(r['east'], r['north']) else "N",
                     f"{r['gv']:.2f}", f"{r['rodv']:.1f}", f"{r['ft']:.1f}"])
out(f"\n總表：{cp}")
print(open(OUT, encoding="utf-8").read())
