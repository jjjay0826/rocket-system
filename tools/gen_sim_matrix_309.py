# -*- coding: utf-8 -*-
"""用 3.0.9.ork 產生 81 組模擬矩陣（3 推力 × 3 仰角 × 9 風況）。

流程與 3.0.8 那次相同：產出三個檔 → 在 OpenRocket 開起來按「Run all
simulations」→ 存檔 → 我再分析。OpenRocket 24.12 沒有 headless 模式
（試過 --help，它直接開 GUI），所以中間這一步必須手動。

沿用上次的兩個修正：
  · 發射座標 22.1029 / 120.5333 是把 DMS 當十進位打進去的，實際旭海是
    22.174833 / 120.892722（差約 40 公里）。
  · 開傘事件設成固定高度時，低頂點的組別會整組不開傘 —— 改成 apogee，
    與韌體行為（頂點後才開）一致。
"""
import zipfile, re, pathlib

SRC = r"D:\Downloads\3.0.9.ork"
OUT = pathlib.Path(r"D:\Downloads\sim_309")
OUT.mkdir(exist_ok=True)

x = zipfile.ZipFile(SRC).read("rocket.ork").decode("utf-8", "ignore")

n_lat = x.count("<launchlatitude>22.1029</launchlatitude>")
x = x.replace("<launchlatitude>22.1029</launchlatitude>",
              "<launchlatitude>22.174833</launchlatitude>")
x = x.replace("<launchlongitude>120.5333</launchlongitude>",
              "<launchlongitude>120.892722</launchlongitude>")
n_alt = x.count("<deployevent>altitude</deployevent>")
x = x.replace("<deployevent>altitude</deployevent>", "<deployevent>apogee</deployevent>")

m = re.search(r"<configid>([0-9a-f-]+)</configid>", x)
CONFIGID = m.group(1)
print(f"  configid = {CONFIGID}")

WINDS = [("CALM", 0.0, 0.0), ("N2", 2.0, 0.0), ("N4", 4.0, 0.0),
         ("E2", 2.0, 1.5707963267948966), ("E4", 4.0, 1.5707963267948966),
         ("S2", 2.0, 3.141592653589793), ("S4", 4.0, 3.141592653589793),
         ("W2", 2.0, 4.71238898038469), ("W4", 4.0, 4.71238898038469)]
ANGLES = [("e79", 11.0), ("e80", 10.0), ("e81", 9.0)]   # 仰角 = 90 − rodangle


def sim_xml(name, rodangle, wspeed, wdir):
    return f"""    <simulation status="outdated">
      <name>{name}</name>
      <simulator>RK4Simulator</simulator>
      <calculator>BarrowmanCalculator</calculator>
      <conditions>
        <configid>{CONFIGID}</configid>
        <launchrodlength>4.0</launchrodlength>
        <launchintowind>false</launchintowind>
        <launchrodangle>{rodangle}</launchrodangle>
        <launchroddirection>90.0</launchroddirection>
        <windaverage>{wspeed}</windaverage>
        <windturbulence>0.1</windturbulence>
        <winddirection>{wdir}</winddirection>
        <wind model="average">
          <speed>{wspeed}</speed>
          <direction>{wdir}</direction>
          <standarddeviation>{wspeed * 0.1:.4f}</standarddeviation>
        </wind>
        <windmodeltype>Average</windmodeltype>
        <launchaltitude>0.0</launchaltitude>
        <launchlatitude>22.174833</launchlatitude>
        <launchlongitude>120.892722</launchlongitude>
        <geodeticmethod>spherical</geodeticmethod>
        <atmosphere model="isa"/>
        <timestep>0.05</timestep>
        <maxtime>1200.0</maxtime>
        <geodeticmethod>spherical</geodeticmethod>
      </conditions>
    </simulation>
"""


for thrust in ("T090", "T100", "T110"):
    body = x.replace("<designation>Pioneer_5K</designation>",
                     f"<designation>Pioneer_5K_{thrust}</designation>")
    # digest 對不上會讓 OpenRocket 跳「找不到馬達」；清掉讓它照 designation 找
    body = re.sub(r"<digest>[0-9a-f]+</digest>", "", body)
    h = body[:body.index("<simulations>")]
    t = body[body.index("</simulations>") + len("</simulations>"):]
    sims = ["<simulations>\n"]
    for aname, rod in ANGLES:
        for wname, wsp, wdr in WINDS:
            sims.append(sim_xml(f"{thrust}_{aname}_{wname}", rod, wsp, wdr))
    sims.append("  </simulations>")
    out = h + "".join(sims) + t
    p = OUT / f"m309_{thrust}.ork"
    with zipfile.ZipFile(p, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("rocket.ork", out)
    print(f"  {p.name}   {out.count('<simulation ')} 組   馬達 Pioneer_5K_{thrust}")

print(f"\n座標修正 {n_lat} 處｜deployevent altitude→apogee {n_alt} 處")
print(f"輸出目錄：{OUT}")
