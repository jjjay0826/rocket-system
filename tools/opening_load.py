"""opening_load.py — 主傘開傘時繩子的峰值拉力（簡化動態模型）

用法:
  python tools/opening_load.py                       # 預設值，印出速度 × 張開時間的表
  python tools/opening_load.py --cda 9.69            # 改用 OpenRocket 模型的傘（3.08 m, Cd 1.3）
  python tools/opening_load.py --mass 25.36 --rho 1.11 --drogue 0.44

模型:
  只算垂直方向。主傘的阻力面積在 tf 秒內從 0 長到全開（--growth 1 = 線性，
  2 = 先慢後快），火箭同時被拉慢：
      m dv/dt = m g - ½ ρ v² (CdA_main(t) + CdA_drogue)
  繩子拉力 = ½ ρ v² (CdA_main + CdA_drogue)，取全程最大值。

  「瞬間全開」欄 = ½ ρ v0² CdA —— 假設傘瞬間張開、火箭完全沒減速，是上限。
  2026-08 文件裡的 8.8 kN 就是這一欄，不是預期值。

沒有算進去的（真實峰值可能更高，設計時建議再乘 1.2～1.5）:
  - 繩子從鬆弛到繃緊那一下的瞬間衝擊（snatch）
  - 傘衣過度充氣（短暫超過全開阻力）
  - 水平速度、擺盪

預設值的出處:
  mass 25.36 kg   燃盡質量，flight_161_summary.md（三條獨立驗證一致）
  rho  1.11       約 800 m、熱天；與 parachute_failure_20260801.md 用的相同
  cda  10.4 m²    主傘設計阻力面積（同上兩份文件）
  drogue 0.44 m²  8/1 實測的終端阻力面積（=拖曳傘）
  8/1 影片：主傘從第一次看得到（T+18.22）到攤得最開（T+18.49）約 0.27 s，
  但那次傘在自旋中只張開一部分，只能當量級參考。
"""
import argparse

G = 9.81


def peak_force(v0, cda, tf, growth, mass, rho, drogue, dt=1e-4):
    v, t, fmax = v0, 0.0, 0.0
    while t < tf + 2.0:
        s = cda * min(t / tf, 1.0) ** growth
        f = 0.5 * rho * v * v * (s + drogue)
        fmax = max(fmax, f)
        v += (G - f / mass) * dt
        t += dt
    return fmax


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mass", type=float, default=25.36)
    ap.add_argument("--rho", type=float, default=1.11)
    ap.add_argument("--cda", type=float, default=10.4, help="主傘全開阻力面積 m²")
    ap.add_argument("--drogue", type=float, default=0.44, help="拖曳傘阻力面積 m²")
    ap.add_argument("--growth", type=float, default=1.0, help="1=線性, 2=先慢後快")
    ap.add_argument("--speeds", default="15,20,25,31,39,41")
    ap.add_argument("--tf", default="0.1,0.2,0.4,0.8", help="張開時間 s")
    a = ap.parse_args()
    speeds = [float(x) for x in a.speeds.split(",")]
    tfs = [float(x) for x in a.tf.split(",")]

    print(f"m={a.mass} kg  rho={a.rho}  主傘 CdA={a.cda} m²  拖曳傘 {a.drogue} m²  growth={a.growth}")
    print("單位 kgf（÷102 = kN）\n")
    print(f"{'速度 m/s':>8} {'瞬間全開':>8} | " + " ".join(f"tf={t:<4}s" for t in tfs))
    for v0 in speeds:
        inst = 0.5 * a.rho * v0 ** 2 * a.cda / G
        row = [peak_force(v0, a.cda, t, a.growth, a.mass, a.rho, a.drogue) / G for t in tfs]
        print(f"{v0:8.0f} {inst:8.0f} | " + " ".join(f"{x:8.0f}" for x in row))


if __name__ == "__main__":
    main()
