#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""產生一個可拖時間軸、可釘選多組並排比對的本機檢視器（單一 HTML）。

## 為什麼是本機 HTML 而不是 base64 內嵌

150 張影像 base64 進 HTML 會是 10 MB 以上，開起來很卡。改成 HTML ＋
同目錄的影像檔，用 file:// 直接開，全解析度、沒有大小限制。

## 產出

    <out>/index.html      檢視器
    <out>/img/<ch>_<n>.jpg  各通道各時間點的影像
    <out>/data.js         遙測 ＋ 索引

## 三個通道

| 通道 | 看什麼 |
|---|---|
| raw   | 原圖（追蹤裁切＋疊幀降噪＋放大） |
| clean | 天空二次曲面擬合扣除後拉對比 —— 綜合判讀 |
| lum   | 亮度殘差 —— **傘繩與白色箭身只在這裡看得到** |

## 影片時間與飛行時間

本次直播片段的 t=0 就是離架（第一幀是箭在架上冒煙），所以
    飛行 T+ = 影片 clip 時間 + OFFSET
OFFSET 預設 0，檢視器裡可以即時微調。
"""
import argparse
import glob
import json
import os
import shutil
import sys

import numpy as np
from PIL import Image, ImageFilter, ImageEnhance

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from track_crop import canopy_box                     # noqa: E402
from separate_objects import fit_sky, rough_mask, norm  # noqa: E402


def build_images(files, out_img, fps, t0, every, stack, win, up, quality,
                 full_height=True):
    """回傳 [(t_flight, basename), ...]。"""
    W, H = Image.open(files[0]).size
    CW, CH = win
    print("【1/3】掃描傘衣位置")
    boxes, cents = {}, {}
    for i, f in enumerate(files):
        b = canopy_box(Image.open(f).convert("RGB"))
        if b:
            boxes[i] = b
            cents[i] = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
    print(f"  {len(boxes)}/{len(files)} 幀偵測到")

    # ★ 裁切吃滿整幅高度。先前把下方橫帶整條排除，但橫幅只佔畫面右側
    #   （x>1020），左邊那一段是乾淨天空 —— 而分離的箭身正好落在那裡，
    #   結果被切掉。偵測仍然避開橫帶（見 canopy_box），只有裁切放寬。
    USE_TOP, USE_BOT = 0, H
    def frame_box(i):
        """full_height=True：縱向固定吃滿整個可用範圍，保證下方的箭身、
        傘繩、分離殘骸全部收進來。框會變成接近方形，不再緊貼傘衣 ——
        代價是放大倍率下降，但不會再切到東西。"""
        b = boxes[i]
        cx = (b[0] + b[2]) // 2
        x0 = int(np.clip(cx - CW // 2, 0, W - CW))
        if full_height:
            return x0, USE_TOP, x0 + CW, USE_BOT
        y0 = int(np.clip(b[1] - 0.25 * CH, USE_TOP,
                         max(USE_TOP, USE_BOT - CH)))
        return x0, y0, x0 + CW, y0 + CH

    half = stack // 2
    idxs = [i for i in range(0, len(files), every) if i in boxes]
    print(f"\n【2/3】疊幀＋分離通道（{len(idxs)} 個時間點）")
    index = []
    for k, i in enumerate(idxs):
        acc = np.asarray(Image.open(files[i]).convert("RGB")).astype(np.float32)
        n = 1
        cx, cy = cents[i]
        for j in range(i - half, i + half + 1):
            if j == i or j not in cents:
                continue
            dx, dy = cx - cents[j][0], cy - cents[j][1]
            if abs(dx) > 220 or abs(dy) > 220:
                continue
            o = Image.open(files[j]).convert("RGB").transform(
                (W, H), Image.AFFINE, (1, 0, -dx, 0, 1, -dy), Image.BICUBIC)
            acc += np.asarray(o).astype(np.float32)
            n += 1
        arr = acc / n
        bx0, by0, bx1, by1 = frame_box(i)
        crop = arr[by0:by1, bx0:bx1]

        # ── 三個通道 ──
        sky = fit_sky(crop, rough_mask(crop))
        resid = crop - sky
        chroma = np.clip(resid[:, :, 0] - resid[:, :, 2], 0, None)
        lum = np.clip(resid.mean(axis=2), 0, None)
        comb = np.maximum(norm(chroma), norm(lum))
        colour = np.stack([norm(crop[:, :, c], 2, 99.8) for c in range(3)], -1) * 255.0
        clean = colour * comb[:, :, None] + 250.0 * (1 - comb[:, :, None])
        lumv = (1 - norm(lum)) * 255.0
        lum3 = np.repeat(lumv[:, :, None], 3, axis=2)

        big = (int(crop.shape[1] * up), int(crop.shape[0] * up))
        for ch, data, sat in (("raw", crop, 1.4), ("clean", clean, 1.0), ("lum", lum3, 1.0)):
            p = Image.fromarray(np.clip(data, 0, 255).astype(np.uint8)).resize(big, Image.LANCZOS)
            if sat != 1.0:
                p = ImageEnhance.Color(p).enhance(sat)
            p = p.filter(ImageFilter.UnsharpMask(radius=2, percent=130, threshold=2))
            p.save(os.path.join(out_img, f"{ch}_{k:03d}.jpg"), quality=quality, optimize=True)
        index.append(round(t0 + i / fps, 3))
        if (k + 1) % 10 == 0:
            print(f"  {k+1}/{len(idxs)}")
    return index


def load_telemetry(csv_path):
    import csv as _csv
    rows = list(_csv.DictReader(open(csv_path, encoding="utf-8")))
    out = []
    for r in rows:
        try:
            t = float(r["t_rel"])
        except (ValueError, KeyError):
            continue
        if not (-2 <= t <= 60):
            continue
        out.append(dict(t=round(t, 3), rh=float(r["rh"]), vz=float(r["vz"]),
                        ga=float(r["ga"]), st=int(r["st"])))
    return out


HTML = r"""<meta charset="utf-8">
<title>開傘影像檢視器 — 161 隊 2026-08-01</title>
<style>
:root{--bg:#12151c;--pan:#1b2030;--ink:#e8ecf5;--dim:#8b95ab;--acc:#f0a848;--bad:#ff6b6b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
     font:14px/1.5 "Microsoft JhengHei","Segoe UI",system-ui,sans-serif}
header{padding:14px 20px 10px;border-bottom:1px solid #2a3145}
h1{margin:0 0 2px;font-size:19px;letter-spacing:.3px}
.sub{color:var(--dim);font-size:12.5px}
.bar{display:flex;gap:16px;align-items:center;padding:12px 20px;background:var(--pan);
     border-bottom:1px solid #2a3145;flex-wrap:wrap;position:sticky;top:0;z-index:9}
input[type=range]{flex:1;min-width:280px;accent-color:var(--acc)}
button{background:#2b3346;color:var(--ink);border:1px solid #3a4560;border-radius:6px;
       padding:6px 12px;cursor:pointer;font:inherit}
button:hover{background:#354061}
button.pri{background:var(--acc);color:#1a1408;border-color:var(--acc);font-weight:600}
.tele{display:flex;gap:18px;flex-wrap:wrap;font-variant-numeric:tabular-nums}
.tele b{color:var(--acc);font-weight:600}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:16px 20px}
.cell{background:var(--pan);border:1px solid #2a3145;border-radius:8px;overflow:hidden}
.cell h3{margin:0;padding:7px 10px;font-size:12.5px;color:var(--dim);
         border-bottom:1px solid #2a3145;font-weight:600}
.cell img{width:100%;display:block;background:#0b0d12}
.pins{padding:0 20px 24px}
.pinrow{display:flex;gap:10px;overflow-x:auto;padding:10px 0}
.pin{flex:0 0 auto;background:var(--pan);border:1px solid #3a4560;border-radius:8px;
     overflow:hidden;position:relative}
.pin img{width:190px;display:block}
.pin .lab{padding:5px 8px;font-size:12px;color:var(--acc);font-variant-numeric:tabular-nums}
.pin .x{position:absolute;top:4px;right:4px;background:#0009;border:none;color:#fff;
        border-radius:4px;padding:1px 6px;cursor:pointer;font-size:13px}
.plot{padding:0 20px 6px}
canvas{width:100%;height:110px;display:block;background:var(--pan);
       border:1px solid #2a3145;border-radius:8px}
.hint{padding:0 20px 18px;color:var(--dim);font-size:12.5px}
kbd{background:#2b3346;border:1px solid #3a4560;border-radius:4px;padding:1px 6px;font-size:11.5px}
</style>

<header>
  <h1>開傘影像檢視器　—　161 隊　2026-08-01 旭海</h1>
  <div class="sub">拖時間軸比對三個通道；按「釘選」把當下這一刻加到下方比較列，可以同時看很多組。</div>
</header>

<div class="bar">
  <button id="prev">◀</button>
  <input type="range" id="sl" min="0" value="0">
  <button id="next">▶</button>
  <button id="play">▶ 播放</button>
  <button class="pri" id="pin">釘選這一刻</button>
  <label style="color:var(--dim)">時間偏移
    <input type="number" id="off" value="0" step="0.1" style="width:66px;background:#2b3346;
           color:var(--ink);border:1px solid #3a4560;border-radius:5px;padding:3px 6px">s</label>
</div>

<div class="bar" style="position:static">
  <div class="tele">
    <span>飛行時間 <b id="tt">—</b></span>
    <span>氣壓高度 <b id="th">—</b></span>
    <span>垂直速度 <b id="tv">—</b></span>
    <span>合加速度 <b id="tg">—</b></span>
    <span>狀態 <b id="ts">—</b></span>
  </div>
</div>

<div class="plot"><canvas id="cv"></canvas></div>

<div class="grid">
  <div class="cell"><h3>原圖（追蹤裁切＋疊幀降噪）</h3><img id="i_raw"></div>
  <div class="cell"><h3>天空擬合扣除＋對比　—　綜合判讀</h3><img id="i_clean"></div>
  <div class="cell"><h3>亮度殘差　—　傘繩／箭身（色度看不到）</h3><img id="i_lum"></div>
</div>

<div class="pins">
  <h3 style="margin:0 0 2px;font-size:14px">釘選比較列</h3>
  <div class="pinrow" id="pinrow"><span style="color:var(--dim)">還沒釘選任何時間點</span></div>
</div>

<div class="hint">
  <kbd>←</kbd><kbd>→</kbd> 前後一格　<kbd>空白</kbd> 播放／暫停　<kbd>P</kbd> 釘選
  ｜　「時間偏移」用來微調影片與遙測的對齊（影片 t=0 = 離架，理論上偏移為 0）
</div>

<script src="data.js"></script>
<script>
const N = IDX.length;
const sl = document.getElementById('sl'); sl.max = N - 1;
const imgs = {raw:document.getElementById('i_raw'), clean:document.getElementById('i_clean'),
              lum:document.getElementById('i_lum')};
const pad = n => String(n).padStart(3,'0');
const ST = {0:'IDLE',1:'LAUNCHED',2:'DEPLOYING',3:'DEPLOYED',4:'LANDED'};
let cur = 0, timer = null;

function tele(t){
  if(!TELE.length) return null;
  let best = TELE[0];
  for(const r of TELE) if(Math.abs(r.t-t) < Math.abs(best.t-t)) best = r;
  return best;
}
function show(i){
  cur = Math.max(0, Math.min(N-1, i)); sl.value = cur;
  for(const k in imgs) imgs[k].src = `img/${k}_${pad(cur)}.jpg`;
  const off = parseFloat(document.getElementById('off').value)||0;
  const t = IDX[cur] + off;
  document.getElementById('tt').textContent = 'T+' + t.toFixed(2) + ' s';
  const r = tele(t);
  document.getElementById('th').textContent = r ? r.rh.toFixed(1)+' m' : '—';
  document.getElementById('tv').textContent = r ? r.vz.toFixed(2)+' m/s' : '—';
  document.getElementById('tg').textContent = r ? r.ga.toFixed(2)+' g' : '—';
  document.getElementById('ts').textContent = r ? (ST[r.st]||r.st) : '—';
  draw();
}
function draw(){
  const c = document.getElementById('cv'), x = c.getContext('2d');
  const W = c.width = c.clientWidth*2, H = c.height = 220;
  x.clearRect(0,0,W,H);
  if(!TELE.length) return;
  const t0 = Math.min(...TELE.map(r=>r.t)), t1 = Math.max(...TELE.map(r=>r.t));
  const h1 = Math.max(...TELE.map(r=>r.rh));
  const X = t => (t-t0)/(t1-t0)*(W-70)+50, Y = h => H-24-(h/h1)*(H-44);
  x.strokeStyle='#3a4560'; x.lineWidth=2; x.beginPath();
  TELE.forEach((r,i)=> i?x.lineTo(X(r.t),Y(r.rh)):x.moveTo(X(r.t),Y(r.rh))); x.stroke();
  // 影像涵蓋區間
  x.fillStyle='#f0a84822';
  x.fillRect(X(IDX[0]), 8, X(IDX[N-1])-X(IDX[0]), H-32);
  // 目前位置
  const off = parseFloat(document.getElementById('off').value)||0;
  const t = IDX[cur]+off;
  x.strokeStyle='#f0a848'; x.lineWidth=3; x.beginPath();
  x.moveTo(X(t),6); x.lineTo(X(t),H-20); x.stroke();
  // 釘選標記
  x.strokeStyle='#ff6b6b'; x.lineWidth=2;
  pins.forEach(p=>{ x.beginPath(); x.moveTo(X(IDX[p]+off),6); x.lineTo(X(IDX[p]+off),H-20); x.stroke(); });
  x.fillStyle='#8b95ab'; x.font='22px sans-serif';
  x.fillText('高度 (m)', 8, 22); x.fillText(h1.toFixed(0), 8, 44);
  x.fillText('T+'+t0.toFixed(0)+'s', 50, H-4);
  x.fillText('T+'+t1.toFixed(0)+'s', W-100, H-4);
}
let pins = [];
function renderPins(){
  const row = document.getElementById('pinrow');
  if(!pins.length){ row.innerHTML='<span style="color:var(--dim)">還沒釘選任何時間點</span>'; draw(); return; }
  const off = parseFloat(document.getElementById('off').value)||0;
  row.innerHTML = pins.map((p,k)=>`<div class="pin">
      <button class="x" data-k="${k}">×</button>
      <img src="img/clean_${pad(p)}.jpg">
      <div class="lab">T+${(IDX[p]+off).toFixed(2)} s</div></div>`).join('');
  row.querySelectorAll('.x').forEach(b=>b.onclick=e=>{
      pins.splice(+e.target.dataset.k,1); renderPins(); });
  draw();
}
sl.oninput = e => show(+e.target.value);
document.getElementById('prev').onclick = ()=>show(cur-1);
document.getElementById('next').onclick = ()=>show(cur+1);
document.getElementById('pin').onclick = ()=>{ if(!pins.includes(cur)){pins.push(cur);pins.sort((a,b)=>a-b);renderPins();} };
document.getElementById('off').oninput = ()=>{ show(cur); renderPins(); };
document.getElementById('play').onclick = function(){
  if(timer){ clearInterval(timer); timer=null; this.textContent='▶ 播放'; }
  else { this.textContent='⏸ 暫停';
         timer=setInterval(()=>show(cur>=N-1?0:cur+1), 160); }
};
addEventListener('keydown', e=>{
  if(e.key==='ArrowLeft') show(cur-1);
  else if(e.key==='ArrowRight') show(cur+1);
  else if(e.key===' '){ e.preventDefault(); document.getElementById('play').click(); }
  else if(e.key.toLowerCase()==='p') document.getElementById('pin').click();
});
addEventListener('resize', draw);
show(0);
</script>
"""


def main():
    ap = argparse.ArgumentParser(description="產生開傘影像檢視器")
    ap.add_argument("pattern", help="原始幀 glob，例如 'chute_video/frames/f_*.png'")
    ap.add_argument("--out", default="viewer")
    ap.add_argument("--telemetry", default=None, help="ch2 的 _parsed.csv")
    ap.add_argument("--fps", type=float, default=15.0, help="抽幀時的幀率")
    ap.add_argument("--t0", type=float, default=16.0, help="第一幀對應的飛行時間")
    ap.add_argument("--every", type=int, default=3)
    ap.add_argument("--stack", type=int, default=1,
                    help="疊幀張數。★只有在抽幀率夠高、時間窗夠短時才有用："
                         "15fps 下疊 7 幀 = 467 ms，傘衣早就變形了，疊起來只會糊。"
                         "59.94fps 下疊 5 幀 = 83 ms 才安全。預設 1（不疊）")
    ap.add_argument("--win", type=int, nargs=2, default=(760, 620),
                    help="裁切窗寬高；full-height 模式下只有寬度生效")
    ap.add_argument("--up", type=float, default=1.6)
    ap.add_argument("--quality", type=int, default=86)
    ap.add_argument("--focus", action="store_true",
                    help="緊貼傘衣裁切（放大倍率高，但下方的箭身可能被切）。"
                         "預設是吃滿整個可用縱向範圍，保證不切到東西")
    ap.add_argument("--tight", action="store_true",
                    help="緊貼傘衣裁切（放大倍率高，但下方可能切到箭身）。"
                         "預設是吃滿整個可用縱向範圍，不切東西")
    a = ap.parse_args()

    files = sorted(glob.glob(a.pattern))
    if not files:
        raise SystemExit(f"找不到 {a.pattern}")
    out_img = os.path.join(a.out, "img")
    if os.path.exists(a.out):
        shutil.rmtree(a.out)
    os.makedirs(out_img)

    idx = build_images(files, out_img, a.fps, a.t0, a.every, a.stack,
                       tuple(a.win), a.up, a.quality, full_height=not a.tight)
    tele = load_telemetry(a.telemetry) if a.telemetry else []

    print("\n【3/3】寫出 HTML")
    with open(os.path.join(a.out, "data.js"), "w", encoding="utf-8") as f:
        f.write("const IDX=" + json.dumps(idx) + ";\n")
        f.write("const TELE=" + json.dumps(tele) + ";\n")
    with open(os.path.join(a.out, "index.html"), "w", encoding="utf-8") as f:
        f.write(HTML)

    size = sum(os.path.getsize(os.path.join(out_img, x)) for x in os.listdir(out_img))
    print(f"  {len(idx)} 個時間點 × 3 通道 = {len(idx)*3} 張，{size/1048576:.1f} MB")
    print(f"  遙測 {len(tele)} 筆")
    print(f"\n→ 用瀏覽器開啟 {os.path.abspath(os.path.join(a.out, 'index.html'))}")


if __name__ == "__main__":
    main()
