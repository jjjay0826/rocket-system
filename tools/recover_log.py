#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""從預分配的 logN.csv 救回飛行資料。

## 為什麼需要這支

韌體開檔後一次預分配 16MB（logger.c:110-127）：

    f_lseek(16MB) → f_write(1 byte) → f_sync() → f_lseek(0)

目錄項的檔案大小在【地面】就寫死成 16MB+1 並落盤，之後飛行中零 sync。
好處：斷電不會孤兒化，資料早就一個磁區一個磁區寫進卡了。
代價：只有走到 LANDED 才會執行 f_truncate（main.c:1330）把尾巴削掉。

沒走到 LANDED（斷電／SD 鎖卡／落在樹上沒偵測到）→ 檔案維持 16MB+1，
資料在最前面，**後面是垃圾**。

⚠ 那些垃圾不是 0。f_lseek 越過 EOF 只配 cluster、不清空（ChaN 文件明說
   expanded area 內容 undefined）。整張卡唯一被寫 0x00 的是 offset 16MB
   那一個 byte。所以尾巴通常是【卡片上一輪的舊 CSV】—— 格式一模一樣，
   肉眼看不出接縫。

## 切點判據（四道，缺一不可）

不能靠「看起來像不像 CSV」—— 垃圾很可能就是上一趟的 CSV。四道一起用：

  ① 欄數 = 25（main.c:2041 的 header）。舊資料不對齊時第一行是半行，
     這道就擋下來了 —— 但這是【運氣】，不能只靠它。

  ② time_ms 嚴格遞增。同一個檔案內必然成立：板子一 reset 就開新的
     logN+1.csv，不會回頭寫。舊資料若從小的 time_ms 起跳，這道擋下。

  ③ time_ms 間隔 ≤ MAX_GAP_MS。韌體最慢的寫入間隔是 LANDED 的 5s
     （main.c:363），加上 SD 卡住最長 ~1.1s 與 reopen 重試，30s 已經
     非常寬鬆。舊資料若 time_ms 比真實結尾【還大】（②放行），這道擋下。

  ④ state 不得非法倒退。0→1→2→3→4，且 LANDED(4) 是終點。
     唯一合法的倒退是 1→0（main.c:1393 的離架撤銷，且只撤銷「推測」
     來的離架）。舊資料幾乎必然從 IDLE 重新開始 → 3→0 或 4→0，擋下。

⚠ 極限：若舊資料剛好對齊行首、time_ms 只比真實結尾大幾秒、且 state 從
   當前值繼續 —— 四道全過。這種巧合需要三個條件同時成立，但不是不可能。
   所以工具永遠印出切點原因與尾巴前 60 bytes，**請看一眼再用**。

用法：
    python recover_log.py log7.csv               # 輸出 log7_clean.csv
    python recover_log.py E:\\                    # 整張卡的 logN.csv 全跑
    python recover_log.py log7.csv --keep-tail   # 另存尾巴供人工檢視
    python recover_log.py log7.csv --max-gap 60  # 放寬 ③（SD 卡狀況很差時）
"""
import argparse
import os
import sys

NCOL = 25          # main.c:2041 的 header 欄數
HEADER_FIRST = "time_ms"
PREALLOC = 16 * 1024 * 1024
MAX_GAP_MS = 30000     # 韌體最慢 5s（LANDED）＋SD 卡住 ~1.1s，30s 極寬鬆
NAMES = {0: "IDLE", 1: "LAUNCHED", 2: "DEPLOYING", 3: "DEPLOYED", 4: "LANDED"}


def parse_rows(raw, max_gap=MAX_GAP_MS):
    """回傳 (header, good_rows, cut_reason, consumed_bytes)。

    raw 是 bytes。用 latin-1 解碼 —— 垃圾區可能有任何位元組，
    latin-1 不會丟例外，而 CSV 本身是純 ASCII 所以不影響判讀。
    """
    text = raw.decode("latin-1")
    lines = text.split("\r\n")

    header = None
    rows = []
    prev_t = -1
    prev_s = -1
    reason = "讀到檔尾（沒有偵測到垃圾）"
    consumed = 0          # 已確認屬於真實資料的位元組數

    for line in lines:
        raw_len = len(line) + 2        # +2 = \r\n

        if not line:
            consumed += raw_len
            continue

        if line.startswith(HEADER_FIRST):
            # 韌體每個檔案世代寫一次 header（main.c:2039）
            if header is None:
                header = line
                consumed += raw_len
                continue
            reason = "遇到第二個 header（AUTO-ROLL 或垃圾區的舊檔）"
            break

        f = line.split(",")
        if len(f) != NCOL:                                          # ── ①
            reason = f"欄數 {len(f)} != {NCOL}（多半是被切斷的半行）"
            break

        try:
            t = int(f[0])
            s = int(f[1])
        except ValueError:
            reason = f"time_ms/state 不是整數：{f[0][:16]!r},{f[1][:8]!r}"
            break

        if not (0 <= s <= 4):
            reason = f"state={s} 不在 0..4"
            break

        if t <= prev_t:                                             # ── ②
            reason = f"time_ms 不再遞增：{prev_t} → {t}"
            break

        if prev_t >= 0 and (t - prev_t) > max_gap:                  # ── ③
            reason = (f"time_ms 跳了 {(t - prev_t)/1000.0:.1f}s "
                      f"（上限 {max_gap/1000.0:.0f}s）：{prev_t} → {t}")
            break

        # ── ④ state 只能前進；唯一合法倒退是 LAUNCHED→IDLE
        #      （main.c:1393 離架撤銷，且只撤銷「推測」來的離架）。
        #      LANDED(4) 是終點，離開 4 一律非法。
        if prev_s >= 0 and s < prev_s and not (prev_s == 1 and s == 0):
            reason = (f"state 非法倒退 {prev_s}={NAMES[prev_s]} → "
                      f"{s}={NAMES[s]}（只有 LAUNCHED→IDLE 合法）")
            break

        prev_t, prev_s = t, s
        rows.append(line)
        consumed += raw_len

    return header, rows, reason, consumed


def human(n):
    for u in ("B", "KB", "MB", "GB"):
        if n < 1024 or u == "GB":
            return f"{n:.1f} {u}" if u != "B" else f"{n} B"
        n /= 1024.0


def recover(path, keep_tail=False, max_gap=MAX_GAP_MS):
    size = os.path.getsize(path)
    raw = open(path, "rb").read()

    print(f"\n=== {os.path.basename(path)} ===")
    print(f"  檔案大小 {human(size)} ({size} bytes)")
    if size == PREALLOC + 1:
        print("  → 大小正好 16MB+1：**f_truncate 沒跑過**，尾巴是垃圾，需要切")
    elif size > PREALLOC:
        print(f"  → 大於 16MB：預分配用完後繼續寫（韌體有退回週期 sync）")
    else:
        print("  → 已被 f_truncate 收斂過（有走到 LANDED），理論上乾淨")

    header, rows, reason, consumed = parse_rows(raw, max_gap)

    if header is None:
        print("  🔴 找不到 CSV header，這個檔可能整個壞掉")
        return False
    if not rows:
        print("  🔴 一筆資料都沒有")
        return False

    t0, t1 = int(rows[0].split(",")[0]), int(rows[-1].split(",")[0])
    dur = (t1 - t0) / 1000.0
    states = sorted({int(r.split(",")[1]) for r in rows})

    print(f"  有效資料 {len(rows)} 筆，{human(consumed)}"
          f"（佔檔案 {100.0*consumed/size:.2f}%）")
    print(f"  時間 {t0/1000.0:.1f}s → {t1/1000.0:.1f}s，共 {dur:.1f}s")
    print(f"  出現過的 state：" + "、".join(f"{s}={NAMES.get(s,'?')}" for s in states))
    if 4 not in states:
        print("    ⚠ 沒有 LANDED —— 落地偵測沒成立，所以尾巴才沒被削掉")
    print(f"  切點原因：{reason}")

    tail = raw[consumed:]
    if tail:
        nz = tail.lstrip(b"\x00")
        if not nz:
            print(f"  尾巴 {human(len(tail))} 全是 0x00（卡片是乾淨的）")
        else:
            print(f"  尾巴 {human(len(tail))}，開頭 60 bytes：")
            print(f"    {tail[:60]!r}")
            print("    ↑ 若這看起來像 CSV，就是卡片上一輪的舊資料")

    out = os.path.splitext(path)[0] + "_clean.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        f.write(header + "\r\n")
        for r in rows:
            f.write(r + "\r\n")
    print(f"  ✓ 寫出 {out}（{human(os.path.getsize(out))}）")

    if keep_tail and tail:
        tp = os.path.splitext(path)[0] + "_tail.bin"
        open(tp, "wb").write(tail)
        print(f"  ✓ 尾巴另存 {tp}")

    return True


def main():
    ap = argparse.ArgumentParser(
        description="從預分配的 logN.csv 切掉垃圾尾巴，救回飛行資料")
    ap.add_argument("path", help="logN.csv 檔案，或含有 logN.csv 的目錄")
    ap.add_argument("--keep-tail", action="store_true",
                    help="把切掉的尾巴另存 _tail.bin 供人工檢視")
    ap.add_argument("--max-gap", type=float, default=MAX_GAP_MS / 1000.0,
                    metavar="SEC",
                    help="容許的最大 time_ms 間隔，秒（預設 30；SD 狀況很差時放寬）")
    a = ap.parse_args()
    gap = int(a.max_gap * 1000)

    if os.path.isdir(a.path):
        names = sorted(
            (n for n in os.listdir(a.path)
             if n.lower().startswith("log") and n.lower().endswith(".csv")
             and "_clean" not in n.lower()),
            key=lambda n: int("".join(c for c in n[3:] if c.isdigit()) or 0))
        if not names:
            sys.exit(f"{a.path} 裡沒有 logN.csv")
        print(f"找到 {len(names)} 個 log 檔：{', '.join(names)}")
        ok = sum(recover(os.path.join(a.path, n), a.keep_tail, gap)
                 for n in names)
        print(f"\n{ok}/{len(names)} 個成功")
    else:
        recover(a.path, a.keep_tail, gap)


if __name__ == "__main__":
    main()
