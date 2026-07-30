# -*- coding: utf-8 -*-
"""驗證 2026-07-31 審查修復的五項。對原始碼做結構斷言（韌體無法在此執行）。"""
import re, sys, pathlib


def strip_comments(src):
    """把 C 註解換成等長空白 —— 位置不變，但註解裡的字串不會被找到。
    三個誤判都是這樣來的：故障處理器的註解寫著「原本這裡是 while (1)」、
    cmd.c 的檔頭寫著「never call HAL_Delay」。"""
    out, i, n = [], 0, len(src)
    while i < n:
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(" " if ch != "\n" else "\n" for ch in src[i:j]))
            i = j
        elif src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i)); i = j
        else:
            out.append(src[i]); i += 1
    return "".join(out)

R = pathlib.Path(r"C:\Users\Jay0826\Desktop\rocket-system\firmware-rocket\Core")
_RAW = {p.name: (R / p).read_text(encoding="utf-8") for p in
     (pathlib.Path("Src/main.c"), pathlib.Path("Src/cmd.c"), pathlib.Path("Src/gnss.c"),
      pathlib.Path("Src/lora_e22.c"), pathlib.Path("Src/stm32f4xx_it.c"),
      pathlib.Path("Inc/cmd.h"), pathlib.Path("Inc/gnss.h"), pathlib.Path("Inc/lora_e22.h"))}
S = {k: strip_comments(v) for k, v in _RAW.items()}   # 結構斷言一律用去註解版
fails = []


def chk(name, ok, note=""):
    print(("  ✓ " if ok else "  ✗ ") + name)
    if note:
        print("        " + note)
    if not ok:
        fails.append(name)


def body(src, fn):
    """抓出函式主體（到下一個頂層 } 為止，夠用）"""
    i = src.index(f"void {fn}(void)\n{{")
    d, j = 0, i
    while True:
        if src[j] == "{": d += 1
        elif src[j] == "}":
            d -= 1
            if d == 0: return src[i:j + 1]
        j += 1


print("=" * 70)
print("【1】故障處理器改成重開機（原本是 while(1) 永遠卡死）")
print("=" * 70)
it = S["stm32f4xx_it.c"]
for h in ("HardFault_Handler", "MemManage_Handler", "BusFault_Handler", "UsageFault_Handler"):
    b = body(it, h)
    chk(f"{h} 呼叫 NVIC_SystemReset", "NVIC_SystemReset()" in b)
    # reset 必須在 while(1) 之前
    chk(f"{h} 的 reset 在 while(1) 之前",
        b.index("NVIC_SystemReset()") < b.index("while (1)"))
chk("NMI 維持不重開（CSS 降級要能返回）",
    "NVIC_SystemReset" not in body(it, "NMI_Handler"),
    "HSE 失效走 NMI，降到 HSI 之後要繼續飛，不能 reset")

print()
print("=" * 70)
print("【2】UART 溢位後重新掛上接收（原本永久失聰）")
print("=" * 70)
chk("實作了 HAL_UART_ErrorCallback", "void HAL_UART_ErrorCallback" in it)
eb = it[it.index("void HAL_UART_ErrorCallback"):]
chk("USART1 → LoRa_RearmRx", "USART1) LoRa_RearmRx()" in eb.replace("      ", " ").replace("  ", " "))
chk("USART2 → GNSS_RearmRx", "GNSS_RearmRx()" in eb[:600])
chk("有清 ORE 旗標", "__HAL_UART_CLEAR_OREFLAG" in eb[:600])
chk("有清 ErrorCode", "ErrorCode = HAL_UART_ERROR_NONE" in eb[:600])
chk("LoRa_RearmRx 已實作", "void LoRa_RearmRx(void)" in S["lora_e22.c"])
chk("LoRa_RearmRx 已宣告", "LoRa_RearmRx" in S["lora_e22.h"])
chk("GNSS_RearmRx 已實作", "void GNSS_RearmRx(void)" in S["gnss.c"])
chk("GNSS_RearmRx 已宣告", "GNSS_RearmRx" in S["gnss.h"])
chk("重掛前先 Abort（把 HAL 狀態機拉回 READY）",
    "AbortReceive_IT" in S["lora_e22.c"][S["lora_e22.c"].index("void LoRa_RearmRx"):][:300])

print()
print("=" * 70)
print("【3】PB6 手動點火加閘門（原本 IDLE 就能同時點燃兩路）")
print("=" * 70)
m = S["main.c"]
i = m.index("stable_btn_state == GPIO_PIN_RESET && manual_fire_btn_last")
blk = m[i:m.index("manual_fire_btn_last = stable_btn_state;", i)]
for cond, why in (("flight_state == FLIGHT_IDLE", "只准地面"),
                  ("manual_armed", "必須先 ARM"),
                  ("gnd_test_active()", "必須在地面測試模式")):
    chk(f"要求 {cond}", cond in blk, why)
chk("★不再檢查『只要不是 DEPLOYING 就可以』",
    "flight_state != FLIGHT_DEPLOYING" not in blk,
    "舊條件讓 IDLE/LAUNCHED/DEPLOYED/LANDED 全部可觸發")
fires = re.findall(r"FIRE_7V_(\d)_Pin, GPIO_PIN_SET", blk)
chk("★只點燃一路火工品", fires == ["1"],
    f"實際拉高的通道 {fires}（舊碼是 ['1','2'] 兩路同時）")
chk("被拒時有回饋", "MANUAL FIRE REJECTED" in blk)

print()
print("=" * 70)
print("【4】離架時強制退出 BRIDGE（原本只有 USB 能退出）")
print("=" * 70)
i = m.index("flight_state   = FLIGHT_LAUNCHED;\n              is_boosting")
chk("2.5g 離架偵測會呼叫 cmd_exit_bridge()",
    "cmd_exit_bridge()" in m[i:i + 1200],
    "BRIDGE 會靜音本板全部遙測，USB 拔掉後就退不出來")
chk("同一處也清 gnd_test_until", "gnd_test_until  = 0" in m[i:i + 1200])
chk("cmd_exit_bridge 已實作", "void cmd_exit_bridge(void)" in S["cmd.c"])
chk("cmd_exit_bridge 已宣告", "cmd_exit_bridge" in S["cmd.h"])

print()
print("=" * 70)
print("【5】維修指令只准在地面（會阻塞主迴圈 16~180 秒）")
print("=" * 70)
c = S["cmd.c"]
chk("有 is_maintenance_cmd 分類函式", "is_maintenance_cmd" in c)
for cmd in ("PINTEST", "PINHOLD", "BUSFLOAT", "READ", "CLEAR", "TRUNC", "BRIDGE"):
    chk(f"  {cmd} 在清單內", f'"{cmd}"' in c[c.index("static int is_maintenance_cmd"):
                                            c.index("static void process_command_exec")])
gate = c[c.index("static void process_command_exec"):][:900]
chk("閘門在 process_command_exec 最前面",
    "is_maintenance_cmd(cmd) && !flight_is_idle()" in gate)
chk("main.c 匯出 flight_is_idle()", "uint8_t flight_is_idle(void)" in m)
# 閘門必須在任何 HAL_Delay 之前
chk("閘門位置早於所有阻塞呼叫",
    c.index("!flight_is_idle()") < c.index("HAL_Delay("))

print()
print("=" * 70)
print("【6】沒有動到的東西（確認這次沒有誤傷）")
print("=" * 70)
for k, v in (("DEPLOY_TB_MS", "18000UL"), ("LAUNCH_AZ_G", "2.5f"),
             ("DEPLOY_DROP_M", "10.0f"), ("DEPLOY_PEAK_MIN_M", "20.0f"),
             ("AIRBAG_IMPACT_G", "5.0f")):
    got = re.search(rf"#define {k}\s+(\S+)", m)
    chk(f"{k} 仍是 {v}", got and got.group(1) == v, got.group(1) if got else "找不到")
chk("開傘決策式未改動", "int deploy_main = (cond_A_eff && cond_B_eff);" in m)
chk("降級規則未改動", "return mod.bmp585 ? cond_A : 0;" in m)

print()
print("=" * 70)
print("【7】第二輪審查（lora_e22 / cmd / logger / gnss / cdc_write）")
print("=" * 70)
lg = strip_comments((R / "Src/logger.c").read_text(encoding="utf-8"))
chk("logger 行緩衝放大到裝得下整行 CSV", "char line[288];" in lg,
    "舊值 128；CSV 一行 135 + 前綴 9 = 144 → 每行都被截斷，連 CRLF 都沒了")
chk("截斷時會告警", "LINE TRUNCATED" in lg)
chk("截斷時不寫入（避免無換行的黏行）",
    lg.index("LINE TRUNCATED") < lg.index("f_write(&file, line"))

gn = strip_comments((R / "Src/gnss.c").read_text(encoding="utf-8"))
for v in ("gnss_byte_cnt", "gnss_line_cnt", "last_valid_fix_time"):
    chk(f"{v} 補上 volatile", f"volatile uint32_t {v}" in gn, "ISR 寫、主迴圈讀")

lo = strip_comments((R / "Src/lora_e22.c").read_text(encoding="utf-8"))
i = lo.index("int LoRa_Send(")
chk("LoRa_Send 逾時會中止卡住的 IT 傳送",
    "AbortTransmit_IT" in lo[i:i + 700],
    "只清 tx_busy 不夠：HAL 的 gState 仍 BUSY，接著的阻塞傳送必回 HAL_BUSY")

mn = strip_comments((R / "Src/main.c").read_text(encoding="utf-8"))
i = mn.index("size_t remaining = strlen(s);")
seg = mn[i:i + 900]
chk("cdc_write 在沒有主機時停止空轉", "cdc_dead" in seg,
    "VBUS sensing 關閉 → 拔線後 dev_state=SUSPENDED 仍會嘗試傳送，"
    "每 500ms 燒 80ms")
chk("有主機時行為不變（成功即歸零）", "cdc_dead = 0;" in seg)

print()
print("PASS" if not fails else f"FAIL: {fails}")
sys.exit(1 if fails else 0)
