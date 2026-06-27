/**
 * protocol.h — 火箭 ⇄ 地面 LoRa 遙測協定（SHARED 共用契約）
 * ===========================================================================
 * 目的：firmware-rocket（發送端）與 firmware-ground（接收端）共用「同一份」
 *       封包格式定義。改格式只改這裡，兩端一起重編 → 避免一端漏改解出垃圾。
 *
 * 鏈路：E22-900T22D，UART 透傳模式，ASCII 文字封包，以 '\n' 結尾。
 *
 * ⚠ 現況（誠實標註，2026-06-27）：
 *   本檔是「依 firmware-rocket/Core/Src/main.c 現有 TX 程式碼抽出」的契約，
 *   但目前兩端「尚未」實際 #include 它——
 *     · 發送端的格式字串仍寫死在 main.c（lora_pkt snprintf）。
 *     · 接收端有自己的解析碼。
 *   「讓兩端都改用本檔的巨集」是建議的下一步，不是已接好的狀態。
 *   採用後才真正獲得 monorepo 的協定同步保證。
 * ===========================================================================
 */
#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>

/* 協定版本：封包格式有任何變動就 +1，兩端可在開機 log / 握手時比對 */
#define RKT_PROTO_VERSION   1

/* ── 飛行狀態碼（封包 S 欄，固定 2 字元）──────────────────────────────── */
#define RKT_ST_IDLE    "ID"   /* 待命（尚未起飛）        */
#define RKT_ST_LAUNCH  "LA"   /* 已起飛                  */
#define RKT_ST_DEPLOY  "DP"   /* 開傘觸發中              */
#define RKT_ST_DOWN    "DD"   /* 已開傘 / 下降 / 落地     */

/* ── 模組健康旗標（封包 M 欄，4 碼，1=OK 0=fail）────────────────────────
 *   順序固定：bmp585, imu, lora, sdcard
 *   例：M=1111 全正常； M=1011 → imu 異常                                  */

/* ── 封包欄位語意 ──────────────────────────────────────────────────────
 *   N   seq    uint32   封包序號（接收端用來偵測掉包）
 *   T   ms     uint32   開機毫秒時間戳
 *   P   —      float    氣壓（單位以 bmp585 驅動輸出為準）
 *   RH  m      float    氣壓相對高度
 *   KH  m      float    Kalman 融合高度（KF2）
 *   G   g      float    總加速度量值（|a|）
 *   S   str    2-char   狀態碼（見上）
 *   M   bbbb   4-digit  模組旗標 bmp/imu/lora/sd
 * ──────────────────────────────────────────────────────────────────────── */

/* 發送端格式（與 main.c:933 lora_pkt 完全一致；勿擅改順序）*/
#define RKT_LORA_TX_FMT \
  "N=%lu T=%lu P=%.2f RH=%.1f KH=%.1f G=%.2f S=%s M=%d%d%d%d\n"

/* 接收端建議解析格式（與 TX 對應；S 收 2 字元，M 拆 4 個 1 位數）
 * 用法：sscanf(line, RKT_LORA_RX_FMT, &seq,&t,&p,&rh,&kh,&g, s2, &b,&i,&l,&sd)
 * 注意：%lu 對 uint32 需平台支援；裸機可改用 %u 視 int 寬度而定。           */
#define RKT_LORA_RX_FMT \
  "N=%lu T=%lu P=%f RH=%f KH=%f G=%f S=%2s M=%1d%1d%1d%1d"

/* 封包緩衝上限（與 firmware-rocket lora_pkt[96] 一致）*/
#define RKT_LORA_MAX_LEN    96

/* 接收端解析後的遙測結構（地面站自行填入；發送端不需要）*/
typedef struct {
  uint32_t seq;          /* N  */
  uint32_t t_ms;         /* T  */
  float    press;        /* P  */
  float    rel_alt_m;    /* RH */
  float    kf_alt_m;     /* KH */
  float    total_g;      /* G  */
  char     state[3];     /* S  ："ID"/"LA"/"DP"/"DD" + '\0' */
  uint8_t  ok_bmp;       /* M[0] */
  uint8_t  ok_imu;       /* M[1] */
  uint8_t  ok_lora;      /* M[2] */
  uint8_t  ok_sd;        /* M[3] */
} rkt_telemetry_t;

#endif /* SHARED_PROTOCOL_H */
