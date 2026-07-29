/**
 * protocol.h — 火箭 ⇄ 地面 LoRa 遙測協定（SHARED 共用契約）
 * ===========================================================================
 * 目的：把「封包長什麼樣」寫在一個地方。發送端改格式時改這裡，任何接收端
 *       實作都能對照，不必反推 main.c 的 snprintf。
 *
 * 鏈路：E22-900T22D，UART 透傳模式（M0=M1=0），ASCII 文字，以 "\r\n" 結尾。
 *
 * ⚠ 現況（誠實標註，2026-07-30）
 *   · 發送端：格式字串仍寫死在 firmware-rocket/Core/Src/main.c 的
 *     lora_pkt snprintf。本檔是「照著它抄下來的契約」，不是它的來源。
 *   · firmware-ground：純透傳橋接（LoRa UART → USB CDC），**完全不解析**，
 *     只把 bytes 原樣轉發到電腦。所以它不需要本檔。
 *   · 真正的解析器是 Python 地面站
 *     rocket_system_ground_side/src/core/models.py 的 SensorData。
 *   → 本檔目前的實際用途是「文件」與「未來 C 解析器的規格」，
 *     兩端都還沒 #include。要真正獲得同步保證，發送端得改用本檔的巨集。
 *
 * ⚠ 2026-07-30 大幅修訂：本檔在此之前描述的是一份早已不存在的舊格式
 *   （"N=%lu T=%lu P=.. S=ID/LA/DP/DD M=1111\n"）。實際封包沒有 "=" 分隔、
 *   沒有 2 字元狀態碼、模組旗標是 hex 而非 4 個十進位數字，而且多了
 *   加速度/角速度/GPS/火工品電壓等十幾個欄位。照舊版寫的解析器一個欄位
 *   都讀不到。
 * ===========================================================================
 */
#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>

/* 協定版本：封包格式有任何變動就 +1。
 *   1 = 舊格式 "N= T= P= RH= KH= G= S= M="（已廢止，2026-06 之前）
 *   2 = 現行格式（本檔以下所述）*/
#define RKT_PROTO_VERSION   2

/* ═══════════════════════════════════════════════════════════════════════
 * 封包格式
 * ═══════════════════════════════════════════════════════════════════════
 * 欄位之間以「一個半形空格」分隔，Key 緊貼數值（無 "="），行尾 "\r\n"。
 * GPS 定位成功時尾端多 LAT/LON 兩欄，未定位時沒有——**這是唯一的變體**。
 *
 * 範例（有定位）：
 *   T28386 SQ42 AX+0.007 AY+0.026 AZ+0.978 GX+6.09 GY-1.05 GZ-2.80
 *   P997.92 RH-0.1 KH-0.1 VZ+0.00 GA0.98 ST:0 MOD:F GPS:1,8 C:0
 *   VF8.12 VA7.98 LAT+22.17483 LON+120.89272
 * ─────────────────────────────────────────────────────────────────────── */

/* 發送端格式（與 main.c 的 lora_pkt snprintf 逐字一致；勿擅改順序）*/
#define RKT_LORA_TX_FMT_GPS \
  "T%lu SQ%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f " \
  "P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f ST:%d MOD:%X GPS:1,%u C:%X " \
  "VF%.2f VA%.2f LAT%+0.5f LON%+0.5f\r\n"

#define RKT_LORA_TX_FMT_NOGPS \
  "T%lu SQ%lu AX%+0.3f AY%+0.3f AZ%+0.3f GX%+0.2f GY%+0.2f GZ%+0.2f " \
  "P%.2f RH%.1f KH%.1f VZ%+0.2f GA%.2f ST:%d MOD:%X GPS:0,0 C:%X " \
  "VF%.2f VA%.2f\r\n"

/* 封包緩衝上限（與 firmware-rocket 的 lora_pkt[256] 一致）*/
#define RKT_LORA_MAX_LEN    256

/* ⚠ 沒有 RKT_LORA_RX_FMT。這是刻意的——單一個 sscanf 解不了這個格式：
 *    ① GPS 定位與否是兩種不同長度的封包
 *    ② 尾端欄位可能被 RF 切掉（截斷幀）
 *    ③ sscanf 遇到一個欄位不合就整串放棄，後面全丟
 *   舊版的 RKT_LORA_RX_FMT 給了「一行就能解完」的錯覺，實際只要 RF 稍差
 *   就會靜默產生一筆全 0 的假資料。Python 端已改成「逐欄位前綴搜尋 +
 *   缺關鍵欄位就整幀丟棄」，C 解析器請照同樣的策略寫。 */

/* ═══════════════════════════════════════════════════════════════════════
 * 欄位語意
 * ═══════════════════════════════════════════════════════════════════════
 *   T    ms      uint32  開機毫秒時間戳
 *   SQ   —       uint32  封包序號，遞增。接收端用來算掉包率；
 *                        數值倒退 = 火箭端重開機
 *   AX/AY/AZ  g   float  三軸加速度（機體座標）
 *   GX/GY/GZ deg/s float 三軸角速度（機體座標）
 *   P    hPa     float   Kalman 平滑後氣壓
 *   RH   m       float   相對起飛點高度（**裸氣壓**，不經 KF——開傘條件 A 用它）
 *   KH   m       float   KF2 融合高度（IMU 積分 + 氣壓修正）
 *   VZ   m/s     float   KF2 垂直速度（開傘條件 B 用它）
 *   GA   g       float   合加速度 |a|
 *   ST   —       int     飛行狀態，見 RKT_ST_*
 *   MOD  hex     1 nibble 模組存活旗標，見下
 *   GPS  int,int —       定位狀態,衛星數。未定位時固定 "GPS:0,0"
 *   C    hex     1 nibble 開傘條件狀態，見下
 *   VF   V       float   保險絲後端電壓（-1.00 = ADC 不可用）
 *   VA   V       float   arming 開關後端電壓（-1.00 = ADC 不可用）
 *   LAT/LON deg  float   （選填）GPS 座標，只在 GPS:1 時出現
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ST：飛行狀態（封包送整數，不是字串）────────────────────────────────
 * ⚠ doc/telemetry_format.md 曾記載一套 12 狀態的方案
 *   （IGNITION/BURNOUT/APOGEE/…）——那份**從未實作**。實際只有這 5 個。
 *   照 12 狀態版判讀會把 ST:2 誤讀成「點火」，實際是「正在放傘」。*/
#define RKT_ST_IDLE       0   /* 地面靜置，等待離架                       */
#define RKT_ST_LAUNCHED   1   /* 離架後，監視高度與時間                   */
#define RKT_ST_DEPLOYING  2   /* 開傘訊號輸出中（DEPLOY_PULSE_MS = 1 秒）  */
#define RKT_ST_DEPLOYED   3   /* 開傘完成，等待落地                       */
#define RKT_ST_LANDED     4   /* 落地確認，低功耗記錄模式（終端狀態）      */

/* ── MOD：模組存活旗標（1 = 正常）─────────────────────────────────────── */
#define RKT_MOD_BMP585    0x8   /* bit3 氣壓計 */
#define RKT_MOD_IMU       0x4   /* bit2 IMU    */
#define RKT_MOD_LORA      0x2   /* bit1 LoRa   */
#define RKT_MOD_SD        0x1   /* bit0 SD 卡  */
/* 常見值：MOD:F 全正常 ／ MOD:7 氣壓計死 ／ MOD:B IMU 死 ／ MOD:E SD 死 */

/* ── C：開傘條件狀態 ───────────────────────────────────────────────────
 * ⚠ 舊文件把這欄稱作「發火迴圈導通狀態」，那是錯的。它是開傘決策的四個
 *   布林值：raw 是感測器實際說的，eff 是套用故障降級後實際採用的。
 * 降級規則（deploy_A_eff/deploy_B_eff，2026-07-30 改為不對稱）：
 *   · 氣壓計死 → A_eff = 0（主路徑關閉，退到 18s 備援計時）
 *     理由：cond_B 讀的 kf2_v 靠氣壓修正，氣壓一死它是漂移量不是量測值。
 *   · IMU 死   → B_eff = 1（cond_A 純氣壓、無積分無漂移，可單獨守門）
 *   · 兩者皆死 → 兩個 eff 都是 0，只剩備援 C。 */
#define RKT_C_COND_A      0x8   /* bit3 cond_A     氣壓：低於峰值 10m       */
#define RKT_C_COND_A_EFF  0x4   /* bit2 cond_A_eff 套用降級後的 A           */
#define RKT_C_COND_B      0x2   /* bit1 cond_B     KF2 速度持續向下 1.5s    */
#define RKT_C_COND_B_EFF  0x1   /* bit0 cond_B_eff 套用降級後的 B           */
/* 判讀：C:0 正常飛行中 ／ C:1 IMU 死 ／ C:F 兩條件皆成立（開傘） */

/* ═══════════════════════════════════════════════════════════════════════
 * 事件訊息（與遙測共用同一條 LoRa，穿插發送）
 * ═══════════════════════════════════════════════════════════════════════
 *   MSG <LEVEL> <CONTENT>\r\n      LEVEL = INFO|WARN|ERROR|SUCCESS
 *   接收端只切前兩個空格，CONTENT 保留其餘所有空格。
 * ─────────────────────────────────────────────────────────────────────── */
#define RKT_MSG_PREFIX    "MSG "

/* 地面站據以自動確認火工品指令的字串（改動須兩端同步）
 * ⚠ 自動開傘送的是 "Parachute deployed (auto A+B ...)" 與
 *   "Parachute deployed (backup timer ...)"，**不含 successfully**，
 *   刻意不觸發下行確認——自動開傘沒有待確認的指令。*/
#define RKT_MSG_DPL_OK    "Parachute deployed successfully"   /* 遠端開傘成功 */
#define RKT_MSG_ABG_OK    "Airbag inflation started"          /* 氣囊充氣中   */
#define RKT_MSG_DPL_DONE  "already deployed"                  /* 傘已開＝證據 */

/* ═══════════════════════════════════════════════════════════════════════
 * 上行指令（地面 → 火箭），burst 4 次、每次間隔 700ms
 *   700ms 是為了與火箭端 2Hz（500ms 一幀）的 TX 窗口錯開——半雙工模組
 *   在自己發送時收不到東西。
 * ═══════════════════════════════════════════════════════════════════════ */
#define RKT_CMD_ARM       "#CMD:ARM_SYSTEM_SALT7763#\r\n"
#define RKT_CMD_DPL       "#CMD:FORCE_DPL_SALT9981#\r\n"
#define RKT_CMD_ABG       "#CMD:OPEN_ABG_SALT8872#\r\n"
#define RKT_CMD_RECAL     "#CMD:RECAL_SALT5566#\r\n"      /* 氣壓零點重校   */
#define RKT_CMD_GNDTEST   "#CMD:GNDTEST_SALT3310#\r\n"    /* 桌測 10 分鐘窗 */
#define RKT_CMD_GNDTEST_OFF "#CMD:GNDTEST_OFF#\r\n"
/* 換頻：#CMD:SETCH_nn#（nn = 00~80，實際頻率 = 850.125 + nn MHz）*/

/* ═══════════════════════════════════════════════════════════════════════
 * 接收端解析後的遙測結構（C 解析器自行填入；發送端不需要）
 * ═══════════════════════════════════════════════════════════════════════ */
typedef struct {
  uint32_t t_ms;         /* T              */
  uint32_t seq;          /* SQ             */
  float    ax, ay, az;   /* AX/AY/AZ   (g) */
  float    gx, gy, gz;   /* GX/GY/GZ (dps) */
  float    press;        /* P      (hPa)   */
  float    rel_alt_m;    /* RH     (m)     */
  float    kf_alt_m;     /* KH     (m)     */
  float    kf_vz;        /* VZ     (m/s)   */
  float    total_g;      /* GA     (g)     */
  uint8_t  state;        /* ST     見 RKT_ST_*  */
  uint8_t  mod_flags;    /* MOD    見 RKT_MOD_* */
  uint8_t  gps_fix;      /* GPS 第一個數字  */
  uint8_t  gps_sats;     /* GPS 第二個數字  */
  uint8_t  cond_flags;   /* C      見 RKT_C_*   */
  float    v_fuse;       /* VF     (V, -1 = 無 ADC) */
  float    v_arm;        /* VA     (V, -1 = 無 ADC) */
  uint8_t  has_coord;    /* LAT/LON 是否存在 */
  float    lat, lon;     /* LAT/LON (deg)  */
} rkt_telemetry_t;

/* ⚠ 截斷偵測（實作解析器時務必照做）
 *   ST / MOD / GA 位在封包後段。任一缺席即代表尾巴被 RF 切掉 → **整幀丟棄**，
 *   不可讓缺席欄位靜默填 0。理由：假的 ST=0 會重設接收端的狀態邊緣偵測基準，
 *   下一筆真封包的 ST:2 就變成「0→2 的轉入」，在沒有新證據的情況下確認了
 *   一道開傘指令——正是下行確認機制要防的假陽性。
 *   同理，宣稱 GPS:1 卻沒有 LAT/LON 的幀必須降級為未定位，不可套用預設座標。*/

#endif /* SHARED_PROTOCOL_H */
