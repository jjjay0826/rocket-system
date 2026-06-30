/*
 * sd_diskio_spi.c - SD/SDHC SPI 模式 diskio（rocket_v2）
 * SPI1 + CS=PA4。基於 ChaN MMC/SDC SPI 參考實作，移植 STM32 HAL。
 *
 * 速度：init 階段 <400kHz（prescaler 256），資料階段 ~5MHz（prescaler 16，
 *       經 Arduino 模組的 74LVC125 電平轉換較保守、可靠）。
 * 注意：需週期呼叫 SD_disk_timerproc()（每 ~10ms）讓逾時計數遞減。
 */
#include "sd_diskio_spi.h"
#include "spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
#define SD_SPI   (&hspi1)

/* ── CS = CubeMX label「SD_CS」(PA4) ── */
#define CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

/* ── 命令 ── */
#define CMD0   (0)          /* GO_IDLE_STATE */
#define CMD1   (1)          /* SEND_OP_COND (MMC) */
#define ACMD41 (0x80+41)    /* SEND_OP_COND (SDC) */
#define CMD8   (8)          /* SEND_IF_COND */
#define CMD9   (9)          /* SEND_CSD */
#define CMD12  (12)         /* STOP_TRANSMISSION */
#define CMD16  (16)         /* SET_BLOCKLEN */
#define CMD17  (17)         /* READ_SINGLE_BLOCK */
#define CMD18  (18)         /* READ_MULTIPLE_BLOCK */
#define CMD24  (24)         /* WRITE_BLOCK */
#define CMD25  (25)         /* WRITE_MULTIPLE_BLOCK */
#define CMD55  (55)         /* APP_CMD */
#define CMD58  (58)         /* READ_OCR */

/* ── 卡種類旗標 ── */
#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_SDC   (CT_SD1|CT_SD2)
#define CT_BLOCK 0x08

static volatile DSTATUS Stat = STA_NOINIT;   /* 磁碟狀態 */
static BYTE  CardType;
volatile uint8_t SD_dbg_cmd0 = 0xEE, SD_dbg_cmd8 = 0xEE;   /* debug：init 階段 CMD0/CMD8 回應 */
/* 逾時一律用 HAL_GetTick()(1ms SysTick) 計算 deadline，不依賴 timerproc 呼叫頻率 */

/* ══════════ SPI 低階 ══════════ */
static void FCLK_SLOW(void)  /* <400kHz：APB2 84MHz / 256 = 328kHz */
{ MODIFY_REG(SD_SPI->Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_256); }
static void FCLK_FAST(void)  /* ~5.25MHz：84MHz / 16，經電平轉換較保守 */
{ MODIFY_REG(SD_SPI->Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_16); }

static BYTE xchg_spi(BYTE dat)
{
    BYTE rx = 0xFF;
    HAL_SPI_TransmitReceive(SD_SPI, &dat, &rx, 1, 50);
    return rx;
}
static void rcvr_spi_multi(BYTE *buff, UINT btr)
{
    for (UINT i = 0; i < btr; i++) buff[i] = xchg_spi(0xFF);
}
static void xmit_spi_multi(const BYTE *buff, UINT btx)
{
    HAL_SPI_Transmit(SD_SPI, (uint8_t*)buff, (uint16_t)btx, 1000);
}

/* 等卡就緒（回傳 0xFF 表示 ready）；wt = 逾時 ms */
static int wait_ready(UINT wt)
{
    BYTE d;
    uint32_t t0 = HAL_GetTick();
    do { d = xchg_spi(0xFF); } while (d != 0xFF && (HAL_GetTick() - t0) < wt);
    return (d == 0xFF) ? 1 : 0;
}

static void deselect(void)
{
    CS_HIGH();
    xchg_spi(0xFF);   /* 釋放 DO，給卡一個時脈 */
}

static int select_card(void)   /* 1=OK, 0=timeout */
{
    CS_LOW();

    /* ================================================= */
    /* 新增延遲區塊：給予電平轉換晶片足夠的反應時間 */
    /* 模擬 Arduino digitalWrite 的延遲 (約幾微秒) */
    for(volatile int i = 0; i < 500; i++) {
        __NOP();
    }
    /* ================================================= */

    xchg_spi(0xFF); // 發送 Dummy Clock 讓 SD 卡準備好
    
    if (wait_ready(500)) return 1;
    
    deselect();
    return 0;
}

/* 讀資料區塊（btr 必為 512 的倍數情境下逐 512）*/
static int rcvr_datablock(BYTE *buff, UINT btr)
{
    BYTE token;
    uint32_t t0 = HAL_GetTick();
    do { token = xchg_spi(0xFF); } while (token == 0xFF && (HAL_GetTick() - t0) < 200);
    if (token != 0xFE) return 0;          /* 非起始 token = 錯誤 */
    rcvr_spi_multi(buff, btr);
    xchg_spi(0xFF); xchg_spi(0xFF);       /* 丟棄 CRC */
    return 1;
}

#if _USE_WRITE
static int xmit_datablock(const BYTE *buff, BYTE token)
{
    if (!wait_ready(500)) return 0;
    xchg_spi(token);                       /* 起始 token */
    if (token != 0xFD) {                   /* 0xFD = 停止 token，無資料 */
        xmit_spi_multi(buff, 512);
        xchg_spi(0xFF); xchg_spi(0xFF);    /* CRC dummy */
        BYTE resp = xchg_spi(0xFF);
        if ((resp & 0x1F) != 0x05) return 0;   /* 0x05 = 資料已接受 */
    }
    return 1;
}
#endif

/* 送命令；回傳 R1（bit7=0 表示有效）*/
static BYTE send_cmd(BYTE cmd, DWORD arg)
{
    BYTE n, res;

    if (cmd & 0x80) {                      /* ACMD：先送 CMD55 */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    if (cmd != CMD12) {
        deselect();
        if (!select_card()) return 0xFF;
    }

    xchg_spi(0x40 | cmd);
    xchg_spi((BYTE)(arg >> 24));
    xchg_spi((BYTE)(arg >> 16));
    xchg_spi((BYTE)(arg >> 8));
    xchg_spi((BYTE)arg);
    n = 0x01;                              /* 預設 CRC */
    if (cmd == CMD0) n = 0x95;             /* CMD0 CRC */
    if (cmd == CMD8) n = 0x87;             /* CMD8(0x1AA) CRC */
    xchg_spi(n);

    if (cmd == CMD12) xchg_spi(0xFF);      /* CMD12：丟一個 stuff byte */
    n = 10;
    do { res = xchg_spi(0xFF); } while ((res & 0x80) && --n);
    return res;
}

/* ══════════ diskio 介面 ══════════ */
DSTATUS SD_disk_status(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DSTATUS SD_disk_initialize(BYTE pdrv)
{
    BYTE n, cmd, ty, ocr[4];

    if (pdrv) return STA_NOINIT;

    FCLK_SLOW();
    CS_HIGH();
    for (n = 10; n; n--) xchg_spi(0xFF);   /* ≥74 時脈喚醒卡 */

    ty = 0;
    SD_dbg_cmd0 = send_cmd(CMD0, 0);
    SD_dbg_cmd8 = 0xEE;
    if (SD_dbg_cmd0 == 1) {                /* 進入 idle */
        uint32_t it0 = HAL_GetTick();      /* 初始化 1s 預算 */
        SD_dbg_cmd8 = send_cmd(CMD8, 0x1AA);
        if (SD_dbg_cmd8 == 1) {            /* SDv2? */
            for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {        /* 2.7-3.6V */
                while ((HAL_GetTick() - it0) < 1000 && send_cmd(ACMD41, 0x40000000)) ;  /* HCS */
                if ((HAL_GetTick() - it0) < 1000 && send_cmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
                    ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
                }
            }
        } else {                            /* SDv1 或 MMC */
            if (send_cmd(ACMD41, 0) <= 1) { ty = CT_SD1; cmd = ACMD41; }
            else                          { ty = CT_MMC; cmd = CMD1;   }
            while ((HAL_GetTick() - it0) < 1000 && send_cmd(cmd, 0)) ;
            if ((HAL_GetTick() - it0) >= 1000 || send_cmd(CMD16, 512) != 0) ty = 0;  /* 設 512 區塊 */
        }
    }
    CardType = ty;
    deselect();

    if (ty) {                               /* 初始化成功 */
        FCLK_FAST();
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;   /* 非 SDHC：位元組定址 */

    if (count == 1) {
        if (send_cmd(CMD17, sector) == 0 && rcvr_datablock(buff, 512))
            count = 0;
    } else {
        if (send_cmd(CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);              /* 停止連續讀取 */
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if (send_cmd(CMD24, sector) == 0 && xmit_datablock(buff, 0xFE))
            count = 0;
    } else {
        if (CardType & CT_SDC) send_cmd(ACMD41, 0);   /* (可選) 預擦除提示略 */
        if (send_cmd(CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            /* 停止 token：buff 內容不使用，用 dummy 避免傳入 NULL */
            static const BYTE sd_stop_dummy[1] = {0};
            if (!xmit_datablock(sd_stop_dummy, 0xFD)) count = 1;
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}
#endif

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    BYTE csd[16];
    DWORD cs;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {
    case CTRL_SYNC:                          /* 等寫入完成 */
        if (select_card()) res = RES_OK;
        deselect();
        break;
    case GET_SECTOR_COUNT:                   /* 總磁區數 */
        if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {        /* CSD v2 (SDHC) */
                cs = csd[9] + ((DWORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(DWORD*)buff = cs << 10;
            } else {                          /* CSD v1 */
                BYTE n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                cs = (csd[8] >> 6) + ((DWORD)csd[7] << 2) + ((DWORD)(csd[6] & 3) << 10) + 1;
                *(DWORD*)buff = cs << (n - 9);
            }
            res = RES_OK;
        }
        deselect();
        break;
    case GET_BLOCK_SIZE:                     /* 抹除區塊大小（磁區）*/
        *(DWORD*)buff = 128;
        res = RES_OK;
        break;
    default:
        res = RES_PARERR;
    }
    return res;
}

/* 逾時改用 HAL_GetTick() 直接計算，本函式保留為相容空殼（可不呼叫）*/
void SD_disk_timerproc(void)
{
    /* no-op：時序不再依賴此函式的呼叫頻率 */
}
