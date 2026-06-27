/*
 * sd_diskio_spi.h - SD/SDHC 卡 SPI 模式 FatFS diskio（rocket_v2）
 *
 * 取代舊板 SDIO diskio。SPI1 + CS=PA4。基於 ChaN MMC/SDC SPI 參考實作，
 * 移植到 STM32 HAL。
 *
 * ── 與 CubeMX FatFS(User-defined) 整合步驟 ──────────────────────────
 * 產碼後編輯 FATFS/Target/user_diskio.c，把 USER_* 轉呼本檔函式：
 *
 *   #include "sd_diskio_spi.h"
 *   DSTATUS USER_initialize(BYTE pdrv){ return SD_disk_initialize(pdrv); }
 *   DSTATUS USER_status    (BYTE pdrv){ return SD_disk_status(pdrv); }
 *   DRESULT USER_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
 *                                     { return SD_disk_read(pdrv,buff,sector,count); }
 *   DRESULT USER_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
 *                                     { return SD_disk_write(pdrv,buff,sector,count); }
 *   DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void* buff)
 *                                     { return SD_disk_ioctl(pdrv,cmd,buff); }
 *
 *   ※ 若你的 FatFS 版本 disk_read 用 LBA_t，把上面 DWORD 改成 LBA_t 即可。
 *   ※ 逾時已改用 HAL_GetTick() 計算，SD_disk_timerproc() 為相容空殼，不必呼叫。
 *      （前提：SysTick 正常運作，SD 操作不在中斷停用的情境下進行）
 */
#ifndef __SD_DISKIO_SPI_H
#define __SD_DISKIO_SPI_H

#include "diskio.h"

DSTATUS SD_disk_initialize(BYTE pdrv);
DSTATUS SD_disk_status(BYTE pdrv);
DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

/* 每 ~10ms 呼叫一次（逾時計數遞減）*/
void    SD_disk_timerproc(void);

#endif /* __SD_DISKIO_SPI_H */
