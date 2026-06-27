/*
 * adcard.c
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#include "main.h"
#include "sdcard.h"
#include <stdint.h>
#include "fatfs.h"


FIL file;
UINT bw;

void sd_write_log(void) {
	MX_FATFS_Init();
	f_mount(&USERFatFS, (TCHAR const*)USERPath, 1);
}

