/*
 * sdcard.h
 *
 *  Created on: Jun 2, 2025
 *      Author: user
 */
#include "fatfs.h"
#ifndef __SDCARD_H
#define __SDCARD_H

extern FIL file;

void sd_write_log(void);

#endif
