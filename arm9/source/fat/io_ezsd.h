/*
	io_m3sd.h  by SaTa.

	Hardware Routines for reading a compact flash card
	using the GBA Movie Player

	This software is completely free. No warranty is provided.
	If you use it, please give me credit and email me about your
	project at chishm@hotmail.com

	See gba_nds_fat.txt for help and license details.
*/

#ifndef IO_EZSD_H
#define IO_EZSD_H

#include "disc_io.h"

// 'EZSD'
#define DEVICE_TYPE_EZSD 0x44535A45

// export interface
extern LPIO_INTERFACE EZSD_GetInterface(void) ;

#endif	// define IO_EZSD_H
