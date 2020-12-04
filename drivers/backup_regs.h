/*
 * backup_regs.h
 *
 *  Created on: Jun 21, 2018
 *      Author: Marco
 *
 *	Current version: v0.1.1
 *
 *	-v0.1.1:
 *		- Fixed bug when writing to backup registers numbers 11 and up. Writing to
 *		those register was causing the system to overwrite registers 1-10.
 */

#ifndef BACKUP_REGS_H_
#define BACKUP_REGS_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void backupRegsEnableAccess(void);
uint16_t backupRegsRead(uint8_t reg);
void backupRegsWrite(uint8_t reg, uint16_t data);
//=============================

#endif /* BACKUP_REGS_H_ */
