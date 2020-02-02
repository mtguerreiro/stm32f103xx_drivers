/*
 * boot.h
 *
 *  Created on: Jan 25, 2020
 *      Author: marco
 */

#ifndef BOOT_H_
#define BOOT_H_

//=============================
/*-------- Functions --------*/
//=============================
void __attribute__ ((section(".boot"))) boot(void);
//=============================

#endif /* BOOT_H_ */
