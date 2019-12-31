/**
 * @file startup.h
 * @brief Simple startup file for STM32F103 devices.
 *
 * This is a simple initialization file for STM32F103 devices, more
 * specifically, for the STM32F103C8T6.
 *
 * Initialization consists of setting the system clock, flash wait-states and
 * setting the RAM memory (data and bss sections). After initialization, the
 * main() function is called.
 *
 *  Created on: Dec 7, 2019
 *      Author: marco
 */

#ifndef STARTUP_H_
#define STARTUP_H_

//=============================
/*-------- Functions --------*/
//=============================
void startup(void);
//=============================

#endif /* STARTUP_H_ */
