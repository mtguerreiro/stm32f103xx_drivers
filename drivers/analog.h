/*
 * analog.h
 *
 *  Created on: March 10, 2019
 *      Author: Marco
 *
 *	This is the ADC1 driver for STM32F103xx devices.
 *
 * v0.1.0:
 * 		- Initial version
 *
 * v0.1.1:
 * 		- Updated formatting
 */

#ifndef ANALOG_H_
#define ANALOG_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/** Sets time out for calibration */
#define configANALOG_CAL_TIMEOUT	0xFFFF

/** Sets time out for an analog read */
#define configANALOG_READ_TIMEOUT	0xFFFF
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
uint8_t analogInitialize(uint32_t channels);
uint8_t analogRead(uint8_t channel, uint16_t *buffer);
uint8_t analogMutexTake(uint32_t ticks);
uint8_t analogMutexGive(void);
//===========================================================================

#endif /* ANALOG_H_ */
