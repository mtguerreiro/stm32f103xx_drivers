/*
 * @file freq_meter.h
 * @brief Frequency meter.
 *
 * Measures the frequency of a PWM on the selected input pin.
 *
 *  Created on: Jun 28, 2019
 *      Author: Marco
 */

#ifndef LIBS_FREQ_METER_H_
#define LIBS_FREQ_METER_H_

//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>
//=============================

//=============================
/*--------- Defines ---------*/
//=============================

//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void freqMeterInitialize(void);
//-----------------------------
uint32_t freqMeterGet(void);
//-----------------------------
//=============================

#endif /* LIBS_FREQ_METER_H_ */
