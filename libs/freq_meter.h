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
/** @brief Initializes the input and timer for the frequency meter. */
void freqMeterInitialize(uint16_t timerPrescaler);
//-----------------------------
/** @brief Starts the frequency meter. */
void freqMeterStart(void);
//-----------------------------
/** @brief Stops the frequency meter. */
void freqMeterStop(void);
//-----------------------------
/** @brief Gets the current timer counter.
 *
 * The timer value represents the signal's period.
 */
uint32_t freqMeterGet(void);
//-----------------------------
//=============================

#endif /* LIBS_FREQ_METER_H_ */
