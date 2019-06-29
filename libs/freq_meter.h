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
/** @brief Function pointer for the frequency meter hook. */
typedef void(*freqMeterHook_t)(uint32_t);

/** @brief Defines if frequency meter hook should be used.
 *
 * The hook is called each time the freqMeter measures a new frequency.
 * The counter's value is passed to the hook.
 */
#define configFREQ_METER_USE_HOOK	1
//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
/** @brief Initializes the input and timer for the frequency meter. */
void freqMeterInitialize(uint16_t timerPrescaler, freqMeterHook_t);
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
