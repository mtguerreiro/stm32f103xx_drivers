/*
 * pulse_counter.h
 *
 *	Counts the number of falling edges on a selected input.
 *
 *  Created on: Jul 20, 2019
 *      Author: marco
 */

#ifndef LIBS_PULSE_COUNTER_H_
#define LIBS_PULSE_COUNTER_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/** @brief Function pointer for the frequency meter hook. */
typedef void(*pulseCounterHook_t)(uint32_t);

/** @brief Defines if frequency meter hook should be used.
 *
 * The hook is called each time the freqMeter measures a new frequency.
 * The counter's value is passed to the hook.
 */
#define PULSE_CONFIG_COUNTER_USE_HOOK		1

/** @brief Defines which edge should trigger an event.
 *
 * If 0, rising edge triggers an event. If 1 (or any value other than 0),
 * the falling edge triggers an event.
 */
#define PULSE_CONFIG_EDGE_DETECT			0

/** @brief Defines interrupt priority of pulse detection. */
#define PULSE_CONFIG_IRQ_PRIO				0x06
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/** @brief Initializes the input and timer for the pulse counter.
 *
 * @param hook Function to be called at each new pulse is detected. This
 * 	function must accept an uint32_t as variable, since the hook is called
 * 	with the counter's value as a 32-bit variable.
 */
void pulseCounterInitialize(pulseCounterHook_t hook);
//---------------------------------------------------------------------------
/** @brief Starts the pulse counter. */
void pulseCounterStart(void);
//---------------------------------------------------------------------------
/** @brief Stops the pulse counter. */
void pulseCounterStop(void);
//---------------------------------------------------------------------------
/** @brief Gets the current pulse count. */
uint32_t pulseCounterGet(void);
//---------------------------------------------------------------------------
/** @brief Sets the current pulse count. */
void pulseCounterSet(uint32_t value);
//---------------------------------------------------------------------------
/** @brief Clears the pulse counter. */
void pulseCounterClear(void);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* LIBS_PULSE_COUNTER_H_ */
