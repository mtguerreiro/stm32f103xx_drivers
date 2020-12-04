/*
 * rtc.h
 *
 *  Created on: Jun 16, 2018
 *      Author: Marco
 *
 * Current version: v0.1.4
 *
 * 	-Possible issues:
 * 		- (v0.1.0 - unresolved)
 * 		While the RTC is powered by the battery, the RTC counter will keep
 * 		incrementing. Once the microcontroller is powered by the main supply system,
 * 		the RTC will generate an interrupt, which will update the calendar. However,
 * 		in the current implementation, the counter is stored in a variable and the
 * 		counter register is cleared. After this procedure, the calendar hook is called
 * 		with the counter's value. If, however, power goes down before the calendar
 * 		can account for all seconds, the calendar will not be updated correctly. The
 * 		correct procedure would be for the calendar application to decrement the counter
 * 		each time a second is accounted for. This can be very inefficient and time consuming,
 * 		since writing to the counter requires to enter configuration mode and wait for each
 * 		write operation to be completed.
 *
 * 		- (v0.1.1 - unresolved)
 * 		When the LSE clock fails to start, HSE/128 clock will be selected. However, this
 * 		needs to be resolved before release, since it does not guarantee that the RTC will
 * 		keep working when the power goes down, even if a battery is present. If HSE clock
 * 		is selected and the power goes down, the RTC will not keep counting since the HSE
 * 		clock will be inactive.
 *
 * 		- (v0.1.3 - solved in v0.1.4)
 * 		When the RTC is initialized, it appears that an interrupt is not generated immediately.
 * 		This can cause issues in the calendar, since the calendar will not be updated when the
 * 		RTC is initialized, only when an interrupt occurs, possibly one second after enabling
 * 		the interrupt. (Now I am in doubt, whether the interrupt will always occur one second
 * 		after it is enabled or if one second is the worst case scenario, since the counter may
 * 		be in mid count.
 *
 * 	v0.1:
 * 		- Initial version
 *
 * 	v0.1.1:
 * 		- Added functions to automatically select RTC clock and enable RTC clock.
 *
 * 	v0.1.2:
 * 		- Added prototype for IRQ handler.
 *
 * 	v0.1.3:
 * 		- rtcInitializes returns 1 if LSE crystal fails to initialize. However, rtcReinitialize remains the same,
 * 		since it only enables the peripheral power.
 *
 *	v0.1.4:
 *		- Added flag to indicate that the RTC entered the interrupt and the calendar
 *		has been updated. This can be verified through the rtcIsReady function.
 */

#ifndef RTC_H_
#define RTC_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*---- Function pointers ----*/
//=============================
typedef void(*rtcISR_t)(uint32_t);
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configRTC_USE_ISR_HOOK		1
//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t rtcInitialize(rtcISR_t isrHook);
uint8_t rtcReinitialize(rtcISR_t isrHook);
uint32_t rtcRead(void);
uint8_t rtcIsReady(void);
//=============================

#endif /* RTC_H_ */
