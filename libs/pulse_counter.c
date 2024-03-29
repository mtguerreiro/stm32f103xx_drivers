/*
 * pulse_counter.c
 *
 *  Created on: Jul 20, 2019
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "pulse_counter.h"

/* Device */
#include "stm32f10x.h"

/* Drivers */
#include "gpio.h"
//=============================

//=============================
/*--------- Defines ---------*/
//=============================

//=============================

//=============================
/*--------- Globals ---------*/
//=============================
uint32_t count = 0;

pulseCounterHook_t pulseHook;
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
//-----------------------------
/** @brief Sets GPIOB P3 as floating input to generate interrupts. */
static void pulseCounterInitializePort(void);
//-----------------------------
///** @brief Initializes the timer to measure the frequency.
// *
// * @param timerPrescaler Defines the prescaler for TIM2's clock (which is
// * 	expected to be 72 MHz).
// */
//static void pulseCounterInitializeTimer(uint16_t timerPrescaler);
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void pulseCounterInitialize(pulseCounterHook_t hook){

	pulseCounterInitializePort();
	pulseHook = hook;
}
//-----------------------------
void pulseCounterStart(void){

	/* To start the pulse counter meter, we just enable input interrupts and clear the counter */

	count = 0;

	/* Clears interrupt request for EXTII line 4*/
	EXTI->PR |= (1U << 4);
	/* Interrupt for line 4 is not masked */
	EXTI->IMR |= (1U << 4);
}
//-----------------------------
void pulseCounterStop(void){

	/* To stop the pulse counter, we just disable input interrupts */

	/* Interrupt for line 4 is masked */
	EXTI->IMR &= ~(1U << 4);
}
//-----------------------------
uint32_t pulseCounterGet(void){

	return count;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void pulseCounterInitializePort(void){

	/* Sets GPIO P4 as floating input */
	gpioPortEnable(GPIOB);
	gpioConfig(GPIOB, GPIO_P4, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT);

	/* Selects GPIOB pin 4 as external source for line 4 */
	AFIO->EXTICR[1] = (1U << 0);
	/* Interrupt for line 4 is not masked */
	//EXTI->IMR |= (1U << 4);
	/* Sets falling edge as trigger for line 4 */
	EXTI->FTSR |= (1U << 4);
	/* Clears pending register */
	EXTI->PR |= (1U << 4);

	/* Sets NVIC priority and enables interrupt */
	NVIC_SetPriority(EXTI4_IRQn, 6);
	NVIC_EnableIRQ(EXTI4_IRQn);
}
//-----------------------------
//=============================


//=============================
/*------- IRQ Handlers ------*/
//=============================
//-----------------------------
void EXTI4_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void EXTI4_IRQHandler(void){

	/* Clears interrupt request */
	EXTI->PR |= (1U << 4);

	count++;

#if configPULSE_COUNTER_USE_HOOK == 1
	pulseHook(count);
#endif
}
//-----------------------------
//=============================
