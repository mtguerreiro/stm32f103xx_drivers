/*
 * freq_meter.c
 *
 *  Created on: Jun 28, 2019
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "freq_meter.h"

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
uint32_t freq = 0;

freqMeterHook_t freqHook;
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static void freqMeterInitializePort(void);
static void freqMeterInitializeTimer(uint16_t timerPrescaler);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void freqMeterInitialize(uint16_t timerPrescaler, freqMeterHook_t hook){

	freqMeterInitializePort();
	freqMeterInitializeTimer(timerPrescaler);
	freqHook = hook;
}
//-----------------------------
void freqMeterStart(void){

	/* To start the freq meter, we just enable input interrupts and the timer */

	/* Clears interrupt request for EXTII */
	EXTI->PR |= (1U << 3);
	/* Interrupt for line 3 is not masked */
	EXTI->IMR |= (1U << 3);

	/* Clears and enables timer counter */
	TIM2->EGR |= 1;
	TIM2->CR1 |= (1 << 0);
}
//-----------------------------
void freqMeterStop(void){

	/* To stop the freq meter, we just disable input interrupts and the timer */

	/* Interrupt for line 3 is masked */
	EXTI->IMR &= ~(1U << 3);

	/* Clears and enables timer counter */
	TIM2->CR1 &= (uint16_t)(~(1U << 0));
}
//-----------------------------
uint32_t freqMeterGet(void){

	return freq;
}
//-----------------------------
//=============================


//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void freqMeterInitializePort(void){

	/* Sets GPIO P3 as floating input */
	gpioPortEnable(GPIOB);
	gpioConfig(GPIOB, GPIO_P3, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT_INPUT);

	/* Selects GPIOB pin 3 as external source for line 3 */
	AFIO->EXTICR[0] = (1U << 12);
	/* Interrupt for line 3 is not masked */
	//EXTI->IMR |= (1U << 3);
	/* Sets falling edge as trigger for line 2 */
	EXTI->FTSR |= (1U << 3);
	/* Clears pending register */
	EXTI->PR |= (1U << 3);

	/* Sets NVIC priority and enables interrupt */
	NVIC_SetPriority(EXTI3_IRQn, 6);
	NVIC_EnableIRQ(EXTI3_IRQn);
}
//-----------------------------
static void freqMeterInitializeTimer(uint16_t timerPrescaler){

	/* Here, timer 2 is enabled */

	/* Disables TIM2 just in case */
	TIM2->CR1 = 0;

	/*
	 * Enables clock to timer.
	 * The clock should be 36 MHz.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/*
	 * Bit 7 - ARPE (Auto Reload Preload Enable): Register is not buffered (bit clear)
	 * Bit 3 - OPM (One-pulse mode): Counter does not stops at update event (bit clear)
	 * Bit 2 - URS (Update request source): Only counter overflow generates interrupt request (bit set)
	 * Bit 1 - UDIS (Update disable): UEV is enabled and buffered registers are reloaded at UEV (bit clear)
	 * Bit 0 - CEN (Counter enabled): Counter disabled (bit clear)
	 */
	TIM2->CR1 = (1 << 2);

	/*
	 * Bit 8 - UDE (Update DMA enable): Update DMA request disabled (bit clear)
	 * Bit 0 - UIE (Update interrupt enable): Update interrupt disabled (bit clear)
	 */
	TIM2->DIER = 0;

	/*
	 * Sets prescaler.
	 * The TIMER clock will be:
	 * f = (36*2 MHz)/(prescaler)
	 *
	 * The clock is 36*2 due to the APB1 prescaler being 2.
	 * In this case, the timer clock is 2 times the APB1
	 * clock. Since the APB1 clock is 36 MHz and the prescaler
	 * is different than 1. (See Figure 11 in the RCC section
	 * of the user guide.)
	 */
	TIM2->PSC = (uint16_t)(timerPrescaler - 1);

	/*
	 * Sets Auto-Reload Register so the counter will count 65536 times.
	 * The time until the counter overflows will be:
	 * t = (65536)/(250000) = 0.262144 s or ~3.81 Hz
	 */
	TIM2->ARR = 0xFFFF;

	/* Clears and enables timer counter */
	TIM2->EGR |= 1;
	TIM2->CR1 |= (1 << 0);
}
//-----------------------------
//=============================


//=============================
/*------- IRQ Handlers ------*/
//=============================
//-----------------------------
void EXTI3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void EXTI3_IRQHandler(void){

	/* Clears interrupt request */
	EXTI->PR |= (1U << 3);

	/* Saves timer value */
	freq = TIM2->CNT;

	/* Clears timer value to start counting for the next cycle */
	TIM2->EGR |= 1;

#if configFREQ_METER_USE_HOOK == 1
	freqHook(freq);
#endif
}
//-----------------------------
//=============================
