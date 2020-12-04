/*
 * rtc.c
 *
 *  Created on: Jun 16, 2018
 *      Author: Marco
 */

#include "rtc.h"

rtcISR_t rtcISRHook;
uint8_t _rtcReady = 1;

//=============================
/*-------- Prototypes -------*/
//=============================
static uint8_t rtcHWInitialize(void);
static uint8_t rtcHWReinitialize(void);
static uint8_t rtcSWInitialize(rtcISR_t isrHook);
static uint8_t rtcSetClock(void);
static void rtcEnableClock(void);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t rtcInitialize(rtcISR_t isrHook){

	if( rtcSWInitialize(isrHook) ) return 1;

	if( rtcHWInitialize() ) return 2;

	return 0;
}
//-----------------------------
uint8_t rtcReinitialize(rtcISR_t isrHook){

	if( rtcSWInitialize(isrHook) ) return 1;

	if( rtcHWReinitialize() ) return 2;

	return 0;
}
//-----------------------------
uint32_t rtcRead(void){

	uint32_t counter;

	counter = RTC->CNTL;
	counter |= (uint32_t)(RTC->CNTH << 16U);

	return counter;
}
//-----------------------------
uint8_t rtcIsReady(void){

	return _rtcReady;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
uint8_t rtcHWInitialize(void){

	uint8_t clock;

	/* Enables power and backup clocks */
	RCC->APB1ENR |= (1 << 28) | (1 << 27);

	/* Enables access to RTC */
	PWR->CR |= (1 << 8);

	/* Sets priority and enables RTC interrupt in the NVIC Core */
	NVIC_SetPriority(RTC_IRQn, 6);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Enters configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL |= (1 << 4);

	/*
	 * Sets RTC clock. Attempts to set LSE as oscillator. If RTC
	 * fails, sets HSE/128 as clock. The prescaler to generate
	 * a 1 second interrupt will be set accordingly.
	 */
	clock = rtcSetClock();

	/* Clears counter */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CNTL = 0;
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CNTH = 0;

	/* Enables second interrupt */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRH = (1 << 0);

	/* Exits configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL &= (uint16_t)~(1 << 4);

	rtcEnableClock();

	/* RTC calibration */
	//-------------------
	/* Writes 72 to CAL bits */
	//BKP->RTCCR = 72 >> 1;
	/* Outputs RTC_CLK/64 on TAMPER pin */
	//BKP->RTCCR |= 1 << 7;
	//-------------------

	if(clock) return 1;

	return 0;
}
//-----------------------------
uint8_t rtcHWReinitialize(void){

	/* Enables power and backup clocks */
	RCC->APB1ENR |= (1 << 28) | (1 << 27);

	/* Enables access to RTC */
	PWR->CR |= (1 << 8);

	/* Sets priority and enables RTC interrupt in the NVIC Core */
	NVIC_SetPriority(RTC_IRQn, 6);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Enters configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL |= (1 << 4);

	/* Enables second interrupt */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRH = (1 << 0);

	/* Exits configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL &= (uint16_t)~(1 << 4);

	/* RTC calibration */
	//-------------------
	/* Outputs RTC_CLK/64 on TAMPER pin */
	//BKP->RTCCR |= 1 << 7;
	//-------------------

	return 0;
}
//-----------------------------
uint8_t rtcSWInitialize(rtcISR_t isrHook){

	rtcISRHook = isrHook;

	return 0;
}
//-----------------------------
static uint8_t rtcSetClock(void){

	uint32_t timeout = 0xFFFFFF;

	/* Clears RTC clock selection */
	RCC->BDCR &= ~(0x03U << 8U);

	/* Turns 32 kHz oscillator on, waits until it is stable */
	RCC->BDCR |= (1 << 0);
	while( (!(RCC->BDCR & (1 << 1))) && (--timeout) );


	if( timeout ){
		/*
		 * If 32 kHz got stable, choose LSE as clock source and sets
		 * prescaler to generate 1 second interrupt.
		 */
		while( !(RTC->CRL & (1 << 5)) );
		RTC->PRLL = 32768 - 1;
		RCC->BDCR |= (0x01 << 8);
	}
	else{
		/*
		 * If 32 kHz did not get stable, we disable LSE and
		 * choose HSE/128 as clock source. Also, sets prescaler
		 * to generate 1 second interrupt.
		 */
		RCC->BDCR &= (uint32_t)(~(1 << 0));
		while( !(RTC->CRL & (1 << 5)) );
		RTC->PRLL = 0xF423;
		RCC->BDCR |= (0x03 << 8);
		return 1;
	}

	return 0;
}
//-----------------------------
static void rtcEnableClock(void){

	RCC->BDCR |= (1 << 15);
}
//-----------------------------
//=============================

//===========================
/*------ ISR Handler ------*/
//===========================
//---------------------------
void RTC_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void RTC_IRQHandler(void){

	uint32_t counter;

	_rtcReady = 0;

	while( !(RTC->CRL & (1 << 3)) );

	counter = RTC->CNTL;
	counter |= (uint32_t)(RTC->CNTH << 16U);

	/* Enters configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL |= (1 << 4);

	/* Clears second interrupt flag */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL &= (uint16_t)~(1 << 0);

	/* Clears sync flag and exits configuration mode */
	while( !(RTC->CRL & (1 << 5)) );
	RTC->CRL &= (uint16_t)~((1 << 4) | (1 << 3));

	/* Application hook */
#if (configRTC_USE_ISR_HOOK == 1)
	rtcISRHook(counter);
#endif
}
//---------------------------
//===========================
