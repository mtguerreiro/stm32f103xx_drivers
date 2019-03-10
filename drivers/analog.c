/*
 * analog.c
 *
 *  Created on: May 2, 2018
 *      Author: Marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "analog.h"

/* Drivers */
#include "gpio.h"
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static uint8_t analogHWInitialize(uint32_t channels);
static uint8_t analogSWInitialize(void);
static void analogConfigChannels(uint32_t channels);
//=============================

//=============================
/*-------- Semaphores -------*/
//=============================
static SemaphoreHandle_t analogMutex;
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t analogInitialize(uint32_t channels){

	if( analogHWInitialize(channels) ) return 1;

	if( analogSWInitialize() ) return 2;

	return 0;
}
//-----------------------------
uint8_t analogRead(uint8_t channel, uint16_t *buffer){

	/* Selects channel to be first in conversion sequence */
	ADC1->SQR3 = channel & 0x1F;

	/* Clears end-of-conversion flag and starts a new conversion */
	ADC1->SR &= (uint16_t)(~ADC_SR_EOC);
	ADC1->CR2 |= ADC_CR2_ADON;

	/* Waits for conversion */
#if configANALOG_READ_TIMEOUT == 0
	while(!(ADC1->SR & ADC_SR_EOC));
#else
	uint32_t k;
	k = configANALOG_READ_TIMEOUT;
	while( !(ADC1->SR & ADC_SR_EOC) && (--k) );
	if(!k) return 1;
#endif

	*buffer = (uint16_t)ADC1->DR;
	return 0;
}
//-----------------------------
uint8_t analogMutexTake(uint32_t ticks){

	if( xSemaphoreTake(analogMutex, ticks) != pdTRUE ) return 1;

	return 0;
}
//-----------------------------
uint8_t analogMutexGive(void){

	if( xSemaphoreGive(analogMutex) != pdTRUE) return 1;

	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static uint8_t analogHWInitialize(uint32_t channels){
	uint32_t k;

	/* Initialize analog inputs */
	analogConfigChannels(channels);

	/* Selects clock divider and enables the peripheral */
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	/* Turns ADC on and waits initialization */
	ADC1->CR2 = ADC_CR2_ADON;
	k = 0xFF;
	while(k--);

	ADC1->CR2 = ADC_CR2_CAL | ADC_CR2_ADON;
#if configANALOG_CAL_TIMEOUT == 0
	while(ADC1->CR2 & ADC_CR2_CAL);
#else
	k = configANALOG_CAL_TIMEOUT;
	while( (ADC1->CR2 & ADC_CR2_CAL) && (--k) );
	if(!k) return 1;
#endif

	/* Sets sampling time for all channels to be 239.5 ADC cycles */
	ADC1->SMPR1 = 0x00FFFFFF;
	ADC1->SMPR2 = 0x3FFFFFFF;

	return 0;
}
//-----------------------------
static uint8_t analogSWInitialize(void){

	analogMutex = xSemaphoreCreateMutex();
	if(analogMutex == NULL) return 1;

	return 0;
}
//-----------------------------
static void analogConfigChannels(uint32_t channels){

	/* First, sets channels 0~7, which are all in PORTA, pins 0~7 */
	if( channels & (0xFFU) ){
		gpioPortEnable(GPIOA);
		gpioConfig(GPIOA, (uint16_t)(channels & 0xFFU), GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_ANALOG);
	}

	/* Now, sets channels 8~9, which are all in PORTB, pins 0~1 */
	if( channels & (0x300U) ){
		gpioPortEnable(GPIOB);
		gpioConfig(GPIOB, (uint16_t)((channels & 0x300U) >> 8U), GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_ANALOG);
	}

	/* Lastly, sets channels 10~15, which are all in PORTC, pins 0~5 */
	if( channels & (0xFC00U) ){
		gpioPortEnable(GPIOC);
		gpioConfig(GPIOC, (uint16_t)((channels & 0xFC00U) >> 10U), GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_ANALOG);
	}
}
//-----------------------------
//=============================
