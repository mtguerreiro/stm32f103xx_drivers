/*
 * nrf24l01_stm32.c
 *
 *  Created on: 15 de abr de 2023
 *      Author: marco
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "nrf24l01_stm32.h"

#include "nrf24l01.h"

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"

/* Drivers */
#include "stm32f10x.h"
#include "gpio.h"
#include "spihl.h"
//=============================================================================

//=============================================================================
/*------------------------------- Definitions -------------------------------*/
//=============================================================================
#define configNRF24L01_SPI			SPI1

#define configNRF24L01_CE_PORT		GPIOB
#define configNRF24L01_CE_PIN		GPIO_P0

#define configNRF24L01_CSN_PORT		GPIOB
#define configNRF24L01_CSN_PIN		GPIO_P1

#define configNRF24L01_IRQ_PORT		GPIOB
#define configNRF24L01_IRQ_PIN		GPIO_P3

#define configNRF24L01_CE_SET		gpioOutputSet(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN)
#define configNRF24L01_CE_RESET		gpioOutputReset(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN)

#define configNRF24L01_CSN_SET		gpioOutputSet(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN)
#define configNRF24L01_CSN_RESET	gpioOutputReset(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN)

//=============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32InitializeHw(void);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32InitializeSw(void);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32SpiWrite(uint8_t *buffer, uint16_t size, uint32_t to);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32SpiRead(uint8_t *buffer, uint16_t size, uint32_t to);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32CSNWrite(uint8_t level);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32CEWrite(uint8_t level);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32DelayMicroSec(uint32_t delay);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32IrqClear(void);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32IrqPend(uint32_t to);
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
SemaphoreHandle_t nrf24l01Semaphore;
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
uint8_t nrf24l01Stm32Initialize(void){

	nrf24l01Stm32InitializeHw();
	nrf24l01Stm32InitializeSw();

	nrf24l01Initialize(nrf24l01Stm32SpiWrite, nrf24l01Stm32SpiRead,
			nrf24l01Stm32CSNWrite, nrf24l01Stm32CEWrite,
			nrf24l01Stm32DelayMicroSec,
			nrf24l01Stm32IrqClear, nrf24l01Stm32IrqPend);

	return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*----------------------------- Static functions ----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32InitializeHw(void){

	/* SPI */
	spihlInitialize(configNRF24L01_SPI, SPIHL_BR_CLK_DIV_128, SPIHL_POLL_PHAF);

	/* GPIOs */
	gpioPortEnable(configNRF24L01_CE_PORT);
	gpioConfig(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(configNRF24L01_CSN_PORT);
	gpioConfig(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(configNRF24L01_IRQ_PORT);
	gpioConfig(configNRF24L01_IRQ_PORT, configNRF24L01_IRQ_PIN, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);

	/* Interrupt on IRQ port */

	/* Selects GPIOB pin 3 as external source for line 3 */
	AFIO->EXTICR[0] = (1U << 12);
	/* Interrupt for line 3 is not masked */
	EXTI->IMR |= (1U << 3);
	/* Sets falling edge as trigger for line 2 */
	EXTI->FTSR |= (1U << 3);
	/* Clears pending register */
	EXTI->PR |= (1U << 3);

	/* Sets NVIC priority and enables interrupt */
	NVIC_SetPriority(EXTI3_IRQn, 6);
	NVIC_EnableIRQ(EXTI3_IRQn);

	return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32InitializeSw(void){

	nrf24l01Semaphore = xSemaphoreCreateBinary();
	if(nrf24l01Semaphore == NULL) return 2;
	xSemaphoreTake(nrf24l01Semaphore, 0);

	return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32SpiWrite(uint8_t *buffer, uint16_t size, uint32_t to){

	if( spihlWriteBare(configNRF24L01_SPI, buffer, size, to) != 0 ) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32SpiRead(uint8_t *buffer, uint16_t size, uint32_t to){

	if( spihlReadBare(configNRF24L01_SPI, buffer, size, to) != 0 ) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32CSNWrite(uint8_t level){

	if( level == 0 )
		gpioOutputReset(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN);
	else
		gpioOutputSet(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN);

	return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32CEWrite(uint8_t level){

	if( level == 0 )
		gpioOutputReset(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN);
	else
		gpioOutputSet(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN);

	return 0;
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32DelayMicroSec(uint32_t delay){

	delaysus(delay);
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32IrqClear(void){

	xSemaphoreTake( nrf24l01Semaphore, 0 );
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01Stm32IrqPend(uint32_t to){

	if( xSemaphoreTake( nrf24l01Semaphore, to ) != pdPASS ) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*------------------------------ IRQ Handlers -------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void EXTI3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void EXTI3_IRQHandler(void){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	EXTI->PR |= (1U << 3);

	xSemaphoreGiveFromISR(nrf24l01Semaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
//-----------------------------------------------------------------------------
//=============================================================================
