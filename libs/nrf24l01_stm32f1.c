/*
 * nrf24l01_stm32f1.c
 *
 *  Created on: 15 de abr de 2023
 *      Author: marco
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "nrf24l01_stm32f1.h"

#include "nrf24l01.h"

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"

/* Drivers */
#include "stm32f10x.h"
#include "gpio.h"
#include "spihl.h"

/* Libs */
#include "delays.h"
//=============================================================================

//=============================================================================
/*------------------------------- Definitions -------------------------------*/
//=============================================================================
#define NRF24L01_STM32F1_CONFIG_SPI			SPI1

#define NRF24L01_STM32F1_CONFIG_CE_PORT		GPIOB
#define NRF24L01_STM32F1_CONFIG_CE_PIN		GPIO_P0

#define NRF24L01_STM32F1_CONFIG_CSN_PORT	GPIOB
#define NRF24L01_STM32F1_CONFIG_CSN_PIN		GPIO_P1

/* Do not change this as the change is not fully automated yet */
#define NRF24L01_STM32F1_CONFIG_IRQ_PORT	GPIOB
#define NRF24L01_STM32F1_CONFIG_IRQ_PIN		GPIO_P3
//=============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1InitializeHw(void);
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1InitializeSw(void);
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1SpiWrite(uint8_t *buffer, uint16_t size, uint32_t to);
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1SpiRead(uint8_t *buffer, uint16_t size, uint32_t to);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1CSNWrite(uint8_t level);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1CEWrite(uint8_t level);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1DelayMicroSec(uint32_t delay);
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1IrqClear(void);
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1IrqPend(uint32_t to);
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
int8_t nrf24l01Stm32F1Initialize(void){

	nrf24l01Stm32F1InitializeHw();
	nrf24l01Stm32F1InitializeSw();

	nrf24l01Initialize(nrf24l01Stm32F1SpiWrite, nrf24l01Stm32F1SpiRead,
			nrf24l01Stm32F1CSNWrite, nrf24l01Stm32F1CEWrite,
			nrf24l01Stm32F1DelayMicroSec,
			nrf24l01Stm32F1IrqClear, nrf24l01Stm32F1IrqPend);

	return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*----------------------------- Static functions ----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1InitializeHw(void){

	/* SPI */
	spihlInitialize(NRF24L01_STM32F1_CONFIG_SPI, SPIHL_BR_CLK_DIV_128, SPIHL_POLL_PHAF);

	/* GPIOs */
	gpioPortEnable(NRF24L01_STM32F1_CONFIG_CE_PORT);
	gpioConfig(NRF24L01_STM32F1_CONFIG_CE_PORT,
			NRF24L01_STM32F1_CONFIG_CE_PIN,
			GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(NRF24L01_STM32F1_CONFIG_CSN_PORT);
	gpioConfig(NRF24L01_STM32F1_CONFIG_CSN_PORT,
			NRF24L01_STM32F1_CONFIG_CSN_PIN,
			GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(NRF24L01_STM32F1_CONFIG_IRQ_PORT);
	gpioConfig(NRF24L01_STM32F1_CONFIG_IRQ_PORT,
			NRF24L01_STM32F1_CONFIG_IRQ_PIN,
			GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);

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
static int8_t nrf24l01Stm32F1InitializeSw(void){

	nrf24l01Semaphore = xSemaphoreCreateBinary();
	if(nrf24l01Semaphore == NULL) return -1;
	xSemaphoreTake(nrf24l01Semaphore, 0);

	return 0;
}
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1SpiWrite(uint8_t *buffer, uint16_t size, uint32_t to){

	if( spihlWriteBare(NRF24L01_STM32F1_CONFIG_SPI, buffer, size, to) != 0 ) return -1;

	return 0;
}
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1SpiRead(uint8_t *buffer, uint16_t size, uint32_t to){

	if( spihlReadBare(NRF24L01_STM32F1_CONFIG_SPI, buffer, size, to) != 0 ) return -1;

	return 0;
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1CSNWrite(uint8_t level){

	if( level == 0 )
		gpioOutputReset(NRF24L01_STM32F1_CONFIG_CSN_PORT, NRF24L01_STM32F1_CONFIG_CSN_PIN);
	else
		gpioOutputSet(NRF24L01_STM32F1_CONFIG_CSN_PORT, NRF24L01_STM32F1_CONFIG_CSN_PIN);
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1CEWrite(uint8_t level){

	if( level == 0 )
		gpioOutputReset(NRF24L01_STM32F1_CONFIG_CE_PORT, NRF24L01_STM32F1_CONFIG_CE_PIN);
	else
		gpioOutputSet(NRF24L01_STM32F1_CONFIG_CE_PORT, NRF24L01_STM32F1_CONFIG_CE_PIN);
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1DelayMicroSec(uint32_t delay){

	delaysus(delay);
}
//-----------------------------------------------------------------------------
static void nrf24l01Stm32F1IrqClear(void){

	xSemaphoreTake( nrf24l01Semaphore, 0 );
}
//-----------------------------------------------------------------------------
static int8_t nrf24l01Stm32F1IrqPend(uint32_t to){

	if( xSemaphoreTake( nrf24l01Semaphore, to ) != pdPASS ) return -1;

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
