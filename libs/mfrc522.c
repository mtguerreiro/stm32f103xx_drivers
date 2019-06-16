/*
 * mfrc522.c
 *
 *  Created on: June 15, 2018
 *      Author: Marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include <mfrc522.h>
#include <stdint.h>

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"

/* Drivers */
#include "spi.h"
#include "gpio.h"
//=============================

#define configMFRC522_NSS_PORT		GPIOB
#define configMFRC522_NSS_PIN		GPIO_P1

#define configMFRC522_IRQ_PORT		GPIOB
#define configMFRC522_IRQ_PIN		GPIO_P3

#define configMFRC522_NSS_SET		gpioOutputSet(configMFRC522_NSS_PORT, configMFRC522_NSS_PIN)
#define configMFRC522_NSS_RESET		gpioOutputReset(configMFRC522_NSS_PORT, configMFRC522_NSS_PIN)


//=============================
/*-------- Prototypes -------*/
//=============================
//-----------------------------
static void mfrc522PortInitialize(void);
//-----------------------------
static void mfrc522EXTIInitialize(void);
//-----------------------------
//=============================

//=============================
/*--------- Globals ---------*/
//=============================
SemaphoreHandle_t mfrc522Semaphore;
//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t mfrc522Initialize(void){

	if( spiInitialize(SPI1, SPI_CLK_DIV_128) ) return 1;

	mfrc522PortInitialize();

	mfrc522Semaphore = xSemaphoreCreateBinary();
	if(mfrc522Semaphore == NULL) return 2;
	xSemaphoreTake(mfrc522Semaphore, 0);

	configMFRC522_NSS_SET;

	return 0;
}
//-----------------------------
uint8_t mfrc522Read(uint8_t *buffer, uint8_t size, uint32_t pendTicks){


    return 0;
}
//-----------------------------
uint8_t mfrc522Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks){


    return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void mfrc522PortInitialize(void){

	gpioPortEnable(configMFRC522_NSS_PORT);
	gpioConfig(configMFRC522_NSS_PORT, configMFRC522_NSS_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(configMFRC522_IRQ_PORT);
	gpioConfig(configMFRC522_IRQ_PORT, configMFRC522_IRQ_PIN, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);

	/* Configures EXTI line 3 as external source */
	mfrc522EXTIInitialize();
}
//-----------------------------
static void mfrc522EXTIInitialize(void){

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
}
//-----------------------------
//=============================

//=============================
/*------- IRQ Handlers ------*/
//=============================
//-----------------------------
void EXTI3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void EXTI3_IRQHandler(void){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	EXTI->PR |= (1U << 3);

	xSemaphoreGiveFromISR(mfrc522Semaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
//-----------------------------
//=============================
