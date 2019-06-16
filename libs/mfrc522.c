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

//=============================
/*--------- Defines ---------*/
//=============================
/* SPI driver */
//-----------------------------
#define MFRC522_SPI					configMFRC522_SPI

#define MFRC522_SPI_CLK_DIV		 	configMFRC522_SPI_CLK_DIV
//-----------------------------

#define MFRC522_NSS_PORT			configMFRC522_NSS_PORT
#define MFRC522_NSS_PIN				configMFRC522_NSS_PIN

#define MFRC522_IRQ_PORT			configMFRC522_IRQ_PORT
#define MFRC522_IRQ_PIN				configMFRC522_IRQ_PIN

#define MFRC522_NSS_SET				gpioOutputSet(MFRC522_NSS_PORT, MFRC522_NSS_PIN)
#define MFRC522_NSS_RESET			gpioOutputReset(MFRC522_NSS_PORT, MFRC522_NSS_PIN)

/* MFRC522 registers */
//-----------------------------
//#define configMFRC522_STATUS1_REG	0x07
//#define configMFRC522_STATUS2_REG	0x08
//-----------------------------
//=============================

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

	if( spiInitialize(MFRC522_SPI, MFRC522_SPI_CLK_DIV) ) return 1;

	mfrc522PortInitialize();

	mfrc522Semaphore = xSemaphoreCreateBinary();
	if(mfrc522Semaphore == NULL) return 2;
	xSemaphoreTake(mfrc522Semaphore, 0);

	MFRC522_NSS_SET;

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
uint8_t mfrc522ReadRegister(uint8_t reg, uint8_t *buffer){

	uint8_t command[2];

	/* Formats register for SPI, read command always has the MSB set */
	command[0] = (uint8_t)(0x80 | ((reg & 0x3FU) << 1));
	command[1] = command[0];

	MFRC522_NSS_RESET;

	/* Sends the command twice to provide clock for slave's transmission */
	spiWrite(configMFRC522_SPI, command, 2);
	spiWaitTX(configMFRC522_SPI, 0xFFFF);

	MFRC522_NSS_SET;

	/* Received two bytes, discards the first one and saves the second */
	if( spiRead(configMFRC522_SPI, buffer, 0) ) return 2;
	if( spiRead(configMFRC522_SPI, buffer, 0) ) return 2;

	return 0;
}
//-----------------------------
void mfrc522WriteRegister(uint8_t reg, uint8_t data){

	uint8_t command[2];

	/* Formats register for SPI, write command always has the MSB cleared */
	command[0] = (uint8_t)((reg & 0x3FU) << 1);
	command[1] = data;

	MFRC522_NSS_RESET;

	/* Sends the command and waits until transmission is over */
	spiWrite(configMFRC522_SPI, command, 2);
	spiWaitTX(configMFRC522_SPI, 0xFFFF);

	MFRC522_NSS_SET;

	/* Discards the received bytes */
	spiRead(configMFRC522_SPI, command, 0);
	spiRead(configMFRC522_SPI, command, 0);
}
//-----------------------------
void mfrc522SoftReset(void){

	uint32_t delay;

	/* Sends reset command */
	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_SOFT_RESET);

	/* Waits for reset */
	delay = 0x4FFFB;
	while(delay--);
}
//-----------------------------
uint8_t mfrc522SelfTest(uint8_t* buffer, uint32_t timeout){

	uint16_t k;
	//uint8_t buffer[64];
	uint8_t fifoSize;

	k = 64;
	while(k--){
		buffer[k] = 0x00;
	}

	/* Step 1: perform a soft-reset */
	mfrc522SoftReset();

	/*
	 * Step 2: clear the internal buffer
	 * Clears the internal buffer by writing 25 bytes of 0x00 to it.
	 * Note that the bytes must first be written to the FIFO buffer and then
	 * transferred to the internal buffer with the "Mem" command
	 */
	mfrc522FIFOFlush();
	mfrc522FIFOWrite(buffer, 25);
	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_MEM);

	/* Step 3: Enable the self test */
	mfrc522WriteRegister(MFRC522_REG_AUTO_TEST, 0x09);

	/* Step 4: Write 0x00 to the FIFO buffer */
	mfrc522FIFOWrite(buffer, 1);

	/* Step 5: Enable CALC_CRC command */
	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_CALC_CRC);

	/* Step 6: wait until FIFO buffer contains 64 bytes */
	fifoSize = 0;
	while( (fifoSize != 64) && (timeout > 0) ){
		k = 0xFFFF;
		while(k--);
		mfrc522ReadRegister(MFRC522_REG_FIFO_LEVEL, &fifoSize);
		timeout--;
	}

	if(!timeout) return 1;

	/* Reads data from FIFO */
	mfrc522FIFORead(buffer, 64);

	return 0;
}
//-----------------------------
void mfrc522FIFOFlush(void){

	mfrc522WriteRegister(MFRC522_REG_FIFO_LEVEL, 0x80);
}
//-----------------------------
void mfrc522FIFOWrite(uint8_t* buffer, uint8_t nbytes){

	uint8_t *ptr;

	ptr = buffer;
	while(nbytes--){
		mfrc522WriteRegister(MFRC522_REG_FIFO_DATA, *ptr++);
	}
}
//-----------------------------
void mfrc522FIFORead(uint8_t* buffer, uint8_t nbytes){

	uint8_t *ptr;

	ptr = buffer;
	while(nbytes--){
		mfrc522ReadRegister(MFRC522_REG_FIFO_DATA, ptr++);
	}
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void mfrc522PortInitialize(void){

	gpioPortEnable(MFRC522_NSS_PORT);
	gpioConfig(MFRC522_NSS_PORT, MFRC522_NSS_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(MFRC522_IRQ_PORT);
	gpioConfig(MFRC522_IRQ_PORT, MFRC522_IRQ_PIN, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);

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
