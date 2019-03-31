/*
 * nrf24l01.c
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "nrf24l01.h"

#include <stdint.h>

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"

/* Drivers */
#include "spi.h"
#include "gpio.h"
//=============================

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


//=============================
/*-------- Prototypes -------*/
//=============================
static void nrf24l01PortInitialize(void);
static void nrf24l01EXTIInitialize(void);
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer);
//=============================

//=============================
/*--------- Globals ---------*/
//=============================
SemaphoreHandle_t nrf24l01Semaphore;
//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t nrf24l01Initialize(void){

	uint8_t buffer;
	uint8_t data;

	if( spiInitialize(SPI1, SPI_CLK_DIV_128) ) return 1;

	nrf24l01PortInitialize();

	nrf24l01Semaphore = xSemaphoreCreateBinary();
	if(nrf24l01Semaphore == NULL) return 2;
	xSemaphoreTake(nrf24l01Semaphore, 0);

	configNRF24L01_CSN_SET;
	configNRF24L01_CE_RESET;

	/* Power up module
	 * - Bit 6: Mask data received interrupt
	 * 		If 0, RX_DR will be reflected on IRQn pin. If 0, RX_DR is not
	 * 		reflected on IRQn pin.
	 *
	 * - Bit 5: Mask data sent interrupt
	 * 		If 0, TX_DS will be reflected on IRQn pin. If 0, TX_DS is not
	 * 		reflected on IRQn pin.
	 *
	 * 	- Bit 4: Mask maximum retries interrupt
	 * 		 If 0, MAX_RT will be reflected on IRQn pin. If 0, MAX_RT is not
	 * 		reflected on IRQn pin.
	 *
	 * 	- Bit 3: Enables CRC
	 * 		If 1, CRC is enabled. This will also be forced to 1 if EN_AA is high.
	 *
	 * 	- Bit 2: CRC encoding scheme
	 * 		If 0, 1 byte is used for CRC. If 1, two bytes are used.
	 *
	 * 	- Bit 1: Power up
	 * 		If 1, device is powered up. If 0, device is off.
	 *
	 * 	- Bit 0: Primary receiver
	 * 		If 1, device is configured as primary receiver. If 0, device is
	 * 		configured as primary transmitter.
	 * */
	data = 0x0A;
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

	/* Check if module has been correctly configured */
	nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);

	if(buffer != data) return 3;

	return 0;
}
//-----------------------------
uint8_t nrf24l01SetTX(uint8_t *address, uint8_t plSize){

	uint8_t buffer[5];
	uint8_t *addrBuffer;
	uint8_t data;
	//uint8_t rxtxAddress[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
	uint8_t k;

	/* Enables auto-ack on data pipe 0 */
	data = 0x01;
	nrf24l01WriteRegister(NRF24L01_REG_EN_AA, &data);
	nrf24l01ReadRegister(NRF24L01_REG_EN_AA, buffer);
	if( *buffer != data ){
		return 1;
	}

	/* Enables RX address on data pipe 0 */
	data = 0x01;
	nrf24l01WriteRegister(NRF24L01_REG_EN_RXADDR, &data);
	nrf24l01ReadRegister(NRF24L01_REG_EN_RXADDR, buffer);
	if( *buffer != data ){
		return 2;
	}

	/* Waits 2000 us for retransmission, tries up to 5 times */
	data = 0x65;
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_RETR, &data);
	nrf24l01ReadRegister(NRF24L01_REG_SETUP_RETR, buffer);
	if( *buffer != data ){
		return 3;
	}

	/* Sets RF channel (frequency) */
	data = 0x07;
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_RETR, &data);
	nrf24l01ReadRegister(NRF24L01_REG_SETUP_RETR, buffer);
	if( *buffer != data ){
		return 4;
	}

	/* Sets RX address */
	addrBuffer = address;
	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P0, addrBuffer);
	nrf24l01ReadRegister(NRF24L01_REG_RX_ADDR_P0, buffer);
	addrBuffer = address;
	k = 5;
	while(k--){
		if(buffer[k] != addrBuffer[k]){
			return 5;
		}
	}

	/* Sets TX address */
	addrBuffer = address;
	nrf24l01WriteRegister(NRF24L01_REG_TX_ADDR, addrBuffer);
	nrf24l01ReadRegister(NRF24L01_REG_TX_ADDR, buffer);
	addrBuffer = address;
	k = 5;
	while(k--){
		if(buffer[k] != addrBuffer[k]){
			return 6;
		}
	}

	/* Sets number of bytes in RX payload data pipe 0 to 5 */
	data = plSize;
	if(data > 32) data = 32;
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P0, &data);
	nrf24l01ReadRegister(NRF24L01_REG_RX_PW_P0, buffer);
	if(data != *buffer){
		return 7;
	}

	nrf24l01FlushTX();
	nrf24l01FlushRX();

	nrf24l01StatusClear();

	return 0;
}
//-----------------------------
uint8_t nrf24l01ReadSR(uint8_t *status){

	uint8_t command;

	/* NOP operation command */
	command = 0xFF;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &command, 1);
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI1, status, 0) ) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks){

	uint8_t nrfStatus;
	uint8_t retries;
	nrf24l01TransmitPayload(buffer, size);

	if( nrf24l01Pend(pendTicks) ){
		/*
		 * Pend should always return 0, because if transmission fails, it
		 * will be due to maximum retries being reached (assuming enough
		 * ticks have been provided). Thus, we should never get here. If
		 * we do, we'll clear everything related to transmission.
		 */
		nrf24l01FlushTX();
		nrf24l01StatusClearMaxRT();
		nrf24l01StatusClearTXDS();
		return 1;
	}

	/*
	 * Now, we check the status. If maximum number of retries were reached,
	 * we'll flush the TX FIFO and clear the corresponding flag. Else, data
	 * was sent and we clear the TXDS flag only.
	 * We'll try to read the status a couple of times. If we do not succeed,
	 * we'll just clear status and FIFO to be sure, although this is definitely
	 * not the best solution.
	 */
	retries = 5;
	while(retries){
		if( !nrf24l01ReadSR(&nrfStatus) ) break;
		retries--;
	}
	if(retries == 0){
		nrf24l01FlushTX();
		nrf24l01StatusClearMaxRT();
		nrf24l01StatusClearTXDS();
		return 2;
	}
	if( nrfStatus & (1 << 4U) ){
		/* Maximum number of retries exceeded */
		nrf24l01FlushTX();
		nrf24l01StatusClearMaxRT();
		return 3;
	}

	/* Data sent successfully */
	nrf24l01StatusClearTXDS();

	return 0;
}
//-----------------------------
uint8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer){

	if( (reg == NRF24L01_REG_RX_ADDR_P0) || (reg == NRF24L01_REG_RX_ADDR_P1) || (reg == NRF24L01_REG_TX_ADDR) ){
		if( nrf24l01ReadRegisterMultiple(reg, buffer) ) return 1;
	}
	else{
		if( nrf24l01ReadRegisterSingle(reg, buffer) ) return 1;
	}

	return 0;
}
//-----------------------------
uint8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer){

	if( (reg == NRF24L01_REG_RX_ADDR_P0) || (reg == NRF24L01_REG_RX_ADDR_P1) || (reg == NRF24L01_REG_TX_ADDR) ){
		if( nrf24l01WriteRegisterMultiple(reg, buffer) ) return 1;
	}
	else{
		if( nrf24l01WriteRegisterSingle(reg, buffer) ) return 1;
	}

	return 0;
}
//-----------------------------
uint8_t nrf24l01TransmitPayload(uint8_t *buffer, uint8_t size){

	uint8_t dummy = 0xFF;
	uint8_t command;
	uint8_t k;

	configNRF24L01_CSN_RESET;

	command = 0xA0;
	spiWrite(SPI1, &command, 1);
	k = size;
	while(k--){
		spiWrite(SPI1, buffer++, 1);
	}
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	configNRF24L01_CE_SET;
	/*
	 * While sending the data (command + data), we also received one
	 * byte for each byte transmitted. We just clear them.
	 */
	k = (uint8_t)(size + 1U);
	while(k--){
		if( spiRead(SPI1, &dummy, 0) ) return 2;
	}

	configNRF24L01_CE_RESET;

	return 0;
}
//-----------------------------
uint8_t nrf24l01ReceivePayload(uint8_t *buffer, uint8_t size){

	uint8_t dummy = 0xFF;
	uint8_t command;
	uint8_t k;

	configNRF24L01_CSN_RESET;

	command = 0x61;
	spiWrite(SPI1, &command, 1);
	k = size;
	while(k--){
		spiWrite(SPI1, &dummy, 1);
	}
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

//	configNRF24L01_CE_SET;
	/*
	 * While sending the data (command + data), we also received one
	 * byte for each byte transmitted. We clear the first one and
	 * save the rest.
	 */
	if( spiRead(SPI1, &dummy, 0) ) return 2;
	k = size;
	while(k--){
		if( spiRead(SPI1, buffer++, 0) ) return 3;
	}

//	configNRF24L01_CE_RESET;

	return 0;
}
//-----------------------------
uint8_t nrf24l01StatusClear(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 4U) | (0x01U << 5U) | (0x01U << 6U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01StatusClearMaxRT(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 4U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01StatusClearTXDS(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 5U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01StatusClearRXDR(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 6U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01FlushTX(void){

	uint8_t command;

	command = 0xE1;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &command, 1);
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI1, &command, 0) ) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01FlushRX(void){

	uint8_t command;

	command = 0xE2;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &command, 1);
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI1, &command, 0) ) return 1;

	return 0;
}
//-----------------------------
void nr24l01SetCE(void){

	configNRF24L01_CE_SET;
}
//-----------------------------
void nr24l01ResetCE(void){

	configNRF24L01_CE_RESET;
}
//-----------------------------
uint8_t nrf24l01Pend(uint32_t ticks){

	if(xSemaphoreTake(nrf24l01Semaphore, ticks) != pdPASS) return 1;

	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void nrf24l01PortInitialize(void){

	gpioPortEnable(configNRF24L01_CE_PORT);
	gpioConfig(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(configNRF24L01_CSN_PORT);
	gpioConfig(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioPortEnable(configNRF24L01_IRQ_PORT);
	gpioConfig(configNRF24L01_IRQ_PORT, configNRF24L01_IRQ_PIN, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);

	/* Configures EXTI line 3 as external source */
	nrf24l01EXTIInitialize();
}
//-----------------------------
static void nrf24l01EXTIInitialize(void){

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
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &reg, 1);
	spiWrite(SPI1, &dummy, 1);
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending two bytes (register + dummy), we also received two
	 * bytes. The first one is the status register. The second byte
	 * should be the register we are trying to read. So we save the first
	 * byte in the buffer, which should be the status register, and then
	 * overwrite it with the second byte, which should be the data we
	 * are looking for.
	 */
	if( spiRead(SPI1, buffer, 0) ) return 1;
	if( spiRead(SPI1, buffer, 0) ) return 2;

	return 0;
}
//-----------------------------
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;
	uint8_t k;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &reg, 1);
	k = 5;
	while(k--){
		spiWrite(SPI1, &dummy, 1);
	}
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending five bytes (register + dummies), we also received five
	 * bytes. The first one is the status register. The 2~5 bytes
	 * should be the register we are trying to read. So we save the first
	 * byte in the buffer, which should be the status register, and then
	 * overwrite it with the 2~5 bytes, which should be the data we
	 * are looking for.
	 */
	if( spiRead(SPI1, buffer, 0) ) return 1;
	k = 5;
	while(k--){
		if( spiRead(SPI1, &buffer[k], 0) ) return 2;
	}

	return 0;
}
//-----------------------------
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;

	reg |= 0x20;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &reg, 1);
	spiWrite(SPI1, buffer, 1);
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending two bytes (register + data), we also received two
	 * bytes. The first one is the status register. The second byte
	 * is undefined. We just clear them.
	 */
	if( spiRead(SPI1, &dummy, 0) ) return 1;
	if( spiRead(SPI1, &dummy, 0) ) return 2;

	return 0;
}
//-----------------------------
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;
	uint8_t k;

	reg |= 0x20;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI1, &reg, 1);
	k = 5;
	while(k--){
		spiWrite(SPI1, &buffer[k], 1);
	}
	spiWaitTX(SPI1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending five bytes (register + data), we also received five
	 * bytes. The first one is the status register. The 2~5 bytes
	 * are undefined. We just clear them.
	 */
	k = 6;
	while(k--){
		if( spiRead(SPI1, &dummy, 0) ) return 2;
	}

	return 0;
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

	xSemaphoreGiveFromISR(nrf24l01Semaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
//-----------------------------
//=============================
