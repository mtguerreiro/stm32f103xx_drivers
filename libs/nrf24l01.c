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

/* Drivers */
#include "spi.h"
#include "gpio.h"
//=============================

#define configNRF24L01_CE_PORT		GPIOB
#define configNRF24L01_CE_PIN		GPIO_P0

#define configNRF24L01_CSN_PORT		GPIOB
#define configNRF24L01_CSN_PIN		GPIO_P1

#define configNRF24L01_CE_SET		gpioOutputSet(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN)
#define configNRF24L01_CE_RESET		gpioOutputReset(configNRF24L01_CE_PORT, configNRF24L01_CE_PIN)

#define configNRF24L01_CSN_SET		gpioOutputSet(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN)
#define configNRF24L01_CSN_RESET	gpioOutputReset(configNRF24L01_CSN_PORT, configNRF24L01_CSN_PIN)


//=============================
/*-------- Prototypes -------*/
//=============================
static void nrf24l01PortInitialize(void);
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer);
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer);
//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t nrf24l01Initialize(void){

	uint8_t buffer;

	if( spiInitialize(SPI_1, SPI_CLK_DIV_128) ) return 1;

	nrf24l01PortInitialize();

	configNRF24L01_CSN_SET;
	configNRF24L01_CE_RESET;

	/* Power up module */
	uint8_t data = 0x7A;
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

	/*Check if module has been configured */
	nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);

	if(buffer != data) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01ReadSR(uint8_t *status){

	uint8_t command;

	/* NOP operation command */
	command = 0xFF;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &command, 1, 0);
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI_1, status, 0) ) return 1;

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
	spiWrite(SPI_1, &command, 1, 0);
	k = size;
	while(k--){
		spiWrite(SPI_1, buffer++, 1, 0);
	}
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	configNRF24L01_CE_SET;
	/*
	 * While sending the data (command + data), we also received one
	 * byte for each byte transmitted. We just clear them.
	 */
	k = (uint8_t)(size + 1U);
	while(k--){
		if( spiRead(SPI_1, &dummy, 0) ) return 2;
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
	spiWrite(SPI_1, &command, 1, 0);
	k = size;
	while(k--){
		spiWrite(SPI_1, &dummy, 1, 0);
	}
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

//	configNRF24L01_CE_SET;
	/*
	 * While sending the data (command + data), we also received one
	 * byte for each byte transmitted. We clear the first one and
	 * save the rest.
	 */
	if( spiRead(SPI_1, &dummy, 0) ) return 2;
	k = size;
	while(k--){
		if( spiRead(SPI_1, buffer++, 0) ) return 3;
	}

//	configNRF24L01_CE_RESET;

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

	spiWrite(SPI_1, &command, 1, 0);
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI_1, &command, 0) ) return 1;

	return 0;
}
//-----------------------------
uint8_t nrf24l01FlushRX(void){

	uint8_t command;

	command = 0xE2;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &command, 1, 0);
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	if( spiRead(SPI_1, &command, 0) ) return 1;

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
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void nrf24l01PortInitialize(void){

	gpioPortEnable(configNRF24L01_CE_PORT);
	gpioMode(configNRF24L01_CE_PORT, GPIO_MODE_OUTPUT_10MHZ, configNRF24L01_CE_PIN);
	gpioConfig(configNRF24L01_CE_PORT, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL, configNRF24L01_CE_PIN);

	gpioPortEnable(configNRF24L01_CSN_PORT);
	gpioMode(configNRF24L01_CSN_PORT, GPIO_MODE_OUTPUT_10MHZ, configNRF24L01_CSN_PIN);
	gpioConfig(configNRF24L01_CSN_PORT, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL, configNRF24L01_CSN_PIN);
}
//-----------------------------
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &reg, 1, 0);
	spiWrite(SPI_1, &dummy, 1, 0);
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending two bytes (register + dummy), we also received two
	 * bytes. The first one is the status register. The second byte
	 * should be the register we are trying to read. So we save the first
	 * byte in the buffer, which should be the status register, and then
	 * overwrite it with the second byte, which should be the data we
	 * are looking for.
	 */
	if( spiRead(SPI_1, buffer, 0) ) return 1;
	if( spiRead(SPI_1, buffer, 0) ) return 2;

	return 0;
}
//-----------------------------
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;
	uint8_t k;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &reg, 1, 0);
	k = 5;
	while(k--){
		spiWrite(SPI_1, &dummy, 1, 0);
	}
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending five bytes (register + dummies), we also received five
	 * bytes. The first one is the status register. The 2~5 bytes
	 * should be the register we are trying to read. So we save the first
	 * byte in the buffer, which should be the status register, and then
	 * overwrite it with the 2~5 bytes, which should be the data we
	 * are looking for.
	 */
	if( spiRead(SPI_1, buffer, 0) ) return 1;
	k = 5;
	while(k--){
		if( spiRead(SPI_1, &buffer[k], 0) ) return 2;
	}

	return 0;
}
//-----------------------------
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;

	reg |= 0x20;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &reg, 1, 0);
	spiWrite(SPI_1, buffer, 1, 0);
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending two bytes (register + data), we also received two
	 * bytes. The first one is the status register. The second byte
	 * is undefined. We just clear them.
	 */
	if( spiRead(SPI_1, &dummy, 0) ) return 1;
	if( spiRead(SPI_1, &dummy, 0) ) return 2;

	return 0;
}
//-----------------------------
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t dummy = 0xFF;
	uint8_t k;

	reg |= 0x20;

	configNRF24L01_CSN_RESET;

	spiWrite(SPI_1, &reg, 1, 0);
	k = 5;
	while(k--){
		spiWrite(SPI_1, &buffer[k], 1, 0);
	}
	spiWaitTX(SPI_1, 0xFFFF);

	configNRF24L01_CSN_SET;

	/*
	 * While sending five bytes (register + data), we also received five
	 * bytes. The first one is the status register. The 2~5 bytes
	 * are undefined. We just clear them.
	 */
	k = 6;
	while(k--){
		if( spiRead(SPI_1, &dummy, 0) ) return 2;
	}

	return 0;
}
//-----------------------------
//=============================
