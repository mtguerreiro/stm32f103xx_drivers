/*
 * spi_bare.c
 *
 *  Created on: Apr 21, 2020
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "spi_bare.h"

/* Device */
#include "stm32f10x.h"

/* Drivers */
#include "gpio.h"
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
//-----------------------------
static void spibareHWInit(void);
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void spibareInitialize(void){

	spibareHWInit();
}
//-----------------------------
void spibareWrite(uint8_t data){

	uint8_t d;

	/* Waits until TX is available, then writes data to data register */
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = (uint16_t)data;

	/* Waits until there is data available in the data register, then clears it */
	while(!(SPI1->SR & SPI_SR_RXNE));
	d = (uint8_t)SPI1->DR;
}
//-----------------------------
uint8_t spibareRead(void){

	uint8_t data;

	/* Waits until TX is available, then writes data to data register */
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = (uint16_t)0xFF;

	/* Waits until there is data available in the data register, then reads it */
	while(!(SPI1->SR & SPI_SR_RXNE));
	data = (uint8_t)SPI1->DR;

	return data;
}
//-----------------------------

//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void spibareHWInit(void){

	GPIO_TypeDef *port = 0;
	SPI_TypeDef *spi = (SPI_TypeDef *)SPI1;
	uint32_t irqn = 0;
	uint16_t mosi = 0;
	uint16_t ck = 0;
	uint16_t miso = 0;
	uint16_t clockDiv = 36;

	/* Enables clock to SPI peripheral */
	RCC->APB2ENR |= (uint32_t)(1U << 12);

	/* Selects GPIO pins*/
	port = GPIOA;
	mosi = GPIO_P7;
	miso = GPIO_P6;
	ck = GPIO_P5;

	/* IRQ priority */
	irqn = SPI1_IRQn;

	/* Sets SPI pins */
	gpioPortEnable(port);
	gpioConfig(port, mosi | ck, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(port, miso, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);

	/* Sets SPI configs */
	spi->CR1 = (uint16_t)(clockDiv << 3U) | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // fpclk/32 -> 1125000 bps
	//spi->CR2 = SPI_CR2_RXNEIE;
	spi->CR1 |= SPI_CR1_SPE;
}
//-----------------------------
//=============================
