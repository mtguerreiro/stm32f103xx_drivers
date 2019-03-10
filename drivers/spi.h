/*
 * spi.h
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 *
 *	Melhorias
 *		- Baudrate configur�vel
 *		- Melhorar uso das filas (criar somente a quantidade
 *		necess�ria)
 *		- "Yield from interrupt" individual
 */

#ifndef SPI_H_
#define SPI_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configSPI1_ENABLED			1
#define configSPI2_ENABLED			0
#define configSPI3_ENABLED			0

/* RX and TX queue size */
#define configSPI1_RXQ_SIZE			25
#define configSPI1_TXQ_SIZE			25

#define configSPI2_RXQ_SIZE			25
#define configSPI2_TXQ_SIZE			25

#define configSPI3_RXQ_SIZE			25
#define configSPI3_TXQ_SIZE			25

#define configUART_INTERRUPT_YIELD	1
//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t spiInitialize(uint32_t spi, uint16_t clockDiv);
uint8_t spiWrite(uint32_t spi, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles);
uint8_t spiRead(uint32_t spi, uint8_t *buffer, uint32_t waitcycles);
uint8_t spiWaitTX(uint32_t spi, uint32_t waitcycles);
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef enum{
	SPI_1 = SPI1_BASE,
	SPI_2 = SPI2_BASE,
	SPI_3 = SPI3_BASE
}spiN_t;
//-----------------------------
typedef enum{
	SPI_CLK_DIV_2 = 0,
	SPI_CLK_DIV_4,
	SPI_CLK_DIV_8,
	SPI_CLK_DIV_16,
	SPI_CLK_DIV_32,
	SPI_CLK_DIV_64,
	SPI_CLK_DIV_128,
	SPI_CLK_DIV_256
}spiClockDiv_t;
//-----------------------------
//=============================

#endif /* SPI_H_ */