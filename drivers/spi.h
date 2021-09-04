/**
 * @file spi.h
 * @brief Simple SPI driver for STM32F103x devices.
 *
 */

#ifndef SPI_H_
#define SPI_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
#include "spihl.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
typedef enum{
	SPI_POLL_PHAF = SPIHL_POLL_PHAF,
	SPI_POLL_PHAS = SPIHL_POLL_PHAS,
	SPI_POLH_PHAF = SPIHL_POLH_PHAF,
	SPI_POLH_PHAS = SPIHL_POLH_PHAS,
}spiPP_t;

typedef enum{
	SPI_BR_CLK_DIV_2 = SPIHL_BR_CLK_DIV_2,
	SPI_BR_CLK_DIV_4 = SPIHL_BR_CLK_DIV_4,
	SPI_BR_CLK_DIV_8 = SPIHL_BR_CLK_DIV_8,
	SPI_BR_CLK_DIV_16 = SPIHL_BR_CLK_DIV_16,
	SPI_BR_CLK_DIV_32 = SPIHL_BR_CLK_DIV_32,
	SPI_BR_CLK_DIV_64 = SPIHL_BR_CLK_DIV_64,
	SPI_BR_CLK_DIV_128 = SPIHL_BR_CLK_DIV_128,
	SPI_BR_CLK_DIV_256 = SPIHL_BR_CLK_DIV_256,
}spiBR_t;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes the specified SPI.
 *
 * @param SPI SPI to be initialized.
 * @param clockDiv Clock prescaler.
 * @param clockPP Clock polarity and phase.
 * @param rxBuffer Buffer to hold data received.
 * @param rxBufferSize Size of buffer to hold received data.
 * @param txBuffer Buffer to hold data to be transmitted.
 * @param txBufferSize Size of buffer to hold data to be transmitted.
 * @result 0 if SPI was initialized successfully, otherwise an error code.
 */
int32_t spiInitialize(SPI_TypeDef *spi, spiBR_t clockDiv, \
		spiPP_t clockPP, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
/**
 * @brief Sends data through the specified SPI.
 *
 * The data is actually written to the TX FIFO queue, and sent through SPI
 * by an interrupt mechanism.
 *
 * @param spi SPI to send data.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Number of ticks to wait to add an item to the TX queue.
 * @result If a positive number, it is the number of bytes successfully
 * 		   enqueued. If it is a negative number, it is an error code.
 */
int32_t spiWrite(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
					uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the specified SPI.
 *
 * The data is actually read from the RX FIFO queue.
 *
 * @param spi SPI to read data.
 * @param buffer Pointer to buffer to hold the data read.
 * @param nbytes Number of bytes to read.
 * @param timeout Number of ticks to wait to remove an item from the RX queue.
 * @result If a positive number, it is the number of bytes read. If it
 * 		   is a negative number, it is an error code.
 */
int32_t spiRead(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* SPI_H_ */
