/**
 * @file spi.c
 * @brief Source file for SPI driver.
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "spi.h"

/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"

#include "gpio.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t spiInitialize(SPI_TypeDef *spi, spiBR_t clockDiv, \
		spiPP_t clockPP, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret = 0;

	ret = spihlInitialize(spi, clockDiv, clockPP, rxBuffer, rxBufferSize, txBuffer, txBufferSize);

	return ret;
}
//---------------------------------------------------------------------------
int32_t spiWrite(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
					uint32_t timeout){

	int32_t bytesWritten;
	int32_t ret;
	uint8_t *p;

	p = buffer;
	bytesWritten = 0;
	while( bytesWritten < nbytes ){
		/* Tries to get the TX semaphore */
		if( spihlPendTXSemaphore(spi, timeout) != 0 ) break;
		/*
		 * If we got the semaphore, we should be able to write at least one
		 * byte, so no need to specify a number of attempts higher than 1.
		 */
		ret = spihlWrite(spi, p, (int32_t)(nbytes - bytesWritten), 1);

		bytesWritten += ret;
		p += (uint32_t)ret;
	}

	return bytesWritten;
}
//---------------------------------------------------------------------------
int32_t spiRead(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
				   uint32_t timeout){

	int32_t bytesRead;
	int32_t ret;
	uint8_t *p;

	p = buffer;
	bytesRead = 0;
	while( bytesRead < nbytes ){
		/* Tries to get the RX semaphore */
		if( spihlPendRXSemaphore(spi, timeout) != 0 ) break;

		/*
		 * If we got the semaphore, we should be able to read at least one
		 * byte, so no need to specify a number of attempts higher than 1.
		 */
		gpioOutputSet(GPIOA, GPIO_P0);
		ret = spihlRead(spi, p, (uint16_t)(nbytes - bytesRead), 1);
		gpioOutputReset(GPIOA, GPIO_P0);

		bytesRead += ret;
		p += (uint32_t)ret;
	}

	return bytesRead;
}
//---------------------------------------------------------------------------
//===========================================================================
