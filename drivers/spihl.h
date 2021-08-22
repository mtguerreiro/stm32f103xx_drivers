/*
 * @file spihl.h
 * @brief Provides an SPI driver for STM32F103 devices.
 *
 * The driver expects that the system clock is 72 MHz. In this case, we
 * consider that the clock for SPI1 is also 72 MHz but for the other SPIs we
 * expect it to be 36 MHz. Maybe we'll consider other clocks in the future
 * (probably 8 MHz, which is the standard if the HSE fails).
 *
 * The SPI is always configured as master.
 *
 *  Created on: 8 de mai de 2021
 *      Author: marco
 */

#ifndef DRIVERS_SPIHL_H_
#define DRIVERS_SPIHL_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
/* Enable SPIs */
#define SPIHL_CONFIG_SPI1_ENABLED 		/**< Enables SPI1. */
#define SPIHL_CONFIG_SPI1_RTOS_EN 		/**< Enables FreeRTOS integration for SPI1. */

//#define SPIHL_CONFIG_SPI2_ENABLED 	/**< Enables SPI2. */

/* Priority for SPI interrupt */
#define SPIHL_CONFIG_SPI1_NVIC_PRIO		0x06 /**< NVIC SPI1 priority. */
#define SPIHL_CONFIG_SPI2_NVIC_PRIO		0x06 /**< NVIC SPI2 priority. */
#define SPIHL_CONFIG_SPI3_NVIC_PRIO		0x06 /**< NVIC SPI3 priority. */

/* Error codes */
#define SPIHL_ERR_INVALID_SPI				-0x01 /**< Invalid SPI. */
#define SPIHL_ERR_TX_NO_SPACE				-0x02 /**< TX queue not large enough. */
#define SPIHL_ERR_SEMPH_CREATE				-0x03 /**< Could no create semaphores.*/

#if defined(SPIHL_CONFIG_SPI1_RTOS_EN) || \
	defined(SPIHL_CONFIG_SPI2_RTOS_EN) || \
	defined(SPIHL_CONFIG_SPI3_RTOS_EN)
#define SPIHL_CONFIG_FREE_RTOS_ENABLED
#endif
//===========================================================================

//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	SPIHL_POLL_PHAF = 0,
	SPIHL_POLL_PHAS,
	SPIHL_POLH_PHAF,
	SPIHL_POLH_PHAS,

}spihlPP_t;

typedef enum{
	SPIHL_BR_CLK_DIV_2 = 0,
	SPIHL_BR_CLK_DIV_4,
	SPIHL_BR_CLK_DIV_8,
	SPIHL_BR_CLK_DIV_16,
	SPIHL_BR_CLK_DIV_32,
	SPIHL_BR_CLK_DIV_64,
	SPIHL_BR_CLK_DIV_128,
	SPIHL_BR_CLK_DIV_256,
}spihlBR_t;
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
int32_t spihlInitialize(SPI_TypeDef *spi, spihlBR_t clockDiv, \
		spihlPP_t clockPP, \
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
 * @param timeout Number of attempts to add an item to the TX queue.
 * @result If a positive number, it is the number of bytes successfully
 * 		   enqueued. If it is a negative number, it is an error code.
 */
int32_t spihlWrite(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
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
 * @param timeout Number of attempts to remove an item from the RX queue.
 * @result If a positive number, it is the number of bytes read. If it
 * 		   is a negative number, it is an error code.
 */
int32_t spihlRead(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
/**
 * @brief Pends on the RX semaphore of the specified SPI.
 *
 * Whenever a new byte is received through SPI, the RX semaphore is given.
 * This function is only available if the FreeRTOS is enabled for the
 * specified SPI.
 *
 * @param spi SPI to pend.
 * @param timeout RTOS ticks to wait for the semaphore.
 */
int32_t spihlPendRXSemaphore(SPI_TypeDef *spi, uint32_t timeout);
#endif
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
/**
 * @brief Pends on the TX semaphore of the specified SPI.
 *
 * Whenever a new byte is sent through SPI, the TX semaphore is given.
 * This function is only available if the FreeRTOS is enabled for the
 * specified spi.
 *
 * @param spi SPI to pend.
 * @param timeout RTOS ticks to wait for the semaphore.
 */
int32_t spihlPendTXSemaphore(SPI_TypeDef *spi, uint32_t timeout);
#endif
//---------------------------------------------------------------------------
int32_t spihlFlushRXBuffer();
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_SPIHL_H_ */
