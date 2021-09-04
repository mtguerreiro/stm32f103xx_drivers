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
 * The maximum frequency for the SPI is 2.25 MHz, if using functions that
 * write/read through the queue. If the frequency is higher, the interrupt
 * mechanism will not have enough time to read incoming data.
 *
 * When using the bare write/read functions, the clock can be higher.
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


//#define SPIHL_CONFIG_SPI2_ENABLED 	/**< Enables SPI2. */

/* Priority for SPI interrupt */
#define SPIHL_CONFIG_SPI1_NVIC_PRIO		0x06 /**< NVIC SPI1 priority. */
#define SPIHL_CONFIG_SPI2_NVIC_PRIO		0x06 /**< NVIC SPI2 priority. */
#define SPIHL_CONFIG_SPI3_NVIC_PRIO		0x06 /**< NVIC SPI3 priority. */

/* Error codes */
#define SPIHL_ERR_INVALID_SPI				-0x01 /**< Invalid SPI. */
#define SPIHL_ERR_BUSY						-0x02 /**< SPI is busy. */
#define SPIHL_ERR_TX_TO						-0x03 /**< Timed-out during transmission. */
#define SPIHL_ERR_RX_TO						-0x04 /**< Timed-out during reception. */
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
 * @result 0 if SPI was initialized successfully, otherwise an error code.
 */
int32_t spihlInitialize(SPI_TypeDef *spi, spihlBR_t clockDiv,
						spihlPP_t clockPP);
//---------------------------------------------------------------------------
/**
 * @brief Sends data through the specified SPI.
 *
 * This function will return immediately, and the data will be sent from the
 * buffer through an interrupt mechanism.
 *
 * @param spi SPI to send data.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait in case the SPI is busy.
 * @result 0 if successful, otherwise and error code.
 */
int32_t spihlWrite(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sends data through the specified SPI.
 *
 * This function will block and return only after the data is sent.
 *
 * This function should not be used in conjunction with the normal write
 * function.
 *
 * @param spi SPI to send data.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait to send one byte.
 * @result 0 if successful, otherwise and error code.
 */
int32_t spihlWriteBare(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
					   uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the specified SPI.
 *
 * This function will return immediately, and the data will be saved to the
 * buffer through an interrupt mechanism.
 *
 * @param spi SPI to read data.
 * @param buffer Pointer to buffer to hold the data read.
 * @param nbytes Number of bytes to read.
 * @param timeout Timeout to wait in case the SPI is busy.
 * @result 0 if successful, otherwise and error code.
 */
int32_t spihlRead(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
				  uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data for the specified SPI.
 *
 * This function will block and return only after the data is read.
 *
 * This function should not be used in conjunction with the normal read
 * function.
 *
 * @param spi SPI to send data.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait to read one byte.
 * @result 0 if successful, otherwise and error code.
 */
int32_t spihlReadBare(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
				      uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_SPIHL_H_ */
