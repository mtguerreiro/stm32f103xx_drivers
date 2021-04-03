/*
 * @file uarthl.h
 * @brief Provides a simple bare metal uart driver for STM32F103 devices.
 *
 * The driver expects that the system clock is 72 MHz. In this case, we
 * consider that the clock for UART1 is also 72 MHz but for the other UARTs
 *  we expect it to be 36 MHz. Maybe we'll consider other clocks in the
 *  future (probably 8 MHz, which is the standard if the HSE fails).
 *
 * For now, the acceptable baud rates are given by uarthlBR_t. All UARTs
 * settings are hard-coded as follows:
 * 	- 1 start bit, 8 data bits, 1 stop bit.
 * 	- Clock phase and clock polarity are set to low.
 * 	- Least significant bit is sent first.
 * 	- NVIC priority is the same for all UARTs and defined by
 * 	  UARTHL_CONFIG_NVIC_PRIO.
 *
 * To save code space, UARTs must be enabled by defining:
 * 	- UARTHL_CONFIG_UARTx_ENABLED,
 * where x = {1, 2, 3, 4, 5}.
 *
 *  Created on: March 14, 2021
 *      Author: Marco
 */

#ifndef DRIVERS_UARTHL_H_
#define DRIVERS_UARTHL_H_


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
#define UARTHL_CONFIG_UART1_ENABLED /**< Enables UART1. */
#define UARTHL_CONFIG_UART2_ENABLED /**< Enables UART2. */

/* Error codes */
#define UARTHL_ERR_INVALID_UART				-0x01 /**< Invalid UART. */
#define UARTHL_ERR_INVALID_BAUD_RATE		-0x02 /**< Invalid baud rate. */
#define UARTHL_ERR_TX_NO_SPACE				-0x03 /**< TX queue not large enough. */
//===========================================================================

//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	UARTHL_BAUD_9600 = 0,
	UARTHL_BAUD_115200,
	UARTHL_BAUD_460800
}uarthlBR_t;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes the specified uart.
 *
 * @param uart UART to be initialized.
 * @param baud Baud rate for transmission/reception.
 * @param rxBuffer Buffer to hold data received.
 * @param rxBufferSize Size of buffer to hold received data.
 * @param txBuffer Buffer to hold data to be transmitted.
 * @param txBufferSize Size of buffer to hold data to be transmitted.
 * @result 0 if UART was initialized successfully, otherwise an error code.
 */
int32_t uarthlInitialize(USART_TypeDef *uart, uarthlBR_t baud, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
/**
 * @brief Sends data through the specified uart.
 *
 * The data is actually written to the TX FIFO queue, and sent through UART
 * by an interrupt mechanism.
 *
 * @param uart UART to send data.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Specifies a timeout for writing. If FreeRTOS is used,
 * 		  this is the number in ticks to wait. If FreeRTOS is not used, this
 * 		  is the number of attempts to add an item to the TX queue.
 * @result 0 if data was enqueued successfully. If a positive number, it is
 * 		   the number of bytes not sent. If it is a negative number, it is an
 * 		   error code.
 */
int32_t uarthlWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
					uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the specified uart.
 *
 * The data is actually read from the RX FIFO queue.
 *
 * @param uart UART to send data.
 * @param buffer Pointer to buffer to hold the data read.
 * @param nbytes Number of bytes to read.
 * @param timeout Specifies a timeout for reading. If FreeRTOS is used, this
 * 		  is the number in ticks to wait. If FreeRTOS is not used, this is
 * 		  the number of attempts to remove an item from the RX queue.
 * @result 0 if all data was read. If a positive number, it is the number of
 * 		   bytes not read. If it is a negative number, it is an error code.
 */
int32_t uarthlRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_UARTHL_H_ */
