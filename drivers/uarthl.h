/*
 * @file uarthl.h
 * @brief Provides a simple bare metal uart driver for STM32F103 devices.
 *
 * The driver expects that the system clock is 72 MHz. In this case, we
 * consider that the clock for UART1 is also 72 MHz but for the other UARTs
 * we expect it to be 36 MHz. Maybe we'll consider other clocks in the
 * future (probably 8 MHz, which is the standard if the HSE fails).
 *
 * For now, the acceptable baud rates are given by uarthlBR_t. All UARTs
 * settings are hard-coded as follows:
 * 	- 1 start bit, 8 data bits, 1 stop bit.
 * 	- Clock phase and clock polarity are set to low.
 * 	- Least significant bit is sent first.
 * 	- NVIC priority is defined by UARTHL_CONFIG_UARTx_NVIC_PRIO.
 *
 * To save code space, UARTs must be enabled by defining:
 * 	- UARTHL_CONFIG_UARTx_ENABLED,
 * where x = {1, 2, 3, 4, 5}.
 *
 * It is also possible to enable integration with FreeRTOS. In this case,
 * any byte received or loaded into the transmitter register gives the
 * corresponding RX/TX semaphore.
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
/* Enable UARTs */
#define UARTHL_CONFIG_UART1_ENABLED 		/**< Enables UART1. */
//#define UARTHL_CONFIG_UART1_RTOS_EN 		/**< Enables FreeRTOS integration for UART1. */

#define UARTHL_CONFIG_UART2_ENABLED 		/**< Enables UART2. */

/* Priority for UART interrupt */
#define UARTHL_CONFIG_UART1_NVIC_PRIO		0x07 /**< NVIC UART1 priority. */
#define UARTHL_CONFIG_UART2_NVIC_PRIO		0x07 /**< NVIC UART2 priority. */
#define UARTHL_CONFIG_UART3_NVIC_PRIO		0x07 /**< NVIC UART3 priority. */
#define UARTHL_CONFIG_UART4_NVIC_PRIO		0x07 /**< NVIC UART4 priority. */
#define UARTHL_CONFIG_UART5_NVIC_PRIO		0x07 /**< NVIC UART5 priority. */

/* Error codes */
#define UARTHL_ERR_INVALID_UART				-0x01 /**< Invalid UART. */
#define UARTHL_ERR_INVALID_BAUD_RATE		-0x02 /**< Invalid baud rate. */
#define UARTHL_ERR_TX_NO_SPACE				-0x03 /**< TX queue not large enough. */
#define UARTHL_ERR_SEMPH_CREATE				-0x04 /**< Could no create semaphores.*/

#if defined(UARTHL_CONFIG_UART1_RTOS_EN) || \
	defined(UARTHL_CONFIG_UART2_RTOS_EN) || \
	defined(UARTHL_CONFIG_UART3_RTOS_EN) || \
	defined(UARTHL_CONFIG_UART4_RTOS_EN) || \
	defined(UARTHL_CONFIG_UART5_RTOS_EN)
#define UARTHL_CONFIG_FREE_RTOS_ENABLED
#endif
//===========================================================================

//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	UARTHL_BAUD_9600 = 0,
	UARTHL_BAUD_10400,
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
 * @param timeout Number of attempts to add an item to the TX queue.
 * @result If a positive number, it is the number of bytes successfully
 * 		   enqueued. If it is a negative number, it is an error code.
 */
int32_t uarthlWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
					uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the specified uart.
 *
 * The data is actually read from the RX FIFO queue.
 *
 * @param uart UART to read data.
 * @param buffer Pointer to buffer to hold the data read.
 * @param nbytes Number of bytes to read.
 * @param timeout Number of attempts to remove an item from the RX queue.
 * @result If a positive number, it is the number of bytes read. If it
 * 		   is a negative number, it is an error code.
 */
int32_t uarthlRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
/**
 * @brief Pends on the RX semaphore of the specified UART.
 *
 * Whenever a new byte is received through UART, the RX semaphore is given.
 * This function is only available if the FreeRTOS is enabled for the
 * specified UART.
 *
 * @param uart UART to pend.
 * @param timeout RTOS ticks to wait for the semaphore.
 */
int32_t uarthlPendRXSemaphore(USART_TypeDef *uart, uint32_t timeout);
#endif
//---------------------------------------------------------------------------
#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
/**
 * @brief Pends on the TX semaphore of the specified UART.
 *
 * Whenever a new byte is sent through UART, the TX semaphore is given.
 * This function is only available if the FreeRTOS is enabled for the
 * specified UART.
 *
 * @param uart UART to pend.
 * @param timeout RTOS ticks to wait for the semaphore.
 */
int32_t uarthlPendTXSemaphore(USART_TypeDef *uart, uint32_t timeout);
#endif
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_UARTHL_H_ */
