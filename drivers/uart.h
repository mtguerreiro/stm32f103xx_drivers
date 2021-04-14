/*
 * uart.h
 *
 *	UART driver for STM32F103xx devices.
 *
 */

#ifndef DRIVERS_UART_H_
#define DRIVERS_UART_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
#include "uarthl.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
typedef enum{
	UART_BAUD_9600 = UARTHL_BAUD_9600,
	UART_BAUD_115200 = UARTHL_BAUD_115200,
	UART_BAUD_460800 = UARTHL_BAUD_460800
}uartBR_t;
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
int32_t uartInitialize(USART_TypeDef *uart, uartBR_t baud, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
int32_t uartWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
					uint32_t timeout);
//---------------------------------------------------------------------------
int32_t uartRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
				   uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================


#endif /* DRIVERS_UART_H_ */
