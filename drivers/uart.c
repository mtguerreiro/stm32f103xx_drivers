/*
 * uart.c
 *
 *  Created on: Apr 29, 2018
 *      Author: Marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "uart.h"

/* Device */
#include "gpio.h"

/* Kernel */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
//===========================================================================


//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t uartInitialize(USART_TypeDef *uart, uartBR_t baud, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret = 0;

	ret = uarthlInitialize(uart, baud, rxBuffer, rxBufferSize, txBuffer, txBufferSize);

	return ret;
}
//---------------------------------------------------------------------------
int32_t uartWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
					uint32_t timeout){

	int32_t bytesWritten;
	int32_t ret;
	uint8_t *p;

	p = buffer;
	bytesWritten = 0;
	while( bytesWritten < nbytes ){
		gpioOutputSet(GPIOA, GPIO_P7);
		/* Tries to get the TX semaphore */
		if( uarthlPendTXSemaphore(uart, timeout) != 0 ) break;
		gpioOutputReset(GPIOA, GPIO_P7);
		/*
		 * If we got the semaphore, we should be able to write at least one
		 * byte, so no need to specify a number of attempts higher than 1.
		 */
		ret = uarthlWrite(uart, p, (uint16_t)(nbytes - bytesWritten), 1);

		bytesWritten += ret;
		p += (uint32_t)ret;
	}

	return bytesWritten;
}
//---------------------------------------------------------------------------
int32_t uartRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
				   uint32_t timeout){

	int32_t bytesRead;
	int32_t ret;
	uint8_t *p;

	p = buffer;
	bytesRead = 0;
	while( bytesRead < nbytes ){
		/* Tries to get the RX semaphore */
		if( uarthlPendRXSemaphore(uart, timeout) != 0 ) break;

		/*
		 * If we got the semaphore, we should be able to read at least one
		 * byte, so no need to specify a number of attempts higher than 1.
		 */
		ret = uarthlRead(uart, p, (uint16_t)(nbytes - bytesRead), 1);

		bytesRead += ret;
		p += (uint32_t)ret;
	}

	return bytesRead;
}
//---------------------------------------------------------------------------
//uint8_t uartWriteString(USART_TypeDef *uart, void *str){
//
//	uint8_t qidx;
//	uint16_t strSize;
//	uint32_t queueSize;
//	uint8_t *buffer = (uint8_t *)str;
//
//	qidx = uartQueueIndex(uart);
//
//	strSize = 0;
//	while(*buffer++) strSize++;
//
//	queueSize = (uint32_t)uxQueueSpacesAvailable(uartTXQueue[qidx]);
//	if(strSize > queueSize) return 1;
//
//	buffer -= (strSize + 1);
//
//	while(*buffer){
//		if( xQueueSendToBack(uartTXQueue[qidx], buffer++, 0) != pdTRUE ) return 2;
//	}
//
//	uartTriggerTransmission(uart);
//
//	return 0;
//}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//===========================================================================
