/*
 * uarthl.c
 *
 *  Created on: 14 de mar de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "uarthl.h"

/* Libs */
#include "cqueue.h"

/* Drivers */
#include "gpio.h"
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes hardware for the specified UART.
 *
 * GPIOs are selected as UART TX/RX, registers are set and interruptions are
 * enabled.
 *
 * @param uart UART to be initialized.
 * @param baud Baud rate for transmission/reception.
 * @result 0 if hardware was initialized successfully, otherwise an error
 *         code.
 */
int32_t uarthlInitializeHW(USART_TypeDef *uart, uarthlBR_t baud);
//---------------------------------------------------------------------------
/**
 * @brief Initializes software for the specified UART.
 *
 * Basically, the queues are set and, if enabled, FreeRTOS stuff is
 * configured.
 *
 * @param uart UART to be initialized.
 * @param rxBuffer Buffer to hold data received.
 * @param rxBufferSize Size of buffer to hold received data.
 * @param txBuffer Buffer to hold data to be transmitted.
 * @param txBufferSize Size of buffer to hold data to be transmitted.
 * @result 0 if software was initialized successfully, otherwise an error
 *         code.
 */
int32_t uarthlInitializeSW(USART_TypeDef *uart,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
/**
 * @brief Gets pointer for the RX queue of the specified uart.
 *
 * @param uart UART.
 * @result Pointer to buffer or 0 if buffer was not found.
 */
cqueue_t* uarthlGetRXQueue(USART_TypeDef *uart);
//---------------------------------------------------------------------------
/**
 * @brief Gets pointer for the TX queue of the specified uart.
 *
 * @param uart UART.
 * @result Pointer to buffer or 0 if buffer was not found.
 */
cqueue_t* uarthlGetTXQueue(USART_TypeDef *uart);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
#define UARTHL_CONFIG_NVIC_PRIO		0x06 /**< NVIC UART priority. */
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
#ifdef UARTHL_CONFIG_UART1_ENABLED
cqueue_t uarthlQueueUART1RX;
cqueue_t uarthlQueueUART1TX;
#endif

#ifdef UARTHL_CONFIG_UART2_ENABLED
cqueue_t uarthlQueueUART2RX;
cqueue_t uarthlQueueUART2TX;
#endif

#ifdef UARTHL_CONFIG_UART3_ENABLED
cqueue_t uarthlQueueUART3RX;
cqueue_t uarthlQueueUART3TX;
#endif

#ifdef UARTHL_CONFIG_UART4_ENABLED
cqueue_t uarthlQueueUART4RX;
cqueue_t uarthlQueueUART4TX;
#endif

#ifdef UARTHL_CONFIG_UART5_ENABLED
cqueue_t uarthlQueueUART5RX;
cqueue_t uarthlQueueUART5TX;
#endif
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t uarthlInitialize(USART_TypeDef *uart, uarthlBR_t baud, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret;

	ret = uarthlInitializeHW(uart, baud);
	if( ret != 0 ) return ret;

	ret = uarthlInitializeSW(uart, rxBuffer, rxBufferSize, txBuffer, txBufferSize);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t uarthlWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes){

	cqueue_t *tx = 0;
	int32_t ret;
	uint8_t *p;

	tx = uarthlGetTXQueue(uart);
	if( tx == 0 ) return UARTHL_ERR_INVALID_UART;

	p = buffer;
	while( nbytes != 0 ){
		/* Adds item to the TX queue */
		gpioOutputSet(GPIOA, GPIO_P6);
		while( cqueueAdd(tx, p) != 0 );
		gpioOutputReset(GPIOA, GPIO_P6);
		/* Enables tx interrupt if necessary */
		if( !(uart->CR1 & USART_CR1_TXEIE) ) uart->CR1 |= USART_CR1_TXEIE;
		p++;
		nbytes--;
	}

	return 0;
}
//---------------------------------------------------------------------------
int32_t uarthlRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes){

	cqueue_t *rx = 0;
	uint8_t ret;
	uint8_t *p;

	rx = uarthlGetRXQueue(uart);
	if( rx == 0 ) return UARTHL_ERR_INVALID_UART;

	p = buffer;
	while( nbytes != 0 ){
		/* Removes an item from the RX queue */
		gpioOutputSet(GPIOA, GPIO_P6);
		while( cqueueRemove(rx, p) != 0 );
		gpioOutputReset(GPIOA, GPIO_P6);
		p++;
		nbytes--;
	}

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================


//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t uarthlInitializeHW(USART_TypeDef *uart, uarthlBR_t baud){

	uint32_t _uart = (uint32_t)uart;
	GPIO_TypeDef *portTX = 0;
	uint16_t portTXPin = 0;
	GPIO_TypeDef *portRX = 0;
	uint16_t portRXPin = 0;
	uint32_t irqn = 0;

	switch (_uart){
#ifdef UARTHL_CONFIG_UART1_ENABLED
	case USART1_BASE:
		/* Enables clock to USART peripheral */
		RCC->APB2ENR |= (uint32_t)(1U << 14);

		/* Selects GPIO pins*/
		portTX = GPIOA;
		portTXPin = GPIO_P9;
		portRX = GPIOA;
		portRXPin = GPIO_P10;

		/* IRQ priority */
		irqn = USART1_IRQn;
		break;
#endif
#ifdef UARTHL_CONFIG_UART2_ENABLED
	case USART2_BASE:
		/* Enables clock to USART peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 17);

		/* Selects GPIO pins*/
		portTX = GPIOA;
		portTXPin = GPIO_P2;
		portRX = GPIOA;
		portRXPin = GPIO_P3;

		/* IRQ priority */
		irqn = USART2_IRQn;
		break;
#endif
#ifdef UARTHL_CONFIG_UART3_ENABLED
	case USART3_BASE:
		/* Enables clock to USART peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 18);

		/* Selects GPIO pins*/
		portTX = GPIOB;
		portTXPin = GPIO_P10;
		portRX = GPIOB;
		portRXPin = GPIO_P11;

		/* IRQ priority */
		irqn = USART3_IRQn;
		break;
#endif
#ifdef UARTHL_CONFIG_UART4_ENABLED
	case UART4_BASE:
		/* Enables clock to UART peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 19);

		/* Selects GPIO pins*/
		portTX = GPIOC;
		portTXPin = GPIO_P10;
		portRX = GPIOC;
		portRXPin = GPIO_P11;

		/* IRQ priority */
		//irqn = UART4_IRQn;
		break;
#endif
#ifdef UARTHL_CONFIG_UART5_ENABLED
	case UART5_BASE:
		/* Enables clock to UART peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 20);

		/* Selects GPIO pins*/
		portTX = GPIOC;
		portTXPin = GPIO_P12;
		portRX = GPIOD;
		portRXPin = GPIO_P2;

		/* IRQ priority */
		irqn = UART5_IRQn;
		break;
#endif
	default:
		return UARTHL_ERR_INVALID_UART;
	}

	/* Sets baud rate register */
	if( baud == UARTHL_BAUD_9600 ){
		if( uart == USART1 ) uart->BRR = (uint32_t)0x1D4C;
		else uart->BRR = (uint32_t)0xEA6;
	}
	else if( baud == UARTHL_BAUD_115200 ){
		if( uart == USART1 ) uart->BRR = (uint32_t)0x271;
		else uart->BRR = (uint32_t)0x138;
	}
	else if( baud == UARTHL_BAUD_460800 ){
		if( uart == USART1 ) uart->BRR = (uint32_t)0x9C;
		else uart->BRR = (uint32_t)0x4E;
	}
	else{
		return UARTHL_ERR_INVALID_BAUD_RATE;
	}

	/* Sets GPIO pins */
	gpioPortEnable(portTX);
	if(portTX != portRX) gpioPortEnable(portRX);
	gpioConfig(portTX, portTXPin, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(portRX, portRXPin, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, UARTHL_CONFIG_NVIC_PRIO);
	NVIC_EnableIRQ(irqn);

	/* Sets and enable USART/UART */
	uart->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE | USART_CR1_TE;

	return 0;
}
//---------------------------------------------------------------------------
int32_t uarthlInitializeSW(USART_TypeDef *uart,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	cqueue_t *rx = 0;
	cqueue_t *tx = 0;

	rx = uarthlGetRXQueue(uart);
	tx = uarthlGetTXQueue(uart);

	if( (rx == 0) || (tx == 0) ) return UARTHL_ERR_INVALID_UART;

	cqueueInitialize(rx, rxBuffer, rxBufferSize);
	cqueueInitialize(tx, txBuffer, txBufferSize);

	return 0;
}
//---------------------------------------------------------------------------
cqueue_t* uarthlGetRXQueue(USART_TypeDef *uart){

	uint32_t _uart = (uint32_t)uart;
	cqueue_t *rx = 0;

	switch (_uart){

#ifdef UARTHL_CONFIG_UART1_ENABLED
	case USART1_BASE:
		rx = &uarthlQueueUART1RX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART2_ENABLED
	case USART2_BASE:
		rx = &uarthlQueueUART2RX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART3_ENABLED
	case USART3_BASE:
		rx = &uarthlQueueUART3RX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART4_ENABLED
	case UART4_BASE:
		rx = &uarthlQueueUART4RX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART5_ENABLED
	case UART5_BASE:
		rx = &uarthlQueueUART5RX;
		break;
#endif
	}

	return rx;
}
//---------------------------------------------------------------------------
cqueue_t* uarthlGetTXQueue(USART_TypeDef *uart){

	uint32_t _uart = (uint32_t)uart;
	cqueue_t *tx = 0;

	switch (_uart){

#ifdef UARTHL_CONFIG_UART1_ENABLED
	case USART1_BASE:
		tx = &uarthlQueueUART1TX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART2_ENABLED
	case USART2_BASE:
		tx = &uarthlQueueUART2TX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART3_ENABLED
	case USART3_BASE:
		tx = &uarthlQueueUART3TX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART4_ENABLED
	case UART4_BASE:
		tx = &uarthlQueueUART4TX;
		break;
#endif
#ifdef UARTHL_CONFIG_UART5_ENABLED
	case UART5_BASE:
		tx = &uarthlQueueUART5TX;
		break;
#endif
	}

	return tx;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
#ifdef UARTHL_CONFIG_UART1_ENABLED
void USART1_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void USART1_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t usartStatus;

	gpioOutputSet(GPIOA, GPIO_P7);

	usartStatus = USART1->SR;

	/* Data received */
	if( usartStatus & USART_SR_RXNE ){
		rxData = (uint8_t) USART1->DR;
#if (configUART_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(uartRXQueue[0], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		cqueueAdd(&uarthlQueueUART1RX, &rxData);
#endif
	}
	/* Transmitter ready */
	if( usartStatus & USART_SR_TXE ){
		if( cqueueRemove(&uarthlQueueUART1TX, &txData) == 0 ){
			USART1->DR = (uint16_t)txData;
		}
		else{
			/* Disables TX interrupt if queue is empty */
			USART1->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
		}
	}

	gpioOutputReset(GPIOA, GPIO_P7);

}
#endif
//-----------------------------
