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

#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
/* Kernel */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#endif
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{
	cqueue_t rxQueue;			/**< RX queue. */
	cqueue_t txQueue;			/**< TX queue. */

#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
	SemaphoreHandle_t rxSemph;	/**< RX semaphore. */
	SemaphoreHandle_t txSemph;	/**< TX semaphore. */
#endif
}uarthlControl_t;
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
/**
 * @brief Gets pointer for the TX queue of the specified uart.
 *
 * @param uart UART.
 * @result Pointer to buffer or 0 if buffer was not found.
 */
uarthlControl_t* uarthlGetControlStruct(USART_TypeDef *uart);
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
uarthlControl_t uarthlUART1Control;
#endif

#ifdef UARTHL_CONFIG_UART2_ENABLED
uarthlControl_t uarthlUART2Control;
#endif

#ifdef UARTHL_CONFIG_UART3_ENABLED
uarthlControl_t uarthlUART3Control;
#endif

#ifdef UARTHL_CONFIG_UART4_ENABLED
uarthlControl_t uarthlUART4Control;
#endif

#ifdef UARTHL_CONFIG_UART5_ENABLED
uarthlControl_t uarthlUART5Control;
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
int32_t uarthlWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
					uint32_t timeout){

	uarthlControl_t *uartControl = 0;
	uint8_t *p;
	uint32_t to;

	uartControl = uarthlGetControlStruct(uart);
	if( uartControl == 0 ) return UARTHL_ERR_INVALID_UART;

	p = buffer;
	while( nbytes != 0 ){
		/* Adds item to the TX queue */
		to = timeout;
		while( (cqueueAdd(&uartControl->txQueue, p) != 0) && (to != 0) ) to--;
		if( to == 0 ) break;

		/* Enables tx interrupt if necessary */
		if( !(uart->CR1 & USART_CR1_TXEIE) ) uart->CR1 |= USART_CR1_TXEIE;

		p++;
		nbytes--;
	}

	return nbytes;
}
//---------------------------------------------------------------------------
int32_t uarthlRead(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes,
				   uint32_t timeout){

	uarthlControl_t *uartControl = 0;
	uint8_t *p;
	uint32_t to;

	uartControl = uarthlGetControlStruct(uart);
	if( uartControl == 0 ) return UARTHL_ERR_INVALID_UART;

	p = buffer;
	while( nbytes != 0 ){
		/* Removes an item from the RX queue */
		to = timeout;
		while( (cqueueRemove(&uartControl->rxQueue, p) != 0) && (to != 0) ) to--;
		if( to == 0 ) break;

		p++;
		nbytes--;
	}

	return nbytes;
}
//---------------------------------------------------------------------------
int32_t uarthlPendRXSemaphore(USART_TypeDef *uart, uint32_t timeout){

	uarthlControl_t *uartControl = 0;
	uartControl = uarthlGetControlStruct(uart);
	if( uartControl == 0 ) return UARTHL_ERR_INVALID_UART;

	if( xSemaphoreTake(uartControl->rxSemph, timeout) != pdTRUE ) return 1;

	return 0;
}
//---------------------------------------------------------------------------
int32_t uarthlPendTXSemaphore(USART_TypeDef *uart, uint32_t timeout){

	uarthlControl_t *uartControl = 0;
	uartControl = uarthlGetControlStruct(uart);
	if( uartControl == 0 ) return UARTHL_ERR_INVALID_UART;

	if( xSemaphoreTake(uartControl->txSemph, timeout) != pdTRUE ) return 1;

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

	uarthlControl_t *uartControl = 0;

	uartControl = uarthlGetControlStruct(uart);
	if( uartControl == 0 ) return UARTHL_ERR_INVALID_UART;

	cqueueInitialize(&uartControl->rxQueue, rxBuffer, rxBufferSize);
	cqueueInitialize(&uartControl->txQueue, txBuffer, txBufferSize);

#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
	uartControl->txSemph = xSemaphoreCreateBinary();
	uartControl->rxSemph = xSemaphoreCreateBinary();
	if( (uartControl->txSemph == NULL) || (uartControl->rxSemph == NULL) ){
		return UARTHL_ERR_SEMPH_CREATE;
	}
	xSemaphoreGive(uartControl->rxSemph);
	xSemaphoreGive(uartControl->txSemph);
#endif

	return 0;
}
//---------------------------------------------------------------------------
uarthlControl_t* uarthlGetControlStruct(USART_TypeDef *uart){

	uint32_t _uart = (uint32_t)uart;
	uarthlControl_t *uartControl = 0;

	switch (_uart){

#ifdef UARTHL_CONFIG_UART1_ENABLED
	case USART1_BASE:
		uartControl = &uarthlUART1Control;
		break;
#endif
#ifdef UARTHL_CONFIG_UART2_ENABLED
	case USART2_BASE:
		uartControl = &uarthlUART2Control;
		break;
#endif
#ifdef UARTHL_CONFIG_UART3_ENABLED
	case USART3_BASE:
		uartControl = &uarthlUART3Control;
		break;
#endif
#ifdef UARTHL_CONFIG_UART4_ENABLED
	case UART4_BASE:
		uartControl = &uarthlUART4Control;
		break;
#endif
#ifdef UARTHL_CONFIG_UART5_ENABLED
	case UART5_BASE:
		uartControl = &uarthlUART5Control;
		break;
#endif
	}

	return uartControl;
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
		cqueueAdd(&uarthlUART1Control.rxQueue, &rxData);
#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(uarthlUART1Control.rxSemph, &xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
	} //if( usartStatus & USART_SR_RXNE )

	/* Transmitter ready */
	else if( usartStatus & USART_SR_TXE ){
		if( cqueueRemove(&uarthlUART1Control.txQueue, &txData) == 0 ){
			USART1->DR = (uint16_t)txData;
#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(uarthlUART1Control.txSemph, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Disables TX interrupt if queue is empty */
			USART1->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
		}
	}//else if( usartStatus & USART_SR_TXE )

	gpioOutputReset(GPIOA, GPIO_P7);
}
#endif
//-----------------------------
