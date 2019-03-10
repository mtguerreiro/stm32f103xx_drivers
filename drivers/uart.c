/*
 * uart.c
 *
 *  Created on: Apr 29, 2018
 *      Author: Marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "uart.h"

/* Device */
#include "gpio.h"

/* Kernel */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
//=============================

//=============================
/*---------- Queues ---------*/
//=============================
QueueHandle_t uartTXQueue[5];
QueueHandle_t uartRXQueue[5];
//=============================

//=============================
/*-------- Semaphores -------*/
//=============================
//static SemaphoreHandle_t uartMutex;
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static uint8_t uartHWInitialize(USART_TypeDef *uart, uint32_t baud);
static uint8_t uartSWInitialize(USART_TypeDef *uart);
static void uartTriggerTransmission(USART_TypeDef *uart);
static uint8_t uartQueueIndex(USART_TypeDef *uart);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t uartInitialize(USART_TypeDef *uart, uint32_t baud){

	if( uartHWInitialize(uart, baud) ) return 1;

	if( uartSWInitialize(uart) ) return 2;

	return 0;
}
//-----------------------------
uint8_t uartWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles){

	uint8_t qidx;

	qidx = uartQueueIndex(uart);

	//	uint32_t queueSize;
//	queueSize = (uint32_t)uxQueueSpacesAvailable(uartTXQueue);
//
//	if(nbytes > queueSize) return 1;

	while(nbytes--){
		if( xQueueSendToBack(uartTXQueue[qidx], buffer++, waitcycles) != pdTRUE ) return 2;
	}

	uartTriggerTransmission(uart);

	return 0;
}
//-----------------------------
uint8_t uartRead(USART_TypeDef *uart, uint8_t *buffer, uint32_t waitcycles){

	uint8_t qidx;

	qidx = uartQueueIndex(uart);

	if( xQueueReceive(uartRXQueue[qidx], buffer, waitcycles) != pdTRUE ) return 1;

	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static uint8_t uartHWInitialize(USART_TypeDef *uart, uint32_t baud){

	uint32_t _uart = (uint32_t)uart;
	GPIO_TypeDef *portTX = 0;
	uint16_t portTXPin = 0;
	GPIO_TypeDef *portRX = 0;
	uint16_t portRXPin = 0;
	uint32_t irqn = 0;

	switch (_uart){

#if (configUART1_ENABLED)
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

#if (configUART2_ENABLED)
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

#if (configUART3_ENABLED)
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

#if (configUART4_ENABLED)
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

#if (configUART5_ENABLED)
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
		return 1;
	}

	switch(baud){

	case 9600:
		if( uart == USART1 ) uart->BRR = (uint32_t)0x1D4C;
		else uart->BRR = (uint32_t)0xEA6;
		break;

	case 19200:
		if( uart == USART1 ) uart->BRR = (uint32_t)0xEA6;
		else uart->BRR = (uint32_t)0x753;
		break;

	case 57600:
		if( uart == USART1 ) uart->BRR = (uint32_t)0x4E2;
		else uart->BRR = (uint32_t)0x271;
		break;

	case 115200:
		if( uart == USART1 ) uart->BRR = (uint32_t)0x271;
		else uart->BRR = (uint32_t)0x138;
		break;

	default:
		return 2;
	}

	/* Sets GPIO pins */
	gpioPortEnable(portTX);
	if(portTX != portRX) gpioPortEnable(portRX);
	gpioConfig(portTX, portTXPin, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(portRX, portRXPin, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT_INPUT);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);

	/* Sets and enable USART/UART */
	uart->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;

	return 0;
}
//-----------------------------
static uint8_t uartSWInitialize(USART_TypeDef *uart){

	uint8_t qidx = 0;
	uint32_t qTXSize = 0;
	uint32_t qRXSize = 0;
	uint32_t _uart = (uint32_t)uart;

	switch(_uart){

#if (configUART1_ENABLED)
	case USART1_BASE:
		qidx = 0;
		qTXSize = configUART1_TXQ_SIZE;
		qRXSize = configUART1_RXQ_SIZE;
		break;
#endif

#if (configUART2_ENABLED)
	case USART2_BASE:
		qidx = 1;
		qTXSize = configUART2_TXQ_SIZE;
		qRXSize = configUART2_RXQ_SIZE;
		break;
#endif

#if (configUART3_ENABLED)
	case USART3_BASE:
		qidx = 2;
		qTXSize = configUART3_TXQ_SIZE;
		qRXSize = configUART3_RXQ_SIZE;
		break;
#endif

#if (configUART4_ENABLED)
	case UART4_BASE:
		qidx = 3;
		qTXSize = configUART4_TXQ_SIZE;
		qRXSize = configUART4_RXQ_SIZE;
		break;
#endif

#if (configUART5_ENABLED)
	case UART5_BASE:
		qidx = 4;
		qTXSize = configUART5_TXQ_SIZE;
		qRXSize = configUART5_RXQ_SIZE;
		break;
#endif

	default:
		return 1;
	}

	uartTXQueue[qidx] = xQueueCreate(qTXSize, 1);
	if(uartTXQueue[qidx] == NULL) return 2;

	uartRXQueue[qidx] = xQueueCreate(qRXSize, 1);
	if(uartRXQueue[qidx] == NULL) return 3;

	return 0;
}
//-----------------------------
static void uartTriggerTransmission(USART_TypeDef *uart){

	uart->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;
}
//-----------------------------
static uint8_t uartQueueIndex(USART_TypeDef *uart){

	uint8_t qidx = 0;
	uint32_t _uart = (uint32_t)uart;

	switch(_uart){

#if (configUART1_ENABLED)
	case USART1_BASE:
		qidx = 0;
		break;
#endif

#if (configUART2_ENABLED)
	case USART2_BASE:
		qidx = 1;
		break;
#endif

#if (configUART3_ENABLED)
	case USART3_BASE:
		qidx = 2;
		break;
#endif

#if (configUART4_ENABLED)
	case UART4_BASE:
		qidx = 3;
		break;
#endif

#if (configUART5_ENABLED)
	case UART5_BASE:
		qidx = 4;
		break;
#endif

	default:
		return 0;
	}

	return qidx;
}
//-----------------------------
//=============================

//=============================
/*------- IRQ Handlers ------*/
//=============================
//-----------------------------
#if (configUART1_ENABLED == 1)
void USART1_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void USART1_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t usartStatus;

	usartStatus = USART1->SR;

	/* Data received */
	if( usartStatus & USART_SR_RXNE ){
		rxData = (uint8_t) USART1->DR;
#if (configUART_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(uartRXQueue[0], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(uartRXQueue[0], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( usartStatus & USART_SR_TXE ){
		if( xQueueReceiveFromISR(uartTXQueue[0], &txData, NULL) == pdTRUE){
			USART1->DR = (uint16_t)txData;
		}
		else{
			USART1->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
		}
	}
}
#endif
//-----------------------------
#if (configUART2_ENABLED == 1)
void USART2_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void USART2_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t usartStatus;

	usartStatus = USART2->SR;

	/* Data received */
	if( usartStatus & USART_SR_RXNE ){
		rxData = (uint8_t) USART2->DR;
#if (configUART_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(uartRXQueue[1], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(uartRXQueue[1], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( usartStatus & USART_SR_TXE ){
		if( xQueueReceiveFromISR(uartTXQueue[1], &txData, NULL) == pdTRUE){
			USART2->DR = (uint16_t)txData;
		}
		else{
			USART2->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
		}
	}
}
#endif
//-----------------------------
#if (configUART3_ENABLED == 1)
void USART3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void USART3_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t usartStatus;

	usartStatus = USART3->SR;

	/* Data received */
	if( usartStatus & USART_SR_RXNE ){
		rxData = (uint8_t) USART3->DR;
#if (configUART_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(uartRXQueue[2], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(uartRXQueue[2], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( usartStatus & USART_SR_TXE ){
		if( xQueueReceiveFromISR(uartTXQueue[2], &txData, NULL) == pdTRUE){
			USART3->DR = (uint16_t)txData;
		}
		else{
			USART3->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
		}
	}
}
#endif
//-----------------------------
//=============================
