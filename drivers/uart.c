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
static uint8_t uartHWInitialize(uint32_t uart);
static uint8_t uartSWInitialize(uint32_t uart);
static void uartTriggerTransmission(uint32_t uart);
static void uartClockEnable(uint32_t uart);
static void uartPinsSet(uint32_t uart);
static void uartPrioSet(uint32_t uart);
static uint8_t uartQueueIndex(uint32_t uart);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t uartInitialize(uint32_t uart){

	uartHWInitialize(uart);

	uartSWInitialize(uart);

	return 0;
}
//-----------------------------
uint8_t uartWrite(uint32_t uart, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles){

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
uint8_t uartRead(uint32_t uart, uint8_t *buffer, uint32_t waitcycles){

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
static uint8_t uartHWInitialize(uint32_t uart){

	USART_TypeDef *_uart = (USART_TypeDef *)uart;

	uartClockEnable(uart);

	uartPinsSet(uart);

	//USART1->CR1 = USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
	_uart->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
	//USART1->BRR = (uint32_t)0x1D4C; //9600 bps / clk = 72e6
	_uart->BRR = (uint32_t)0x271; // 115200 bps / clk = 72e6

	uartPrioSet(uart);

	return 0;
}
//-----------------------------
static uint8_t uartSWInitialize(uint32_t uart){

	uint8_t qidx = 0;
	uint32_t qTXSize = 0;
	uint32_t qRXSize = 0;

	switch(uart){

#if (configUART1_ENABLED)
	case UART_1:
		qidx = 0;
		qTXSize = configUART1_TXQ_SIZE;
		qRXSize = configUART1_RXQ_SIZE;
		break;
#endif

#if (configUART2_ENABLED)
	case UART_2:
		qidx = 1;
		qTXSize = configUART2_TXQ_SIZE;
		qRXSize = configUART2_RXQ_SIZE;
		break;
#endif

#if (configUART3_ENABLED)
	case UART_3:
		qidx = 2;
		qTXSize = configUART3_TXQ_SIZE;
		qRXSize = configUART3_RXQ_SIZE;
		break;
#endif

#if (configUART4_ENABLED)
	case UART_4:
		qidx = 3;
		qTXSize = configUART4_TXQ_SIZE;
		qRXSize = configUART4_RXQ_SIZE;
		break;
#endif

#if (configUART5_ENABLED)
	case UART_5:
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
static void uartTriggerTransmission(uint32_t uart){

	USART_TypeDef *_uart = (USART_TypeDef *)uart;

	_uart->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;
}
//-----------------------------
static void uartClockEnable(uint32_t uart){

	switch(uart){

#if (configUART1_ENABLED)
	case UART_1:
		RCC->APB2ENR |= (uint32_t)(1U << 14);
		break;
#endif

#if (configUART2_ENABLED)
	case UART_2:
		RCC->APB1ENR |= (uint32_t)(1U << 17);
		break;
#endif

#if (configUART3_ENABLED)
	case UART_3:
		RCC->APB1ENR |= (uint32_t)(1U << 18);
		break;
#endif

#if (configUART4_ENABLED)
	case UART_4:
		RCC->APB1ENR |= (uint32_t)(1U << 19);
		break;
#endif

#if (configUART5_ENABLED)
	case UART_5:
		RCC->APB1ENR |= (uint32_t)(1U << 20);
		break;
#endif

	default:
		break;
	}
}
//-----------------------------
static void uartPinsSet(uint32_t uart){

	uint32_t portTX = 0;
	uint16_t portTXPin = 0;
	uint32_t portRX = 0;
	uint16_t portRXPin = 0;

	switch(uart){

#if (configUART1_ENABLED)
	case UART_1:
		portTX = GPIO_PA;
		portTXPin = GPIO_PA_9;
		portRX = GPIO_PA;
		portRXPin = GPIO_PA_10;
		break;
#endif

#if (configUART2_ENABLED)
	case UART_2:
		portTX = GPIO_PA;
		portTXPin = GPIO_PA_2;
		portRX = GPIO_PA;
		portRXPin = GPIO_PA_3;
		break;
#endif

#if (configUART3_ENABLED)
	case UART_3:
		portTX = GPIO_PB;
		portTXPin = GPIO_PB_10;
		portRX = GPIO_PB;
		portRXPin = GPIO_PB_11;
		break;
#endif

#if (configUART4_ENABLED)
	case UART_4:
		portTX = GPIO_PC;
		portTXPin = GPIO_PC_10;
		portRX = GPIO_PC;
		portRXPin = GPIO_PC_11;
		break;
#endif

#if (configUART5_ENABLED)
	case UART_5:
		portTX = GPIO_PC;
		portTXPin = GPIO_PC_12;
		portRX = GPIO_PD;
		portRXPin = GPIO_PD_2;
		break;
#endif

	default:
		return;
	}

	gpioPortEnable(portTX);
	if(portTX != portRX) gpioPortEnable(portRX);

	gpioMode(portTX, GPIO_MODE_OUTPUT_10MHZ, portTXPin);
	gpioConfig(portTX, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL, portTXPin);

	gpioMode(portRX, GPIO_MODE_INPUT, portRXPin);
	gpioConfig(portRX, GPIO_CONFIG_INPUT_FLOAT_INPUT, portRXPin);
}
//-----------------------------
static void uartPrioSet(uint32_t uart){

	uint32_t irqn = 0;

	switch(uart){

#if (configUART1_ENABLED)
	case UART_1:
		irqn = USART1_IRQn;
		break;
#endif

#if (configUART2_ENABLED)
	case UART_2:
		irqn = USART2_IRQn;
		break;
#endif

#if (configUART3_ENABLED)
	case UART_3:
		irqn = USART3_IRQn;
		break;
#endif

#if (configUART4_ENABLED)
	case UART_4:
		//irqn = UART4_IRQn;
		break;
#endif

#if (configUART5_ENABLED)
	case UART_5:
		//irqn = UART5_IRQn;
		break;
#endif

	default:
		return;
	}

	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);
}
//-----------------------------
static uint8_t uartQueueIndex(uint32_t uart){

	uint8_t qidx = 0;

	switch(uart){

#if (configUART1_ENABLED)
	case UART_1:
		qidx = 0;
		break;
#endif

#if (configUART2_ENABLED)
	case UART_2:
		qidx = 1;
		break;
#endif

#if (configUART3_ENABLED)
	case UART_3:
		qidx = 2;
		break;
#endif

#if (configUART4_ENABLED)
	case UART_4:
		qidx = 3;
		break;
#endif

#if (configUART5_ENABLED)
	case UART_5:
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
