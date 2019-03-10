/*
 * spi.c
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "spi.h"

/* Drivers */
#include "gpio.h"

/* Kernel */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
//=============================

//=============================
/*---------- Queues ---------*/
//=============================
QueueHandle_t spiTXQueue[3];
QueueHandle_t spiRXQueue[3];
//=============================

//=============================
/*-------- Semaphores -------*/
//=============================
//static SemaphoreHandle_t spiMutex;
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static uint8_t spiHWInitialize(SPI_TypeDef *spi, uint16_t clockDiv);
static uint8_t spiSWInitialize(SPI_TypeDef *spi);
static void spiTriggerTransmission(SPI_TypeDef *spi);
static void spiClockEnable(SPI_TypeDef *spi);
static void spiPinsSet(SPI_TypeDef *spi);
static void spiPrioSet(SPI_TypeDef *spi);
static uint8_t spiQueueIndex(SPI_TypeDef *spi);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t spiInitialize(SPI_TypeDef *spi, uint16_t clockDiv){

	if( spiHWInitialize(spi, clockDiv) ) return 1;

	if( spiSWInitialize(spi) ) return 2;

	return 0;
}
//-----------------------------
uint8_t spiWrite(SPI_TypeDef *spi, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles){

	uint8_t qidx;

	qidx = spiQueueIndex(spi);

	//	uint32_t queueSize;
//	queueSize = (uint32_t)uxQueueSpacesAvailable(spiTXQueue);
//
//	if(nbytes > queueSize) return 1;

	while(nbytes--){
		if( xQueueSendToBack(spiTXQueue[qidx], buffer++, waitcycles) != pdTRUE ) return 2;
	}

	spiTriggerTransmission(spi);

	return 0;
}
//-----------------------------
uint8_t spiRead(SPI_TypeDef *spi, uint8_t *buffer, uint32_t waitcycles){

	uint8_t qidx;

	qidx = spiQueueIndex(spi);

	if( xQueueReceive(spiRXQueue[qidx], buffer, waitcycles) != pdTRUE ) return 1;

	return 0;
}
//-----------------------------
uint8_t spiWaitTX(SPI_TypeDef *spi, uint32_t waitcycles){

	while((spi->SR & SPI_SR_BSY) && (--waitcycles));

	if(!waitcycles) return 1;

	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static uint8_t spiHWInitialize(SPI_TypeDef *spi, uint16_t clockDiv){

	spiClockEnable(spi);

	spiPinsSet(spi);

	//_spi->CR1 = SPI_CR1_LSBFIRST | (SPI_CR1_BR_2); // fpclk/32 -> 1125000 bps
	spi->CR1 = (uint16_t)(clockDiv << 3U) | SPI_CR1_MSTR; // fpclk/32 -> 1125000 bps

	spi->CR2 = SPI_CR2_RXNEIE;

	spi->CR1 |= SPI_CR1_SPE;

	spiPrioSet(spi);

	return 0;
}
//-----------------------------
static uint8_t spiSWInitialize(SPI_TypeDef *spi){

	uint32_t _spi = (uint32_t)spi;
	uint8_t qidx = 0;
	uint32_t qTXSize = 0;
	uint32_t qRXSize = 0;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		qidx = 0;
		qTXSize = configSPI1_TXQ_SIZE;
		qRXSize = configSPI1_RXQ_SIZE;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		qidx = 1;
		qTXSize = configSPI2_TXQ_SIZE;
		qRXSize = configSPI2_RXQ_SIZE;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		qidx = 2;
		qTXSize = configSPI3_TXQ_SIZE;
		qRXSize = configSPI3_RXQ_SIZE;
		break;
#endif

	default:
		return 1;
	}

	spiTXQueue[qidx] = xQueueCreate(qTXSize, 1);
	if(spiTXQueue[qidx] == NULL) return 2;

	spiRXQueue[qidx] = xQueueCreate(qRXSize, 1);
	if(spiRXQueue[qidx] == NULL) return 3;

	return 0;
}
//-----------------------------
static void spiTriggerTransmission(SPI_TypeDef *spi){

	spi->CR2 |= SPI_CR2_TXEIE;
}
//-----------------------------
static void spiClockEnable(SPI_TypeDef *spi){

	uint32_t _spi = (uint32_t)spi;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		RCC->APB2ENR |= (uint32_t)(1U << 12);
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		RCC->APB1ENR |= (uint32_t)(1U << 14);
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		RCC->APB1ENR |= (uint32_t)(1U << 15);
		break;
#endif

	default:
		break;
	}
}
//-----------------------------
static void spiPinsSet(SPI_TypeDef *spi){

	GPIO_TypeDef *port = 0;
	uint32_t _spi = (uint32_t)spi;

	uint16_t mosi = 0;
	uint16_t ck = 0;
	uint16_t miso = 0;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		port = GPIOA;
		mosi = GPIO_P7;
		miso = GPIO_P6;
		ck = GPIO_P5;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		port = GPIOB;
		mosi = GPIO_P15;
		miso = GPIO_P14;
		ck = GPIO_P13;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		port = GPIOB;
		mosi = GPIO_P5;
		miso = GPIO_P4;
		ck = GPIO_P3;
		break;
#endif

	default:
		return;
	}

	gpioPortEnable(port);
	gpioConfig(port, mosi | ck, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(port, miso, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT_INPUT);
}
//-----------------------------
static void spiPrioSet(SPI_TypeDef *spi){

	uint32_t _spi = (uint32_t)spi;
	uint32_t irqn = 0;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		irqn = SPI1_IRQn;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		irqn = SPI2_IRQn;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		irqn = 1;
		//irqn = SPI3_IRQn;
		break;
#endif

	default:
		return;
	}

	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);
}
//-----------------------------
static uint8_t spiQueueIndex(SPI_TypeDef *spi){

	uint32_t _spi = (uint32_t)spi;

	uint8_t qidx = 0;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		qidx = 0;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		qidx = 1;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		qidx = 2;
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
#if (configSPI1_ENABLED == 1)
void SPI1_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void SPI1_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t spiStatus;

	spiStatus = SPI1->SR;

	/* Data received */
	if( spiStatus & SPI_SR_RXNE ){
		rxData = (uint8_t) SPI1->DR;
#if (configSPI_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(spiRXQueue[0], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(spiRXQueue[0], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( spiStatus & SPI_SR_TXE ){
		if( xQueueReceiveFromISR(spiTXQueue[0], &txData, NULL) == pdTRUE){
			SPI1->DR = (uint16_t)txData;
		}
		else{
			SPI1->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
		}
	}
}
#endif
//-----------------------------
#if (configSPI2_ENABLED == 1)
void SPI2_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void SPI2_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t spiStatus;

	spiStatus = SPI2->SR;

	/* Data received */
	if( spiStatus & SPI_SR_RXNE ){
		rxData = (uint8_t) SPI2->DR;
#if (configSPI_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(spiRXQueue[1], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(spiRXQueue[1], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( spiStatus & SPI_SR_TXE ){
		if( xQueueReceiveFromISR(spiTXQueue[1], &txData, NULL) == pdTRUE){
			SPI2->DR = (uint16_t)txData;
		}
		else{
			SPI2->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
		}
	}
}
#endif
//-----------------------------
#if (configSPI3_ENABLED == 1)
void SPI3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void SPI3_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t spiStatus;

	spiStatus = SPI3->SR;

	/* Data received */
	if( spiStatus & SPI_SR_RXNE ){
		rxData = (uint8_t) SPI3->DR;
#if (configSPI_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(spiRXQueue[2], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(spiRXQueue[2], &rxData, NULL);
#endif

	}
	/* Transmitter ready */
	if( spiStatus & SPI_SR_TXE ){
		if( xQueueReceiveFromISR(spiTXQueue[2], &txData, NULL) == pdTRUE){
			SPI3->DR = (uint16_t)txData;
		}
		else{
			SPI3->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
		}
	}
}
#endif
//-----------------------------
//=============================
