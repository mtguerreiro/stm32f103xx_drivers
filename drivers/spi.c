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
static uint8_t spiHWInitialize(uint32_t spi, uint16_t clockDiv);
static uint8_t spiSWInitialize(uint32_t spi);
static void spiTriggerTransmission(uint32_t spi);
static void spiClockEnable(uint32_t spi);
static void spiPinsSet(uint32_t spi);
static void spiPrioSet(uint32_t spi);
static uint8_t spiQueueIndex(uint32_t spi);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t spiInitialize(uint32_t spi, uint16_t clockDiv){

	if( spiHWInitialize(spi, clockDiv) ) return 1;

	if( spiSWInitialize(spi) ) return 2;

	return 0;
}
//-----------------------------
uint8_t spiWrite(uint32_t spi, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles){

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
uint8_t spiRead(uint32_t spi, uint8_t *buffer, uint32_t waitcycles){

	uint8_t qidx;

	qidx = spiQueueIndex(spi);

	if( xQueueReceive(spiRXQueue[qidx], buffer, waitcycles) != pdTRUE ) return 1;

	return 0;
}
//-----------------------------
uint8_t spiWaitTX(uint32_t spi, uint32_t waitcycles){

	SPI_TypeDef *_spi = (SPI_TypeDef *)spi;

	while((_spi->SR & SPI_SR_BSY) && (--waitcycles));

	if(!waitcycles) return 1;

	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static uint8_t spiHWInitialize(uint32_t spi, uint16_t clockDiv){

	SPI_TypeDef *_spi = (SPI_TypeDef *)spi;

	spiClockEnable(spi);

	spiPinsSet(spi);

	//_spi->CR1 = SPI_CR1_LSBFIRST | (SPI_CR1_BR_2); // fpclk/32 -> 1125000 bps
	_spi->CR1 = (uint16_t)(clockDiv << 3U) | SPI_CR1_MSTR; // fpclk/32 -> 1125000 bps

	_spi->CR2 = SPI_CR2_RXNEIE;

	_spi->CR1 |= SPI_CR1_SPE;

	spiPrioSet(spi);

	return 0;
}
//-----------------------------
static uint8_t spiSWInitialize(uint32_t spi){

	uint8_t qidx = 0;
	uint32_t qTXSize = 0;
	uint32_t qRXSize = 0;

	switch(spi){

#if (configSPI1_ENABLED == 1)
	case SPI_1:
		qidx = 0;
		qTXSize = configSPI1_TXQ_SIZE;
		qRXSize = configSPI1_RXQ_SIZE;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI_2:
		qidx = 1;
		qTXSize = configSPI2_TXQ_SIZE;
		qRXSize = configSPI2_RXQ_SIZE;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI_3:
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
static void spiTriggerTransmission(uint32_t spi){

	SPI_TypeDef *_spi = (SPI_TypeDef *)spi;

	_spi->CR2 |= SPI_CR2_TXEIE;
}
//-----------------------------
static void spiClockEnable(uint32_t spi){

	switch(spi){

#if (configSPI1_ENABLED == 1)
	case SPI_1:
		RCC->APB2ENR |= (uint32_t)(1U << 12);
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI_2:
		RCC->APB1ENR |= (uint32_t)(1U << 14);
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI_3:
		RCC->APB1ENR |= (uint32_t)(1U << 15);
		break;
#endif

	default:
		break;
	}
}
//-----------------------------
static void spiPinsSet(uint32_t spi){

	uint32_t port = 0;

	uint16_t mosi = 0;
	uint16_t ck = 0;
	uint16_t miso = 0;

	switch(spi){

#if (configSPI1_ENABLED == 1)
	case SPI_1:
		port = GPIO_PA;
		mosi = GPIO_PA_7;
		miso = GPIO_PA_6;
		ck = GPIO_PA_5;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI_2:
		port = GPIO_PB;
		mosi = GPIO_PB_15;
		miso = GPIO_PB_14;
		ck = GPIO_PB_13;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI_3:
		port = GPIO_PB;
		mosi = GPIO_PB_5;
		miso = GPIO_PB_4;
		ck = GPIO_PB_3;
		break;
#endif

	default:
		return;
	}

	gpioPortEnable(port);

	gpioMode(port, GPIO_MODE_OUTPUT_10MHZ, mosi | ck);
	gpioConfig(port, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL, mosi | ck);

	gpioMode(port, GPIO_MODE_INPUT, miso);
	gpioConfig(port, GPIO_CONFIG_INPUT_FLOAT_INPUT, miso);
}
//-----------------------------
static void spiPrioSet(uint32_t spi){

	uint32_t irqn = 0;

	switch(spi){

#if (configSPI1_ENABLED == 1)
	case SPI_1:
		irqn = SPI1_IRQn;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI_2:
		irqn = SPI2_IRQn;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI_3:
		irqn = SPI3_IRQn;
		break;
#endif

	default:
		return;
	}

	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);
}
//-----------------------------
static uint8_t spiQueueIndex(uint32_t spi){

	uint8_t qidx = 0;

	switch(spi){

#if (configSPI1_ENABLED == 1)
	case SPI_1:
		qidx = 0;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI_2:
		qidx = 1;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI_3:
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
