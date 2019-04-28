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
/** @brief SPI TX queues. */
QueueHandle_t spiTXQueue[3];
/** @brief SPI RX queues */
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
//-----------------------------
/** @brief Initializes SPI hardware.
 *
 * SPI settings, interrupts and GPIO are set.
 *
 * @param spi SPI to be initialized.
 * @param clockDiv Clock divisor for the selected SPI peripheral. Must be one
 *                 of spiClockDiv_t values.
 *
 * @return 0 if initialization succeeded, 1 if initialization failed. This
 *         can only occur if the SPI informed is not valid.
 * */
static uint8_t spiHWInitialize(SPI_TypeDef *spi, uint16_t clockDiv);
//-----------------------------
/** @brief Initializes the RX and TX queues.
 *
 * @param spi SPI to be initialized.
 *
 * @return 0 if initialization suceedded, 1 if failed to create any of the
 *         queues.
 */
static uint8_t spiSWInitialize(SPI_TypeDef *spi);
//-----------------------------
/** @brief Triggers SPI transmission.
 *
 * Calling this functions enables TX empty interruption.
 *
 * @param spi SPI to enable TX empty interruption.
 */
static void spiTriggerTransmission(SPI_TypeDef *spi);
//-----------------------------
/** @brief Returns queue index for the informed SPI.
 *
 * This is used to index the RX/TX queue vector.
 *
 * @param spi SPI to determine index.
 * @return Index of informed SPI.
 */
static uint8_t spiQueueIndex(SPI_TypeDef *spi);
//-----------------------------
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
uint8_t spiWrite(SPI_TypeDef *spi, uint8_t *buffer, uint16_t nbytes){

    uint8_t qidx;
    uint8_t *txbuffer;
	uint32_t queueSize;

	qidx = spiQueueIndex(spi);

	queueSize = (uint32_t)uxQueueSpacesAvailable(spiTXQueue[qidx]);
	if(nbytes > queueSize) return 1;

    /*
     * We save the buffer pointer in a local variable to preserve the
     * original pointer's value.
     */
    txbuffer = buffer;
	while(nbytes--){
		xQueueSendToBack(spiTXQueue[qidx], txbuffer++, 0);
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

	GPIO_TypeDef *port = 0;
	uint32_t _spi = (uint32_t)spi;
	uint32_t irqn = 0;
	uint16_t mosi = 0;
	uint16_t ck = 0;
	uint16_t miso = 0;

	switch(_spi){

#if (configSPI1_ENABLED == 1)
	case SPI1_BASE:
		/* Enables clock to SPI peripheral */
		RCC->APB2ENR |= (uint32_t)(1U << 12);

		/* Selects GPIO pins*/
		port = GPIOA;
		mosi = GPIO_P7;
		miso = GPIO_P6;
		ck = GPIO_P5;

		/* IRQ priority */
		irqn = SPI1_IRQn;
		break;
#endif

#if (configSPI2_ENABLED == 1)
	case SPI2_BASE:
		/* Enables clock to SPI peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 14);

		/* Selects GPIO pins*/
		port = GPIOB;
		mosi = GPIO_P15;
		miso = GPIO_P14;
		ck = GPIO_P13;

		/* IRQ priority */
		irqn = SPI2_IRQn;
		break;
#endif

#if (configSPI3_ENABLED == 1)
	case SPI3_BASE:
		/* Enables clock to SPI peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 15);

		/* Selects GPIO pins*/
		port = GPIOB;
		mosi = GPIO_P5;
		miso = GPIO_P4;
		ck = GPIO_P3;

		/* IRQ priority */
		irqn = SPI3_IRQn;
		break;
#endif

	default:
		return 1;
	}

	/* Sets SPI pins */
	gpioPortEnable(port);
	gpioConfig(port, mosi | ck, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(port, miso, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT_INPUT);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);

	/* Sets SPI configs */
	spi->CR1 = (uint16_t)(clockDiv << 3U) | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // fpclk/32 -> 1125000 bps
	spi->CR2 = SPI_CR2_RXNEIE;
	spi->CR1 |= SPI_CR1_SPE;

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
	}

	spiTXQueue[qidx] = xQueueCreate(qTXSize, 1);
	if(spiTXQueue[qidx] == NULL) return 1;

	spiRXQueue[qidx] = xQueueCreate(qRXSize, 1);
	if(spiRXQueue[qidx] == NULL) return 1;

	return 0;
}
//-----------------------------
static void spiTriggerTransmission(SPI_TypeDef *spi){

	spi->CR2 |= SPI_CR2_TXEIE;
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
#if (configSPI1_INTERRUPT_YIELD == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendToBackFromISR(spiRXQueue[0], &rxData, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		xQueueSendToBackFromISR(spiRXQueue[0], &rxData, NULL);
#endif
	}
	/* Transmitter ready */
	if( (spiStatus & SPI_SR_TXE) && (SPI1->CR2 & SPI_CR2_TXEIE) ){
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
	if( (spiStatus & SPI_SR_TXE) && (SPI2->CR2 & SPI_CR2_TXEIE) ){
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
	if( (spiStatus & SPI_SR_TXE) && (SPI3->CR2 & SPI_CR2_TXEIE) ){
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
