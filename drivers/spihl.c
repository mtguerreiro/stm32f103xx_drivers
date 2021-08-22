/*
 * spihl.c
 *
 *  Created on: 8 de mai de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "spihl.h"

/* Libs */
#include "cqueue.h"

/* Drivers */
#include "gpio.h"

#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"
#endif
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{
	cqueue_t rxQueue;			/**< RX queue. */
	cqueue_t txQueue;			/**< TX queue. */

#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
	SemaphoreHandle_t rxSemph;	/**< RX semaphore. */
	SemaphoreHandle_t txSemph;	/**< TX semaphore. */
#endif
}spihlControl_t;
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes hardware for the specified SPI.
 *
 * GPIOs are selected as SPI MISO, MOSI and CLK, registers are set and
 * interruptions are enabled.
 *
 * @param spi SPI to be initialized.
 * @param div Clock prescaler
 * @param pp Clock phase and polarity.
 * @result 0 if hardware was initialized successfully, otherwise an error
 *         code.
 */
static int32_t spihlInitializeHW(SPI_TypeDef *spi, spihlBR_t div, spihlPP_t pp);
//---------------------------------------------------------------------------
/**
 * @brief Initializes software for the specified SPI.
 *
 * Basically, the queues are set and, if enabled, FreeRTOS stuff is
 * configured.
 *
 * @param spi SPI to be initialized.
 * @param rxBuffer Buffer to hold data received.
 * @param rxBufferSize Size of buffer to hold received data.
 * @param txBuffer Buffer to hold data to be transmitted.
 * @param txBufferSize Size of buffer to hold data to be transmitted.
 * @result 0 if software was initialized successfully, otherwise an error
 *         code.
 */
static int32_t spihlInitializeSW(SPI_TypeDef *spi,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
/**
 * @brief Gets pointer for the control structure the specified SPI.
 *
 * @param spi SPI.
 * @result Pointer to structure or 0 if structure was not found.
 */
static spihlControl_t* spihlGetControlStruct(SPI_TypeDef *spi);
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
/**
 * @brief Initializes the semaphores for the specified SPI.
 *
 * @param spi SPI.
 * @param spiControl Pointer to control structure of the specified SPI.
 * @ result 0 if the semaphores were successfully initialized, otherwise an
 * 			error code.
 */
static int32_t spihlInitializeSWSemph(SPI_TypeDef *spi,
		spihlControl_t* spiControl);
#endif
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
#ifdef SPIHL_CONFIG_SPI1_ENABLED
spihlControl_t spihlSPI1Control;
#endif

#ifdef SPIHL_CONFIG_SPI2_ENABLED
spihlControl_t spihlSPI2Control;
#endif

#ifdef SPIHL_CONFIG_SPI3_ENABLED
spihlControl_t spihlSPI3Control;
#endif
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t spihlInitialize(SPI_TypeDef *spi, spihlBR_t clockDiv, \
		spihlPP_t clockPP, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret;

	ret = spihlInitializeHW(spi, clockDiv, clockPP);
	if( ret != 0 ) return ret;

	ret = spihlInitializeSW(spi, rxBuffer, rxBufferSize, txBuffer, txBufferSize);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t spihlWrite(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
					uint32_t timeout){

	uint8_t emptyByte = 0xFF;
	spihlControl_t *spiControl = 0;
	uint8_t *p;
	uint32_t to;
	int32_t bytesWritten;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	if( buffer != 0 ) p = buffer;
	else p = &emptyByte;

	bytesWritten = 0;
	while( bytesWritten < nbytes ){
		/* Adds item to the TX queue */
		to = timeout;
		while( (cqueueAdd(&spiControl->txQueue, p) != 0) && (to != 0) ) to--;
		if( to == 0 ) break;

		/* Enables tx interrupt if necessary */
		if( !(spi->CR2 & SPI_CR2_TXEIE) ) spi->CR2 |= SPI_CR2_TXEIE;

		if( buffer != 0 ) p++;
		bytesWritten++;
	}

	return bytesWritten;
}
//---------------------------------------------------------------------------
int32_t spihlRead(SPI_TypeDef *spi, uint8_t *buffer, int32_t nbytes,
				   uint32_t timeout){

	spihlControl_t *spiControl = 0;
	uint8_t *p;
	uint32_t to;
	int32_t bytesRead;
	int32_t bytesWritten;
	int32_t bytesToWrite;
	int32_t bytesToRead;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	/*
	 * Waits until any ongoing transmission ends. To do this, we must wait
	 * until TXEIE is cleared (meaning there are no more bytes waiting in the
	 * TX queue) and then we wait until the SPI's BUSY flag is cleared,
	 * because when TXEIE is cleared, the last byte is still being shifted by
	 * the SPI hardware.
     */
	to = timeout;
	while( (spi->CR2 & SPI_CR2_TXEIE) && (to != 0 ) ) to--;
	if( to == 0 ) return 0;

	to = timeout;
	while( (spi->SR & SPI_SR_BSY ) && (to != 0 ) ) to--;
	if( to == 0 ) return 0;

	/*
	 * Reads the SPI data register to make sure there is nothing there so
	 * that we can enable the RX interrupt and not trigger an erroneous
	 * interrupt.
	 */
	p = buffer;
	*p = (uint8_t)spi->DR;
	spi->CR2 |= SPI_CR2_RXNEIE;

	/*
	 * Here, we'll provide as many 0xFF bytes as possible, to provide
	 * clock for reading. Then, we'll read as many bytes as we wrote.
	 * In the next iteration, we'll write 0xFF for the bytes still to be
	 * read, repeating this until we read all the bytes or until a
	 * time-out occurred, which is an error.
	 */
	bytesRead = 0;
	bytesWritten = 0;
	while( 1 ){
		bytesToRead = 0;
		bytesToWrite = nbytes - bytesRead;
		if( bytesToWrite > spiControl->rxQueue.size ) bytesToWrite = spiControl->rxQueue.size;
		bytesWritten = spihlWrite(spi, 0, bytesToWrite, 1);
		while( bytesToRead < bytesWritten ){
			/* Removes an item from the RX queue */
			to = timeout;
			while( (cqueueRemove(&spiControl->rxQueue, p) != 0) && (to != 0) ) to--;
			if( to == 0 ) break;

			p++;
			bytesToRead++;
		}
		if( to == 0 ) break;
		bytesRead += bytesToRead;
		if( bytesRead == nbytes ) break;
	}

	/* Disables RX interrupt */
	spi->CR2 &= ((uint16_t)(~SPI_CR2_RXNEIE));

	return bytesRead;
}
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
int32_t spihlPendRXSemaphore(SPI_TypeDef *spi, uint32_t timeout){

	spihlControl_t *spiControl = 0;
	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	if( xSemaphoreTake(spiControl->rxSemph, timeout) != pdTRUE ) return 1;

	return 0;
}
#endif
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
int32_t spihlPendTXSemaphore(SPI_TypeDef *spi, uint32_t timeout){

	spihlControl_t *spiControl = 0;
	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	if( xSemaphoreTake(spiControl->txSemph, timeout) != pdTRUE ) return 1;

	return 0;
}
#endif
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t spihlInitializeHW(SPI_TypeDef *spi, spihlBR_t div, spihlPP_t pp){

	GPIO_TypeDef *port = 0;
	uint32_t _spi = (uint32_t)spi;
	uint16_t mosi = 0;
	uint16_t ck = 0;
	uint16_t miso = 0;
	IRQn_Type irqn = (IRQn_Type) 0;
	IRQn_Type irqnPrio = (IRQn_Type) 0;

	switch(_spi){

#ifdef SPIHL_CONFIG_SPI1_ENABLED
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
		irqnPrio = (IRQn_Type) SPIHL_CONFIG_SPI1_NVIC_PRIO;
		break;
#endif

#ifdef SPIHL_CONFIG_SPI2_ENABLED
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
		irqnPrio = (IRQn_Type) SPIHL_CONFIG_SPI2_NVIC_PRIO;
		break;
#endif

#ifdef SPIHL_CONFIG_SPI3_ENABLED
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
		irqnPrio = (IRQn_Type) SPIHL_CONFIG_SPI3_NVIC_PRIO;
		break;
#endif

	default:
		return SPIHL_ERR_INVALID_SPI;
	}

	/* Sets SPI pins */
	gpioPortEnable(port);
	gpioConfig(port, mosi | ck, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_PUSH_PULL);
	gpioConfig(port, miso, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, irqnPrio);
	NVIC_EnableIRQ(irqn);

	/* Sets SPI configs */
	spi->CR1 = (uint16_t)((div << 3U) | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | pp); // fpclk/32 -> 1125000 bps
	//spi->CR2 = SPI_CR2_RXNEIE;
	spi->CR1 |= SPI_CR1_SPE;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t spihlInitializeSW(SPI_TypeDef *spi,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	spihlControl_t *spiControl = 0;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	cqueueInitialize(&spiControl->rxQueue, rxBuffer, rxBufferSize);
	cqueueInitialize(&spiControl->txQueue, txBuffer, txBufferSize);

#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
	if( spihlInitializeSWSemph(spi, spiControl) != 0 ){
		return SPIHL_ERR_SEMPH_CREATE;
	}
#endif

	return 0;
}
//---------------------------------------------------------------------------
static spihlControl_t* spihlGetControlStruct(SPI_TypeDef *spi){

	uint32_t _spi = (uint32_t)spi;
	spihlControl_t *spiControl = 0;

	switch (_spi){

#ifdef SPIHL_CONFIG_SPI1_ENABLED
	case SPI1_BASE:
		spiControl = &spihlSPI1Control;
		break;
#endif
#ifdef SPIHL_CONFIG_SPI2_ENABLED
	case SPI2_BASE:
		spiControl = &spihlSPI2Control;
		break;
#endif
#ifdef SPIHL_CONFIG_SPI3_ENABLED
	case SPI3_BASE:
		spiControl = &spihlSPI3Control;
		break;
#endif
	}

	return spiControl;
}
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
static int32_t spihlInitializeSWSemph(SPI_TypeDef *spi,
		spihlControl_t* spiControl){

	uint32_t _spi = (uint32_t)spi;
	uint32_t semCreate = 0;

	switch (_spi){

#ifdef SPIHL_CONFIG_SPI1_RTOS_EN
	case SPI1_BASE:
		semCreate = 1;
		break;
#endif
#ifdef SPIHL_CONFIG_SPI2_RTOS_EN
	case SPI2_BASE:
		semCreate = 1;
		break;
#endif
#ifdef SPIHL_CONFIG_SPI3_RTOS_EN
	case SPI3_BASE:
		semCreate = 1;
		break;
#endif
	}

	if( semCreate == 1 ){
		spiControl->txSemph = xSemaphoreCreateBinary();
		spiControl->rxSemph = xSemaphoreCreateBinary();
		if( (spiControl->txSemph == NULL) || (spiControl->rxSemph == NULL) ){
			return SPIHL_ERR_SEMPH_CREATE;
		}
		xSemaphoreGive(spiControl->rxSemph);
		xSemaphoreGive(spiControl->txSemph);
	}

	return 0;
}
#endif
//---------------------------------------------------------------------------
//===========================================================================


//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_SPI1_ENABLED
void SPI1_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void SPI1_IRQHandler(void){

	uint8_t txData, rxData;
	uint32_t spiStatus;

	spiStatus = SPI1->SR;

	/* Transmitter ready */
	if( (SPI1->CR2 & SPI_CR2_TXEIE) && (spiStatus & SPI_SR_TXE) ){
		if( cqueueRemove(&spihlSPI1Control.txQueue, &txData) == 0 ){
			SPI1->DR = (uint16_t)txData;
#ifdef SPIHL_CONFIG_SPI1_RTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(spihlSPI1Control.txSemph, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Disables TX interrupt if queue is empty */
			SPI1->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
		}
	} // if( (SPI1->CR2 & SPI_CR2_TXEIE) && (spiStatus & SPI_SR_TXE) )

	/* Data received */
	else if( (SPI1->CR2 & SPI_CR2_RXNEIE) && (spiStatus & SPI_SR_RXNE) ){
		rxData = (uint8_t) SPI1->DR;
		cqueueAdd(&spihlSPI1Control.rxQueue, &rxData);
#ifdef SPIHL_CONFIG_SPI1_RTOS_EN
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(spihlSPI1Control.rxSemph, &xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
	} // else if( (SPI1->CR2 & SPI_CR2_RXNEIE) && (spiStatus & SPI_SR_RXNE) )
}
#endif
//---------------------------------------------------------------------------
//===========================================================================
