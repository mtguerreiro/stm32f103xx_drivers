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

/* Drivers */
#include "gpio.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{
	volatile uint8_t busy;
	uint8_t mode;
	uint32_t nbytes;
	uint8_t *p;
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
 * THe control structure of the selected SPI is initialized.
 *
 * @param spi SPI to be initialized.
 * @result 0 if software was initialized successfully, otherwise an error
 *         code.
 */
static int32_t spihlInitializeSW(SPI_TypeDef *spi);
//---------------------------------------------------------------------------
/**
 * @brief Gets pointer for the control structure the specified SPI.
 *
 * @param spi SPI.
 * @result Pointer to structure or 0 if structure was not found.
 */
static spihlControl_t* spihlGetControlStruct(SPI_TypeDef *spi);
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
int32_t spihlInitialize(SPI_TypeDef *spi, spihlBR_t clockDiv, spihlPP_t clockPP){

	int32_t ret;

	ret = spihlInitializeHW(spi, clockDiv, clockPP);
	if( ret != 0 ) return ret;

	ret = spihlInitializeSW(spi);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t spihlWrite(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
					uint32_t timeout){

	spihlControl_t *spiControl = 0;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	while( (spiControl->busy != 0) && (timeout != 0 ) ) timeout--;
	if( timeout == 0 ) return SPIHL_ERR_BUSY;

	while( (spi->SR & SPI_SR_BSY ) && (timeout != 0 ) ) timeout--;
	if( timeout == 0 ) return 0;

	spiControl->busy = 1;
	spiControl->mode = 0;
	spiControl->nbytes = nbytes;
	spiControl->p = buffer;

	/* Enables SPI TX interrupt */
	spi->CR2 |= SPI_CR2_TXEIE;

	return 0;
}
//---------------------------------------------------------------------------
int32_t spihlRead(SPI_TypeDef *spi, uint8_t *buffer, uint32_t nbytes,
				   uint32_t timeout){

	spihlControl_t *spiControl = 0;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	while( (spiControl->busy != 0) && (timeout != 0 ) ) timeout--;
	if( timeout == 0 ) return SPIHL_ERR_BUSY;

	while( (spi->SR & SPI_SR_BSY ) && (timeout != 0 ) ) timeout--;
	if( timeout == 0 ) return 0;

	spiControl->busy = 1;
	spiControl->mode = 1;
	spiControl->nbytes = nbytes;
	spiControl->p = buffer;

	/*
	 * Reads the SPI data register to make sure there is nothing there so
	 * that we can enable the RX interrupt and not trigger an erroneous
	 * interrupt.
	 */
	*buffer = (uint8_t)spi->DR;
	spi->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_TXEIE);

	return 0;
}
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
static int32_t spihlInitializeSW(SPI_TypeDef *spi){

	spihlControl_t *spiControl = 0;

	spiControl = spihlGetControlStruct(spi);
	if( spiControl == 0 ) return SPIHL_ERR_INVALID_SPI;

	spiControl->busy = 0;

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
//===========================================================================


//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
#ifdef SPIHL_CONFIG_SPI1_ENABLED
void SPI1_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void SPI1_IRQHandler(void){

	uint32_t spiStatus;

	gpioOutputSet(GPIOA, GPIO_P0);

	spiStatus = SPI1->SR;

	if( spihlSPI1Control.mode == 0 ){
		/* Transmitter ready */
		if( (SPI1->CR2 & SPI_CR2_TXEIE) && (spiStatus & SPI_SR_TXE) ){
			if( spihlSPI1Control.nbytes != 0 ){
				spihlSPI1Control.nbytes--;
				SPI1->DR = (uint16_t)*spihlSPI1Control.p++;
			}
			else{
				/* Disables TX interrupt if all bytes were sent */
				SPI1->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
				spihlSPI1Control.busy = 0;
			}
		} // if( (SPI1->CR2 & SPI_CR2_TXEIE) && (spiStatus & SPI_SR_TXE) )
	} // if( spihlSPI1Control.mode == 0 )

	else{
		/* Transmitter ready */
		if( (SPI1->CR2 & SPI_CR2_TXEIE) && (spiStatus & SPI_SR_TXE) ){
			if( spihlSPI1Control.nbytes != 0 ){
				SPI1->DR = (uint16_t)0xFF;
				spihlSPI1Control.nbytes--;
			}
			else{
				/* Disables RX and TX interrupt if all bytes were received */
				SPI1->CR2 &= (uint16_t)(~SPI_CR2_TXEIE);
			}
		}
		/* Data received */
		else if( (SPI1->CR2 & SPI_CR2_RXNEIE) && (spiStatus & SPI_SR_RXNE) ){
			*spihlSPI1Control.p++ = (uint8_t) SPI1->DR;
			if( (SPI1->SR & SPI_SR_BSY) == 0 ){
				SPI1->CR2 &= (uint16_t)(~SPI_CR2_RXNEIE);
				spihlSPI1Control.busy = 0;
			}
		} // else if( (SPI1->CR2 & SPI_CR2_RXNEIE) && (spiStatus & SPI_SR_RXNE) )
	} // else -> if( spihlSPI1Control.mode == 0 )

	gpioOutputReset(GPIOA, GPIO_P0);

}
#endif
//---------------------------------------------------------------------------
//===========================================================================
