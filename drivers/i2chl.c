/*
 * i2chl.c
 *
 *  Created on: 18 de set de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <i2chl.h>

/* Drivers */
#include "gpio.h"

/* Libs */
#include "cqueue.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================

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
}i2chlControl_t;
//===========================================================================


//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t i2chlInitializeHW(I2C_TypeDef *i2c);
static int32_t i2chlInitializeSW(I2C_TypeDef *i2c,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
static i2chlControl_t* i2chlGetControlStruct(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
#ifdef I2CHL_CONFIG_I2C1_ENABLED
i2chlControl_t i2chlI2C1Control;
#endif

#ifdef I2CHL_CONFIG_I2C2_ENABLED
i2chlControl_t i2chlI2C2Control;
#endif
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t i2chlInitialize(I2C_TypeDef *i2c,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret;

	ret = i2chlInitializeHW(i2c);
	if( ret != 0 ) return ret;

	ret = i2chlInitializeSW(i2c, rxBuffer, rxBufferSize, txBuffer, txBufferSize);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t i2chlWrite(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes){

	i2chlControl_t *i2chlControl = 0;
	uint8_t *p;
	uint16_t space;

	i2chlControl = i2chlGetControlStruct(i2c);
	if( i2chlControl == 0 ) return I2CHL_ERR_INVALID_I2C;

	space = cqueueSpace(&i2chlControl->txQueue);
	if( space < (nbytes + 1) ) return I2CHL_ERR_TX_BUFFER_SIZE;

	address = (uint8_t)(address << 1);
	if( cqueueAdd(&i2chlControl->txQueue, &address) != 0 ) return I2CHL_ERR_TX;

	p = buffer;
	while( nbytes-- ){
		if( cqueueAdd(&i2chlControl->txQueue, p++) != 0 ) return I2CHL_ERR_TX;
	}

	/* Generates start condition */
	i2c->CR1 |= I2C_CR1_START;

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t i2chlInitializeHW(I2C_TypeDef *i2c){

	GPIO_TypeDef *portSDA = 0;
	uint16_t portSDAPin = 0;
	GPIO_TypeDef *portSCL = 0;
	uint16_t portSCLPin = 0;
	IRQn_Type irqn = (IRQn_Type) 0;
	IRQn_Type irqnPrio = (IRQn_Type) 0;

	uint32_t _i2c = (uint32_t)i2c;

	if( _i2c == I2C1_BASE ){
		/* Enables clock to I2C1 peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 21);

		/* Selects GPIO pins*/
		portSDA = GPIOB;
		portSDAPin = GPIO_P7;
		portSCL = GPIOB;
		portSCLPin = GPIO_P6;

		/* IRQ priority */
		irqn = I2C1_EV_IRQn;
		irqnPrio = (IRQn_Type) I2CHL_CONFIG_I2C1_NVIC_PRIO;
	}
	else if( _i2c == I2C2_BASE ){
		/* Enables clock to I2C2 peripheral */
		RCC->APB1ENR |= (uint32_t)(1U << 22);

		/* Selects GPIO pins*/
		portSDA = GPIOB;
		portSDAPin = GPIO_P11;
		portSCL = GPIOB;
		portSCLPin = GPIO_P10;

		/* IRQ priority */
		irqn = I2C2_EV_IRQn;
		irqnPrio = (IRQn_Type) I2CHL_CONFIG_I2C2_NVIC_PRIO;
	}
	else{
		return I2CHL_ERR_INVALID_I2C;
	}

	/* Sets GPIO pins */
	gpioPortEnable(portSCL);
	if(portSDA != portSCL) gpioPortEnable(portSDA);
	gpioConfig(portSDA, portSDAPin, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_OPEN_DRAIN);
	gpioConfig(portSCL, portSCLPin, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_AF_OPEN_DRAIN);

	/* Sets NVIC priority */
	NVIC_SetPriority(irqn, irqnPrio);
	NVIC_EnableIRQ(irqn);

	/*
	 * Puts the I2C peripheral under reset and sets it. CR2 is set with the
	 * clock frequency supplied by APB to the I2C peripheral. Note that both
	 * I2C1 and I2C2 are clocked from APB1, which runs at a maximum of
	 * 36 MHz. So, we set that as the frequency of the I2C peripheral.
	 */
	i2c->CR1 = I2C_CR1_SWRST;
	i2c->CR1 = 0;
	i2c->CR2 = I2C_CR2_ITEVTEN | (36);

	/*
	 * Sets the CCR with a value of 180, such that the clock frequency for
	 * if the I2C bus is 100 kHz
	 */
	i2c->CCR = 90;
	//i2c->CCR = 180;

	/*
	 * Sets the maximum rise time of the SCL line. For now, it is hard-coded
	 * as 1us.
	 */
	i2c->TRISE = 36;

	/* Enables the peripheral */
	i2c->CR1 = I2C_CR1_PE;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t i2chlInitializeSW(I2C_TypeDef *i2c,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	i2chlControl_t *i2cControl = 0;

	i2cControl = i2chlGetControlStruct(i2c);
	if( i2cControl == 0 ) return I2CHL_ERR_INVALID_I2C;

	cqueueInitialize(&i2cControl->rxQueue, rxBuffer, rxBufferSize);
	cqueueInitialize(&i2cControl->txQueue, txBuffer, txBufferSize);

	return 0;
}
//---------------------------------------------------------------------------
static i2chlControl_t* i2chlGetControlStruct(I2C_TypeDef *i2c){

	uint32_t _i2c = (uint32_t)i2c;
	i2chlControl_t *i2chlControl = 0;

	switch (_i2c){

	case I2C1_BASE:
		i2chlControl = &i2chlI2C1Control;
		break;
	case I2C2_BASE:
		i2chlControl = &i2chlI2C2Control;
		break;
	}

	return i2chlControl;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void I2C1_EV_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void I2C1_EV_IRQHandler(void){

	uint8_t txData, rxData;
	uint16_t sr1, sr2;

	gpioOutputSet(GPIOA, GPIO_P0);

	sr1 = I2C1->SR1;

	if( sr1 & I2C_SR1_SB ){
		if( cqueueRemove(&i2chlI2C1Control.txQueue, &txData) == 0 ){
			I2C1->DR = txData;
		}
	}
	else if( sr1 & I2C_SR1_ADDR ){
		sr2 = I2C1->SR2;
		if( cqueueRemove(&i2chlI2C1Control.txQueue, &txData) == 0 ){
			I2C1->DR = txData;
		}
	}
	else if( sr1 & I2C_SR1_BTF ){
		if( cqueueRemove(&i2chlI2C1Control.txQueue, &txData) == 0 ){
			I2C1->DR = txData;
		}
		else{
			rxData = (uint8_t)I2C1->DR;
			I2C1->CR1 |= I2C_CR1_STOP;
		}
	}

	gpioOutputReset(GPIOA, GPIO_P0);

//	usartStatus = USART1->SR;
//
//	/* Data received */
//	if( usartStatus & USART_SR_RXNE ){
//		rxData = (uint8_t) USART1->DR;
//		cqueueAdd(&uarthlUART1Control.rxQueue, &rxData);
//#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(uarthlUART1Control.rxSemph, &xHigherPriorityTaskWoken);
//		if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//	} //if( usartStatus & USART_SR_RXNE )
//
//	/* Transmitter ready */
//	else if( usartStatus & USART_SR_TXE ){
//		if( cqueueRemove(&uarthlUART1Control.txQueue, &txData) == 0 ){
//			USART1->DR = (uint16_t)txData;
//#ifdef UARTHL_CONFIG_FREE_RTOS_ENABLED
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			xSemaphoreGiveFromISR(uarthlUART1Control.txSemph, &xHigherPriorityTaskWoken);
//			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//		}
//		else{
//			/* Disables TX interrupt if queue is empty */
//			USART1->CR1 &= (uint16_t)(~USART_CR1_TXEIE);
//		}
//	}//else if( usartStatus & USART_SR_TXE )
}
//---------------------------------------------------------------------------
//===========================================================================
