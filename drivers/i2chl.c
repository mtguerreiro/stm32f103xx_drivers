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

#ifdef I2CHL_CONFIG_FREE_RTOS_ENABLED
/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"
#endif
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================

//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{

	/*
	 * Indicates if I2C is busy sending/receiving data. This is not the same
	 * as the hardware busy flag. A value of 1 indicates that I2C is busy and
	 * a value of zero indicates that the I2C is free.
	 */
	volatile uint8_t busy;

	/*
	 * Number of bytes for the current operation. If writing, this is the
	 * number of bytes to be written. If reading, this is the number of
	 * bytes to be read.
	 */
	uint32_t nbytes;

	/*
	 * Buffer to write data to or read data from. This pointer changes values
	 * as it is incremented.
	 */
	uint8_t *p;

	/*
	 * Slave address.
	 */
	uint8_t slaveAddress;

	/* I2C busy semaphore is given when the I2C is about to become free */
#ifdef I2CHL_CONFIG_FREE_RTOS_ENABLED
	SemaphoreHandle_t semaphore;
#endif
}i2chlControl_t;
//===========================================================================


//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes the HW for the specified I2C.
 *
 * @param i2c I2C to be initialized.
 * @result 0 if I2C H@ was initialized successfully, otherwise an error code.
 */
static int32_t i2chlInitializeHW(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
/**
 * @brief Initializes the SW for the specified I2C.
 *
 * @param i2c I2C to be initialized.
 * @result 0 if I2C SW was initialized successfully, otherwise an error code.
 */
static int32_t i2chlInitializeSW(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
/**
 * @brief Gets pointer for the control structure the specified I2C.
 *
 * @param i2c I2C.
 * @result Pointer to structure or 0 if structure was not found.
 */
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
int32_t i2chlInitialize(I2C_TypeDef *i2c){

	int32_t ret;

	ret = i2chlInitializeHW(i2c);
	if( ret != 0 ) return ret;

	ret = i2chlInitializeSW(i2c);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t i2chlWrite(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout){

	i2chlControl_t *i2chlControl = 0;

	i2chlControl = i2chlGetControlStruct(i2c);
	if( i2chlControl == 0 ) return I2CHL_ERR_INVALID_I2C;

	if( nbytes == 0 ) return I2CHL_ERR_TX_0;

	while( ((i2c->SR2 & I2C_SR2_BUSY) || (i2chlControl->busy == 1)) && (timeout != 0) ) timeout--;
	if( timeout == 0 ) return I2CHL_ERR_BUSY;

	//if( i2chlWaitWhileBusy(i2c, timeout) != 0 ) return I2CHL_ERR_BUSY;

	i2chlControl->slaveAddress = (uint8_t)(address << 1U);
	i2chlControl->busy = 1;
	i2chlControl->nbytes = nbytes;
	i2chlControl->p = buffer;

	/* Enables TX/RX interrupt */
	i2c->CR2 |= I2C_CR2_ITBUFEN;

	/* Generates start condition */
	i2c->CR1 |= I2C_CR1_START;

	return 0;
}
//---------------------------------------------------------------------------
int32_t i2chlRead(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout){

	i2chlControl_t *i2chlControl = 0;

	i2chlControl = i2chlGetControlStruct(i2c);
	if( i2chlControl == 0 ) return I2CHL_ERR_INVALID_I2C;

	if( nbytes == 0 ) return I2CHL_ERR_RX_0;

	while( ((i2c->SR2 & I2C_SR2_BUSY) || (i2chlControl->busy == 1)) && (timeout != 0) ) timeout--;
	if( timeout == 0 ) return I2CHL_ERR_BUSY;

	//if( i2chlWaitWhileBusy(i2c, timeout) != 0 ) return I2CHL_ERR_BUSY;

	i2chlControl->slaveAddress = ((uint8_t)(address << 1U) | 0x01U);
	i2chlControl->busy = 1;
	i2chlControl->nbytes = nbytes;
	i2chlControl->p = buffer;

	/* Enables TX/RX interrupt */
	i2c->CR2 |= I2C_CR2_ITBUFEN;

	/* Generates start condition */
	i2c->CR1 |= I2C_CR1_START;

	return 0;
}
//---------------------------------------------------------------------------
int32_t i2chlWaitWhileBusy(I2C_TypeDef *i2c, uint32_t timeout){

	i2chlControl_t *i2chlControl = 0;

	i2chlControl = i2chlGetControlStruct(i2c);
	if( i2chlControl == 0 ) return I2CHL_ERR_INVALID_I2C;

#ifdef I2CHL_CONFIG_FREE_RTOS_ENABLED
	if( i2chlControl->semaphore == 0 ){
		while( ((i2c->SR2 & I2C_SR2_BUSY) || (i2chlControl->busy == 1)) && (timeout != 0) ) timeout--;
		if( timeout == 0 ) return I2CHL_ERR_WAIT_TO;
	}
	else{
		if( xSemaphoreTake(i2chlControl->semaphore, timeout) != pdTRUE ) return I2CHL_ERR_WAIT_TO;
		/*
		 * When the semaphore is released, either I2C hardware has to send
		 * the last byte, wait for ack and send the stop condition, or it
		 * still has to send nack and the stop condition. Thus, after the
		 * semaphore is released, we still have to wait a bit longer.
		 */
		timeout = 0x1FFF;
		while( (i2c->SR2 & I2C_SR2_BUSY) && (timeout != 0) ) timeout--;
		if( timeout == 0 ) return I2CHL_ERR_WAIT_TO;
	}
#else
	while( ((i2c->SR2 & I2C_SR2_BUSY) || (i2chlControl->busy == 1)) && (timeout != 0) ) timeout--;
	if( timeout == 0 ) return I2CHL_ERR_WAIT_TO;
#endif

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

	switch(_i2c){

#ifdef I2CHL_CONFIG_I2C1_ENABLED
	case I2C1_BASE:
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

		break;
#endif

#ifdef I2CHL_CONFIG_I2C2_ENABLED
	case I2C2_BASE:
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

		break;
#endif

	default:
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
static int32_t i2chlInitializeSW(I2C_TypeDef *i2c){

#ifdef I2CHL_CONFIG_FREE_RTOS_ENABLED

#ifdef I2CHL_CONFIG_I2C1_ENABLED
#ifdef I2CHL_CONFIG_I2C1_RTOS_EN
	i2chlI2C1Control.semaphore = xSemaphoreCreateBinary();
	if( i2chlI2C1Control.semaphore == NULL ) return 1;
	xSemaphoreGive(i2chlI2C1Control.semaphore);
#else
	i2chlI2C1Control.semaphore = 0;
#endif
#endif

#ifdef I2CHL_CONFIG_I2C2_ENABLED
#ifdef I2CHL_CONFIG_I2C2_RTOS_EN
	i2chlI2C2Control.semaphore = xSemaphoreCreateBinary();
	if( i2chlI2C2Control.semaphore == NULL ) return 1;
	xSemaphoreGive(i2chlI2C2Control.semaphore);
#else
	i2chlI2C2Control.semaphore = 0;
#endif
#endif

#endif

	return 0;
}
//---------------------------------------------------------------------------
static i2chlControl_t* i2chlGetControlStruct(I2C_TypeDef *i2c){

	uint32_t _i2c = (uint32_t)i2c;
	i2chlControl_t *i2chlControl = 0;

	switch (_i2c){

#ifdef I2CHL_CONFIG_I2C1_ENABLED
	case I2C1_BASE:
		i2chlControl = &i2chlI2C1Control;
		break;
#endif

#ifdef I2CHL_CONFIG_I2C2_ENABLED
	case I2C2_BASE:
		i2chlControl = &i2chlI2C2Control;
		break;
#endif

	}

	return i2chlControl;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
#ifdef I2CHL_CONFIG_I2C1_ENABLED
void I2C1_EV_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void I2C1_EV_IRQHandler(void){

	uint16_t sr1, sr2;

	gpioOutputSet(GPIOA, GPIO_P0);

	sr1 = I2C1->SR1;

	if( sr1 & I2C_SR1_SB ){
		/* Start-condition sent, now send address */
		I2C1->DR = i2chlI2C1Control.slaveAddress;
	}
	else if( sr1 & I2C_SR1_ADDR ){
		/*
		 * Address sent, now start transmission or reception. In addition, we
		 * read the SR2 register, in order to clear the flag, per manual.
		 */
		sr2 = I2C1->SR2;
		if( i2chlI2C1Control.slaveAddress & 0x01 ){
			/*
			 * If we are just looking to receive one byte, we'll set set the
			 * stop condition already, according to the reference manual.
			 * Otherwise, we set the ack bit.
			 */
			if( i2chlI2C1Control.nbytes == 1 ){
				I2C1->CR1 &= (uint16_t)(~I2C_CR1_ACK);
				I2C1->CR1 |= I2C_CR1_STOP;
			}
			else{
				I2C1->CR1 &= (uint16_t)(~I2C_CR1_STOP);
				I2C1->CR1 |= I2C_CR1_ACK;
			}
		}
		else{
			/* Transmitter mode. Address sent, now send data */
			I2C1->DR = *i2chlI2C1Control.p++;
			i2chlI2C1Control.nbytes--;
		}
	}
	else if( sr1 & I2C_SR1_TXE ){
		if( i2chlI2C1Control.nbytes == 0 ){
			/*
			 * If there are no more bytes, program the stop condition. We
			 * also write 0 to DR to clear the TX interrupt flag.
			 */
			I2C1->DR = 0;
			I2C1->CR2 &= (uint16_t)(~I2C_CR2_ITBUFEN);
			I2C1->CR1 |= I2C_CR1_STOP;
			i2chlI2C1Control.busy = 0;
#ifdef I2CHL_CONFIG_I2C1_RTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(i2chlI2C1Control.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Continues to send data */
			I2C1->DR = *i2chlI2C1Control.p++;
			i2chlI2C1Control.nbytes--;
		}
	}
	else if( sr1 & I2C_SR1_RXNE ){
		/* Receiver mode */
		*i2chlI2C1Control.p++ = (uint8_t)I2C1->DR;
		i2chlI2C1Control.nbytes--;

		if ( i2chlI2C1Control.nbytes == 0 ){
			/* Received all bytes, disable ER interrupt */
			i2chlI2C1Control.busy = 0;
#ifdef I2CHL_CONFIG_I2C1_RTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(i2chlI2C1Control.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		if( i2chlI2C1Control.nbytes == 1 ){
			/* One byte remaining to be received, sets stop condition */
			I2C1->CR1 &= (uint16_t)(~I2C_CR1_ACK);
			I2C1->CR1 |= I2C_CR1_STOP;
		}
	}

	gpioOutputReset(GPIOA, GPIO_P0);
}
#endif
//---------------------------------------------------------------------------
#ifdef I2CHL_CONFIG_I2C2_ENABLED
void I2C2_EV_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void I2C2_EV_IRQHandler(void){

	uint16_t sr1, sr2;

	//gpioOutputSet(GPIOA, GPIO_P0);

	sr1 = I2C2->SR1;

	if( sr1 & I2C_SR1_SB ){
		/* Start-condition sent, now send address */
		I2C2->DR = i2chlI2C2Control.slaveAddress;
	}
	else if( sr1 & I2C_SR1_ADDR ){
		/*
		 * Address sent, now start transmission or reception. In addition, we
		 * read the SR2 register, in order to clear the flag, per manual.
		 */
		sr2 = I2C2->SR2;
		if( i2chlI2C2Control.slaveAddress & 0x01 ){
			/*
			 * If we are just looking to receive one byte, we'll set set the
			 * stop condition already, according to the reference manual.
			 * Otherwise, we set the ack bit.
			 */
			if( i2chlI2C2Control.nbytes == 1 ){
				I2C2->CR1 &= (uint16_t)(~I2C_CR1_ACK);
				I2C2->CR1 |= I2C_CR1_STOP;
			}
			else{
				I2C2->CR1 &= (uint16_t)(~I2C_CR1_STOP);
				I2C2->CR1 |= I2C_CR1_ACK;
			}
		}
		else{
			/* Transmitter mode. Address sent, now send data */
			I2C2->DR = *i2chlI2C2Control.p++;
			i2chlI2C2Control.nbytes--;
		}
	}
	else if( sr1 & I2C_SR1_TXE ){
		if( i2chlI2C2Control.nbytes == 0 ){
			/*
			 * If there are no more bytes, program the stop condition. We
			 * also write 0 to DR to clear the TX interrupt flag.
			 */
			I2C2->DR = 0;
			I2C2->CR2 &= (uint16_t)(~I2C_CR2_ITBUFEN);
			I2C2->CR1 |= I2C_CR1_STOP;
			i2chlI2C2Control.busy = 0;
#ifdef I2CHL_CONFIG_I2C2_RTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(i2chlI2C2Control.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Continues to send data */
			I2C2->DR = *i2chlI2C2Control.p++;
			i2chlI2C2Control.nbytes--;
		}
	}
	else if( sr1 & I2C_SR1_RXNE ){
		/* Receiver mode */
		*i2chlI2C2Control.p++ = (uint8_t)I2C2->DR;
		i2chlI2C2Control.nbytes--;

		if ( i2chlI2C2Control.nbytes == 0 ){
			/* Received all bytes, disable ER interrupt */
			i2chlI2C2Control.busy = 0;
#ifdef I2CHL_CONFIG_I2C2_RTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(i2chlI2C2Control.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		if( i2chlI2C2Control.nbytes == 1 ){
			/* One byte remaining to be received, sets stop condition */
			I2C2->CR1 &= (uint16_t)(~I2C_CR1_ACK);
			I2C2->CR1 |= I2C_CR1_STOP;
		}
	}

	//gpioOutputReset(GPIOA, GPIO_P0);
}
#endif
//---------------------------------------------------------------------------
//===========================================================================
