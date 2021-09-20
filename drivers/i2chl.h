/*
 * i2chl.h
 *
 *  Created on: 18 de set de 2021
 *      Author: marco
 */

#ifndef DRIVERS_I2CHL_H_
#define DRIVERS_I2CHL_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
/* Enable I2Cs */
#define I2CHL_CONFIG_I2C1_ENABLED			 		/**< Enables I2C1. */
#define I2CHL_CONFIG_I2C2_ENABLED			 		/**< Enables I2C2. */

/* Priority for I2C interrupt */
#define I2CHL_CONFIG_I2C1_NVIC_PRIO				0x06 /**< NVIC I2C1 priority. */
#define I2CHL_CONFIG_I2C2_NVIC_PRIO				0x06 /**< NVIC I2C2 priority. */

/* Error codes */
#define I2CHL_ERR_INVALID_I2C					-0x01 /**< Invalid I2C. */
#define I2CHL_ERR_TX_BUFFER_SIZE				-0x02 /**< Not enough space in TX buffer. */
#define I2CHL_ERR_TX							-0x03 /**< Couldn't enqueue TX data. */
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t i2chlInitialize(I2C_TypeDef *i2c,\
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize);
//---------------------------------------------------------------------------
int32_t i2chlWrite(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_I2CHL_H_ */
