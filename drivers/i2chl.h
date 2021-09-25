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
#define I2CHL_ERR_BUSY							-0x02 /**< I2C busy. */
#define I2CHL_ERR_TX_0							-0x03 /**< Trying to transmit 0 bytes. */
#define I2CHL_ERR_RX_0							-0x04 /**< Trying to receive 0 bytes. */
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t i2chlInitialize(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
int32_t i2chlWrite(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
int32_t i2chlRead(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_I2CHL_H_ */
