/*
 * @file i2chl.h
 * @brief Provides an I2C driver for STM32F103 devices.
 *
 * Only works in master mode, without multiple masters on the bus.
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
#define I2CHL_CONFIG_I2C1_ENABLED			 	/**< Enables I2C1. */
#define I2CHL_CONFIG_I2C2_ENABLED			 	/**< Enables I2C2. */

//#define I2CHL_CONFIG_I2C1_RTOS_EN				/**< Enables FreeRTOS integration for I2C1. */
#define I2CHL_CONFIG_I2C2_RTOS_EN				/**< Enables FreeRTOS integration for I2C2. */

/* Priority for I2C interrupt */
#define I2CHL_CONFIG_I2C1_NVIC_PRIO				0x06 /**< NVIC I2C1 priority. */
#define I2CHL_CONFIG_I2C2_NVIC_PRIO				0x06 /**< NVIC I2C2 priority. */

/* Error codes */
#define I2CHL_ERR_INVALID_I2C					-0x01 /**< Invalid I2C. */
#define I2CHL_ERR_BUSY							-0x02 /**< I2C busy. */
#define I2CHL_ERR_WAIT_TO						-0x03 /**< Timed-out while waiting. */
#define I2CHL_ERR_TX_0							-0x04 /**< Trying to transmit 0 bytes. */
#define I2CHL_ERR_RX_0							-0x05 /**< Trying to receive 0 bytes. */
#define I2CHL_BUSY_RECOVER_ERR					-0x06 /**< Couldn't recover from busy locked state. */

#if defined(I2CHL_CONFIG_I2C1_RTOS_EN) || defined(I2CHL_CONFIG_I2C2_RTOS_EN)
#define I2CHL_CONFIG_FREE_RTOS_ENABLED
#endif
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes the specified I2C.
 *
 * @param i2c I2C to be initialized.
 * @result 0 if I2C was initialized successfully, otherwise an error code.
 */
int32_t i2chlInitialize(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
/**
 * @brief Sends data through the specified I2C.
 *
 * This function will return immediately, and the data will be sent from the
 * buffer through an interrupt mechanism.
 *
 * @param i2c I2C to send data.
 * @param address Slave address.
 * @param buffer Pointer to buffer holding data to be transmitted.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait in case the I2C is busy. CPU is blocked
 * 		  during this time.
 * @result 0 if successful, otherwise and error code.
 */
int32_t i2chlWrite(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the specified I2C.
 *
 * This function will return immediately, and the data will be saved to the
 * buffer through an interrupt mechanism.
 *
 * @param i2c I2C to read data.
 * @param address Slave address.
 * @param buffer Pointer to buffer to hold the data read.
 * @param nbytes Number of bytes to read.
 * @param timeout Timeout to wait in case the I2C is busy. CPU is blocked
 * 		  during this time.
 * @result 0 if successful, otherwise and error code.
 */
int32_t i2chlRead(I2C_TypeDef *i2c, uint8_t address, uint8_t *buffer,
		uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Waits until the specified I2C becomes free.
 *
 *
 * @param i2c I2C to wait.
 * @param timeout Timeout.
 * @result 0 if I2C became free before the timeout expired, 1 otherwise.
 */
int32_t i2chlWaitWhileBusy(I2C_TypeDef *i2c, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Returns the status of the last transaction.
 *
 * A value of 0 indicates that the last transaction was successful, and a
 * value of 1 indicates otherwise.
 *
 * @param i2c I2C to check status.
 * @result 0 if last transaction was successful, 1 otherwise.
 */
int32_t i2chlStatusLastTransaction(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
/**
 * @brief Attempts to recover the I2C peripheral from a wrong busy condition.
 *
 * According to the errata, the I2C busy flag may lock on a busy state, even
 * though the data and clock lines are free. This function performs the
 * procedure given in the erratasheet, in order to recover from this locked
 * condition.
 */
int32_t i2chlBusyRecover(I2C_TypeDef *i2c);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* DRIVERS_I2CHL_H_ */
