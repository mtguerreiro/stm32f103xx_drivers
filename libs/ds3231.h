/*
 * ds3231.h
 *
 *  Created on: 28 de set de 2021
 *      Author: marco
 */

#ifndef LIBS_DS3231_H_
#define LIBS_DS3231_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//===========================================================================


//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================

//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
#define DS3231_CONFIG_I2C			I2C2 /**< Sets the I2C used for communication.*/
#define DS3231_CONFIG_ADDRESS		0x68 /**< DS3231 address.*/

/* Error codes */
#define DS3231_ERR_INIT				-0x01 /**< Failed to initialize DS3231. */
#define DS3231_ERR_TX				-0x02 /**< Transmission error. */
#define DS3231_ERR_TX_TO			-0x03 /**< Timed-out before transmission was completed. */
#define DS3231_ERR_RX				-0x04 /**< Reception error. */
#define DS3231_ERR_RX_TO			-0x05 /**< Timed-out before reception was completed. */
#define DS3231_ERR_WRITE_TO			-0x06 /**< Write time-out. */
#define DS3231_ERR_READ_TO			-0x07 /**< Read time-out. */
#define DS3231_ERR_STATUS_READ		-0x08 /**< Error reading status. */
#define DS3231_ERR_STATUS_WRITE 	-0x09 /**< Error writing status. */
#define DS3231_ERR_CMD				-0x0A /**< Failed to complete command. */

/* DS3231 address map */
#define DS3231_ADD_STATUS			0x0F /**< Status register */

/* DS3231 status bits */
#define DS3231_STATUS_EN32KHZ_OFFS	(3)
#define DS3231_STATUS_OSF_OFFS		(7)

#define DS3231_STATUS_EN32KHZ		(1U << DS3231_STATUS_EN32KHZ_OFFS)
#define DS3231_STATUS_OSF			(1U << DS3231_STATUS_OSF_OFFS)
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes DS3231.
 *
 * Basically, just the I2C is initialized.
 *
 * @result 0 of initialization was successful, an error code otherwise.
 */
int32_t ds3231Initialize(void);
//---------------------------------------------------------------------------
/**
 * @brief Reads DS3231's status register.
 *
 * @param status Buffer to save status.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231StatusRead(uint8_t *status, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads EN32kHz flag.
 *
 * @param status Buffer to save flag.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231EN32kHzRead(uint8_t *en32kHz, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Clears EN32kHz flag.
 *
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231EN32kHzClear(uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets EN32kHz flag.
 *
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231EN32kHzSet(uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads OSF flag.
 *
 * @param osf Buffer to save flag.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231OSFRead(uint8_t *osf, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Clears OSF flag.
 *
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231OSFClear(uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* LIBS_DS3231_H_ */
