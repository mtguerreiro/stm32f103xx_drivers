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
#define DS3231_ERR_INVALID_SEC		-0x0B /**< Invalid value for seconds. */

/* DS3231 address map */
#define DS3231_ADD_SECONDS			0x00 /**< Upper byte of the temperature register. */
#define DS3231_ADD_TEMP_UB			0x11 /**< Upper byte of the temperature register. */
#define DS3231_ADD_CONTROL			0x0E /**< Control register. */
#define DS3231_ADD_STATUS			0x0F /**< Status register. */

/* DS3231 status bits */
#define DS3231_STATUS_OSF			(1U << 7) /**< Oscillator stop flag. */
#define DS3231_STATUS_EN32KHZ		(1U << 3) /**< Enable 32kHz output flag. */
#define DS3231_STATUS_BSY			(1U << 2) /**< Busy flag. */
#define DS3231_STATUS_A2F			(1U << 1) /**< Alarm 2 flag. */
#define DS3231_STATUS_A1F			(1U << 0) /**< Alarm 1 flag. */

/* DS3231 control bits */
#define DS3231_CONTROL_EOSC			(1U << 7) /**< Enable oscillator flag. */
#define DS3231_CONTROL_BBSQW		(1U << 6) /**< Battery-backed square-wave enable flag. */
#define DS3231_CONTROL_CONV			(1U << 5) /**< Convert temperature flag. */
#define DS3231_CONTROL_RS2			(1U << 4) /**< Rate select 2 flag. */
#define DS3231_CONTROL_RS1			(1U << 3) /**< Rate select 1 flag. */
#define DS3231_CONTROL_INTCN		(1U << 2) /**< Interrupt control flag. */
#define DS3231_CONTROL_A2IE			(1U << 1) /**<  Alarm 2 interrupt enable flag. */
#define DS3231_CONTROL_A1IE			(1U << 0) /**<  Alarm 1 interrupt enable flag. */

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
/**
 * @brief Reads the control register.
 *
 * @param buffer Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231ControlRead(uint8_t *buffer, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Clears the selected bits of the control register.
 *
 * The CONV bit is ignored by this function.
 *
 * @param bits Bits to be cleared.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231ControlBitsClear(uint8_t bits, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the selected bits of the control register.
 *
 * @param bits Bits to be cleared.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231ControlBitsSet(uint8_t bits, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the temperature register.
 *
 * This function just reads the temperature already converted, i.e. no new
 * conversion is started prior to reading the temperature registers.
 *
 * @param buffer Buffer to save temperature.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231TemperatureRead(int8_t *temp, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads seconds.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231SecondsRead(uint8_t *sec, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets seconds.
 *
 * @param sec Value to set. Must be between 0 and 59.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231SecondsSet(uint8_t sec, uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* LIBS_DS3231_H_ */
