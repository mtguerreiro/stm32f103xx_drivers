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
#define DS3231_ERR_INIT				-1 /**< Failed to initialize DS3231. */
#define DS3231_ERR_TX				-2 /**< Transmission error. */
#define DS3231_ERR_TX_TO			-3 /**< Timed-out before transmission was completed. */
#define DS3231_ERR_RX				-4 /**< Reception error. */
#define DS3231_ERR_RX_TO			-5 /**< Timed-out before reception was completed. */
#define DS3231_ERR_WRITE_TO			-6 /**< Write time-out. */
#define DS3231_ERR_READ_TO			-7 /**< Read time-out. */
#define DS3231_ERR_STATUS_READ		-8 /**< Error reading status. */
#define DS3231_ERR_STATUS_WRITE 	-9 /**< Error writing status. */
#define DS3231_ERR_CMD				-10 /**< Failed to complete command. */
#define DS3231_ERR_SET_SECONDS		-11 /**< Invalid value for seconds. */
#define DS3231_ERR_SET_MINUTES		-12 /**< Invalid value for minutes. */
#define DS3231_ERR_SET_HOUR			-13 /**< Invalid value for hour. */
#define DS3231_ERR_SET_DAYOFWEEK	-14 /**< Invalid value for day of the week. */
#define DS3231_ERR_SET_DAY			-15 /**< Invalid value for day. */
#define DS3231_ERR_SET_MONTH		-16 /**< Invalid value for month. */
#define DS3231_ERR_SET_YEAR			-17 /**< Invalid value for month. */
#define DS3231_ERR_TIME_SEC			-18 /**< Error setting/reading seconds. */
#define DS3231_ERR_TIME_MIN			-19 /**< Error setting/reading minutes. */
#define DS3231_ERR_TIME_HOUR		-20 /**< Error setting/reading hour. */
#define DS3231_ERR_DATE_DOW			-21 /**< Error setting/reading day of the week. */
#define DS3231_ERR_DATE_DAY			-22 /**< Error setting/reading day. */
#define DS3231_ERR_DATE_MONTH		-23 /**< Error setting/reading month. */
#define DS3231_ERR_DATE_YEAR		-24 /**< Error setting/reading year. */

/* DS3231 address map */
#define DS3231_ADD_SECONDS			0x00 /**< Seconds register. */
#define DS3231_ADD_MINUTES			0x01 /**< Minutes register. */
#define DS3231_ADD_HOUR				0x02 /**< Minutes register. */
#define DS3231_ADD_DAYOFWEEK		0x03 /**< Day of the week register. */
#define DS3231_ADD_DAY				0x04 /**< Day register. */
#define DS3231_ADD_MONTH			0x05 /**< Date register. */
#define DS3231_ADD_YEAR				0x06 /**< Date register. */
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

typedef struct{
	/* Time */
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;

	/* Date */
	uint8_t dayofweek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
}ds3231DateTime_t;
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
 * Also, only the integer part of the temperature is read. The decimal part
 * is ignored.
 *
 * @param buffer Buffer to save temperature.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231TemperatureRead(int8_t *temp, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the seconds register.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231SecondsRead(uint8_t *sec, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the seconds register.
 *
 * @param sec Value to set. Must be between 0 and 59.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231SecondsSet(uint8_t sec, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the minutes register.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231MinutesRead(uint8_t *min, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the minutes register.
 *
 * @param sec Value to set. Must be between 0 and 59.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231MinutesSet(uint8_t min, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the hours register.
 *
 * Hour is always read in 24-hour format.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231HoursRead(uint8_t *hour, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the hours register.
 *
 * Hour is always written in 24-hour format.
 *
 * @param sec Value to set. Must be between 0 and 23.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231HoursSet(uint8_t hour, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the day of the week register.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DayOfWeekRead(uint8_t *day, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the day of the week register.
 *
 * @param sec Value to set. Must be between 1 and 7.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DayOfWeekSet(uint8_t day, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the day register.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DayRead(uint8_t *day, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the day register.
 *
 * @param sec Value to set. Must be between 1 and 31.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DaySet(uint8_t day, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the month register.
 *
 * Obs.: century is ignored.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231MonthRead(uint8_t *month, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the month register.
 *
 * Obs.: century is ignored.
 *
 * @param sec Value to set. Must be between 1 and 12.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231MonthSet(uint8_t month, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the year register.
 *
 * The year is read as a value between 0 and 99. The actual year will then be
 * 2000 + the read value.
 *
 * @param sec Buffer to save data to.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231YearRead(uint8_t *year, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the year register.
 *
 * The year is written as a value between 0 and 99. The actual year will then
 * be 2000 + the written value.
 *
 * @param sec Value to set. Must be between 0 and 99.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231YearSet(uint8_t year, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the time.
 *
 * Reading the time consists of reading the seconds, minutes and hour.
 *
 * @param time Date-time structure to save the time.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231TimeRead(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the time.
 *
 * Setting the time consists of setting the seconds, minutes and hour.
 *
 * @param time Date-time structure containing the time to be set.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231TimeSet(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the date.
 *
 * Reading the date consists of reading the day of the week, day, month and
 * year.
 *
 * @param time Date-time structure to save the date.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DateRead(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the date.
 *
 * Setting the date consists of reading the day of the week, day, month and
 * year.
 *
 * @param time Date-time structure containing the date to be set.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DateSet(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads the date and time.
 *
 * Reading the date and time consists of reading the seconds, minutes, hour,
 * day of the week, day, month and year.
 *
 * @param time Date-time structure to save the date and time.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DateTimeRead(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets the date and time.
 *
 * Sets the date and time consists of reading the seconds, minutes, hour,
 * day of the week, day, month and year.
 *
 * @param time Date-time structure containing the date and time to be set.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 of command was successful, an error code otherwise.
 */
int32_t ds3231DateTimeSet(ds3231DateTime_t *datetime, uint32_t timeout);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* LIBS_DS3231_H_ */
