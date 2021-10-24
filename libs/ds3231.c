/*
 * ds3231.c
 *
 *  Created on: 28 de set de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "ds3231.h"

/* Drivers */
#include "i2chl.h"
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Sends data to the DS3231.
 *
 * A sufficient timeout must be specified.
 *
 * @param buffer Pointer to buffer holding data.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait until transmission is completed.
 * @result 0 if data transmission started, an error code otherwise.
 */
static int32_t ds3231Write(uint8_t *buffer, uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads data from the DS3231.
 *
 * A sufficient timeout must be specified.
 *
 * @param buffer Pointer to buffer holding data.
 * @param nbytes Number of bytes to send.
 * @param timeout Timeout to wait until reception is completed.
 * @result 0 if data transmission started, an error code otherwise.
 */
static int32_t ds3231Read(uint8_t *buffer, uint16_t nbytes, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Sets bit of a register
 *
 * @param reg Register to set bits.
 * @param bits Bits to set.
 * @timeout Timeout to wait until procedure is completed.
 * @result 0 if command was successful, an error code otherwise.
 */
static int32_t ds3231RegisterBitsSet(uint8_t reg, uint8_t bits, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Clears bit of a register
 *
 * @param reg Register to clear bits.
 * @param bits Bits to clear.
 * @timeout Timeout to wait until procedure is completed.
 * @result 0 if command was successful, an error code otherwise.
 */
static int32_t ds3231RegisterBitsClear(uint8_t reg, uint8_t bits, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Reads a register.
 *
 * @param reg Register to be read.
 * @param data Buffer to save data.
 * @param timeout Timeout to wait until procedure is completed.
 * @result 0 if data was read, an error code otherwise.
 */
static int32_t ds3231RegisterRead(uint8_t reg, uint8_t *data, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Writes to a register.
 *
 * @param reg Register to be written.
 * @param data Buffer containing the data to be written.
 * @param check Flag to check if written value was successful. If 0, no
 * 		  checking is performed. If 1, the written register is read and the
 * 		  result is compared to the value that was supposed to be written.
 * @result 0 if data was written, an error code otherwise.
 */
static int32_t ds3231RegisterWrite(uint8_t reg, uint8_t *data, uint8_t check, uint32_t timeout);
//---------------------------------------------------------------------------
/**
 * @brief Converts a decimal to the time format of the time registers.
 *
 * @param dec Decimal to be converted.
 * @result Converted register value.
 */
static uint8_t ds3231ConvertDecReg(uint8_t dec);
//---------------------------------------------------------------------------
/**
 * @brief Converts a register time value to the a decimal.
 *
 * @param val Register value to be converter.
 * @result Converted decimal value.
 */
static uint8_t ds3231ConvertRegDec(uint8_t reg);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================


//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t ds3231Initialize(void){

	if( i2chlInitialize(DS3231_CONFIG_I2C) != 0 ) return DS3231_ERR_INIT;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231StatusRead(uint8_t *status, uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterRead(DS3231_ADD_STATUS, status, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231EN32kHzRead(uint8_t *en32kHz, uint32_t timeout){

	int32_t ret;
	uint8_t status;

	ret = ds3231RegisterRead(DS3231_ADD_STATUS, &status, timeout);

	if( ret != 0 ) return ret;

	if( status & DS3231_STATUS_EN32KHZ ) *en32kHz = 1;
	else *en32kHz = 0;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231EN32kHzClear(uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterBitsClear(DS3231_ADD_STATUS, DS3231_STATUS_EN32KHZ, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231EN32kHzSet(uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterBitsSet(DS3231_ADD_STATUS, DS3231_STATUS_EN32KHZ, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231OSFRead(uint8_t *osf, uint32_t timeout){

	int32_t ret;
	uint8_t status;

	ret = ds3231RegisterRead(DS3231_ADD_STATUS, &status, timeout);

	if( ret != 0 ) return ret;

	if( status & DS3231_STATUS_OSF ) *osf = 1;
	else *osf = 0;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231OSFClear(uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterBitsClear(DS3231_ADD_STATUS, DS3231_STATUS_OSF, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231ControlRead(uint8_t *buffer, uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterRead(DS3231_ADD_CONTROL, buffer, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231ControlBitsClear(uint8_t bits, uint32_t timeout){

	int32_t ret;

	/* Ignores CONV bit */
	bits &= (uint8_t)(~DS3231_CONTROL_CONV);

	ret = ds3231RegisterBitsClear(DS3231_ADD_CONTROL, bits, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231ControlBitsSet(uint8_t bits, uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterBitsSet(DS3231_ADD_CONTROL, bits, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231TemperatureRead(int8_t *temp, uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterRead(DS3231_ADD_TEMP_UB, (uint8_t *)temp, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231SecondsRead(uint8_t *sec, uint32_t timeout){

	int32_t ret;
	uint8_t secsreg;

	ret = ds3231RegisterRead(DS3231_ADD_SECONDS, &secsreg, timeout);
	if( ret != 0 ) return ret;

	*sec = ds3231ConvertRegDec(secsreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231SecondsSet(uint8_t sec, uint32_t timeout){

	int32_t ret;
	uint8_t secsreg;

	if( sec > 59 ) return DS3231_ERR_SET_SECONDS;

	secsreg = ds3231ConvertDecReg(sec);

	ret = ds3231RegisterWrite(DS3231_ADD_SECONDS, &secsreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231MinutesRead(uint8_t *min, uint32_t timeout){

	int32_t ret;
	uint8_t minreg;

	ret = ds3231RegisterRead(DS3231_ADD_MINUTES, &minreg, timeout);
	if( ret != 0 ) return ret;

	*min = ds3231ConvertRegDec(minreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231MinutesSet(uint8_t min, uint32_t timeout){

	int32_t ret;
	uint8_t minreg;

	if( min > 59 ) return DS3231_ERR_SET_MINUTES;

	minreg = ds3231ConvertDecReg(min);

	ret = ds3231RegisterWrite(DS3231_ADD_MINUTES, &minreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231HoursRead(uint8_t *hour, uint32_t timeout){

	int32_t ret;
	uint8_t hourreg;

	ret = ds3231RegisterRead(DS3231_ADD_HOUR, &hourreg, timeout);
	if( ret != 0 ) return ret;

	*hour = ds3231ConvertRegDec(hourreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231HoursSet(uint8_t hour, uint32_t timeout){

	int32_t ret;
	uint8_t hourreg;

	if( hour > 23 ) return DS3231_ERR_SET_HOUR;

	hourreg = ds3231ConvertDecReg(hour);

	ret = ds3231RegisterWrite(DS3231_ADD_HOUR, &hourreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231DayOfWeekRead(uint8_t *day, uint32_t timeout){

	int32_t ret;

	ret = ds3231RegisterRead(DS3231_ADD_DAYOFWEEK, day, timeout);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DayOfWeekSet(uint8_t day, uint32_t timeout){

	int32_t ret;

	if( (day == 0) || (day > 7) ) return DS3231_ERR_SET_DAYOFWEEK;

	ret = ds3231RegisterWrite(DS3231_ADD_DAYOFWEEK, &day, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231DayRead(uint8_t *day, uint32_t timeout){

	int32_t ret;
	uint8_t dayreg;

	ret = ds3231RegisterRead(DS3231_ADD_DAY, &dayreg, timeout);
	if( ret != 0 ) return ret;

	*day = ds3231ConvertRegDec(dayreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DaySet(uint8_t day, uint32_t timeout){

	int32_t ret;
	uint8_t dayreg;

	if( day > 31 ) return DS3231_ERR_SET_DAY;

	dayreg = ds3231ConvertDecReg(day);

	ret = ds3231RegisterWrite(DS3231_ADD_DAY, &dayreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231MonthRead(uint8_t *month, uint32_t timeout){

	int32_t ret;
	uint8_t monthreg;

	ret = ds3231RegisterRead(DS3231_ADD_MONTH, &monthreg, timeout);
	if( ret != 0 ) return ret;

	*month = ds3231ConvertRegDec(monthreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231MonthSet(uint8_t month, uint32_t timeout){

	int32_t ret;
	uint8_t monthreg;

	if( (month == 0) || (month > 12) ) return DS3231_ERR_SET_MONTH;

	monthreg = ds3231ConvertDecReg(month);

	ret = ds3231RegisterWrite(DS3231_ADD_MONTH, &monthreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231YearRead(uint8_t *year, uint32_t timeout){

	int32_t ret;
	uint8_t yearreg;

	ret = ds3231RegisterRead(DS3231_ADD_YEAR, &yearreg, timeout);
	if( ret != 0 ) return ret;

	*year = ds3231ConvertRegDec(yearreg);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231YearSet(uint8_t year, uint32_t timeout){

	int32_t ret;
	uint8_t yearreg;

	if( year > 99 ) return DS3231_ERR_SET_YEAR;

	yearreg = ds3231ConvertDecReg(year);

	ret = ds3231RegisterWrite(DS3231_ADD_YEAR, &yearreg, 1, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231TimeRead(ds3231DateTime_t *time, uint32_t timeout){

	int32_t ret;

	ret = ds3231SecondsRead(&time->seconds, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_SEC;

	ret = ds3231MinutesRead(&time->minutes, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_MIN;

	ret = ds3231HoursRead(&time->hours, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_HOUR;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231TimeSet(ds3231DateTime_t *datetime, uint32_t timeout){

	int32_t ret;

	ret = ds3231SecondsSet(datetime->seconds, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_SEC;

	ret = ds3231MinutesSet(datetime->minutes, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_MIN;

	ret = ds3231HoursSet(datetime->hours, timeout);
	if( ret != 0 ) return DS3231_ERR_TIME_HOUR;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DateRead(ds3231DateTime_t *datetime, uint32_t timeout){

	int32_t ret;
	uint8_t year;

	ret = ds3231DayOfWeekRead(&datetime->dayofweek, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_DOW;

	ret = ds3231DayRead(&datetime->day, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_DAY;

	ret = ds3231MonthRead(&datetime->month, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_MONTH;

	ret = ds3231YearRead(&year, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_YEAR;
	datetime->year = (uint16_t)(year + 2000U);

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DateSet(ds3231DateTime_t *datetime, uint32_t timeout){

	int32_t ret;
	uint8_t year;

	ret = ds3231DayOfWeekSet(datetime->dayofweek, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_DOW;

	ret = ds3231DaySet(datetime->day, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_DAY;

	ret = ds3231MonthSet(datetime->month, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_MONTH;

	year = (uint8_t)(datetime->year - 2000U);
	ret = ds3231YearSet(year, timeout);
	if( ret != 0 ) return DS3231_ERR_DATE_YEAR;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DateTimeRead(ds3231DateTime_t *datetime, uint32_t timeout){

	int32_t ret;

	ret = ds3231TimeRead(datetime, timeout);
	if( ret != 0 ) return ret;

	ret = ds3231DateRead(datetime, timeout);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231DateTimeSet(ds3231DateTime_t *datetime, uint32_t timeout){

	int32_t ret;

	ret = ds3231TimeSet(datetime, timeout);
	if( ret != 0 ) return ret;

	ret = ds3231DateSet(datetime, timeout);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231I2CBusyRecover(void){

	int32_t ret;

	ret = i2chlBusyRecover(DS3231_CONFIG_I2C);

	return ret;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t ds3231Write(uint8_t *buffer, uint16_t nbytes, uint32_t timeout){

	if( i2chlWrite(DS3231_CONFIG_I2C, DS3231_CONFIG_ADDRESS, buffer, nbytes, timeout) != 0 )
		return DS3231_ERR_TX_TO;

	if( i2chlWaitWhileBusy(DS3231_CONFIG_I2C, timeout) != 0 )
		return DS3231_ERR_TX_TO;

	if( i2chlStatusLastTransaction(DS3231_CONFIG_I2C) != 0 )
		return DS3231_ERR_TX;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231Read(uint8_t *buffer, uint16_t nbytes, uint32_t timeout){

	if( i2chlRead(DS3231_CONFIG_I2C, DS3231_CONFIG_ADDRESS, buffer, nbytes, timeout) != 0 )
		return DS3231_ERR_RX_TO;

	if( i2chlWaitWhileBusy(DS3231_CONFIG_I2C, timeout) != 0 )
		return DS3231_ERR_RX_TO;

	if( i2chlStatusLastTransaction(DS3231_CONFIG_I2C) != 0 )
		return DS3231_ERR_RX;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterBitsSet(uint8_t reg, uint8_t bits, uint32_t timeout){

	int32_t ret;
	uint8_t data;

	/*
	 * First, we read the register, so that we can modify only the selected
	 * bits.
	 */
	ret = ds3231RegisterRead(reg, &data, timeout);
	if( ret != 0 ) return ret;

	/* Sets selected bits */
	data = data | bits;

	/* Writes updated value to register */
	ret = ds3231RegisterWrite(reg, &data, 0, timeout);
	if( ret != 0 ) return ret;

	/*
	 * Now, we'll read the register again to make sure the bits were set
	 * correctly.
	 */
	ret = ds3231RegisterRead(reg, &data, timeout);
	if( ret != 0 ) return ret;

	if( (data & bits) != bits ) return DS3231_ERR_CMD;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterBitsClear(uint8_t reg, uint8_t bits, uint32_t timeout){

	int32_t ret;
	uint8_t data;

	/*
	 * First, we read the register, so that we can modify only the selected
	 * bits.
	 */
	ret = ds3231RegisterRead(reg, &data, timeout);
	if( ret != 0 ) return ret;

	/* Clears selected bits */
	data = (uint8_t)(data & (~bits));

	/* Writes updated value to register */
	ret = ds3231RegisterWrite(reg, &data, 0, timeout);
	if( ret != 0 ) return ret;

	/*
	 * Now, we'll read the register again to make sure the bits were set
	 * correctly.
	 */
	ret = ds3231RegisterRead(reg, &data, timeout);
	if( ret != 0 ) return ret;

	if( (data & bits) != 0 ) return DS3231_ERR_CMD;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterRead(uint8_t reg, uint8_t *data, uint32_t timeout){

	int32_t ret;

	/* First, set DS3231's address pointer */
	ret = ds3231Write(&reg, 1, timeout);
	if( ret != 0 ) return ret;

	/* Now, read the register */
	ret = ds3231Read(data, 1, timeout);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterWrite(uint8_t reg, uint8_t *data, uint8_t check, uint32_t timeout){

	int32_t ret;
	uint8_t buffer[2];

	buffer[0] = reg;
	buffer[1] = *data;

	/* First, write to the register */
	ret = ds3231Write(buffer, 2, timeout);
	if( ret != 0 ) return ret;

	/* Checks written data, if required */
	if( check != 0 ){
		ret = ds3231RegisterRead(reg, buffer, timeout);
		if( ret != 0 ) return ret;

		if( buffer[0] != *data ) return DS3231_ERR_CMD;
	}

	return 0;
}
//---------------------------------------------------------------------------
static uint8_t ds3231ConvertDecReg(uint8_t dec){

	uint8_t decUn, decDez, decData;

	decUn = dec % 10;
	decDez = (uint8_t)(dec / 10);

	decData = (uint8_t)(decUn | (decDez << 4));

	return decData;
}
//---------------------------------------------------------------------------
static uint8_t ds3231ConvertRegDec(uint8_t reg){

	uint8_t decUn, decDez, decData;

	decUn = reg & 0x0F;
	decDez = (reg & 0xF0) >> 4;

	decData = (uint8_t)(decUn + (decDez * 10));

	return decData;
}
//---------------------------------------------------------------------------
//===========================================================================
