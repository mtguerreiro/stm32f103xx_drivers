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

	uint32_t ret;

	ret = ds3231RegisterRead(DS3231_ADD_STATUS, status, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231EN32kHzRead(uint8_t *en32kHz, uint32_t timeout){

	int32_t ret;
	uint8_t status;

	ret = ds3231StatusRead(&status, timeout);

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

	ret = ds3231StatusRead(&status, timeout);

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

	ret = ds3231RegisterRead(DS3231_ADD_TEMP_UB, temp, timeout);

	return ret;
}
//---------------------------------------------------------------------------
int32_t ds3231SecondsRead(uint8_t *sec, uint32_t timeout){

	int32_t ret;

	uint8_t secUn, secDez, secData;

	ret = ds3231RegisterRead(DS3231_ADD_SECONDS, &secData, timeout);
	if( ret != 0 ) return ret;

	secUn = secData & 0x0F;
	secDez = (secData & 0x70) >> 4;

	*sec = (uint8_t)(secUn + (secDez * 10));

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231SecondsSet(uint8_t sec, uint32_t timeout){

	int32_t ret;

	uint8_t secUn, secDez, secData;

	uint8_t cmd[2];

	if( sec > 59 ) return DS3231_ERR_INVALID_SEC;

	secUn = sec % 10;
	secDez = (uint8_t)(sec / 10);

	secData = (uint8_t)(secUn | (secDez << 4));

	cmd[0] = DS3231_ADD_SECONDS;
	cmd[1] = secData;

	ret = ds3231Write(cmd, 2, timeout);

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
	uint8_t data[2];

	/*
	 * First, we read the register, so that we can modify only the selected
	 * bits.
	 */
	data[0] = reg;
	ret =  ds3231Write(data, 1, timeout);
	if( ret != 0 ) return DS3231_ERR_WRITE_TO;

	ret = ds3231Read(&data[1], 1, timeout);
	if( ret != 0 ) return ret;

	/* Sets selected bits */
	data[1] = data[1] | bits;

	/* Writes updated status to register */
	ret = ds3231Write(data, 2, timeout);
	if( ret != 0 ) return ret;

	/* Now, reads status to make sure command was executed properly */
	ret = ds3231Read(&data[1], 1, timeout);
	if( ret != 0 ) return ret;

	if( (data[1] & bits) != bits ) return DS3231_ERR_CMD;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterBitsClear(uint8_t reg, uint8_t bits, uint32_t timeout){

	int32_t ret;
	uint8_t data[2];

	/*
	 * First, we read the register, so that we can modify only the selected
	 * bits.
	 */
	data[0] = reg;
	ret =  ds3231Write(data, 1, timeout);
	if( ret != 0 ) return DS3231_ERR_WRITE_TO;

	ret = ds3231Read(&data[1], 1, timeout);
	if( ret != 0 ) return ret;

	/* Clears selected bits */
	data[1] = (uint8_t)(data[1] & (~bits));

	/* Writes updated status to register */
	ret = ds3231Write(data, 2, timeout);
	if( ret != 0 ) return ret;

	/* Now, reads status to make sure command was executed properly */
	ret = ds3231Read(&data[1], 1, timeout);
	if( ret != 0 ) return ret;

	if( (data[1] & bits) != 0 ) return DS3231_ERR_CMD;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t ds3231RegisterRead(uint8_t reg, uint8_t *data, uint32_t timeout){

	/* First, set DS3231's address pointer */
	if( ds3231Write(&reg, 1, timeout) != 0 ) return DS3231_ERR_WRITE_TO;

	/* Now, read the register */
	if( ds3231Read(data, 1, timeout) != 0 ) return DS3231_ERR_READ_TO;

	return 0;
}
//---------------------------------------------------------------------------
//static int32_t ds3231RegisterWrite(uint8_t reg, uint8_t *data, uint32_t timeout){
//
//	/* First, set DS3231's address pointer */
//	if( ds3231Write(&reg, 1, timeout) != 0 ) return DS3231_ERR_WRITE_TO;
//
//	/* Now, read the register */
//	if( ds3231Read(data, 1, timeout) != 0 ) return DS3231_ERR_READ_TO;
//
//	return 0;
//}
//---------------------------------------------------------------------------
//===========================================================================
