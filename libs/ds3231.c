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

	uint8_t data;

	/* First, set DS3231's address pointer */
	data = DS3231_ADD_STATUS;
	if( ds3231Write(&data, 1, timeout) != 0 ) return DS3231_ERR_WRITE_TO;

	/* Now, read the register */
	if( ds3231Read(status, 1, timeout) != 0 ) return DS3231_ERR_READ_TO;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds3231EN32kHz(uint8_t en, uint32_t timeout){

	int32_t ret;
	uint8_t status;
	uint8_t data[2];

	/* Makes sure en is either 0 or 1 */
	en = en & 0x01;
	en = (uint8_t)(en << DS3231_STATUS_EN32KHZ_OFFS);

	/*
	 * First, we read the status register, so that we can modify only the
	 * EN32kHz bit.
	 */
	ret = ds3231StatusRead(&status, timeout);
	if( ret != 0 ) return ret;

	/* If the saved value matches the one to be written, skips writing */
	if( (status & DS3231_STATUS_EN32KHZ) == en ) return 0;

	if( en != 0 ){
		status |= DS3231_STATUS_EN32KHZ;
	}
	else{
		status &= (uint8_t)(~DS3231_STATUS_EN32KHZ);
	}

	/* Sends status address followed by the data */
	data[0] = DS3231_ADD_STATUS;
	data[1] = status;
	ret = ds3231Write(data, 2, timeout);
	if( ret != 0 ) return ret;

	/* Now, reads status to make sure command was executed properly */
	ret = ds3231StatusRead(&status, timeout);
	if( ret != 0 ) return ret;

	if( (status & DS3231_STATUS_EN32KHZ) != en ) return DS3231_ERR_CMD;

	return 0;
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
//===========================================================================
