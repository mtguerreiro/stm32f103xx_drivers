/*
 * ds18b20.c
 *
 *  Created on: 2 de mai de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "ds18b20.h"

/* Drivers */
#include "onewire.h"
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static uint8_t ds18b20crc8(uint8_t *data, uint16_t len);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
#define DS18B20_CRC_POLY 			0x18 	/** Polynomial for CRC computation. */

/* Commands */
#define DS18B20_CMD_READ_ROM		0x33	/** Read ROM command. */
#define DS18B20_CMD_READ_ROM_SIZE	8 		/** Number of bytes expected from read ROM command. */

#define DS18B20_CMD_READ_SP			0xBE	/** Read scratch pad command. */
#define DS18B20_CMD_READ_SP_SIZE	9 		/** Number of bytes expected from read SP command. */

#define DS18B20_CMD_SKIP_ROM		0xCC	/** Skip ROM command. */

#define DS18B20_CMD_WRITE_SP		0x4E	/** Write scratch pad command. */
#define DS18B20_CMD_WRITE_SP_SIZE	3		/** Write scratch pad command. */

#define DS18B20_CMD_CONVERT_T		0x44	/** Initiate temperature conversion command. */

//===========================================================================

//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	DS12B20_TEMP_RES_9 = 0,
	DS12B20_TEMP_RES_10,
	DS12B20_TEMP_RES_11,
	DS12B20_TEMP_RES_12
}ds18b20TempRes_t;
//===========================================================================


//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t ds18b20Initialize(void *gpio, uint8_t pin){

	if( onewireInitialize(gpio, pin) ) return DS18B20_ERR_1W_INIT;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds18b20ReadROM(uint8_t *buffer, uint32_t to){

	int32_t ret;
	uint8_t cmd;

	ret = onewireReset(to);
	if( ret != 0 ) return DS18B20_ERR_RESET;

	cmd = DS18B20_CMD_READ_ROM;
	ret = onewireWrite(&cmd, 1, to);
	if( ret != 0 ) return DS18B20_ERR_WRITE;

	ret = onewireRead(buffer, DS18B20_CMD_READ_ROM_SIZE, to);
	if( ret != 0 ) return DS18B20_ERR_READ;

	ret = ds18b20crc8(buffer, DS18B20_CMD_READ_ROM_SIZE);
	if( ret != 0 ) return DS18B20_ERR_READ_CRC;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds18b20WriteSP(uint8_t *buffer, uint32_t to){

	int32_t ret;
	uint8_t cmd[4];
	uint8_t rbuffer[9];
	uint8_t *p;

	/* First, writes the scratch pad */
	ret = ds18b20SkipROM(to);
	if( ret != 0 ) return ret;

	p = buffer;
	cmd[0] = DS18B20_CMD_WRITE_SP;
	cmd[1] = *p++;
	cmd[2] = *p++;
	cmd[3] = *p++;
	ret = onewireWrite(cmd, 4, to);
	if( ret != 0 ) return DS18B20_ERR_WRITE;

	/* Now, reads the scratch pad to check if data is written correctly */
	ret = ds18b20ReadSP(rbuffer, to);
	if( ret != 0 ) return ret;

	if(cmd[1] != rbuffer[2]) return DS18B20_ERR_WRITE_SP;
	if(cmd[2] != rbuffer[3]) return DS18B20_ERR_WRITE_SP;
	if(cmd[3] != rbuffer[4]) return DS18B20_ERR_WRITE_SP;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds18b20ReadTemp(uint16_t *dez, uint16_t *dec, uint32_t to){

	int32_t ret;
	uint8_t cmd;
	uint8_t rbuffer[9];
	uint16_t temp;
	uint16_t aux;
	uint8_t bit;
	uint8_t k;

	ret = ds18b20SkipROM(to);
	if( ret != 0 ) return ret;

	cmd = DS18B20_CMD_CONVERT_T;
	ret = onewireWrite(&cmd, 1, to);
	if( ret != 0 ) return DS18B20_ERR_WRITE;

	ret = 0;
	while(1){
		ret = onewireRead(&cmd, 1, to);
		if( ret != 0 ) break;
		if( cmd == 0xFF ) break;
	}

	if( ret != 0 ) return DS18B20_ERR_READ;

	ret = ds18b20ReadSP(rbuffer, to);
	if( ret != 0 ) return ret;

	temp = (uint16_t)((rbuffer[1] << 8) | rbuffer[0]);
	*dez = (uint16_t)( temp >> 4);

	*dec = 0;
	for(k = 0; k < 4; k++){
		bit = 1 << k;
		if( (temp & (bit)) ){
			aux = ( 0xFF - (bit << (k + 2)) ) >> 4;
			*dec |= aux;
		}
	}


//	*p++ = rbuffer[0];
//	*p++ = rbuffer[1];

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds18b20ReadSP(uint8_t *buffer, uint32_t to){

	int32_t ret;
	uint8_t cmd;

	ret = ds18b20SkipROM(to);
	if( ret != 0 ) return ret;

	cmd = DS18B20_CMD_READ_SP;
	ret = onewireWrite(&cmd, 1, to);
	if( ret != 0 ) return DS18B20_ERR_WRITE;

	ret = onewireRead(buffer, DS18B20_CMD_READ_SP_SIZE, to);
	if( ret != 0 ) return DS18B20_ERR_READ;

	ret = ds18b20crc8(buffer, DS18B20_CMD_READ_SP_SIZE);
	if( ret != 0 ) return DS18B20_ERR_READ_CRC;

	return 0;
}
//---------------------------------------------------------------------------
int32_t ds18b20SkipROM(uint32_t to){

	int32_t ret;
	uint8_t cmd;

	ret = onewireReset(to);
	if( ret != 0 ) return DS18B20_ERR_RESET;

	cmd = DS18B20_CMD_SKIP_ROM;
	ret = onewireWrite(&cmd, 1, to);
	if( ret != 0 ) return DS18B20_ERR_WRITE;

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static uint8_t ds18b20crc8(uint8_t *data, uint16_t len){

	uint8_t *p;
	uint16_t k, i;
	uint8_t crc;
	uint8_t d;

	crc = 0;
	p = data;
	for(i = 0; i < len; i++){
		d = *p++;
		for(k = 0; k < 8; k++){
			if( (d ^ crc) & 1 ){
				crc = crc ^ DS18B20_CRC_POLY;
				crc = crc >> 1;
				crc = crc | 0x80;
			}
			else{
				crc = crc >> 1;
			}
			d = d >> 1;
		}
	}

	return crc;
}
//---------------------------------------------------------------------------
//===========================================================================
