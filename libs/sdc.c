/*
 * sdc.c
 *
 *  Created on: 10 de dez de 2021
 *      Author: marco
 *
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "sdc.h"

/* Libs */
#include "delays.h"

/* Drivers */
#include "spihl.h"
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
typedef struct{
	sdcSPIWrite_t spiWrite;
	sdcSPIRead_t spiRead;

	sdcCSSet_t csSet;
	sdcCSReset_t csReset;
}sdcControl_t;
//===========================================================================

//===========================================================================
/*--------------------------------- Globals -------------------------------*/
//===========================================================================
sdcControl_t sdcControl;
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcSendCMD(uint8_t cmd, uint32_t arg);
//---------------------------------------------------------------------------
/**
 * @brief Computes CRC 8.
 *
 * This function computes the CRC8 of a message (considering init and xorout
 * as 0, no reflection of poly/input). This is used to generate the CRC7 for
 * the card commands. The CRC7 will be the upper 7 bits of the CRC8 value,
 * and still needs to be OR'ed with 0x01 before forming the final CRC value.
 *
 * To check if a message has the right CRC, the complete message (including
 * CRC) can be given to this function, but the last bit of the CRC value has
 * to be 0. Then, if the CRC is correct, the function will return 0.
 *
 * @param data Buffer holding data.
 * @param len Size of buffer.
 * @return CRC result.
 */
static uint8_t sdccrc8(uint8_t *data, uint16_t len);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t sdcInitialize(void){

	uint32_t k;
	int32_t status;
	uint8_t data;

	uint8_t buffer[5];

	/* Sends dummy clocks with CS high */
	sdcControl.csSet();
	k = 100;
	data = 0xFF;
	while( k != 0 ){
		status = sdcControl.spiWrite(&data, 1, 10000);
		if( status != 0 ) break;
		k--;
	}
	if( k != 0 ) return SDC_ERR_SPI_WRITE;


	/* Sends the cmd0 command */
	sdcControl.csReset();
	status = sdcSendCMD(0, 0x00000000);
	if( status != 0 ){
		sdcControl.csSet();
		return SDC_ERR_SPI_WRITE;
	}

	/* Now, waits for the card to respond. The response should be "0x01". */
	k = 100;
	data = 0;
	while( k != 0 ){
		status = sdcControl.spiRead(buffer, 1, 10000);
		if( status != 0 ) break;
		if( buffer[0] == 0x01 ) break;
		k--;
	}
	sdcControl.csSet();
	if( k == 0 ) return SDC_ERR_NO_RESP;
	if( buffer[0] != 0x01 ) return SDC_ERR_UNEXPECTED_RESP;

	/* Sends the cmd8 command */
	sdcControl.csReset();
	status = sdcSendCMD(8, 0x000001AA);
	if( status != 0 ){
		sdcControl.csSet();
		return SDC_ERR_SPI_WRITE;
	}
	status = sdcControl.spiRead(buffer, 5, 10000);
	if( status != 0 ){
		sdcControl.csSet();
		return SDC_ERR_SPI_READ;
	}
	sdcControl.csSet();

	if( (buffer[0] != 0x01) && (buffer[0] != 0x05) ) return SDC_ERR_UNEXPECTED_RESP;

	/* Sends initialization command and waits for the proper response */
	sdcControl.csReset();
	k = 10000;
	while( k != 0 ){

		sdcSendCMD(55, 0x00000000);
		status = sdcControl.spiRead(buffer, 1, 10000);
		if( status != 0 ) break;

		sdcSendCMD(41, 0x40000000);
		status = sdcControl.spiRead(buffer, 1, 10000);
		if( status != 0 ) break;
		if( buffer[0] == 0x00 ) break;

		k--;
	}
	sdcControl.csSet();
	if( k == 0 ) return SDC_ERR_INIT;

	/* Last step now, we set block addressing mode */
	sdcControl.csReset();
	status = sdcSendCMD(16, SDC_CONFIG_BLOCK_SIZE);
	if( status != 0 ){
		sdcControl.csSet();
		return SDC_ERR_SPI_WRITE;
	}
	status = sdcControl.spiRead(buffer, 1, 10000);
	sdcControl.csSet();
	if( status != 0 ) return SDC_ERR_SPI_READ;
	if( buffer[0] != 0x00 ) return SDC_ERR_INIT;

	return 0;
}
//---------------------------------------------------------------------------
int32_t sdcReadBlock(uint32_t address, uint8_t *buffer, uint32_t nblocks){

	uint32_t k, i;
	uint8_t cmd = 17;
	int32_t status;
	uint8_t data = 0xFF;

	if( nblocks != 1 ) cmd = 18;

	sdcControl.csReset();

	/* Sends read block command and reads response */
	status = sdcSendCMD(cmd, address);
	if( status != 0 ){
		sdcControl.csSet();
		return status;
	}

	/* Waits for 0x00 response */
	k = 100;
	while(k--){
		status = sdcControl.spiRead(&data, 1, 10000);
		if( status != 0 ){
			sdcControl.csSet();
			return status;
		}

		if( data != 0xFF ) break;
	}
	if( k == 0 ){
		sdcControl.csSet();
		return status;
	}
	if( data != 0 ){
		sdcControl.csSet();
		return SDC_ERR_UNEXPECTED_RESP;
	}
//	status = sdcControl.spiRead(&data, 1, 10000);
//	if( status != 0 ){
//		sdcControl.csSet();
//		return status;
//	}
//
//	if( data != 0 ){
//		sdcControl.csSet();
//		return SDC_ERR_UNEXPECTED_RESP;
//	}

	i = nblocks;
	while(i--){

		/* Waits for data token (should be 0xFE) */
		k = 1000;
		data = 0;
		while(k--){
			status = sdcControl.spiRead(&data, 1, 10000);
			if( (status != 0) || (data == 0xFE) ) break;
		}
		if( status != 0 ){
			sdcControl.csSet();
			return status;
		}
		if( k == 0 ){
			sdcControl.csSet();
			return SDC_ERR_NO_RESP;
		}

		/* Reads block data */
		status = sdcControl.spiRead(buffer, SDC_CONFIG_BLOCK_SIZE, 1000000);
		if( status != 0 ){
			sdcControl.csSet();
			return status;
		}

		/* Reads the 2 CRC byte */
		sdcControl.spiRead(&data, 1, 10000);
		status = sdcControl.spiRead(&data, 1, 10000);
		if( status != 0 ){
			sdcControl.csSet();
			return status;
		}

		buffer += SDC_CONFIG_BLOCK_SIZE;
	}

	if( nblocks == 1 ){
		sdcControl.csSet();
		return 0;
	}

	/* Sends CMD 12 to stop transaction */
	status = sdcSendCMD(12, 0x00000000);
	if( status != 0 ){
		sdcControl.csSet();
		return status;
	}

	/* Waits for 0x00 response */
	k = 100;
	while(k--){
		status = sdcControl.spiRead(&data, 1, 10000);
		if( status != 0 ){
			sdcControl.csSet();
			return status;
		}
		if( data == 0x00 ) break;
	}
	if( k == 0 ){
		sdcControl.csSet();
		return status;
	}

	return 0;
}
//---------------------------------------------------------------------------
void sdcSetSPI(sdcSPIWrite_t spiwrite, sdcSPIRead_t spiread,
			   sdcCSSet_t csset, sdcCSReset_t csreset){

	sdcControl.spiWrite = spiwrite;
	sdcControl.spiRead = spiread;

	sdcControl.csSet = csset;
	sdcControl.csReset = csreset;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcSendCMD(uint8_t cmd, uint32_t arg){

	int32_t status;
	uint32_t k;
	uint8_t buffer[8];

	buffer[0] = 0xFF;
	buffer[7] = 0xFF;

	buffer[1] = cmd | 0x40U;
	k = 0;
	while( k < 4 ){
		buffer[k + 2] = (uint8_t)( (arg & 0xFF000000) >> 24 );
		arg = arg << 8;
		k++;
	}
	buffer[6] = sdccrc8(&buffer[1], 5) | 0x01U;

	status = sdcControl.spiWrite(buffer, 8, 10000);

	return status;
}
//---------------------------------------------------------------------------
static uint8_t sdccrc8(uint8_t *data, uint16_t len){

	uint8_t *p;
	uint8_t d;
    uint8_t crc;
    uint16_t k, i;

    p = data;
    crc = 0;
    for(i = 0; i < len; i++){
    	d = *p++;
    	crc = crc ^ d;
    	for(k = 0; k < 8; k++){
        	if( crc & 0x80 ){
        		crc = (uint8_t)(crc << 1);
        		crc = (uint8_t)(crc ^ SDC_CONFIG_CRC7_POLY);
        	}
        	else{
            	crc = (uint8_t)(crc << 1U);
        	}
    	}
    }

    return crc;
}
//---------------------------------------------------------------------------
//===========================================================================
