/*
 * sdc.c
 *
 *  Created on: 10 de dez de 2021
 *      Author: marco
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

//typedef struct{
//
//}sdcControl_t;

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcInitializeHW(void);
//---------------------------------------------------------------------------
static int32_t sdcInitializeSW(void);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t sdcInitialize(void){

	if( sdcInitializeHW() != 0 ) return SDC_ERR_INIT_HW;

	if( sdcInitializeSW() != 0 ) return SDC_ERR_INIT_SW;


	return 0;
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcInitializeHW(void){

	/* Initializes SPI peripheral */
	spihlInitialize(SDC_CONFIG_SPI, SPIHL_BR_CLK_DIV_256, SPIHL_POLL_PHAF);

	/* Initializes CS pin */
	gpioPortEnable(SDC_CONFIG_SPI_CS_PORT);
	gpioConfig(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);
	gpioOutputSet(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN);

	return 0;
}
//---------------------------------------------------------------------------
static int32_t sdcInitializeSW(void){

	uint32_t k;
	int32_t status;
	uint8_t data;

	uint8_t cmd0[] = {0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x95};

	k = 100;
	data = 0xFF;
	while( k != 0 ){
		status = spihlWrite(SDC_CONFIG_SPI, &data, 1, 10000);
		if( status != 0 ) break;
		k--;
	}
	if( k != 0 ){
		gpioOutputSet(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN);
		return SDC_ERR_INIT_SW;
	}

	status = spihlWaitWhileBusy(SDC_CONFIG_SPI, 10000);
	delaysus(5);
	gpioOutputReset(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN);

	status = spihlWrite(SDC_CONFIG_SPI, cmd0, sizeof(cmd0), 10000);
	if( status != 0 ){
		gpioOutputSet(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN);
		return SDC_ERR_INIT_SW;
	}

	k = 100;
	data = 0;
	while( k != 0 ){
		status = spihlRead(SDC_CONFIG_SPI, &data, 1, 10000);
		if( status != 0 ) break;
		if( data == 0x01 ) break;
		k--;
	}
	gpioOutputSet(SDC_CONFIG_SPI_CS_PORT, SDC_CONFIG_SPI_CS_PIN);
	if( k != 0 ) return SDC_ERR_INIT_SW;
	if( data != 0x01 ) return SDC_ERR_INIT_SW;


	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================
