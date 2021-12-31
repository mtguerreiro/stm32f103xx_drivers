/*
 * sdcstm32.c
 *
 *  Created on: 31 de dez de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "sdcstm32.h"

/* Libs */
#include "sdc.h"
#include "delays.h"

/* Drivers */
#include "spihl.h"
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcstm32InitializeHW(void);
static int32_t sdcstm32spiWrite(uint8_t *buffer, uint32_t nbytes, uint32_t timeout);
static int32_t sdcstm32spiRead(uint8_t *buffer, uint32_t nbytes, uint32_t timeout);
static void sdcstm32csSet(void);
static void sdcstm32csReset(void);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t sdcstm32Initialize(void){

	int32_t status;

	status = sdcstm32InitializeHW();
	if( status != 0 ) return status;

	sdcSetSPI(sdcstm32spiWrite, sdcstm32spiRead, sdcstm32csSet, sdcstm32csReset);

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static int32_t sdcstm32InitializeHW(void){

	/* Initializes SPI peripheral */
	spihlInitialize(SDC_STM32_CONFIG_SPI, SPIHL_BR_CLK_DIV_256, SPIHL_POLL_PHAF);

	/* Initializes CS pin */
	gpioPortEnable(SDC_STM32_CONFIG_SPI_CS_PORT);

	gpioConfig(SDC_STM32_CONFIG_SPI_CS_PORT, SDC_STM32_CONFIG_SPI_CS_PIN,
			   GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	gpioOutputSet(SDC_STM32_CONFIG_SPI_CS_PORT, SDC_STM32_CONFIG_SPI_CS_PIN);

	return 0;
}
//---------------------------------------------------------------------------
static int32_t sdcstm32spiWrite(uint8_t *buffer, uint32_t nbytes, uint32_t timeout){

	int32_t status;

	status = spihlWrite(SDC_STM32_CONFIG_SPI, buffer, nbytes, timeout);
	if( status != 0 ) return SDC_ERR_SPI_WRITE;

	status = spihlWaitWhileBusy(SDC_STM32_CONFIG_SPI, timeout);
	if( status != 0 ) return SDC_ERR_SPI_WRITE;

	return 0;
}
//---------------------------------------------------------------------------
static int32_t sdcstm32spiRead(uint8_t *buffer, uint32_t nbytes, uint32_t timeout){

	int32_t status;

	status = spihlRead(SDC_STM32_CONFIG_SPI, buffer, nbytes, timeout);
	if( status != 0 ) return SDC_ERR_SPI_READ;

	status = spihlWaitWhileBusy(SDC_STM32_CONFIG_SPI, timeout);
	if( status != 0 ) return SDC_ERR_SPI_READ;

	return 0;
}
//---------------------------------------------------------------------------
static void sdcstm32csSet(void){

	gpioOutputSet(SDC_STM32_CONFIG_SPI_CS_PORT, SDC_STM32_CONFIG_SPI_CS_PIN);
}
//---------------------------------------------------------------------------
static void sdcstm32csReset(void){

	gpioOutputReset(SDC_STM32_CONFIG_SPI_CS_PORT, SDC_STM32_CONFIG_SPI_CS_PIN);
}
//---------------------------------------------------------------------------
//===========================================================================
