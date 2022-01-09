/*
 * sdc.h
 *
 *  Created on: 10 de dez de 2021
 *      Author: marco
 */

#ifndef LIBS_SDC_H_
#define LIBS_SDC_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
/* Error codes */
#define SDC_ERR_INIT				-0x01 		/** SD card initialization error. */
#define SDC_ERR_INIT_SW				-0x02		/** Failed to initialize the SD card. */
#define SDC_ERR_NO_RESP				-0x03		/** No response from the SD card. */
#define SDC_ERR_UNEXPECTED_RESP		-0x04		/** Unexpected response from the SD card. */
#define SDC_ERR_WRITE_INVALID		-0x05		/** Write command rejected by card. */
#define SDC_ERR_SPI_WRITE			-0x05		/** SPI write function failed. */
#define SDC_ERR_SPI_READ			-0x06		/** SPI read function failed. */

#define SDC_CONFIG_CRC7_POLY		(0x89<<1) 	/** Polynomial for CRC computation. */
#define SDC_CONFIG_BLOCK_SIZE		512

typedef int32_t(*sdcHWInit_t)(void *params);
typedef int32_t(*sdcSPIWrite_t)(uint8_t *buffer, uint32_t nbytes, uint32_t timeout);
typedef int32_t(*sdcSPIRead_t)(uint8_t *buffer, uint32_t nbytes, uint32_t timeout);
typedef void(*sdcCSSet_t)(void);
typedef void(*sdcCSReset_t)(void);
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t sdcInitialize(void);

void sdcSetSPI(sdcSPIWrite_t spiwrite, sdcSPIRead_t spiread,
			   sdcCSSet_t csset, sdcCSReset_t csreset);

int32_t sdcReadBlock(uint32_t address, uint8_t *buffer, uint32_t nblocks);
int32_t sdcWriteBlock(uint32_t address, uint8_t *buffer, uint32_t nblocks);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//===========================================================================

#endif /* LIBS_SDC_H_ */
