/*
 * spihl.c
 *
 *  Created on: 8 de mai de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "spihl.h"

/* Libs */
#include "cqueue.h"

/* Drivers */
#include "gpio.h"

#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"
#endif
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{
	cqueue_t rxQueue;			/**< RX queue. */
	cqueue_t txQueue;			/**< TX queue. */

#ifdef SPIHL_CONFIG_FREE_RTOS_ENABLED
	SemaphoreHandle_t rxSemph;	/**< RX semaphore. */
	SemaphoreHandle_t txSemph;	/**< TX semaphore. */
#endif
}spihlControl_t;
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
#ifdef SPIHL_CONFIG_SPI1_ENABLED
spihlControl_t spihlSPI1Control;
#endif

#ifdef SPIHL_CONFIG_SPI2_ENABLED
spihlControl_t spihlSPI2Control;
#endif

#ifdef SPIHL_CONFIG_SPI3_ENABLED
spihlControl_t spihlSPI3Control;
#endif
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t spihlInitialize(SPI_TypeDef *spi, SPIHLPP_t clockPP, \
		uint8_t *rxBuffer, uint16_t rxBufferSize, \
		uint8_t *txBuffer, uint16_t txBufferSize){

	int32_t ret;

	ret = spihlInitializeHW(spi, clockPP);
	if( ret != 0 ) return ret;

	ret = uarthlInitializeSW(spi, rxBuffer, rxBufferSize, txBuffer, txBufferSize);
	if( ret != 0 ) return ret;

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================
