/*
 * radio.c
 *
 *  Created on: Oct 12, 2019
 *      Author: marco
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "radio.h"

/* Kernel */
#include "FreeRTOS.h"
#include "task.h"

/* Libs */
#include "nrf24l01.h"
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
int8_t radioInitialize(void){

    uint8_t address[5] = {
    		RADIO_CONFIG_RX_ADDR_B0,
    		RADIO_CONFIG_RX_ADDR_B1,
			RADIO_CONFIG_RX_ADDR_B2,
			RADIO_CONFIG_RX_ADDR_B3,
			RADIO_CONFIG_RX_ADDR_B4};

    /* Powers module up. Keeps trying if initialization fails. */
    while( nrf24l01PowerUp()){
        vTaskDelay(500);
    }

    /* Sets the NRF device as RX. Keeps trying if it fails. */
    while( nrf24l01SetConfigs(address, RADIO_CONFIG_CHANNEL) ) {
        vTaskDelay(500);
    }

    /* Switches back to RX */
    nrf24l01SetPRX();

    return 0;
}
//-----------------------------------------------------------------------------
int8_t radioWrite(uint8_t *data, uint8_t size){

    int8_t status;

    /* Switches to TX */
    status = nrf24l01SetPTX();
    if( status ){
        return status;
    }

    /* Sends data */
    status = nrf24l01Write(data, size, 100);

    /* Switches back to RX */
    nrf24l01SetPRX();

    return status;
}
//-----------------------------------------------------------------------------
int8_t radioRead(uint8_t *buffer, uint32_t ticks){

	int8_t size;
    int8_t status;

    /* Ensures radio is in RX mode and waits for a packet */
    status = nrf24l01SetPRX();
    if( status ){
        return status;
    }

    size = nrf24l01Read(buffer, ticks);

    return size;
}
//-----------------------------------------------------------------------------
//=============================================================================
