/*
 * radio.c
 *
 *  Created on: Oct 12, 2019
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "radio.h"

/* Standard */
#include <stdint.h>

/* Kernel */
#include "FreeRTOS.h"
#include "task.h"

/* Drivers */
#include "gpio.h"

/* Libs */
#include "nrf24l01.h"
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t radioInitialize(void){

    uint8_t address[5] = {configRADIO_RX_ADDR_B0,
                          configRADIO_RX_ADDR_B1,
                          configRADIO_RX_ADDR_B2,
                          configRADIO_RX_ADDR_B3,
                          configRADIO_RX_ADDR_B4};

    /* Initializes the driver/lib */
    nrf24l01Initialize();

    /* Powers module up. Keeps trying if initialization fails. */
    while( nrf24l01PowerUp()){
        vTaskDelay(500);
    }

    /* Sets the NRF device as RX. Keeps trying if it fails. */
    while( nrf24l01SetRX(address, configRADIO_PAYLOAD_SIZE, configRADIO_CHANNEL) ) {
        vTaskDelay(500);
    }

    return 0;
}
//-----------------------------
uint8_t radioWrite(uint8_t *data){

    /* Switches to TX */
    nrf24l01SetPTX();

    /* Sends data */
    if( nrf24l01Write(data, configRADIO_PAYLOAD_SIZE, 100) ){
        nrf24l01SetPRX();
        return 1;
    }

    /* Switches back to RX */
    nrf24l01SetPRX();

    return 0;
}
//-----------------------------
uint8_t radioRead(uint8_t *buffer, uint32_t ticks){

    /* Assures radio is in RX mode and waits for a packet */
    nrf24l01SetPRX();
    if( nrf24l01Read(buffer, configRADIO_PAYLOAD_SIZE, ticks) ) return 1;

    return 0;
}
//-----------------------------
//=============================
