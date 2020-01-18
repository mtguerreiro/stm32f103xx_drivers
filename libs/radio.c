/*
 * radio.c
 *
 *  Created on: Oct 12, 2019
 *      Author: marco
 */

#ifdef RADIO_H_
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

    uint8_t status;

    /* Switches to TX */
    status = radioSetTX();
    if( status ){
        return status;
    }

    /* Sends data */
    status = nrf24l01Write(data, configRADIO_PAYLOAD_SIZE, 100);
    if( status ){
        uint8_t ret;
        if( status == 1 ) ret = configRADIO_ERROR_PEND;
        else if( status == 2 ) ret = configRADIO_ERROR_SPI_COMM;
        else if( status == 3 ) ret = configRADIO_ERROR_MAX_RETRIES;
        else ret = 0xFF;
        nrf24l01SetPRX();
        return ret;
    }

    /* Switches back to RX */
    nrf24l01SetPRX();

    return 0;
}
//-----------------------------
uint8_t radioRead(uint8_t *buffer, uint32_t ticks){

    uint8_t status;

    /* Assures radio is in RX mode and waits for a packet */
    status = radioSetRX();
    if( status ){
        return status;
    }

    if( nrf24l01Read(buffer, configRADIO_PAYLOAD_SIZE, ticks) ) {
        return configRADIO_ERROR_READ_TO;
    }

    return 0;
}
//-----------------------------
uint8_t radioSetTX(void){

    uint8_t status;

    status = nrf24l01SetPTX();
    if( status ){
        uint8_t ret;
        if( status == 1 ) ret = configRADIO_ERROR_SPI_COMM;
        else if( status == 2 ) ret = configRADIO_ERROR_WRITE_REG;
        else ret = 0xFF;
        return ret;
    }

    return 0;
}
//-----------------------------
uint8_t radioSetRX(void){

    uint8_t status;

    status = nrf24l01SetPRX();
    if( status ) {
        uint8_t ret;
        if( status == 1 ) ret = configRADIO_ERROR_SPI_COMM;
        else if( status == 2 ) ret = configRADIO_ERROR_WRITE_REG;
        else ret = 0xFF;
        return ret;
    }

    return 0;
}
//-----------------------------
//=============================

#endif
