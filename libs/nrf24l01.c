/*
 * nrf24l01.c
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "nrf24l01.h"
//=============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------
/* Transmitting/receiving functionalities */
//-----------------------------------------------------------------------------
/** @brief Transmits a payload to the configured TX address.
 *
 * The transmit payload command is sent, along with the data informed. Since
 * SPI transmits and received data simultaneously, all data received while
 * writing to the NRF's register are cleared from the RX queue.
 *
 * Here, data is not actually sent through RF. The payload is only written to
 * the NRF's register. However, actual data transmission should start as soon
 * after the payload is sent.
 *
 * This function is not meant to be used alone, and pending to the IRQ pin
 * should be done right after, to check if data was successfully sent.
 *
 * @param buffer Pointer to buffer containing data to be sent.
 * @param size Number of bytes to transmit.
 * @return 0 if data was sent successfully, 1 if not enough bytes were
 *         received during transmission, which can indicate that
 *         transmission failed.
 *
 */
static uint8_t nrf24l01TransmitPayload(uint8_t *buffer, uint8_t size);
//-----------------------------------------------------------------------------
/** @brief Receives a payload.
 *
 * The NRF's RX buffer is read. It is necessary to inform the number of bytes
 * to be read, which is the same as the configured RX payload size.
 *
 * @param buffer Pointer to buffer that will store the payload bytes.
 * @param size RX payload size.
 *
 * @return 0 if data data was retrieved from the NRF RX buffer, 1 if could
 *         not read data from NRF.
 */
static uint8_t nrf24l01ReceivePayload(uint8_t *buffer, uint8_t size);
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*------------------------------- Definitions -------------------------------*/
//=============================================================================
#define NRF24L01_CONFIG_SPI_READ_TO			0xFFFFF
#define NRF24L01_CONFIG_SPI_WRITE_TO		0xFFFFF
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
nrf24l01SpiWrite_t nrf24l01SpiWrite;
nrf24l01SpiRead_t nrf24l01SpiRead;
nrf24l01CSNWrite_t nrf24l01CEWrite;
nrf24l01CEWrite_t nrf24l01CSNWrite;
nrf24l01DelayMicrosec_t nrf24l01DelayMicroSec;
nrf24l01IrqClear_t nrf24l01IrqClear;
nrf24l01IrqPend_t nrf24l01IrqPend;
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void nrf24l01Initialize(nrf24l01SpiWrite_t spiWrite, nrf24l01SpiRead_t spiRead,
		nrf24l01CSNWrite_t csnWrite, nrf24l01CEWrite_t ceWrite,
		nrf24l01DelayMicrosec_t delay,
		nrf24l01IrqClear_t irqClear, nrf24l01IrqPend_t irqPend){

	nrf24l01SpiWrite = spiWrite;
	nrf24l01SpiRead = spiRead;
	nrf24l01CSNWrite = csnWrite;
	nrf24l01CEWrite = ceWrite;
	nrf24l01DelayMicroSec = delay;
	nrf24l01IrqClear = irqClear;
	nrf24l01IrqPend = irqPend;

	nrf24l01CSNWrite(1);
	nrf24l01CEWrite(0);
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01Read(uint8_t *buffer, uint8_t size, uint32_t pendTicks){

	/* Clears any pending IRQ */
	nrf24l01IrqClear();

    /* Starts listening */
	nrf24l01CEWrite(1);

    if( nrf24l01Pend(pendTicks) ){
        /* Nothing received. Stops listening and returns. */
    	nrf24l01CEWrite(0);
        return 1;
    }

    /*
     * If the device is in RX mode and the pend returned 0, then we certainly
     * received data. Thus, we just retrieve it from the FIFO and clear the
     * data received flag.
     */
    nrf24l01ReceivePayload(buffer, size);
    nrf24l01StatusClearRXDR();

    /* Stops listening */
    nrf24l01CEWrite(0);

    return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks){

    uint8_t nrfStatus;

	/* Clears any pending IRQ */
	nrf24l01IrqClear();

    nrf24l01TransmitPayload(buffer, size);

    if( nrf24l01Pend(pendTicks) ){
        /*
         * Pend should always return 0, because if transmission fails, it
         * will be due to maximum retries being reached (assuming enough
         * ticks have been provided). Thus, we should never get here. If
         * we do, we'll clear everything related to transmission.
         */
        nrf24l01FlushTX();
        nrf24l01StatusClearMaxRT();
        nrf24l01StatusClearTXDS();
        return 1;
    }

    /*
     * Now, we check the status. If maximum number of retries were reached,
     * we'll flush the TX FIFO and clear the corresponding flag. Else, data
     * was sent and we clear the TXDS flag only.
     * Additionally, if we fail to read the status, we'll just clear it
     * and clear the TX FIFO.
     */
    if( nrf24l01ReadSR(&nrfStatus) ){
        nrf24l01FlushTX();
        nrf24l01StatusClearMaxRT();
        nrf24l01StatusClearTXDS();
        return 2;
    }
    if( nrfStatus & (1 << 4) ){
        /* Maximum number of retries exceeded */
        nrf24l01FlushTX();
        nrf24l01StatusClearMaxRT();
        return 3;
    }

    /* Data sent successfully */
    nrf24l01StatusClearTXDS();

    return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetRX(uint8_t *address, uint8_t plSize, uint8_t channel){

	/* Enables auto-ack on data pipe 0 */
	if( nrf24l01EnableAA(0x01) ) return 1;

	/* Enables RX address on data pipe 0 */
	if( nrf24l01EnableRXADDR(0x01) ) return 2;

	/* Waits 2000 us for retransmission, tries up to 5 times */
	if( nrf24l01SetRetryTime(0x65) ) return 3;

	/* Sets RF channel (frequency) */
	if( nrf24l01SetRFChannel(channel) ) return 4;

	/* Sets RX address */
	if( nrf24l01SetRXAdress(address) ) return 5;

	/* Sets TX address */
	if( nrf24l01SetTXAdress(address) ) return 6;

	/* Sets number of bytes in RX payload data pipe 0 to 5 */
	if( nrf24l01SetRXPayloadSize(plSize) ) return 7;

	/* Sets as primary RX (PRX) */
	if( nrf24l01SetPRX() ) return 8;

	if( nrf24l01FlushTX() ) return 9;
	if( nrf24l01FlushRX() ) return 10;

	if( nrf24l01StatusClear()) return 11;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetTX(uint8_t *address, uint8_t plSize, uint8_t channel){

	/* Enables auto-ack on data pipe 0 */
	if( nrf24l01EnableAA(0x01) ) return 1;

	/* Enables RX address on data pipe 0 */
	if( nrf24l01EnableRXADDR(0x01) ) return 2;

	/* Waits 2000 us for retransmission, tries up to 5 times */
	if( nrf24l01SetRetryTime(0x65) ) return 3;

	/* Sets RF channel (frequency) */
	if( nrf24l01SetRFChannel(channel) ) return 4;

	/* Sets RX address */
	if( nrf24l01SetRXAdress(address) ) return 5;

	/* Sets TX address */
	if( nrf24l01SetTXAdress(address) ) return 6;

	/* Sets number of bytes in RX payload data pipe 0 to plSize */
	if( nrf24l01SetRXPayloadSize(plSize) ) return 7;

	/* Sets as primary TX (PTX) */
	if( nrf24l01SetPTX() ) return 8;

	if( nrf24l01FlushTX() ) return 9;
	if( nrf24l01FlushRX() ) return 10;

	if( nrf24l01StatusClear() ) return 11;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetRXPayloadSize(uint8_t size){

	/*
	 * Sets RX payload size for data pipe 0.
	 */
	uint8_t buffer;

	/* Writes to NRF register */
	if(size > 32) size = 32;
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P0, &size);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_RX_PW_P0, &buffer);
	if(size != buffer) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetRXAdress(uint8_t *address){

	/*
	 * Sets RX address for data pipe 0.
	 */

	uint8_t buffer[5];
	uint8_t *addrBuffer;
	uint8_t k;

	/* Writes to NRF register */
	addrBuffer = address;
	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P0, addrBuffer);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_RX_ADDR_P0, buffer);
	addrBuffer = address;
	k = 5;
	while(k--){
		if(buffer[k] != addrBuffer[k]) return 1;
	}

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetTXAdress(uint8_t *address){

	/*
	 * Sets TX address for data pipe 0.
	 */

	uint8_t buffer[5];
	uint8_t *addrBuffer;
	uint8_t k;

	/* Writes to NRF register */
	addrBuffer = address;
	nrf24l01WriteRegister(NRF24L01_REG_TX_ADDR, addrBuffer);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_TX_ADDR, buffer);
	addrBuffer = address;
	k = 5;
	while(k--){
		if(buffer[k] != addrBuffer[k]) return 1;
	}

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetRFChannel(uint8_t channel){

	uint8_t buffer;

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_RF_CH, &channel);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_RF_CH, &buffer);
	if(buffer != channel) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetRetryTime(uint8_t time){

	uint8_t buffer;

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_RETR, &time);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_SETUP_RETR, &buffer);
	if(buffer != time) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetPRX(void){

    uint8_t buffer;
    uint8_t data;

    /* First, we must read the current settings so we don't overwrite them */
    if( nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &data) ) return 1;

    data |= 0x01;

    /* Writes to NRF register */
    nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

    /* Reads from NRF register to make sure we wrote it correctly */
    nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);
    if(buffer != data) return 2;

    return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01SetPTX(void){

    uint8_t buffer;
    uint8_t data;

    /* First, we must read the current settings so we don't overwrite them */
    if( nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &data) ) return 1;

    data &= (uint8_t)(~0x01);

    /* Writes to NRF register */
    nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

    /* Reads from NRF register to make sure we wrote it correctly */
    nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);
    if(buffer != data) return 2;

    return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01EnableRXADDR(uint8_t pipes){

	uint8_t data;
	uint8_t buffer;

	/* First, we must read the current settings so we don't overwrite them */
	if( nrf24l01ReadRegister(NRF24L01_REG_EN_RXADDR, &data) ) return 1;

	/* Sets new pipes */
	data |= pipes;

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_EN_RXADDR, &data);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_EN_RXADDR, &buffer);
	if(buffer != data) return 2;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01DisableRXADDR(uint8_t pipes){

	uint8_t data;
	uint8_t buffer;

	/* First, we must read the current settings so we don't overwrite them */
	if( nrf24l01ReadRegister(NRF24L01_REG_EN_RXADDR, &data) ) return 1;

	/* Clear pipes */
	data &= (uint8_t)(~pipes);

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_EN_RXADDR, &data);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_EN_RXADDR, &buffer);
	if(buffer != data) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01EnableAA(uint8_t pipes){

	uint8_t data;
	uint8_t buffer;

	/* First, we must read the current settings so we don't overwrite them */
	if( nrf24l01ReadRegister(NRF24L01_REG_EN_AA, &data) ) return 1;

	/* Clear pipes */
	data |= pipes;

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_EN_AA, &data);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_EN_AA, &buffer);
	if(buffer != data) return 2;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01DisableAA(uint8_t pipes){

	uint8_t data;
	uint8_t buffer;

	/* First, we must read the current settings so we don't overwrite them */
	if( nrf24l01ReadRegister(NRF24L01_REG_EN_AA, &data) ) return 1;

	/* Clear pipes */
    data &= (uint8_t)(~pipes);

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_EN_AA, &data);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_EN_AA, &buffer);
	if(buffer != data) return 2;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01PowerUp(void){

	uint8_t buffer;
	uint8_t data;

    /* First, we must read the current settings so we don't overwrite them */
    if( nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &data) ) return 1;

	data |= 0x02;

    /* Writes to NRF register */
    nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

    /* After powering the device, we must wait around 1.5ms */
    nrf24l01DelayMicroSec(2000);

    /* Reads from NRF register to make sure we wrote it correctly */
    nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);
    if(buffer != data) return 2;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01PowerDown(void){

	uint8_t buffer;
	uint8_t data;

	/* First, we must read the current settings so we don't overwrite them */
	if( nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &data) ) return 1;

	data &= (uint8_t)(~0x02);

	/* Writes to NRF register */
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, &data);

	/* Reads from NRF register to make sure we wrote it correctly */
	nrf24l01ReadRegister(NRF24L01_REG_CONFIG, &buffer);
	if(buffer != data) return 2;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01ReadSR(uint8_t *status){

	uint8_t cmdstatus;
	nrf24l01CSNWrite(0);

	cmdstatus = nrf24l01SpiRead(status, 1, NRF24L01_CONFIG_SPI_READ_TO);

	nrf24l01CSNWrite(1);

	return cmdstatus;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer){

	if( (reg == NRF24L01_REG_RX_ADDR_P0) || (reg == NRF24L01_REG_RX_ADDR_P1) || (reg == NRF24L01_REG_TX_ADDR) ){
		if( nrf24l01ReadRegisterMultiple(reg, buffer) ) return 1;
	}
	else{
		if( nrf24l01ReadRegisterSingle(reg, buffer) ) return 1;
	}

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer){

	if( (reg == NRF24L01_REG_RX_ADDR_P0) || (reg == NRF24L01_REG_RX_ADDR_P1) || (reg == NRF24L01_REG_TX_ADDR) ){
		if( nrf24l01WriteRegisterMultiple(reg, buffer) ) return 1;
	}
	else{
		if( nrf24l01WriteRegisterSingle(reg, buffer) ) return 1;
	}

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01StatusClear(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 4U) | (0x01U << 5U) | (0x01U << 6U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01StatusClearMaxRT(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 4U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01StatusClearTXDS(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 5U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01StatusClearRXDR(void){

	uint8_t data;
	uint8_t buffer;

	data = (0x01U << 6U);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, &data);
	nrf24l01ReadRegister(NRF24L01_REG_STATUS, &buffer);

	if(buffer & data) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01FlushTX(void){

	uint8_t status;
	uint8_t command;

	command = 0xE1;

	nrf24l01CSNWrite(0);

	status = nrf24l01SpiWrite(&command, 1, NRF24L01_CONFIG_SPI_WRITE_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01FlushRX(void){

	uint8_t command;
	uint8_t status;

	command = 0xE2;

	nrf24l01CSNWrite(0);

	status = nrf24l01SpiWrite(&command, 1, NRF24L01_CONFIG_SPI_WRITE_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01Pend(uint32_t ticks){

	if( nrf24l01IrqPend(ticks) != 0 ) return 1;

	return 0;
}
//-----------------------------------------------------------------------------
uint8_t nrf24l01TXPower(uint8_t power){

    uint8_t buffer;
    uint8_t data;

    /* First, we must read the current settings so we don't overwrite them */
    if( nrf24l01ReadRegister(NRF24L01_REG_RF_SETUP, &data) ) return 1;

    data &= (uint8_t)~(0x03 << 1);
    data |= (uint8_t)(power << 1);

    /* Writes to NRF register */
    nrf24l01WriteRegister(NRF24L01_REG_RF_SETUP, &data);

    /* Reads from NRF register to make sure we wrote it correctly */
    nrf24l01ReadRegister(NRF24L01_REG_RF_SETUP, &buffer);
    if(buffer != data) return 2;

    return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*----------------------------- Static functions ----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static uint8_t nrf24l01ReadRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t status;
	uint8_t txBuffer[2];

    txBuffer[0] = reg;
	txBuffer[1] = 0xFF;

	nrf24l01CSNWrite(0);

	status = nrf24l01SpiWrite(txBuffer, 1, NRF24L01_CONFIG_SPI_WRITE_TO);

	status |= nrf24l01SpiRead(buffer, 1, NRF24L01_CONFIG_SPI_READ_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01ReadRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t status;
    uint8_t txBuffer[6] = {0x00};

	txBuffer[0] = reg;

	nrf24l01CSNWrite(0);

	status = nrf24l01SpiWrite(txBuffer, 1, NRF24L01_CONFIG_SPI_WRITE_TO);
	status |= nrf24l01SpiRead(buffer, 5, NRF24L01_CONFIG_SPI_READ_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01WriteRegisterSingle(uint8_t reg, uint8_t *buffer){

	uint8_t status;
    uint8_t txBuffer[2];

    txBuffer[0] = reg | 0x20;
    txBuffer[1] = *buffer;

    nrf24l01CSNWrite(0);

    status = nrf24l01SpiWrite(txBuffer, 2, NRF24L01_CONFIG_SPI_WRITE_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01WriteRegisterMultiple(uint8_t reg, uint8_t *buffer){

	uint8_t status;
	reg |= 0x20;

	nrf24l01CSNWrite(0);

	status = nrf24l01SpiWrite(&reg, 1, NRF24L01_CONFIG_SPI_WRITE_TO);
	status |= nrf24l01SpiWrite(buffer, 5, NRF24L01_CONFIG_SPI_WRITE_TO);

	nrf24l01CSNWrite(1);

	return status;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01TransmitPayload(uint8_t *buffer, uint8_t size){

	uint8_t status;
    uint8_t command;

    nrf24l01CSNWrite(0);

    command = 0xA0;

    /* Sends TX payload command plus payload */
	status = nrf24l01SpiWrite(&command, 1, NRF24L01_CONFIG_SPI_WRITE_TO);
	status |= nrf24l01SpiWrite(buffer, size, NRF24L01_CONFIG_SPI_WRITE_TO);

    nrf24l01CSNWrite(1);

    if( status != 0 ) return status;

    /*
     * A high pulse must be given to CE so transmission can start. This pulse
     * must last at least 10 us.
     */
    nrf24l01CEWrite(1);

    nrf24l01DelayMicroSec(10);

    nrf24l01CEWrite(0);

    return 0;
}
//-----------------------------------------------------------------------------
static uint8_t nrf24l01ReceivePayload(uint8_t *buffer, uint8_t size){

	uint8_t status;
    uint8_t txBuffer[2] = {0x00};

    nrf24l01CSNWrite(0);

    txBuffer[0] = 0x61;

    /* Sends read RX payload command */
    status = nrf24l01SpiWrite(txBuffer, 1, NRF24L01_CONFIG_SPI_WRITE_TO);
    status |= nrf24l01SpiRead(buffer, size, NRF24L01_CONFIG_SPI_READ_TO);

    nrf24l01CSNWrite(1);

    return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================
