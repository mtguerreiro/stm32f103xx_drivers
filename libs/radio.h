/*
 * radio.h
 *
 *  Created on: Oct 12, 2019
 *      Author: marco
 */

#ifndef RADIO_H_
#define RADIO_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
//-----------------------------
/** @brief Address for the RX/TX channel. */
#define configRADIO_RX_ADDR_B0          0x05
#define configRADIO_RX_ADDR_B1          0x04
#define configRADIO_RX_ADDR_B2          0x03
#define configRADIO_RX_ADDR_B3          0x02
#define configRADIO_RX_ADDR_B4          0x01
//-----------------------------
/** @brief RPpayload size. */
#define configRADIO_PAYLOAD_SIZE        32
//-----------------------------
/** @brief RX/TX channel frequency. */
#define configRADIO_CHANNEL             0x07
//-----------------------------
/** @brief Radio commands. */
#define configRADIO_CMD_SIZE            2
#define configRADIO_DATA_SIZE           (configRADIO_PAYLOAD_SIZE - configRADIO_CMD_SIZE)
//-----------------------------
/** @brief Errors */
#define configRADIO_ERROR_SPI_COMM      0x01
#define configRADIO_ERROR_WRITE_REG     0x02
#define configRADIO_ERROR_PEND          0x03
#define configRADIO_ERROR_MAX_RETRIES   0x04
#define configRADIO_ERROR_READ_TO       0x05
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t radioInitialize(void);
//-----------------------------
/** @brief Sends a packet through radio.
 *
 * @param packet Buffer of size configRADIO_PAYLOAD_SIZE containing the
 *               data to send.
 * @return 0 if the packet was sent successfully, another value otherwise.
 *         -configRADIO_ERROR_SPI_COMM: Failed to read/write to NRF.
 *         -configRADIO_ERROR_WRITE_REG: Written data to NRF's register does
 *         not match the value read.
 *         -configRADIO_ERROR_PEND: Timeout while pending.
 *         -configRADIO_ERROR_MAX_RETRIES: Exceeded maximum number of retries.
 *         -0xFF: Unknown error; possibly due to changes in NRF lib.
 */
uint8_t radioWrite(uint8_t *data);
//-----------------------------
/** @brief Pends on the radio until a packet is received.
 *
 * @param buffer Buffer of length configRADIO_PAYLOAD_SIZE to store data
 *               received from radio.
 * @param ticks Amount of systicks to wait for a packet.
 * @return 0 if a packet was received, another value otherwise.
 *         -configRADIO_ERROR_SPI_COMM: Failed to read/write to NRF.
 *         -configRADIO_ERROR_WRITE_REG: Written data to NRF's register does
 *         not match the value read.
 *         -configRADIO_ERROR_READ_TO: Timeout while waiting for data.
 *         -configRADIO_ERROR_MAX_RETRIES: Exceeded maximum number of retries.
 *         -0xFF: Unknown error; possibly due to changes in NRF lib.
 */
uint8_t radioRead(uint8_t *buffer, uint32_t ticks);
//-----------------------------
uint8_t radioSetTX(void);
//-----------------------------
uint8_t radioSetRX(void);
//-----------------------------
//=============================

#endif /* RADIO_H_ */
