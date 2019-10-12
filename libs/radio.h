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
 * @return 0 if the packet was sent successfully, 1 otherwise.
 */
uint8_t radioWrite(uint8_t *data);
//-----------------------------
/** @brief Pends on the radio until a packet is received.
 *
 * @param buffer Buffer of length configRADIO_PAYLOAD_SIZE to store data
 *               received from radio.
 * @param ticks Amount of systicks to wait for a packet.
 * @return 0 if a packet was received, 1 if systicks expired before a packet
 *         was received.
 */
uint8_t radioRead(uint8_t *buffer, uint32_t ticks);
//-----------------------------
uint8_t radioSetTX(void);
//-----------------------------
uint8_t radioSetRX(void);
//-----------------------------
//=============================

#endif /* RADIO_H_ */
