/*
 * nrf24l01.h
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 *
 * Melhorias
 * 		- Ap�s enviar um comando, realizar a leitura para ver
 * 		se foi configurado corretamente
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_


//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>
//=============================

//=============================
/*--------- Defines ---------*/
//=============================

/* NRF24L01 Register addresses */
#define NRF24L01_REG_CONFIG 		0x00
#define NRF24L01_REG_EN_AA			0x01
#define NRF24L01_REG_EN_RXADDR		0x02
#define NRF24L01_REG_SETUP_AW		0x03
#define NRF24L01_REG_SETUP_RETR		0x04
#define NRF24L01_REG_RF_CH			0x05
#define NRF24L01_REG_RF_SETUP		0x06
#define NRF24L01_REG_STATUS			0x07
#define NRF24L01_REG_OBS_TX			0x08
#define NRF24L01_REG_CD				0x09
#define NRF24L01_REG_RX_ADDR_P0		0x0A
#define NRF24L01_REG_RX_ADDR_P1		0x0B
#define NRF24L01_REG_RX_ADDR_P2		0x0C
#define NRF24L01_REG_RX_ADDR_P3		0x0D
#define NRF24L01_REG_RX_ADDR_P4		0x0E
#define NRF24L01_REG_RX_ADDR_P5		0x0F
#define NRF24L01_REG_TX_ADDR		0x10
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STATUS	0x17


//=============================


//=============================
/*-------- Functions --------*/
//=============================
uint8_t nrf24l01Initialize(void);
uint8_t nrf24l01ReadSR(uint8_t *status);
uint8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer);
uint8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer);
uint8_t nrf24l01TransmitPayload(uint8_t *buffer, uint8_t size);
uint8_t nrf24l01ReceivePayload(uint8_t *buffer, uint8_t size);
uint8_t nrf24l01StatusClearMaxRT(void);
uint8_t nrf24l01StatusClearTXDS(void);
uint8_t nrf24l01StatusClearRXDR(void);
uint8_t nrf24l01FlushTX(void);
uint8_t nrf24l01FlushRX(void);
void nr24l01SetCE(void);
void nr24l01ResetCE(void);
//=============================


#endif /* NRF24L01_H_ */
