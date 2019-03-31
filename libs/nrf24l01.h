/*
 * nrf24l01.h
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 *
 *	Current version: 0.1.1
 *
 *	-v0.1.0:
 *		- Initial version
 *
 *	-v0.1.1:
 *		- Added function to pend on IRQ pin
 *		- Added status clear function
 *		- Added RX and TX initialization functions
 *
 * Melhorias
 * 		- Após enviar um comando, realizar a leitura para ver
 * 		se foi configurado corretamente
 * 		- Power-down? Ativer power-up somente quando for transmitir?
 * 		- Funções nrf24l01Write e nrf24l01Read
 * 			- Em ambos os casos é necessário ler o status. E se houver
 * 			um erro na leitura? Qual o melhor tratamento para isso?
 * 			- Para transmitir, seria possível ligar o dispositivo só
 * 			no momento da transmissão?
 * 			- Para receber, seria possível ligar o dispositivo só durante
 * 			o tempo de recepção?
 * 			- Ambas as funções utilizam as funções received and transmit
 * 			payload. Essas funções podem retornar erro. Como tratar isso?
 *
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
uint8_t nrf24l01SetTX(uint8_t *address, uint8_t plSize);
uint8_t nrf24l01SetRX(uint8_t *address, uint8_t plSize);
uint8_t nrf24l01ReadSR(uint8_t *status);
uint8_t nrf24l01Read(uint8_t *buffer, uint8_t size, uint32_t pendTicks);
uint8_t nrf24l01Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks);
uint8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer);
uint8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer);
uint8_t nrf24l01TransmitPayload(uint8_t *buffer, uint8_t size);
uint8_t nrf24l01ReceivePayload(uint8_t *buffer, uint8_t size);
uint8_t nrf24l01StatusClear(void);
uint8_t nrf24l01StatusClearMaxRT(void);
uint8_t nrf24l01StatusClearTXDS(void);
uint8_t nrf24l01StatusClearRXDR(void);
uint8_t nrf24l01FlushTX(void);
uint8_t nrf24l01FlushRX(void);
void nr24l01SetCE(void);
void nr24l01ResetCE(void);
uint8_t nrf24l01Pend(uint32_t ticks);
//=============================


#endif /* NRF24L01_H_ */
