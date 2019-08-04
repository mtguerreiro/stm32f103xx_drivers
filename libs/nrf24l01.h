/**
 * @file nrf24l01.h
 * @brief Simple driver/library for the NRF24L01 device.
 *
 *
 * This is a simple driver for the NRF24L01 device, designed for the the
 * STM32F103xx family and integrated with the FreeRTOS kernel. Documentation
 * still in its early stages and no examples are provided yet.
 *
 * The library is not very flexible and for now it only works with data
 * pipe 0. The library was designed to work with the IRQ pin as an external
 * interrupt source on the STM32F103 MCU. Transmission and reception relies
 * on this interrupt and to work without it would need serious modifications
 * in the source code.
 *
 * -v0.1.0:
 *  - Initial version
 *
 *	-v0.1.1:
 *		- Added function to pend on IRQ pin
 *		- Added status clear function
 *		- Added RX and TX initialization functions
 *		- Documentation
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
 * 			- Para leitura, é necessário informar a quantidade de bytes a
 * 			serem lidas. Porém, os bytes são apenas recebidos se a quantidade
 * 			configurada no registrador do payload size for correta. Uma
 * 			melhoria é ler esse registrador para não ser necessário informar
 * 			a quantidade a ser lida ao chamar a função nrf24l01Read.
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
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
/* Initialization */
/*
 * These functions are used to initialize the NRF24L01 device and for
 * powering it up/down.
 */
//-----------------------------
/** @brief Prepares MCU's hardware for the NRF24L01 device.
 *
 * Initializes the SPI peripheral (hard-coded as SPI 1) and the CE, CSN and
 * IRQ GPIO pins. CE and CSN can be freely modified in the source file
 * through defines. The IRQ pin, however, is used as external interrupt and
 * should be left with the defined value.
 *
 * @return 0 if hardware was initialized correctly, another value otherwise.
 *         -0x01 if SPI failed to initialized. This will only occurs if
 *          creating the RX and TX queues failed, which can be due to
 *          insufficient memory defined in the heap.
 *         -0x02 if failed to create the NRF's semaphore. This can only occur
 *          if there is insufficient RAM memory.
 */
uint8_t nrf24l01Initialize(void);
//-----------------------------
/** @brief Sets the power up bit in the CONFIG register.
 *
 * First, the CONFIG register is read to ensure that previous bits are not
 * overwritten. The power up bit is then set and the register is read once
 * again to ensure data was written correctly.
 *
 * @return 0 if device was powered up successfully, another value otherwise.
 *         -0x01 if failed to read NRF's CONFIG register
 *         -0x02 if data written/read did not match.
 */
uint8_t nrf24l01PowerUp(void);
//-----------------------------
/** @brief Clears the power down bit in the CONFIG register.
 *
 * First, the CONFIG register is read to ensure that previous bits are not
 * overwritten. The power down bit is then cleared and the register is read
 * once again to ensure data was written correctly.
 *
 * @return 0 if device was powered down successfully, another value otherwise.
 *         -0x01 if failed to read NRF's CONFIG register
 *         -0x02 if data written/read did not match.
 */
uint8_t nrf24l01PowerDown(void);
//-----------------------------
/** @brief Sets device as transmitter.
 *
 * The following configurations are set:
 *
 * - Auto-ack is enabled for data pipe 0.
 * - RX address is enabled on data pipe 0.
 * - RX and TX addresses are set to the same value .
 * - RF channel is set.
 * - Retransmission time is set to 2000 us and number of retries is set to 5.
 * - Number of bytes for RX payload is set.
 * - The device is set as primary TX.
 * - RX and TX FIFOs are flushed.
 * - Status flags are cleared.
 *
 * @param address Pointer to buffer containing address. This is used both for
 *                RX and TX.
 * @param plSize Payload size, in bytes.
 * @param channel RF channel.
 *
 * @return 0 if all settings were successful, another value otherwise.
 *         -0x01 if failed to enable auto-ack.
 *         -0x02 if failed to enable RX address on data pipe 0.
 *         -0x03 if failed to set retransmission time and number of retries.
 *         -0x04 if failed to set RF channel.
 *         -0x05 if failed to set RX address.
 *         -0x06 if failed to set TX address.
 *         -0x07 if failed to set RX payload size.
 *         -0x08 if failed to set as primary TX.
 *         -0x09 if failed to flush TX FIFO.
 *         -0x0A if failed to flush RX FIFO.
 *         -0x0B if failed to clear status.
 */
uint8_t nrf24l01SetTX(uint8_t *address, uint8_t plSize, uint8_t channel);
//-----------------------------
/** @brief Sets device as receiver.
 *
 * The following configurations are set:
 *
 * - Auto-ack is enabled for data pipe 0.
 * - RX address is enabled on data pipe 0.
 * - RX and TX addresses are set to the same value .
 * - RF channel is set.
 * - Retransmission time is set to 2000 us and number of retries is set to 5.
 * - Number of bytes for RX payload is set.
 * - The device is set as primary RX.
 * - RX and TX FIFOs are flushed.
 * - Status flags are cleared.
 *
 * @param address Pointer to buffer containing address. This is used both for
 *                RX and TX.
 * @param plSize Payload size, in bytes.
 * @param channel RF channel.
 *
 * @return 0 if all settings were successful, another value otherwise.
 *         -0x01 if failed to enable auto-ack.
 *         -0x02 if failed to enable RX address on data pipe 0.
 *         -0x03 if failed to set retransmission time and number of retries.
 *         -0x04 if failed to set RF channel.
 *         -0x05 if failed to set RX address.
 *         -0x06 if failed to set TX address.
 *         -0x07 if failed to set RX payload size.
 *         -0x08 if failed to set as primary RX.
 *         -0x09 if failed to flush TX FIFO.
 *         -0x0A if failed to flush RX FIFO.
 *         -0x0B if failed to clear status.
 */
uint8_t nrf24l01SetRX(uint8_t *address, uint8_t plSize, uint8_t channel);
//-----------------------------

/* Transmitting/Receiving */
/*
 * These functions deal with sending/receiving data to/from the NRF radio, as
 * well as dealing with flushing of the RX/TX FIFOs.
 */
//-----------------------------
/** @brief Reads data from the NRF24L01 device.
 *
 * Pends on the IRQ pin to wait for new data to become available. When this
 * function is called, the CE pin is set and the device starts listening for
 * incoming transmissions. As a side note, current consumption increases when
 * the device starts listening.
 *
 * As soon as the IRQ pin is driven low, new data is available and the RX
 * register is read. After reading the RX register, the CE pin in reset and
 * the device stops listening for incoming transmissions.
 *
 * @param buffer Pointer to buffer to store data read from the NRF24L01. The
 *               pointer's value is not modified by this function.
 * @param size RX payload size expected.
 * @param pendTicks Number of ticks to wait for the IRQ pin to be driven low.
 *
 * @return 0 if new data was received and saved to buffer, 1 if number of
 *         ticks expired and the IRQ pin was not driven low.
 */
uint8_t nrf24l01Read(uint8_t *buffer, uint8_t size, uint32_t pendTicks);
//-----------------------------
/** @brief Writes data to the NRF24L01 device.
 *
 * Data is written to the NRF's TX buffer and a high pulse to the CE pin is
 * applied to start radio transmission.
 *
 * After starting transmission, the IRQ pin is expected to be driven low. The
 * IRQ pin should be driven low either if transmission was completed
 * successfully or if the maximum number of retries is exceeded. If the IRQ
 * pin was not driven low, something went wrong, probably with the SPI.
 *
 * Several scenarios are possible after sending data to the NRF device and
 * applying the pulse on the CE pin. Each scenario is described below.
 *
 *  - If the IRQ pin is not drive low, the NRF's TX buffer is flushed and the
 *    maximum retries and TXDS flags are cleared.
 *  - If the IRQ pin is driven low and there was an issue reading the status
 *    register, the TX buffer is flushed and the maximum retries and TXDS
 *    flags are cleared.
 *  - If transmission fails due to the maximum number of retries being
 *    exceeded, the TX buffer is flushed and the maximum number of retries
 *    flag is cleared.
 *  - If data is sent successfully, the TX data sent flag is cleared.
 *
 * @param buffer Pointer to buffer containing data to be transmitted. The
 *               pointer's value is not modified by this function.
 * @param size Number of bytes to send.
 * @param pendTicks Number of ticks to wait for the IRQ pin to be driven low.
 *
 * @return 0 if data was sent successfully, another value otherwise.
 *         -0x01 if number of ticks expired and the IRQ pin was not driven
 *          low.
 *         -0x02 if there was an error reading the status register.
 *         -0x03 if exceeded maximum number of retries.
 */
uint8_t nrf24l01Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks);
//-----------------------------
/** @brief Flushes the NRF's TX FIFO.
 *
 * @return 0 if flush command was sent successfully, 1 otherwise.
 */
uint8_t nrf24l01FlushTX(void);
//-----------------------------
/** @brief Flushes the NRF's RX FIFO.
 *
 * @return 0 if flush command was sent successfully, 1 otherwise.
 */
uint8_t nrf24l01FlushRX(void);
//-----------------------------

/* Settings */
/*
 * These functions are used to set several parameters, such as RX payload
 * size, RX/TX addresses, enabling/disabling auto-ack,RF channel, etc.
 */
//-----------------------------
/** @brief Sets the RX payload size, for data pipe 0.
 *
 * The maximum size is 32 bytes. A value superior to 32 will be truncated to
 * 32.
 *
 * @param size Size of payload for data pipe 0, in bytes
 *
 * @return 0 if register was set correctly, 1 otherwise.
 */
uint8_t nrf24l01SetRXPayloadSize(uint8_t size);
//-----------------------------
/** @brief Sets RX address for data pipe 0.
 *
 * @param pointer to buffer containing address.
 * @return 0 if address was set successfully, 1 otherwise.
 */
uint8_t nrf24l01SetRXAdress(uint8_t *address);
//-----------------------------
/** @brief Sets TX address for data pipe 0.
 *
 * @param pointer to buffer containing address.
 * @return 0 if address was set successfully, 1 otherwise.
 */
uint8_t nrf24l01SetTXAdress(uint8_t *address);
//-----------------------------
/** @brief Sets RF channel.
 *
 * @param channel RF channel.
 * @return 0 if RF channel was set successfully, 1 otherwise.
 */
uint8_t nrf24l01SetRFChannel(uint8_t channel);
//-----------------------------
/** @brief Sets retry time and maximum number of retries.
 *
 * @param time Data to be written to the RETR register.
 * @return 0 if RETR register was set successfully, 1 otherwise.
 */
uint8_t nrf24l01SetRetryTime(uint8_t time);
//-----------------------------
/** @brief Enables RX addresses on data pipes.
 *
 * @param pipes Data pipes to be enabled.
 * @return 0 if data pipes were set successfully, another value otherwise.
 *         -0x01 if failed to read REG_EN_RXADDR register.
 *         -0x02 if failed to set REG_EN_RXADDR register.
 */
uint8_t nrf24l01EnableRXADDR(uint8_t pipes);
//-----------------------------
/** @brief Disables RX addresses on data pipes.
 *
 * @param pipes Data pipes to be disabled.
 * @return 0 if data pipes were cleared successfully, another value otherwise.
 *         -0x01 if failed to read REG_EN_RXADDR register.
 *         -0x02 if failed to set REG_EN_RXADDR register.
 */
uint8_t nrf24l01DisableRXADDR(uint8_t pipes);
//-----------------------------
/** @brief Enables auto-ack on data pipes.
 *
 * @param pipes Data pipes to enable auto-ack.
 * @return 0 if auto-ack were enabled successfully, another value otherwise.
 *         -0x01 if failed to read REG_EN_AA register.
 *         -0x02 if failed to set REG_EN_AA register.
 */
uint8_t nrf24l01EnableAA(uint8_t pipes);
//-----------------------------
/** @brief Disables auto-ack on data pipes.
 *
 * @param pipes Data pipes to disable auto-ack.
 * @return 0 if auto-ack were disabled successfully, another value otherwise.
 *         -0x01 if failed to read REG_EN_AA register.
 *         -0x02 if failed to set REG_EN_AA register.
 */
uint8_t nrf24l01DisableAA(uint8_t pipes);
//-----------------------------
/** @brief Sets device as primary RX.
 *
 * @return 0 if device was set successfully, another value otherwise.
 *         -0x01 if failed to read REG_CONFIG register.
 *         -0x02 if failed to set REG_CONFIG register.
 */
uint8_t nrf24l01SetPRX(void);
//-----------------------------
/** @brief Sets device as primary TX.
 *
 * @return 0 if device was set successfully, another value otherwise.
 *         -0x01 if failed to read REG_CONFIG register.
 *         -0x02 if failed to set REG_CONFIG register.
 */
uint8_t nrf24l01SetPTX(void);
//-----------------------------
/** @brief Sets antenna gain.
 *
 * @param power See manual for more info
 * @return 0 if power was set successfully, another value otherwise.
 *         -0x01 if failed to read REG_RF_SETUP register.
 *         -0x02 if failed to set REG_RF_SETUP register.
 */
uint8_t nrf24l01TXPower(uint8_t power);
//-----------------------------

/* Direct comm */
/*
 * These functions give direct access to the NRF's register.
 */
//-----------------------------
uint8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer);
//-----------------------------
uint8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer);
//-----------------------------

/* Status */
uint8_t nrf24l01ReadSR(uint8_t *status);
uint8_t nrf24l01StatusClear(void);
uint8_t nrf24l01StatusClearMaxRT(void);
uint8_t nrf24l01StatusClearTXDS(void);
uint8_t nrf24l01StatusClearRXDR(void);

/* Control */
void nr24l01SetCE(void);
void nr24l01ResetCE(void);
uint8_t nrf24l01Pend(uint32_t ticks);
//=============================


#endif /* NRF24L01_H_ */
