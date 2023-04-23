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
 *	-v0.1.2 (23/04/2023):
 *		- Decoupling nrf24l01 from MCU-specific functions
 *		- Improving function returns
 *		- TX/RX through dynamic payload only
 *		- Updating documentation
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include <stdint.h>
//=============================================================================

//=============================================================================
/*------------------------------- Definitions -------------------------------*/
//=============================================================================
typedef enum{
	NRF24L01_CMD_STATUS_OK = 0,
	NRF24L01_CMD_STATUS_SPI_ERROR = -1,
	NRF24L01_CMD_STATUS_TX_TO = -2,
	NRF24L01_CMD_STATUS_TX_MAX_RETRY = -3,
	NRF24L01_CMD_STATUS_RX_TO = -4,
	NRF24L01_CMD_STATUS_RX_PAYLOAD_ERROR = -5,
	NRF24L01_CMD_STATUS_PEND_TO = -6
}nrf24l01CmdStatus_t;

/** @brief Writes data through SPI.
 *
 *	@param buffer Pointer to buffer holding data.
 *	@param size Number of bytes to write.
 *	@param to Timeout.
 *
 *	@return 0 of write was successful, another value otherwise.
 */
typedef int8_t (*nrf24l01SpiWrite_t)(uint8_t *buffer, uint16_t size, uint32_t to);

 /** @brief Reads data from SPI.
  *
  *	@param buffer Pointer to buffer to hold data read.
  *	@param size Number of bytes to read.
  *	@param to Timeout.
  *
  *	@return 0 of read was successful, another value otherwise.
  */
typedef int8_t (*nrf24l01SpiRead_t)(uint8_t *buffer, uint16_t size, uint32_t to);

/** @brief Writes to the CSN pin.
 *
 * @param level Level to write to pin (0 - low, 1 - high).
 */
typedef void (*nrf24l01CSNWrite_t)(uint8_t level);

/** @brief Writes to the CE pin.
 *
 * @param level Level to write to pin (0 - low, 1 - high).
 */
typedef void (*nrf24l01CEWrite_t)(uint8_t level);

/** @brief Generates a delay, in microseconds.
 *
 * There are only two places where a delay is required. The first place is
 * when the radio is powered up. In this case, a delay of approximately 2 ms
 * is required.
 *
 * The second place is when a payload is to be sent. In this case, a high
 * level pulse must be generated at the CE pin, lasting at least
 * 10 microseconds.
 *
 * @param delay Delay, in microseconds.
 */
typedef void (*nrf24l01DelayMicrosec_t)(uint32_t delay);

/** @brief Clears any pending IRQ flag.
 *
 * This is related only to how the detection and synchronization of the IRQ
 * pin is implemented on each specific case.
 *
 * For example, a semaphore can be used to pend on the IRQ pin. The pend
 * function waits until the semaphore is release. The semaphore is released
 * when an interrupt on the IRQ pin is detected. At the interrupt handler,
 * the semaphore is released and wakes the function that was pending on it.
 *
 * However, for this to work correctly, the semaphore must be taken before
 * the pending function is called. If the semaphore is not taken, the pend
 * function will immediately return, although there was no change on the
 * IRQ pin.
 *
 * This function is called before any transmission reception event, and this
 * function should implement any pre-pend synchronism here.
 */
typedef void (*nrf24l01IrqClear_t)(void);

/** @brief Pends on the IRQ pin.
 *
 * This is related only to how the detection and synchronization of the IRQ
 * pin is implemented on each specific case.
 *
 * For example, a semaphore can be used to pend on the IRQ pin. The pend
 * function waits until the semaphore is release. The semaphore is released
 * when an interrupt on the IRQ pin is detected. At the interrupt handler,
 * the semaphore is released and wakes the function that was pending on it.
 *
 * This functions is called just after a transmission or receive event. This
 * function is expected to return only after an interrupt on the IRQ pin was
 * detected.
 *
 * @param to Timeout to wait on pend.
 */
typedef int8_t (*nrf24l01IrqPend_t)(uint32_t to);
//=============================================================================


//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
/* Initialization */
/*
 * These functions are used to initialize the NRF24L01 device and for
 * powering it up/down.
 */
//-----------------------------------------------------------------------------
/** @brief Initializes the library.
 *
 * Basically, all the callbacks are set, CSN is set high and CE is set low.
 *
 * @param spiWrite Function to write data through SPI.
 * @param spiRead Function to read data from SPI.
 * @param csnWrite Function to write to the CSN pin.
 * @param ceWrite Function to write to the CE pin.
 * @param delay Function to generate a delay, in microseconds.
 * @param irqClear Function to clear any pending flags related to the IRQ pin.
 * @param irqPend Function to pend on the IRQ pin.
 */
void nrf24l01Initialize(nrf24l01SpiWrite_t spiWrite, nrf24l01SpiRead_t spiRead,
		nrf24l01CSNWrite_t csnWrite, nrf24l01CEWrite_t ceWrite,
		nrf24l01DelayMicrosec_t delay,
		nrf24l01IrqClear_t irqClear, nrf24l01IrqPend_t irqPend);
 //-----------------------------------------------------------------------------
/** @brief Sets the power up bit in the CONFIG register.
 *
 * First, the CONFIG register is read to ensure that previous bits are not
 * overwritten. The power up bit is then set and the register is read once
 * again to ensure data was written correctly.
 *
 * @return 0 if device was powered up successfully, or a negative error code.
 */
int8_t nrf24l01PowerUp(void);
//-----------------------------------------------------------------------------
/** @brief Clears the power down bit in the CONFIG register.
 *
 * First, the CONFIG register is read to ensure that previous bits are not
 * overwritten. The power down bit is then cleared and the register is read
 * once again to ensure data was written correctly.
 *
 * @return 0 if device was powered down successfully, or a negative error code.
 */
int8_t nrf24l01PowerDown(void);
//-----------------------------------------------------------------------------
/** @brief Sets the main settings for the device.
 *
 * This settings are valid for both RX and TX modes.
 *
 * The following configurations are set:
 *
 * - Auto-ack is enabled for data pipe 0.
 * - RX address is enabled on data pipe 0.
 * - RX and TX addresses are set to the same value .
 * - RF channel is set.
 * - Retransmission time is set to 2000 us and number of retries is set to 5.
 * - Dynamic payload is enabled on data pipe 0.
 * - The device is set as primary TX.
 * - RX and TX FIFOs are flushed.
 * - Status flags are cleared.
 *
 * @param address Pointer to buffer containing address. This is used both for
 *                RX and TX.
 * @param channel RF channel.
 *
 * @return 0 if device was set successfully, or a negative error code.
 *
 */
int8_t nrf24l01SetConfigs(uint8_t *address, uint8_t channel);
//-----------------------------------------------------------------------------

/* Transmitting/Receiving */
/*
 * These functions deal with sending/receiving data to/from the NRF radio, as
 * well as dealing with flushing of the RX/TX FIFOs.
 */
//-----------------------------------------------------------------------------
/** @brief Reads data from the NRF24L01 device.
 *
 * Pends on the IRQ pin to wait for new data to become available. When this
 * function is called, the CE pin is set and the device starts listening for
 * incoming transmissions. As a side note, current consumption increases when
 * the device starts listening.
 *
 * Moreover, TX and RX FIFOs are flushed.
 *
 * As soon as the IRQ pin is driven low, new data is available and the RX
 * register is read. After reading the RX register, the CE pin in reset and
 * the device stops listening for incoming transmissions.
 *
 * @param buffer Pointer to buffer to store data read from the NRF24L01. This
 * 				 buffer must be at least 32 bytes long, which is the maximum
 * 				 number of bytes that can be received.
 * @param timeout IRQ timeout. This depends on how the IRQ functions are
 * 				  implemented.
 *
 * @return if data was received, returns the number of bytes received.
 * 		   Otherwise a negative error code.
 */
int8_t nrf24l01Read(uint8_t *buffer, uint32_t timeout);
//-----------------------------------------------------------------------------
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
 *  - If the IRQ pin is not driven low, the NRF's TX buffer is flushed and
 *    the maximum retries and TXDS flags are cleared.
 *  - If the IRQ pin is driven low and there was an issue reading the status
 *    register, the TX buffer is flushed and the maximum retries and TXDS
 *    flags are cleared.
 *  - If transmission fails due to the maximum number of retries being
 *    exceeded, the TX buffer is flushed and the maximum number of retries
 *    flag is cleared.
 *  - If data is sent successfully, the TX data sent flag is cleared.
 *
 * @param buffer Pointer to buffer containing data to be transmitted.
 * @param size Number of bytes to send.
 * @param timeout IRQ timeout. This depends on how the IRQ functions are
 * 				  implemented.
 *
 * @return 0 if data was sent successfully, or a negative error code.
 */
int8_t nrf24l01Write(uint8_t *buffer, uint8_t size, uint32_t timeout);
//-----------------------------------------------------------------------------
/** @brief Flushes the NRF's TX FIFO.
 *
 * @return 0 if flush command was sent successfully, or a negative error code.
 */
int8_t nrf24l01FlushTX(void);
//-----------------------------------------------------------------------------
/** @brief Flushes the NRF's RX FIFO.
 *
 * @return 0 if flush command was sent successfully, or a negative error code.
 */
int8_t nrf24l01FlushRX(void);
//-----------------------------------------------------------------------------

/* Settings */
/*
 * These functions are used to set several parameters, such as RX payload
 * size, RX/TX addresses, enabling/disabling auto-ack,RF channel, etc.
 */
//-----------------------------------------------------------------------------
/** @brief Sets the RX payload size, for data pipe 0.
 *
 * The maximum size is 32 bytes. A value higher then 32 will be truncated to
 * 32.
 *
 * @param size Size of payload for data pipe 0, in bytes
 *
 * @return 0 if register was set correctly, or a negative error code.
 */
int8_t nrf24l01SetRXPayloadSize(uint8_t size);
//-----------------------------------------------------------------------------
/** @brief Sets RX address for data pipe 0.
 *
 * @param pointer to buffer containing address.
 * @return 0 if address was set successfully, or a negative error code.
 */
int8_t nrf24l01SetRXAdress(uint8_t *address);
//-----------------------------------------------------------------------------
/** @brief Sets TX address for data pipe 0.
 *
 * @param pointer to buffer containing address.
 * @return 0 if address was set successfully, or a negative error code.
 */
int8_t nrf24l01SetTXAdress(uint8_t *address);
//-----------------------------------------------------------------------------
/** @brief Sets RF channel.
 *
 * @param channel RF channel.
 * @return 0 if RF channel was set successfully, or a negative error code.
 */
int8_t nrf24l01SetRFChannel(uint8_t channel);
//-----------------------------------------------------------------------------
/** @brief Sets retry time and maximum number of retries.
 *
 * @param time Data to be written to the RETR register.
 * @return 0 if RETR register was set successfully, or a negative error code.
 */
int8_t nrf24l01SetRetryTime(uint8_t time);
//-----------------------------------------------------------------------------
/** @brief Enables RX addresses on data pipes.
 *
 * @param pipes Data pipes to be enabled.
 * @return 0 if data pipes were set successfully, or a negative error code.
 */
int8_t nrf24l01EnableRXADDR(uint8_t pipes);
//-----------------------------------------------------------------------------
/** @brief Disables RX addresses on data pipes.
 *
 * @param pipes Data pipes to be disabled.
 * @return 0 if data pipes were cleared successfully, or a negative error code.
 */
int8_t nrf24l01DisableRXADDR(uint8_t pipes);
//-----------------------------------------------------------------------------
/** @brief Enables auto-ack on data pipes.
 *
 * @param pipes Data pipes to enable auto-ack.
 * @return 0 if auto-ack were enabled successfully, or a negative error code.
 */
int8_t nrf24l01EnableAA(uint8_t pipes);
//-----------------------------------------------------------------------------
/** @brief Disables auto-ack on data pipes.
 *
 * @param pipes Data pipes to disable auto-ack.
 * @return 0 if auto-ack were disabled successfully, or a negative error code.
 */
int8_t nrf24l01DisableAA(uint8_t pipes);
//-----------------------------------------------------------------------------
/** @brief Sets device as primary RX.
 *
 * @return 0 if device was set successfully, or a negative error code.
 */
int8_t nrf24l01SetPRX(void);
//-----------------------------------------------------------------------------
/** @brief Sets device as primary TX.
 *
 * @return 0 if device was set successfully, or a negative error code.
 */
int8_t nrf24l01SetPTX(void);
//-----------------------------------------------------------------------------
/** @brief Sets antenna gain.
 *
 * @param power See manual for more info
 * @return 0 if power was set successfully, or a negative error code.
 */
int8_t nrf24l01TXPower(uint8_t power);
//-----------------------------------------------------------------------------
/** @brief Sets dynamic payload length (DPL).
 *
 * @param dpl 1 to enable DPL, 0 to disable it.
 * @return 0 if DPL was set successfully, or a negative error code.
 */
int8_t nrf24l01SetDPL(uint8_t dpl);
//-----------------------------------------------------------------------------
/** @brief Sets dynamic payload length on data pipe 0.
 *
 * @param dpl 1 to enable DPL, 0 to disable it.
 * @return 0 if DPL was set successfully, or a negative error code.
 */
int8_t nrf24l01SetDynPD(uint8_t dpl);
//-----------------------------------------------------------------------------
/** @brief Reads the received payload's width.
 *
 * @return Payload's width. If an error occurred, or a negative error code is
 * 		   returned instead.
 */
int8_t nrf24l01ReadRXPayloadWidth(void);
//-----------------------------------------------------------------------------
/* Direct comm */
/*
 * These functions give direct access to the NRF's register.
 */
//-----------------------------------------------------------------------------
int8_t nrf24l01ReadRegister(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------
int8_t nrf24l01WriteRegister(uint8_t reg, uint8_t *buffer);
//-----------------------------------------------------------------------------

/* Status */
int8_t nrf24l01ReadSR(uint8_t *status);
int8_t nrf24l01StatusClear(void);
int8_t nrf24l01StatusClearMaxRT(void);
int8_t nrf24l01StatusClearTXDS(void);
int8_t nrf24l01StatusClearRXDR(void);

/* Control */
void nr24l01SetCE(void);
void nr24l01ResetCE(void);
int8_t nrf24l01Pend(uint32_t ticks);

//=============================================================================


#endif /* NRF24L01_H_ */
