/**
 * @file radio.h
 * @brief Implements radio communication with NRF24L01 device for the Garmo module.
 *
 * This is a very simple SPI driver. It is not flexible and most options are
 * hard coded. However, it has integration with the FreeRTOS kernel, using
 * queues to transmit and receive data. Although this is not the best
 * approach, it does allow to pend on reception of data, which can be useful.
 * For a list of hard-coded settings, see spiInitialize.
 *
 * For best performance, data should not be written with multiple calls to
 * spiWrite. Writing all data at once will be much more efficient. All data
 * received for each byte sent is stored in the RX queue and can be retrieved
 * after returning from spiWrite. As a side note, spiWrite function does not
 * return after all data is actually sent. The function returns after all
 * data is actually enqueued. The function spiWaitTX can be used to wait
 * for the peripheral to be idle, ensuring transmissions have been finalized.
 *
 * The spiRead function pends on the RX queue and can be used to wait
 * indefinitely for a byte to arrive, if necessary (although the module only
 * works in master mode).
 *
 *	Current version: 0.1.1
 *
 *	-v0.1.0:
 *		- Adapted to new model
 *
 * -v0.1.1:
 *      - General improvements
 *      - Sets SSI and SSM bits during initialization
 *      - Initial documentation
 *
 *	Melhorias
 *		- Melhorar uso das filas (criar somente a quantidade
 *		necessária)
 *		- "Yield from interrupt" individual
 *
 *  Created on: Dec 9, 2018
 *      Author: Marco
 */

#ifndef SPI_H_
#define SPI_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configSPI1_ENABLED			1
#define configSPI2_ENABLED			0
#define configSPI3_ENABLED			0

/* RX and TX queue size */
#define configSPI1_RXQ_SIZE			40
#define configSPI1_TXQ_SIZE			40

#define configSPI2_RXQ_SIZE			25
#define configSPI2_TXQ_SIZE			25

#define configSPI3_RXQ_SIZE			25
#define configSPI3_TXQ_SIZE			25

#define configSPI1_INTERRUPT_YIELD	0
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
/** @brief Initializes the SPI peripheral.
 *
 * Initialization consists of setting the SPI peripheral, settings GPIO pins
 * and enabling interrupts. In addition, the TX and RX queues are created.
 *
 * This initialization is not flexible. The following settings are
 * hard-coded:
 *
 * - 2-line unidirectional mode is selected.
 * - Output in bidirectional mode is disabled.
 * - Hardware CRC calculation is disabled.
 * - Data frame is 8-bit format.
 * - SSM and SSI bits are set (software slave management enabled).
 * - MSB is transmitted first.
 * - Master mode is enabled.
 * - CK is 0 when idle (CPOL is cleared).
 * - Captures data in the first clock transition (CPHA is cleared).
 * - RX buffer not empty interrupt always enabled.
 * - TX buffer empty enabled when TX queue is not empty.
 * - Error interrupt is masked.
 * - SS output is disabled.
 * - TX buffer DMA is disabled.
 * - RX buffe DMA is disabled.
 *
 * In addition, SPI GPIO pins cannot be remapped. The SPI GPIO pins are:
 *
 * - SPI 1: PORTA
 *  - MOSI: GPIO_7
 *  - MISO: GPIO_6
 *  - CLK:  GPIO_5
 * - SPI 2: PORTB
 *  - MOSI: GPIO_P15
 *  - MISO: GPIO_P14
 *  - CLK:  GPIO_P13
 * - SPI 3: PORTB
 *  - MOSI: GPIO_P5
 *  - MISO: GPIO_P4
 *  - CLK:  GPIO_P3
 *
 * SPIs to be used must be enabled through the defines in this header, along
 * with the RX/TX queue sizes for each one.
 *
 * @param spi SPI to be initialized.
 * @param clockDiv Clock divisor for the selected SPI peripheral. Must be one
 *                 of spiClockDiv_t values.
 *
 * @return 0 if initialization succeeded, another value otherwise.
 *         -0x01 if hardware initialization fails. This can only occur if the
 *          SPI informed is not valid.
 *         -0x02 if creation of queues fail. This can only occur if, somehow,
 *          there is not enough RAM space when initializing the SPI.
 */
uint8_t spiInitialize(SPI_TypeDef *spi, uint16_t clockDiv);
//-----------------------------
/** @brief Sends data through SPI.
 *
 * Writes data to the TX queue. Data is then sent through interruption. This
 * function returns after data is enqueued, not after data is actually sent.
 * To wait until all SPI is idle, use spiWaitTX after calling spiWrite.
 *
 * After all bytes are enqueued, the transmission is triggered and all data
 * is sent through interruptions.
 *
 * For most efficiency, data should be sent with fewer call possible to
 * spiWrite. Although multiple calls work, it is less efficient.
 *
 * @param spi SPI used to send data.
 * @param buffer Pointer to buffer containing data to be sent. This pointer
 *               is not modified by the function.
 * @param nbytes Number of bytes to send though SPI.
 *
 * @return 0 if data was enqueued successfully, 1 if number of bytes exceed
 *         the number of spaces available in the TX queue.
 */
uint8_t spiWrite(SPI_TypeDef *spi, uint8_t *buffer, uint16_t nbytes);
//-----------------------------
/** @brief Reads data from SPI.
 *
 * Retrieves data received through SPI. Transmitting and receiving is
 * simultaneous and for each byte transmitted, one byte is received.
 * These bytes received are stored in the RX queue and can be retrieved with
 * this function.
 *
 * A call to this function removes one byte from the RX queue. This function
 * basically pends on the queue. If it is not empty, it returns right away
 * with the data.
 *
 * @param spi SPI to read data.
 * @param buffer Pointer to buffer to store retrieved data.
 * @param waitcycles Number of ticks to wait if the RX queue is empty.
 *
 * @return 0 if a byte was succesfully received from the queue, 1 if the
 *         number of ticks informed expired and no data was received.
 */
uint8_t spiRead(SPI_TypeDef *spi, uint8_t *buffer, uint32_t waitcycles);
//-----------------------------
/** @brief Waits until transmission is completed.
 *
 * This function can be used to wait until a transmission is completed. More
 * precisely, this function returns only when the SPI peripheral is idle,
 * that is, there is not ongoing transmission/reception.
 *
 * @param spi SPI to wait until idle.
 * @param waitcycles Number of cycles to wait for SPI to become idle.
 *
 * @return 0 if SPI became idle, 1 if number of waitcyles expired and the SPI
 *         did not become idle.
 */
uint8_t spiWaitTX(SPI_TypeDef *spi, uint32_t waitcycles);
//-----------------------------
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
/** @brief Divisor for SPI clock.
 *
 * When SPI 1 is selected, the clock comes from APB2, which can run @ 72 MHz.
 * For all other SPIs, clock comes from APB1, which can be different from
 * APB2 and runs @ 36 MHz maximum.
 */
typedef enum{
	SPI_CLK_DIV_2 = 0,
	SPI_CLK_DIV_4,
	SPI_CLK_DIV_8,
	SPI_CLK_DIV_16,
	SPI_CLK_DIV_32,
	SPI_CLK_DIV_64,
	SPI_CLK_DIV_128,
	SPI_CLK_DIV_256
}spiClockDiv_t;
//-----------------------------
//=============================

#endif /* SPI_H_ */
