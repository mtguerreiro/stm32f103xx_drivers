/**
 * @file mfrc522.h
 * @brief Simple driver/library for the MFRC522 device.
 *
 *
 * This is a simple driver for the MFRC522 device, designed for the the
 * STM32F103xx family and integrated with the FreeRTOS kernel. Documentation
 * still in its early stages and no examples are provided yet.
 *
 * Current version: v0.1.0.
 *
 * -v0.1.0:
 *  - Initial version
 *
 *  Created on: June 15, 2019
 *      Author: Marco
 */

#ifndef MFRC522_H_
#define MFRC522_H_

//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>
//=============================

//=============================
/*--------- Defines ---------*/
//=============================

/* SPI */
//-----------------------------
#define configMFRC522_SPI			SPI2
#define configMFRC522_SPI_CLK_DIV	SPI_CLK_DIV_64
//-----------------------------

/* GPIO */
//-----------------------------
#define configMFRC522_NSS_PORT		GPIOB
#define configMFRC522_NSS_PIN		GPIO_P1

#define configMFRC522_RESET_PORT	GPIOB
#define configMFRC522_RESET_PIN		GPIO_P9

/* Do not change this */
#define configMFRC522_IRQ_PORT		GPIOB
#define configMFRC522_IRQ_PIN		GPIO_P3
//-----------------------------
//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
/** @brief Prepares MCU's hardware for the MFRC522 device.
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
 *         -0x02 if failed to create the MFRC's semaphore. This can only occur
 *          if there is insufficient RAM memory.
 */
uint8_t mfrc522Initialize(void);
//-----------------------------
/** @brief Reads a register from the MFRC522.
 *
 * @param reg Register to read.
 * @param buffer Buffer to save value read from register.
 * @return 0 if register was read successfully, 1 otherwise.
 */
uint8_t mfrc522ReadRegister(uint8_t reg, uint8_t *buffer);
//-----------------------------
/** @brief Writes to a MFRC522's register
 *
 * @param reg Register to write.
 * @param data Data to be written.
 *
 * @return 0 if register was sent through SPI, 1 otherwise.
 */
uint8_t mfrc522WriteRegister(uint8_t reg, uint8_t data);
//-----------------------------
/** @brief Resets the MFRC522. */
void mfrc522SoftReset(void);
//-----------------------------
/** @brief Performs a self digital test.
 *
 * After performing a self-test, the MFRC's version can be obtained with
 * mfrc522GetVersion().
 *
 * @param timeout Attempts to read the FIFO buffer before returning.
 *
 * @return 0 if test passed, 1 otherwise.
 */
uint8_t mfrc522SelfTest(uint32_t timeout);
//-----------------------------
/** @brief Flushes the MFRC522 FIFO buffer. */
void mfrc522FIFOFlush(void);
//-----------------------------
/** @brief Writes to the FIFO buffer.
 *
 * @param buffer Buffer containing data to be written.
 * @param nbytes Number of bytes to write.
 */
void mfrc522FIFOWrite(uint8_t* buffer, uint8_t nbytes);
//-----------------------------
/** @brief Reads data from the FIFO buffer.
 *
 * @param buffer Buffer to store read data.
 * @param nbytes Number of bytes to read.
 */
void mfrc522FIFORead(uint8_t* buffer, uint8_t nbytes);
//-----------------------------
/** @brief Gets the FIFO level.
 *
 * @return FIFO level.
 */
uint8_t mfrc522FIFOLevel(void);
//-----------------------------
/** @brief Gets the MFRC's version.
 *
 * This functions is valid only after performing a self-test.
 *
 * @return MFRC522's version.
 */
uint8_t mfrc522GetVersion(void);
//-----------------------------
//=============================


#endif /* MFRC522_H_ */
