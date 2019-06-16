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
//=============================


#endif /* MFRC522_H_ */
