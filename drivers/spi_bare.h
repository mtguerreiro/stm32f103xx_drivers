/**
 * @file spi.h
 * @brief Simple SPI driver for STM32F103x devices.
 *
 * This is just a quick SPI driver for SPI1. The SPI is set as master, CPHA and
 * CPOL are both set to 0, and the speed is set as APB2_CLK / 32.
 * NSS is not used.
 *
 *  Created on: Apr 21, 2020
 *      Author: marco
 */

#ifndef SPI_BARE_H_
#define SPI_BARE_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void spibareInitialize(void);
void spibareWrite(uint8_t data);
uint8_t spibareRead(void);
//=============================

#endif /* SPI_BARE_H_ */
