/*
 * spi_bare.h
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
