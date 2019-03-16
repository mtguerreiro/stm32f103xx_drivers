/*
 * serial.h
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 */

#ifndef SERIAL_H_
#define SERIAL_H_

//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>

/* Drivers */
#include "uart.h"
//=============================

//=============================
/*-------- Func ptrs --------*/
//=============================
/* Function executed when data is received */
typedef void(*serialHandler_t)(void);
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configSERIAL_MAX_IDS			10

#define configSERIAL_START_MAX_DELAY	2000
#define configSERIAL_ID_MAX_DELAY		100

#define configSERIAL_START_BYTE			0x55
#define configSERIAL_STOP_BYTE			0x77
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef void(*serialHandler_t)(void);
typedef struct{
	uint32_t id;
	serialHandler_t handler;
	uint8_t *buffer;
	uint32_t dataSize;
	uint8_t dataAvailable;
}serial_t;
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void serialInitialize(USART_TypeDef *uart, uint32_t baud);
uint8_t serialRun(void);
uint8_t serialInstallID(serial_t *serial, uint32_t id, uint8_t *buffer, serialHandler_t handler);
//=============================

#endif /* SERIAL_H_ */
