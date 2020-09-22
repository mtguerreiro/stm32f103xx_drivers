/*
 * serial.h
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 *
 *	Current version: v0.1.3
 *
 *	-v0.1.0
 *		- Initial version
 *
 *	-v0.1.1
 *		- Added serialSend
 *		- Handles can be null
 *
 *	-v0.1.2
 *		- Corrected bug in serialSend
 *		- Added serialSendString
 *		- Added serial SendStringRaw
 *
 *	-v0.1.3
 *		- Modularization for serialSend and serialSendString
 *
 *	-v0.1.4
 *		- Added serialRead for master-like serial receive
 *
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
//#define configSERIAL_MAX_SER			2

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
//typedef struct{
//	uint32_t id[configSERIAL_MAX_IDS];
//	serialHandler_t handler[configSERIAL_MAX_IDS];
//	uint8_t *buffer;
//	uint32_t dataSize;
//	uint8_t dataAvailable;
//}serial_t;
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void serialInitialize(USART_TypeDef *uart, uint32_t baud, uint8_t *buffer);
uint8_t serialRun(void);
uint8_t serialInstallID(uint32_t id, serialHandler_t handler);
uint8_t serialSend(uint32_t id, uint8_t *buffer, uint32_t nbytes);
uint8_t serialSendString(uint32_t id, void *string);
uint8_t serialSendStringRaw(void *string);
uint8_t serialReceive(uint32_t id);
//=============================

#endif /* SERIAL_H_ */
