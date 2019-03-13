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
//=============================


//=============================
/*--------- Defines ---------*/
//=============================
#define configSERIAL_MAX_IDS	10

#define configSERIAL_START_MAX_DELAY	2000
#define configSERIAL_ID_MAX_DELAY		100

#define configSERIAL_START_BYTE			0x55
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef void(*serialHandler_t)(void);
typedef struct{
	uint32_t id;
	serialHandler_t ptr;
	uint8_t *buffer;
	uint8_t dataAvailable;
}serial_t;
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t serialInstallID(serial_t *serial, uint32_t id, uint8_t *buffer, serialHandler_t handler);
uint8_t serialRun(void);
//=============================


#endif /* SERIAL_H_ */
