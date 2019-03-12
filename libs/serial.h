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
//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t serialInstallID(serial_t *serial);
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef void(*serialHandler)(void);
typedef struct{
	uint32_t id;
	serialHandler *ptr;
	uint8_t *buffer;
	uint8_t dataAvailable;
}serial_t;
//-----------------------------
//=============================

#endif /* SERIAL_H_ */
