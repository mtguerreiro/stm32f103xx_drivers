/*
 * serial.c
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "serial.h"

/* Drivers */
#include "uart.h"
//=============================

//=============================
/*---------- Enums ----------*/
//=============================
//-----------------------------
typedef enum{
	ST_START,
	ST_ID,
	ST_END
}serialSMStates_t;
//-----------------------------
//=============================


//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef void(*serialHandler)(void);
typedef struct{
	uint32_t n;
	serial_t *serials[configSERIAL_MAX_IDS];
}serialControl_t;
serialControl_t serialControl = {.n = 0};
//-----------------------------
typedef void (*serialSMaction)(void);
typedef void (*serialSMhandle)(void);

typedef struct{
	serialSMStates_t state;
	serialSMaction functions[ST_END];
}serialSM_t;
serialSM_t serialSMControl;
//-----------------------------
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static void serialInitialize(void);
static void serialStateStart(void);
static void serialStateID(void);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t serialInstallID(serial_t *serial){

	if(serialControl.n >= configSERIAL_MAX_IDS) return 0;

	serialControl.serials[serialControl.n++] = serial;

	return 0;
}
//-----------------------------
uint8_t serialRun(void){

	serialInitialize();
	while(1){

	}
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void serialInitialize(void){

	uartInitialize(USART1, 115200);

	serialSMControl.state = ST_START;

	serialSMControl.functions[ST_START] = serialStateStart;
	serialSMControl.functions[ST_ID] = serialStateID;

}
//-----------------------------
static void serialStateStart(void){

	uint8_t buffer;

	if( uartRead(USART1, &buffer, configSERIAL_START_MAX_DELAY) ) return;

	serialSMControl.state = ST_ID;
}
//-----------------------------
//static void serialStateID(void){
//
//	uint
//
//}
//-----------------------------
//=============================
