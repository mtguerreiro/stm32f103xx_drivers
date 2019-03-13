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
//#include "uart.h"
//=============================

//=============================
/*---------- Enums ----------*/
//=============================
//-----------------------------
typedef enum{
	ST_START,
	ST_ID,
	ST_DATA_SIZE,
	ST_DATA,
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
static void serialStataDataSize(void);
static void serialStateData(void);
static uint8_t serialFindID(uint32_t id);
static uint8_t uartRead(uint8_t *buffer);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t serialInstallID(serial_t *serial, uint32_t id, uint8_t *buffer, serialHandler_t handler){

	/* Checks if ID is available */
	if( serialFindID(id) == 0 ){
		return 1;
	}

	if(serialControl.n >= configSERIAL_MAX_IDS) return 2;

	serialControl.serials[serialControl.n++] = serial;

	serial->id = id;
	serial->buffer = buffer;
	serial->ptr = handler;
	serial->dataAvailable = 0;

	return 0;
}
//-----------------------------
uint8_t serialRun(void){

	serialInitialize();
	while(1){
		serialSMControl.functions[serialSMControl.state]();
	}
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void serialInitialize(void){

	//uartInitialize(USART1, 115200);

	serialSMControl.state = ST_START;

	serialSMControl.functions[ST_START] = serialStateStart;
	serialSMControl.functions[ST_ID] = serialStateID;
	serialSMControl.functions[ST_DATA_SIZE] = serialStataDataSize;

}
//-----------------------------
static void serialStateStart(void){

	uint8_t buffer;

	//if( uartRead(USART1, &buffer, configSERIAL_START_MAX_DELAY) ) return;
	if( uartRead(&buffer) ) return;

	if( buffer != configSERIAL_START_BYTE ) return;

	serialSMControl.state = ST_ID;
}
//-----------------------------
static void serialStateID(void){

	uint8_t k;
	uint8_t buffer;
	uint32_t id;

	id = 0;
	k = 0;
	while(k < 4){
		//if( uartRead(USART1, &buffer, configSERIAL_ID_MAX_DELAY) ) break;
		if( uartRead(&buffer) ) break;
		id += (buffer << (k << 3));
		k++;
	}

	/*
	 * If k did not reach 4, we broke from the while loop before receiving
	 * all bytes to form the ID. Thus, we discard this frame.
	 */
	if(k != 4){
		serialSMControl.state = ST_START;
		return;
	}

	/* Checks if ID is valid */
	if( serialFindID(id) ){
		serialSMControl.state = ST_START;
		return;
	}

	serialSMControl.state = ST_DATA_SIZE;

}
//-----------------------------
static void serialStataDataSize(void){

	uint8_t k;
	uint8_t buffer;
	uint32_t size;

	size = 0;
	k = 0;
	while(k < 4){
		//if( uartRead(USART1, &buffer, configSERIAL_ID_MAX_DELAY) ) break;
		if( uartRead(&buffer) ) break;
		size += (buffer << (k << 3));
		k++;
	}

	/*
	 * If k did not reach 4, we broke from the while loop before receiving
	 * all bytes to form the data size. Thus, we discard this frame.
	 */
	if(k != 4){
		serialSMControl.state = ST_START;
		return;
	}

	serialSMControl.state = ST_DATA;
}
//-----------------------------
static void serialStateData(void){

	serialSMControl.state = ST_START;
	return;
}
//-----------------------------
static uint8_t serialFindID(uint32_t id){

	uint32_t k;

	k = 0;

	//if(serialControl.n == 0) return 1;

	while(k < serialControl.n){
		if(id == serialControl.serials[k]->id) break;
		k++;
	}

	if(k == serialControl.n) return 1;

	return 0;
}
//-----------------------------
//=============================

static uint8_t uartRead(uint8_t *buffer){

	static uint8_t buf[30] = {0x55, 0x55, 0x55, 0xAA, 0xAA, 0x04, 0x00, 0x00, 0x00};
	static uint8_t *ptr = buf;

	*buffer = *ptr++;
	return 0;
}
