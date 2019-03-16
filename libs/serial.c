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
/*-------- Func ptrs --------*/
//=============================
typedef void(*serialVoid_t)(void);
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
	ST_STOP,
	ST_END
}serialSMStates_t;
//-----------------------------
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef struct{
	uint32_t n;
	serial_t *serials[configSERIAL_MAX_IDS];
}serialControl_t;
//-----------------------------
typedef struct{
	serialSMStates_t state;
	serialVoid_t functions[ST_END];
}serialSM_t;
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
static void serialStateStop(void);
static uint32_t serialFindID(uint32_t id);
static uint8_t uartRead(uint8_t *buffer);
//=============================

//=============================
/*--------- Globals ---------*/
//=============================
serialSM_t serialSMControl;
serialControl_t serialControl = {.n = 0};
serial_t *serialCurrent;
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t serialInstallID(serial_t *serial, uint32_t id, uint8_t *buffer, serialHandler_t handler){

	/* Checks if ID is available */
	if( serialFindID(id) != serialControl.n ){
		return 1;
	}

	if(serialControl.n >= configSERIAL_MAX_IDS) return 2;

	serialControl.serials[serialControl.n++] = serial;

	serial->id = id;
	serial->buffer = buffer;
	serial->handler = handler;
	serial->dataSize = 0;
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

	/* Initial state */
	serialSMControl.state = ST_START;

	/* State functions */
	serialSMControl.functions[ST_START] = serialStateStart;
	serialSMControl.functions[ST_ID] = serialStateID;
	serialSMControl.functions[ST_DATA_SIZE] = serialStataDataSize;
	serialSMControl.functions[ST_DATA] = serialStateData;
	serialSMControl.functions[ST_STOP] = serialStateStop;
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
	uint32_t ididx;

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

	/*
	 * Gets the index of the received ID. If the received ID is not found,
	 * the function returns n. In this case, we just go back to the first
	 * state.
	 */
	ididx = serialFindID(id);
	if( ididx == serialControl.n ){
		serialSMControl.state = ST_START;
		return;
	}

	serialCurrent = serialControl.serials[ididx];

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

	/*
	 * Checks if there is unread data in the current serial. If there is,
	 * we will just discard the new incoming data.
	 */
	if(serialCurrent->dataAvailable){
		serialSMControl.state = ST_START;
		return;
	}

	serialCurrent->dataSize = size;
	serialSMControl.state = ST_DATA;
}
//-----------------------------
static void serialStateData(void){

	uint32_t k;
	uint8_t *buffer;

	buffer = serialCurrent->buffer;
	k = 0;

	while(k < serialCurrent->dataSize){
		if( uartRead(buffer++) ) break;
		k++;
	}

	/* Checks if we got all the data that we were expecting */
	if(k != serialCurrent->dataSize){
		serialSMControl.state = ST_START;
		return;
	}

	serialSMControl.state = ST_STOP;
}
//-----------------------------
static void serialStateStop(void){

	uint8_t buffer;

	//if( uartRead(USART1, &buffer, configSERIAL_START_MAX_DELAY) ) return;
	if( uartRead(&buffer) ) {
		serialSMControl.state = ST_START;
		return;
	}

	if( buffer != configSERIAL_STOP_BYTE ) {
		serialSMControl.state = ST_START;
		return;
	}

	serialCurrent->dataAvailable = 1;
	serialCurrent->handler();
	serialSMControl.state = ST_START;
}
//-----------------------------
static uint32_t serialFindID(uint32_t id){

	uint32_t k;

	k = 0;

	while(k < serialControl.n){
		if(id == serialControl.serials[k]->id) break;
		k++;
	}

	if(k == serialControl.n) return serialControl.n;

	return k;
}
//-----------------------------
//=============================

static uint8_t uartRead(uint8_t *buffer){

	static uint8_t buf[30] = {0x55, 0x55, 0x55, 0xAA, 0xAA, 0x02, 0x00, 0x00, 0x00, 0x12, 0xB3, 0x77,
							  0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x01};
	static uint8_t *ptr = buf;

	*buffer = *ptr++;
	return 0;
}
