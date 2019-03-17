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
	USART_TypeDef *uart;
	uint32_t txBusy;
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
static void serialStateStart(void);
static void serialStateID(void);
static void serialStataDataSize(void);
static void serialStateData(void);
static void serialStateStop(void);
static uint32_t serialFindID(uint32_t id);
//=============================

//=============================
/*--------- Globals ---------*/
//=============================
serialSM_t serialSMControl;
serialControl_t serialControl = {.n = 0, .txBusy = 0};
serial_t *serialCurrent;
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void serialInitialize(USART_TypeDef *uart, uint32_t baud){

	uartInitialize(uart, baud);

	serialControl.uart = uart;

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
uint8_t serialRun(void){

	while(1){
		serialSMControl.functions[serialSMControl.state]();
	}
}
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
uint8_t serialSend(serial_t *serial, uint8_t *buffer, uint32_t nbytes){

	uint32_t id;
	uint32_t k;
	uint32_t size;
	uint8_t start[9];

	id = serial->id;
	/* Checks if ID is valid */
	if( id == serialControl.n ){
		return 1;
	}

	if(serialControl.txBusy){
		return 2;
	}
	serialControl.txBusy = 1;

	size = nbytes;
	/* Builds start of frame (start, ID, size) */
	start[0] = configSERIAL_START_BYTE;
	for(k = 0; k < 4; k++){
		start[k + 1] = (uint8_t)id;
		id =  id >> 8;
		start[k + 5] = (uint8_t)size;
		size = size >> 8;
	}

	/* Sends start of frame */
	if( uartWrite(serialControl.uart, start, 9) ){
		serialControl.txBusy = 0;
		return 3;
	}

	/* Sends data from buffer */
	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ){
		serialControl.txBusy = 0;
		return 4;
	}

	/* Sends end of frame */
	start[0] = configSERIAL_STOP_BYTE;
	if( uartWrite(serialControl.uart, start, 1) ){
		serialControl.txBusy = 0;
		return 5;
	}

	serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
uint8_t serialSendString(serial_t *serial, void *string){

	uint32_t id;
	uint32_t k;
	uint32_t size;
	uint32_t nbytes;
	uint8_t *buffer;
	uint8_t start[9];

	buffer = (uint8_t *)string;

	id = serial->id;
	/* Checks if ID is valid */
	if( id == serialControl.n ){
		return 1;
	}

	if(serialControl.txBusy){
		return 2;
	}
	serialControl.txBusy = 1;

	nbytes = 0;
	/* Size of string */
	while(*buffer++){
		nbytes++;
	}
	buffer--;
	buffer = buffer - nbytes;

	size = nbytes;
	/* Builds start of frame (start, ID, size) */
	start[0] = configSERIAL_START_BYTE;
	for(k = 0; k < 4; k++){
		start[k + 1] = (uint8_t)id;
		id =  id >> 8;
		start[k + 5] = (uint8_t)size;
		size = size >> 8;
	}

	/* Sends start of frame */
	if( uartWrite(serialControl.uart, start, 9) ){
		serialControl.txBusy = 0;
		return 3;
	}

	/* Sends data from buffer */
	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ){
		serialControl.txBusy = 0;
		return 4;
	}

	/* Sends end of frame */
	start[0] = configSERIAL_STOP_BYTE;
	if( uartWrite(serialControl.uart, start, 1) ){
		serialControl.txBusy = 0;
		return 5;
	}

	serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
uint8_t serialSendStringRaw(void *string){

	uint32_t nbytes;
	uint8_t *buffer;

	buffer = (uint8_t *)string;

	if(serialControl.txBusy){
		return 1;
	}
	serialControl.txBusy = 1;

	nbytes = 0;
	/* Size of string */
	while(*buffer++){
		nbytes++;
	}
	buffer--;
	buffer = buffer - nbytes;

	/* Sends data from buffer */
	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ){
		serialControl.txBusy = 0;
		return 4;
	}

	serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void serialStateStart(void){

	uint8_t buffer;

	if( uartRead(serialControl.uart, &buffer, configSERIAL_START_MAX_DELAY) ) return;

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
		if( uartRead(serialControl.uart, &buffer, configSERIAL_ID_MAX_DELAY) ) break;
		id += (((uint32_t)buffer) << (k << 3));
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
		if( uartRead(serialControl.uart, &buffer, configSERIAL_ID_MAX_DELAY) ) break;
		size += (((uint32_t)buffer) << (k << 3));
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
		if( uartRead(serialControl.uart, buffer++, configSERIAL_START_MAX_DELAY) ) break;
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

	if( uartRead(serialControl.uart, &buffer, configSERIAL_START_MAX_DELAY) ){
		serialSMControl.state = ST_START;
		return;
	}

	if( buffer != configSERIAL_STOP_BYTE ) {
		serialSMControl.state = ST_START;
		return;
	}

	serialCurrent->dataAvailable = 1;

	if( serialCurrent->handler ) serialCurrent->handler();

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
