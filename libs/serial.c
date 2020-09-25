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
	ST_SUCCESSFUL,
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

	uint32_t currentId;

	USART_TypeDef *uart;

	//uint32_t txBusy;

	uint32_t id[configSERIAL_MAX_IDS];

	serialHandler_t handler[configSERIAL_MAX_IDS];

	uint8_t *buffer;

	uint32_t dataSize;

	//uint8_t dataAvailable;
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
static void serialStateDataSize(void);
static void serialStateData(void);
static void serialStateStop(void);
static void serialStateSuccessful(void);
static uint32_t serialFindID(uint32_t id);
static uint8_t serialSendFrame(uint32_t id, uint8_t *buffer, uint32_t nbytes);
//=============================

//=============================
/*--------- Globals ---------*/
//=============================
serialSM_t serialSMControl;
serialControl_t serialControl = {.n = 0};
//serialControl_t serialControl = {.n = 0, .txBusy = 0};
//serial_t *serialCurrent;
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void serialInitialize(USART_TypeDef *uart, uint32_t baud, uint8_t *buffer){

	/* Initializes uart driver */
	uartInitialize(uart, baud);

	/* Initializes control structure */
	serialControl.buffer = buffer;
	serialControl.uart = uart;
	serialControl.dataSize = 0;
	//serialControl.dataAvailable = 0;

	/* Initial state */
	serialSMControl.state = ST_START;

	/* State functions */
	serialSMControl.functions[ST_START] = serialStateStart;
	serialSMControl.functions[ST_ID] = serialStateID;
	serialSMControl.functions[ST_DATA_SIZE] = serialStateDataSize;
	serialSMControl.functions[ST_DATA] = serialStateData;
	serialSMControl.functions[ST_STOP] = serialStateStop;
	serialSMControl.functions[ST_SUCCESSFUL] = serialStateSuccessful;
}
//-----------------------------
uint8_t serialRun(void){

	while(1){
		serialSMControl.functions[serialSMControl.state]();
	}
}
//-----------------------------
uint8_t serialInstallID(uint32_t id, serialHandler_t handler){

	/* Checks if ID is available */
	if( serialFindID(id) != serialControl.n ){
		return 1;
	}

	if(serialControl.n >= configSERIAL_MAX_IDS) return 2;

	serialControl.id[serialControl.n] = id;
	serialControl.handler[serialControl.n] = handler;

	serialControl.n++;

	return 0;
}
//-----------------------------
uint8_t serialSend(uint32_t id, uint8_t *buffer, uint32_t nbytes){

	uint32_t status;

	/* Checks if ID is valid */
	if( serialFindID(id) == serialControl.n ){
		return 1;
	}

//	if(serialControl.txBusy){
//		return 2;
//	}
//	serialControl.txBusy = 1;

	status = serialSendFrame(id, buffer, nbytes);
	if(status){
		//serialControl.txBusy = 0;
		return (uint8_t)(status + 2U);
	}

//	serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
uint8_t serialSendString(uint32_t id, void *string){

	uint32_t status;
	uint32_t nbytes;
	uint8_t *buffer;

	buffer = (uint8_t *)string;

	/* Checks if ID is valid */
	if( serialFindID(id) == serialControl.n ){
		return 1;
	}

//	if(serialControl.txBusy){
//		return 2;
//	}
//	serialControl.txBusy = 1;

	nbytes = 0;
	/* Size of string */
	while(*buffer++){
		nbytes++;
	}
	buffer--;
	buffer = buffer - nbytes;

	status = serialSendFrame(id, buffer, nbytes);
	if(status){
		//serialControl.txBusy = 0;
		return (uint8_t)(status + 2U);
	}

	//serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
uint8_t serialSendStringRaw(void *string){

	uint32_t nbytes;
	uint8_t *buffer;

	buffer = (uint8_t *)string;

//	if(serialControl.txBusy){
//		return 1;
//	}
//	serialControl.txBusy = 1;

	nbytes = 0;
	/* Size of string */
	while(*buffer++){
		nbytes++;
	}
	buffer--;
	buffer = buffer - nbytes;

	/* Sends data from buffer */
	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ){
		//serialControl.txBusy = 0;
		return 4;
	}

//	serialControl.txBusy = 0;
	return 0;
}
//-----------------------------
uint8_t serialReceive(uint32_t id){

	/*
	 * Here, we run state by state.
	 */

	/*
	 * Clears all bytes on the RX queue of UART. Since this is a receive for
	 * a master-like communication, there should be no unprocessed data in
	 * the buffer. However, there may be unprocessed data for the case where
	 * there as an frame error and all the data ended up in the queue.
	 * Anyway, any data in the buffer at this stage is not important as we
	 * are more concerned about the new incoming data.
	 */
	uartRXFlush(serialControl.uart);

	/* Initial state */
	serialSMControl.state = ST_START;

	/*
	 * Runs the start state. If this functions returns and the state still is
	 * ST_START, then we have not received anything from the UART, and we
	 * return an error.
	 */
	serialStateStart();
	if(serialSMControl.state == ST_START) return 1;

	/*
	 * The next state is the ID state, where we expect the ID, or command.
	 * If the function returns the ST_START state, then something went wrong
	 * and we return an error.
	 * In addition, we check the ID. If the ID matches the expected, we
	 * continue with the state machine. However, if something different was
	 * received, we set the state to ST_START and return an error.
	 */
	serialStateID();
	if(serialSMControl.state == ST_START) return 2;
	if(serialControl.id[serialControl.currentId] != id){
		serialSMControl.state = ST_START;
		return 3;
	}

	/*
	 * Execute the next states (data size, data and stop ). After executing
	 * the stop state, we will verify if the entire frame, including the stop
	 * bytes, was received successfully, and return accordingly.
	 */
	serialStateDataSize();
	if(serialSMControl.state == ST_START) return 4;

	serialStateData();
	if(serialSMControl.state == ST_START) return 5;

	serialStateStop();
	if(serialSMControl.state == ST_START) return 6;

	serialSMControl.state = ST_START;

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

	serialControl.currentId = ididx;
	serialSMControl.state = ST_DATA_SIZE;
}
//-----------------------------
static void serialStateDataSize(void){

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
//	if(serialControl.dataAvailable){
//		serialSMControl.state = ST_START;
//		return;
//	}

	serialControl.dataSize = size;
	serialSMControl.state = ST_DATA;
}
//-----------------------------
static void serialStateData(void){

	uint32_t k;
	uint8_t *buffer;

	buffer = serialControl.buffer;
	k = 0;

	while(k < serialControl.dataSize){
		if( uartRead(serialControl.uart, buffer++, configSERIAL_START_MAX_DELAY) ) break;
		k++;
	}

	/* Checks if we got all the data that we were expecting */
	if(k != serialControl.dataSize){
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

//	serialControl.dataAvailable = 1;

	if( serialControl.handler[serialControl.currentId] ) serialControl.handler[serialControl.currentId]();

	serialSMControl.state = ST_SUCCESSFUL;
}
//-----------------------------
static void serialStateSuccessful(void){

	serialSMControl.state = ST_START;
}
//-----------------------------
static uint32_t serialFindID(uint32_t id){

	uint32_t k;

	k = 0;

	while(k < serialControl.n){
		if(id == serialControl.id[k]) break;
		k++;
	}

	if(k == serialControl.n) return serialControl.n;

	return k;
}
//-----------------------------
static uint8_t serialSendFrame(uint32_t id, uint8_t *buffer, uint32_t nbytes){

	uint32_t size;
	uint32_t k;
	uint8_t start[9];

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
	if( uartWrite(serialControl.uart, start, 9) ) return 1;

	/* Sends data from buffer */
	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ) return 2;

	/* Sends end of frame */
	start[0] = configSERIAL_STOP_BYTE;
	if( uartWrite(serialControl.uart, start, 1) ) return 3;

	return 0;
}
//-----------------------------
//=============================
