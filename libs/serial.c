/*
 * serial.c
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "serial.h"

//===========================================================================


//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
typedef void(*serialVoid_t)(void);
//---------------------------------------------------------------------------
typedef enum{
	SERIAL_ST_START,
	SERIAL_ST_ID,
	SERIAL_ST_DATA_SIZE,
	SERIAL_ST_DATA,
	SERIAL_ST_STOP,
	SERIAL_ST_SUCCESSFUL,
	SERIAL_ST_END
}serialSMStates_t;
//---------------------------------------------------------------------------
typedef struct{
	uint32_t n;

	uint32_t currentID;

	serialHWRead_t hwRead;

	serialHWWrite_t hwWrite;

	//uint32_t txBusy;

	uint32_t id[SERIAL_CONFIG_IDS];

	serialHandler_t handler[SERIAL_CONFIG_IDS];

	uint8_t *buffer;
	uint32_t bufferSize;

	uint32_t dataSize;

	serialSMStates_t state;
	serialVoid_t functions[SERIAL_ST_END];

	//uint8_t dataAvailable;
}serialControl_t;
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static void serialStateStart(void);
static void serialStateID(void);
static void serialStateDataSize(void);
static void serialStateData(void);
static void serialStateStop(void);
static uint32_t serialFindID(uint32_t id);
static int32_t serialSendFrame(uint32_t id, uint8_t *buffer, uint32_t nbytes);
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
//serialSM_t serialSMControl;
serialControl_t serialControl = {.n = 0};
//serialControl_t serialControl = {.n = 0, .txBusy = 0};
//serial_t *serialCurrent;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void serialInitialize(uint8_t *buffer, uint32_t size, serialHWRead_t hwRead, serialHWWrite_t hwWrite){

	uint32_t k;

	/* Initializes control structure */
	serialControl.buffer = buffer;
	serialControl.bufferSize = size;

	serialControl.hwRead = hwRead;
	serialControl.hwWrite = hwWrite;

	serialControl.dataSize = 0;

	/* Initial state */
	serialControl.state = SERIAL_ST_START;

	/* State functions */
	serialControl.functions[SERIAL_ST_START] = serialStateStart;
	serialControl.functions[SERIAL_ST_ID] = serialStateID;
	serialControl.functions[SERIAL_ST_DATA_SIZE] = serialStateDataSize;
	serialControl.functions[SERIAL_ST_DATA] = serialStateData;
	serialControl.functions[SERIAL_ST_STOP] = serialStateStop;

	/* Initializes handlers */
	for(k = 0; k < SERIAL_CONFIG_IDS; k++){
		serialControl.handler[k] = 0;
	}
}
//---------------------------------------------------------------------------
uint8_t serialRun(void){

	while(1){
		serialControl.functions[serialControl.state]();
	}
}
//---------------------------------------------------------------------------
int32_t serialInstallID(uint32_t id, serialHandler_t handler){

	/* Checks if ID is available */
	if( serialFindID(id) != serialControl.n ){
		return SERIAL_ERR_INVALID_ID;
	}

	if( serialControl.n >= SERIAL_CONFIG_IDS ) return SERIAL_ERR_EXCEEDED_MAX_ID;

	serialControl.id[serialControl.n] = id;
	serialControl.handler[serialControl.n] = handler;

	serialControl.n++;

	return 0;
}
//---------------------------------------------------------------------------
int32_t serialSend(uint32_t id, uint8_t *buffer, uint32_t nbytes){

	int32_t status;

	/* Checks if ID is valid */
	if( serialFindID(id) == serialControl.n ){
		return SERIAL_ERR_INVALID_ID;
	}

	status = serialSendFrame(id, buffer, nbytes);
	if( status ) return status;

	return 0;
}
//---------------------------------------------------------------------------
//uint8_t serialSendString(uint32_t id, void *string){
//
//	uint32_t status;
//	uint32_t nbytes;
//	uint8_t *buffer;
//
//	buffer = (uint8_t *)string;
//
//	/* Checks if ID is valid */
//	if( serialFindID(id) == serialControl.n ){
//		return 1;
//	}
//
////	if(serialControl.txBusy){
////		return 2;
////	}
////	serialControl.txBusy = 1;
//
//	nbytes = 0;
//	/* Size of string */
//	while(*buffer++){
//		nbytes++;
//	}
//	buffer--;
//	buffer = buffer - nbytes;
//
//	status = serialSendFrame(id, buffer, nbytes);
//	if(status){
//		//serialControl.txBusy = 0;
//		return (uint8_t)(status + 2U);
//	}
//
//	//serialControl.txBusy = 0;
//	return 0;
//}
//---------------------------------------------------------------------------
//uint8_t serialSendStringRaw(void *string){
//
//	uint32_t nbytes;
//	uint8_t *buffer;
//
//	buffer = (uint8_t *)string;
//
////	if(serialControl.txBusy){
////		return 1;
////	}
////	serialControl.txBusy = 1;
//
//	nbytes = 0;
//	/* Size of string */
//	while(*buffer++){
//		nbytes++;
//	}
//	buffer--;
//	buffer = buffer - nbytes;
//
//	/* Sends data from buffer */
//	if( uartWrite(serialControl.uart, buffer, (uint16_t)nbytes) ){
//		//serialControl.txBusy = 0;
//		return 4;
//	}
//
////	serialControl.txBusy = 0;
//	return 0;
//}
//---------------------------------------------------------------------------
//uint8_t serialReceive(uint32_t id){
//
//	/*
//	 * Here, we run state by state.
//	 */
//
//	/*
//	 * Clears all bytes on the RX queue of UART. Since this is a receive for
//	 * a master-like communication, there should be no unprocessed data in
//	 * the buffer. However, there may be unprocessed data for the case where
//	 * there as an frame error and all the data ended up in the queue.
//	 * Anyway, any data in the buffer at this stage is not important as we
//	 * are more concerned about the new incoming data.
//	 */
//	uartRXFlush(serialControl.uart);
//
//	/* Initial state */
//	serialSMControl.state = ST_START;
//
//	/*
//	 * Runs the start state. If this functions returns and the state still is
//	 * ST_START, then we have not received anything from the UART, and we
//	 * return an error.
//	 */
//	serialStateStart();
//	if(serialSMControl.state == ST_START) return 1;
//
//	/*
//	 * The next state is the ID state, where we expect the ID, or command.
//	 * If the function returns the ST_START state, then something went wrong
//	 * and we return an error.
//	 * In addition, we check the ID. If the ID matches the expected, we
//	 * continue with the state machine. However, if something different was
//	 * received, we set the state to ST_START and return an error.
//	 */
//	serialStateID();
//	if(serialSMControl.state == ST_START) return 2;
//	if(serialControl.id[serialControl.currentId] != id){
//		serialSMControl.state = ST_START;
//		return 3;
//	}
//
//	/*
//	 * Execute the next states (data size, data and stop ). After executing
//	 * the stop state, we will verify if the entire frame, including the stop
//	 * bytes, was received successfully, and return accordingly.
//	 */
//	serialStateDataSize();
//	if(serialSMControl.state == ST_START) return 4;
//
//	serialStateData();
//	if(serialSMControl.state == ST_START) return 5;
//
//	serialStateStop();
//	if(serialSMControl.state == ST_START) return 6;
//
//	serialSMControl.state = ST_START;
//
//	return 0;
//}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static void serialStateStart(void){

	int32_t ret;
	uint8_t buffer;

	ret = serialControl.hwRead(&buffer, SERIAL_CONFIG_RX_TO);
	if( ret != 0 ) return;

	if( buffer != SERIAL_CONFIG_START_BYTE ) return;

	serialControl.state = SERIAL_ST_ID;
}
//---------------------------------------------------------------------------
static void serialStateID(void){

	int32_t ret;
	uint8_t k;
	uint8_t buffer;
	uint32_t id;
	uint32_t ididx;

	id = 0;
	k = 0;
	while( k < 4 ){
		ret = serialControl.hwRead(&buffer, SERIAL_CONFIG_RX_TO);
		if( ret != 0 ) break;
		id += (((uint32_t)buffer) << (k << 3));
		k++;
	}

	/*
	 * If k did not reach 4, we broke from the while loop before receiving
	 * all bytes to form the ID. Thus, we discard this frame.
	 */
	if( k != 4 ){
		serialControl.state = SERIAL_ST_START;
		return;
	}

	/*
	 * Gets the index of the received ID. If the received ID is not found,
	 * the function returns n. In this case, we just go back to the first
	 * state.
	 */
	ididx = serialFindID(id);
	if( ididx == serialControl.n ){
		serialControl.state = SERIAL_ST_START;
		return;
	}

	serialControl.currentID = ididx;
	serialControl.state = SERIAL_ST_DATA_SIZE;
}
//---------------------------------------------------------------------------
static void serialStateDataSize(void){

	int32_t ret;
	uint8_t k;
	uint8_t buffer;
	uint32_t size;

	size = 0;
	k = 0;
	while( k < 4 ){
		ret = serialControl.hwRead(&buffer, SERIAL_CONFIG_RX_TO);
		if( ret != 0 ) break;
		size += (((uint32_t)buffer) << (k << 3));
		k++;
	}

	/*
	 * If k did not reach 4, we broke from the while loop before receiving
	 * all bytes to form the data size. Thus, we discard this frame.
	 */
	if(k != 4){
		serialControl.state = SERIAL_ST_START;
		return;
	}

	serialControl.dataSize = size;
	serialControl.state = SERIAL_ST_DATA;
}
//---------------------------------------------------------------------------
static void serialStateData(void){

	int32_t ret;
	uint32_t k;
	uint8_t *buffer;

	buffer = serialControl.buffer;
	k = 0;

	while( k < serialControl.dataSize ){
		ret = serialControl.hwRead(buffer, SERIAL_CONFIG_RX_TO);
		if( ret != 0 ) break;
		k++;
		buffer++;
	}

	/* Checks if we got all the data that we were expecting */
	if( k != serialControl.dataSize ){
		serialControl.state = SERIAL_ST_START;
		return;
	}

	serialControl.state = SERIAL_ST_STOP;
}
//---------------------------------------------------------------------------
static void serialStateStop(void){

	int32_t ret;
	uint8_t buffer;

	ret = serialControl.hwRead(&buffer, SERIAL_CONFIG_RX_TO);
	if( ret != 0 ){
		serialControl.state = SERIAL_ST_START;
		return;
	}

	if( buffer != SERIAL_CONFIG_STOP_BYTE ) {
		serialControl.state = SERIAL_ST_START;
		return;
	}

	/*
	 * At this point, we have received a valid package. Now, we call the
	 * handler, if there is one.
	 */
	if( serialControl.handler[serialControl.currentID] != 0 ) {
		serialControl.handler[serialControl.currentID]();
	}

	serialControl.state = SERIAL_ST_START;
}
//---------------------------------------------------------------------------
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
//---------------------------------------------------------------------------
static int32_t serialSendFrame(uint32_t id, uint8_t *buffer, uint32_t nbytes){

	int32_t ret;
	uint32_t size;
	uint32_t k;
	uint8_t start[9];
	uint8_t *p;

	/* Builds start of frame (start, ID, size) */
	size = nbytes;
	start[0] = SERIAL_CONFIG_START_BYTE;
	for(k = 0; k < 4; k++){
		start[k + 1] = (uint8_t)id;
		id =  id >> 8;
		start[k + 5] = (uint8_t)size;
		size = size >> 8;
	}

	/* Sends start of frame */
	p = start;
	for(k = 0; k < 9; k++){
		ret = serialControl.hwWrite(p, SERIAL_CONFIG_TX_TO);
		if( ret != 0 ) return SERIAL_ERR_WRITE;
		p++;
	}

	/* Sends data from buffer */
	p = buffer;
	for(k = 0; k < nbytes; k++){
		ret = serialControl.hwWrite(p, SERIAL_CONFIG_TX_TO);
		if( ret != 0 ) return SERIAL_ERR_WRITE;
		p++;
	}

	/* Sends end of frame */
	start[0] = SERIAL_CONFIG_STOP_BYTE;
	ret = serialControl.hwWrite(start, SERIAL_CONFIG_TX_TO);
	if( ret != 0 ) return SERIAL_ERR_WRITE;

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================
