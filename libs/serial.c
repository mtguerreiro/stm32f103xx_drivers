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
	SERIAL_ST_DATA_RCVD,
	SERIAL_ST_SEND_START,
	SERIAL_ST_SEND_ID,
	SERIAL_ST_SEND_DATA_SIZE,
	SERIAL_ST_SEND_DATA,
	SERIAL_ST_SEND_STOP,
	SERIAL_ST_END
}serialSMStates_t;
//---------------------------------------------------------------------------
typedef struct{

	serialHWRead_t hwRead;
	serialHWWrite_t hwWrite;

	uint8_t *buffer;
	uint32_t bufferSize;

	uint32_t id[SERIAL_CONFIG_IDS];
	serialHandle_t handle[SERIAL_CONFIG_IDS];

	serialSMStates_t st;
	serialVoid_t state[SERIAL_ST_END];

	uint32_t n;
	uint32_t currentID;
	uint32_t dataSize;
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
static void serialStateDataRcvd(void);
static void serialStateSendStart(void);
static void serialStateSendID(void);
static void serialStateSendDataSize(void);
static void serialStateSendData(void);
static void serialStateSendStop(void);
static uint32_t serialFindID(uint32_t id);
static int32_t serialSendFrame(uint32_t id, uint8_t *buffer, uint32_t nbytes);
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
static serialControl_t serialControl = {.n = 0};
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
	serialControl.st = SERIAL_ST_START;

	/* State functions */
	serialControl.state[SERIAL_ST_START] = serialStateStart;
	serialControl.state[SERIAL_ST_ID] = serialStateID;
	serialControl.state[SERIAL_ST_DATA_SIZE] = serialStateDataSize;
	serialControl.state[SERIAL_ST_DATA] = serialStateData;
	serialControl.state[SERIAL_ST_STOP] = serialStateStop;
	serialControl.state[SERIAL_ST_DATA_RCVD] = serialStateDataRcvd;
	serialControl.state[SERIAL_ST_SEND_START] = serialStateSendStart;
	serialControl.state[SERIAL_ST_SEND_ID] = serialStateSendID;
	serialControl.state[SERIAL_ST_SEND_DATA_SIZE] = serialStateSendDataSize;
	serialControl.state[SERIAL_ST_SEND_DATA] = serialStateSendData;
	serialControl.state[SERIAL_ST_SEND_STOP] = serialStateSendStop;

	/*
	 * Initializes handlers. When we declared the serialControl structure,
	 * we set n to be zero (that's the number of IDs registered). However,
	 * it may be that due to priority issues, the register ID function was
	 * called even before the initialization of the serial structure. This
	 * should be no issue, because the register function will just save the
	 * handle and increment n. So, we can start from n and go up to the
	 * max. number of IDs.
	 */
	k = serialControl.n;
	while( k < SERIAL_CONFIG_IDS ){
		serialControl.handle[k] = 0;
		serialControl.id[k] = 0;
		k++;
	}
}
//---------------------------------------------------------------------------
uint8_t serialRun(void){

	while(1){
		serialControl.state[serialControl.st]();
	}
}
//---------------------------------------------------------------------------
int32_t serialRegisterHandle(uint32_t id, serialHandle_t handle){

	/* Cannot register ID as 0 */
	if( id == 0 ){
		return SERIAL_ERR_INVALID_ID;
	}
	/* Checks if ID is available */
	if( serialFindID(id) != serialControl.n ){
		return SERIAL_ERR_INVALID_ID;
	}

	if( serialControl.n >= SERIAL_CONFIG_IDS ) return SERIAL_ERR_EXCEEDED_MAX_ID;

	SERIAL_CRITICAL_ENTER;

	serialControl.id[serialControl.n] = id;
	serialControl.handle[serialControl.n] = handle;

	serialControl.n++;

	SERIAL_CRITICAL_EXIT;

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

	serialControl.st = SERIAL_ST_ID;
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
		serialControl.st = SERIAL_ST_START;
		return;
	}

	/*
	 * Gets the index of the received ID. If the received ID is not found,
	 * the function returns n. In this case, we just go back to the first
	 * state.
	 */
	ididx = serialFindID(id);
	if( ididx == serialControl.n ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.currentID = ididx;
	serialControl.st = SERIAL_ST_DATA_SIZE;
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
	if( k != 4 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	/* Ensures we don't have buffer overflow */
	if( size > serialControl.bufferSize ){
        serialControl.st = SERIAL_ST_START;
        return;
	}

	serialControl.dataSize = size;
	serialControl.st = SERIAL_ST_DATA;
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
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.st = SERIAL_ST_STOP;
}
//---------------------------------------------------------------------------
static void serialStateStop(void){

	int32_t ret;
	uint8_t buffer;

	ret = serialControl.hwRead(&buffer, SERIAL_CONFIG_RX_TO);
	if( ret != 0 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	if( buffer != SERIAL_CONFIG_STOP_BYTE ) {
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.st = SERIAL_ST_DATA_RCVD;
}
//---------------------------------------------------------------------------
static void serialStateDataRcvd(void){

	uint32_t nbytes;
	uint32_t id;

	id = serialControl.currentID;

	/*
	 * At this point, we have received a valid package. Now, we call the
	 * handler, if there is one.
	 */
	if( serialControl.handle[id] == 0 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	nbytes = serialControl.handle[id](serialControl.buffer, serialControl.dataSize);
	if( nbytes == 0 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.dataSize = nbytes;
	serialControl.st = SERIAL_ST_SEND_START;
}
//---------------------------------------------------------------------------
static void serialStateSendStart(void){

	int32_t ret;
	uint8_t start;

	start = SERIAL_CONFIG_START_BYTE;

	ret = serialControl.hwWrite(&start, SERIAL_CONFIG_TX_TO);

	if( ret != 0 ){
		serialControl.st = SERIAL_ST_START;
	}

	serialControl.st = SERIAL_ST_SEND_ID;
}
//---------------------------------------------------------------------------
static void serialStateSendID(void){

	int32_t ret;
	uint8_t id;
	uint32_t id32;
	uint8_t k;

	id32 = serialControl.id[serialControl.currentID];
	k = 0;
	while( k < 4 ){
		id = (uint8_t)id32;
		ret = serialControl.hwWrite(&id, SERIAL_CONFIG_TX_TO);
		if( ret != 0 ) break;
		id32 = id32 >> 8;
		k++;
	}

	if( k != 4 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.st = SERIAL_ST_SEND_DATA_SIZE;
}
//---------------------------------------------------------------------------
static void serialStateSendDataSize(void){

	int32_t ret;
	uint8_t data;
	uint32_t dataSize;
	uint8_t k;

	dataSize = serialControl.dataSize;
	k = 0;
	while( k < 4 ){
		data = (uint8_t)dataSize;
		ret = serialControl.hwWrite(&data, SERIAL_CONFIG_TX_TO);
		if( ret != 0 ) break;
		dataSize = dataSize >> 8;
		k++;
	}

	if( k != 4 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.st = SERIAL_ST_SEND_DATA;
}
//---------------------------------------------------------------------------
static void serialStateSendData(void){

	int32_t ret;
	uint8_t *p;
	uint32_t k;

	/* Sends data from buffer */
	k = 0;
	p = serialControl.buffer;
	while( k < serialControl.dataSize ){
		ret = serialControl.hwWrite(p, SERIAL_CONFIG_TX_TO);
		if( ret != 0 ) break;
		p++;
		k++;
	}

	if( k != serialControl.dataSize ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.st = SERIAL_ST_SEND_STOP;
}
//---------------------------------------------------------------------------
static void serialStateSendStop(void){

	uint8_t stop;

	stop = SERIAL_CONFIG_STOP_BYTE;

	serialControl.hwWrite(&stop, SERIAL_CONFIG_TX_TO);

	serialControl.st = SERIAL_ST_START;
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
