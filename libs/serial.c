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
	uint8_t *bufferSend;
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

	uint32_t ret;
	uint32_t id;
    serialDataExchange_t dataExchange;

	id = serialControl.currentID;

	/*
	 * At this point, we have received a valid package. Now, we call the
	 * handler, if there is one.
	 */
	if( serialControl.handle[id] == 0 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	dataExchange.buffer = serialControl.buffer;
	dataExchange.size = serialControl.dataSize;
	ret = serialControl.handle[id](&dataExchange);
	if( ret == 0 ){
		serialControl.st = SERIAL_ST_START;
		return;
	}

	serialControl.dataSize = dataExchange.size;
	serialControl.bufferSend = dataExchange.buffer;
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
	p = serialControl.bufferSend;
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

	while( k < serialControl.n ){
		if( id == serialControl.id[k] ) break;
		k++;
	}

	if( k == serialControl.n ) return serialControl.n;

	return k;
}
//---------------------------------------------------------------------------
//===========================================================================
