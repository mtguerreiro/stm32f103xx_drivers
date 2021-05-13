/*
 * @file serial.h
 * @brief Implements a simple serial protocol.
 *
 * A serial protocol is implemented here through a state machine. The
 * protocol is as follows:
 * 	- Start byte (1 byte - 0x55)
 * 	- ID - or cmd (4 bytes)
 * 	- Data size (4 bytes)
 * 	- Data (N bytes)
 * 	- Stop byte (1 byte - 0x77)
 *
 * Whenever data is successfully received, the state machine will call the
 * function corresponding to the ID received.
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 *
 */

#ifndef SERIAL_H_
#define SERIAL_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

//===========================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
/* Function executed when data is received */
typedef void(*serialHandler_t)(void);

/*
 * Function to read a byte from the hardware peripheral. We expect the
 * function to take as argument a pointer and the TO. If the function was
 * able to read one byte from the hardware peripheral, then we expect this
 * function to return 0, and any other value if the no data was read from
 * the peripheral.
 */
typedef int32_t(*serialHWRead_t)(uint8_t *buffer, uint32_t to);

/*
 * Function to write a byte to the hardware peripheral. We expect the
 * function to take as argument a pointer and the TO. If the function was
 * able to write one byte to the hardware peripheral, then we expect this
 * function to return 0, and any other value if the no data was written to
 * the peripheral.
 */
typedef int32_t(*serialHWWrite_t)(uint8_t *buffer, uint32_t to);
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/* Error codes */
#define SERIAL_ERR_INVALID_ID			-0x01
#define SERIAL_ERR_EXCEEDED_MAX_ID		-0x02
#define SERIAL_ERR_WRITE				-0x03


#define SERIAL_CONFIG_IDS				10
#define SERIAL_CONFIG_RX_TO				100
#define SERIAL_CONFIG_TX_TO				100

#define SERIAL_CONFIG_START_BYTE		0x55
#define SERIAL_CONFIG_STOP_BYTE			0x77
//===========================================================================


//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void serialInitialize(uint8_t *buffer, uint32_t size, serialHWRead_t hwRead, serialHWWrite_t hwWrite);
//void serialInitialize(USART_TypeDef *uart, uint32_t baud, uint8_t *buffer);
uint8_t serialRun(void);
int32_t serialInstallID(uint32_t id, serialHandler_t handler);
int32_t serialSend(uint32_t id, uint8_t *buffer, uint32_t nbytes);
//uint8_t serialSendString(uint32_t id, void *string);
//uint8_t serialSendStringRaw(void *string);
//uint8_t serialReceive(uint32_t id);
//===========================================================================

#endif /* SERIAL_H_ */
