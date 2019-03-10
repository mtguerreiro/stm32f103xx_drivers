/*
 * uart.h
 *
 *  Created on: Apr 29, 2018
 *      Author: Marco
 *
 *	UART driver for STM32F103RET6 with FreeRTOS.
 *	In the current implementation, @ 72MHz, the driver takes
 *	around 14 us to enqueue a received byte and around 5 us
 *	to execute the routine that sends a byte.
 *
 *	Current version: 0.0
 *
 *	Melhorias
 *		- Baudrate configurável
 *		- Melhorar uso das filas (criar somente a quantidade
 *		necessária)
 *		- "Yield from interrupt" individual
 *
 */

#ifndef UART_H_
#define UART_H_

//=============================
/*--------- Includes --------*/
//=============================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configUART1_ENABLED			1
#define configUART2_ENABLED			0
#define configUART3_ENABLED			0
#define configUART4_ENABLED			0
#define configUART5_ENABLED			0

/* RX and TX queue size */
#define configUART1_RXQ_SIZE			25
#define configUART1_TXQ_SIZE			25

#define configUART2_RXQ_SIZE			25
#define configUART2_TXQ_SIZE			25

#define configUART3_RXQ_SIZE			25
#define configUART3_TXQ_SIZE			25

#define configUART4_RXQ_SIZE			25
#define configUART4_TXQ_SIZE			25

#define configUART5_RXQ_SIZE			25
#define configUART5_TXQ_SIZE			25

#define configUART_INTERRUPT_YIELD	1


//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t uartInitialize(uint32_t uart);
uint8_t uartWrite(uint32_t uart, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles);
uint8_t uartRead(uint32_t uart, uint8_t *buffer, uint32_t waitcycles);
//=============================

//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef enum{
	UART_1 = USART1_BASE,
	UART_2 = USART2_BASE,
	UART_3 = USART3_BASE,
	UART_4 = UART4_BASE,
	UART_5 = UART5_BASE
}uartN_t;
//-----------------------------
//=============================

#endif /* UART_H_ */
