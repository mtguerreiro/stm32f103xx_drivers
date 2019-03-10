/*
 * uart.h
 *
 *  Created on: March 10, 2019
 *      Author: Marco
 *
 *	UART driver for STM32F103xx devices.
 *
 *	Current version: 0.1
 *
 *	- v0.1:
 *		- Overall improvements in code
 *
 *	Melhorias
 *		- Baudrate configurável
 *		- Melhorar uso das filas (criar somente a quantidade
 *		necessária)
 *		- "Yield from interrupt" individual
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
#define configUART1_ENABLED			0
#define configUART2_ENABLED			0
#define configUART3_ENABLED			1
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
uint8_t uartInitialize(USART_TypeDef *uart, uint32_t baud);
uint8_t uartWrite(USART_TypeDef *uart, uint8_t *buffer, uint16_t nbytes, uint32_t waitcycles);
uint8_t uartRead(USART_TypeDef *uart, uint8_t *buffer, uint32_t waitcycles);
//=============================


#endif /* UART_H_ */
