/*
 * gpio.c
 *
 *  Created on: Feb 24, 2019
 *      Author: marco
 */

//=============================
/*-------- Includes ---------*/
//=============================
#include "gpio.h"
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
static void gpioSetCR(GPIO_TypeDef *port, uint16_t pins, uint8_t conf, uint8_t val);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void gpioPortEnable(GPIO_TypeDef *port){

	uint32_t _port;
	uint32_t bit;

	/* Get port's address as an hex number */
	_port = (uint32_t)port;

	/* Calculates which RCC bit corresponds to the selected port */
	bit = 1U << (((_port - GPIOA_BASE) >> 10) + 2);

	RCC->APB2ENR |= bit;
}
//-----------------------------
void gpioConfig(GPIO_TypeDef *port, uint16_t pins, uint8_t mode, uint8_t conf){

	gpioSetCR(port, pins, 0, mode);
	gpioSetCR(port, pins, 1, conf);
}
//-----------------------------
void gpioOutputToggle(GPIO_TypeDef *port, uint16_t pins){

	port->ODR ^= (uint32_t)pins;
}
//-----------------------------
void gpioOutputSet(GPIO_TypeDef *port, uint16_t pins){

	port->BSRR = (uint32_t)pins;
}
//-----------------------------
void gpioOutputReset(GPIO_TypeDef *port, uint16_t pins){

	port->BRR = (uint32_t)pins;
}
//-----------------------------
void gpioOutputWrite(GPIO_TypeDef *port, uint16_t pins){

	port->ODR = (uint32_t)pins;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void gpioSetCR(GPIO_TypeDef *port, uint16_t pins, uint8_t conf, uint8_t val){

	uint32_t *_port;
	uint16_t _pins;
	uint16_t k;
	uint16_t crOffs, offs;

	/* Points to CRL register (which is the base address) */
	_port = (uint32_t *)((uint32_t)port);

	/* Compensate if we're setting the CONF bits */
	offs = 0;
	if (conf) offs = 2;

	/* Sets the CRL register */
	_pins = pins & 0xFF;
	k = 8;
	while(k--){
		if(_pins & (1U << k)){
			crOffs = (uint16_t)((k << 2) + offs);
			/* Clears bits before setting */
			*_port &= (uint32_t)(~(0x03U << crOffs));
			*_port |= (uint32_t)(val << crOffs);
		}
	}

	/* Points to the CRH register (base address + 4) */
	_port++;

	/* Sets the CRH register */
	_pins = (pins >> 8) & 0xFF;
	k = 8;
	while(k--){
		if(_pins & (1U << k)){
			crOffs = (uint16_t)((k << 2) + offs);
			/* Clears bits before setting */
			*_port &= (uint32_t)(~(0x03U << crOffs));
			*_port |= (uint32_t)(val << crOffs);
		}
	}
}
//-----------------------------
//=============================
