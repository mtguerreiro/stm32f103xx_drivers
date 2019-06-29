/*
 * gpio.h
 *
 *	Current version: v0.1.2.
 *
 *	v0.1.0:
 *		- Initial version
 *
 *	-v0.1.1:
 *		- Input can be configured as pull-up or pull-down.
 *
 *	-v0.1.2:
 *		- Changed GPIO_CONFIG_INPUT_FLOAT_INPUT to GPIO_CONFIG_INPUT_FLOAT.
 *
 *  Created on: March 10, 2019
 *      Author: Marco
 */

#ifndef GPIO_H_
#define GPIO_H_

//=============================
/*-------- Includes ---------*/
//=============================
/* Standard libs */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void gpioPortEnable(GPIO_TypeDef *port);
void gpioConfig(GPIO_TypeDef *port, uint16_t pins, uint8_t mode, uint8_t conf);
void gpioOutputToggle(GPIO_TypeDef *port, uint16_t pins);
void gpioOutputSet(GPIO_TypeDef *port, uint16_t pins);
void gpioOutputReset(GPIO_TypeDef *port, uint16_t pins);
void gpioOutputWrite(GPIO_TypeDef *port, uint16_t pins);
//=============================

//=============================
/*--------- Enums -----------*/
//=============================
//-----------------------------
typedef enum{
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT_10MHZ,
	GPIO_MODE_OUTPUT_2MHZ,
	GPIO_MODE_OUTPUT_50MHZ
}gpioMode_et;
//-----------------------------
typedef enum{
	GPIO_CONFIG_INPUT_ANALOG,
	GPIO_CONFIG_INPUT_FLOAT,
	GPIO_CONFIG_INPUT_PULL_UP,
	GPIO_CONFIG_INPUT_PULL_DOWN
}gpioConfigInput_et;
//-----------------------------
typedef enum{
	GPIO_CONFIG_OUTPUT_GP_PUSH_PULL,
	GPIO_CONFIG_OUTPUT_GP_OPEN_DRAIN,
	GPIO_CONFIG_OUTPUT_AF_PUSH_PULL,
	GPIO_CONFIG_OUTPUT_AF_OPEN_DRAIN
}gpioConfigOutput_et;
//-----------------------------
typedef enum{
	GPIO_P0 = 1U << 0,
	GPIO_P1 = 1U << 1,
	GPIO_P2 = 1U << 2,
	GPIO_P3 = 1U << 3,
	GPIO_P4 = 1U << 4,
	GPIO_P5 = 1U << 5,
	GPIO_P6 = 1U << 6,
	GPIO_P7 = 1U << 7,
	GPIO_P8 = 1U << 8,
	GPIO_P9 = 1U << 9,
	GPIO_P10 = 1U << 10,
	GPIO_P11 = 1U << 11,
	GPIO_P12 = 1U << 12,
	GPIO_P13 = 1U << 13,
	GPIO_P14 = 1U << 14,
	GPIO_P15 = 1U << 15
}gpioPin_et;
//-----------------------------
//=============================

#endif /* GPIO_H_ */
