/*
 * display.h
 *
 *  Created on: May 5, 2018
 *      Author: Marco
 *
 *	Basic functionalities to write do a 20x4 display, in parallel mode, using 4 bits
 *	of data.
 *	All timings between commands have been tuned to work with STM32F103RET6 @ 72 MHz
 *	main clock.
 *
 *	Current version: v0.1.6
 *
 *	Possible issues:
 *		- #1 (v0.1.3 - resolving in v0.1.4)
 *		The display seems to be slow to update. We have measured and verified that the
 *		enable pulse is of about 1 ms, when the minimum value required is around 250 ns
 *		(seen in the datasheet).
 *
 *	v0.1.0:
 *		- Initial version
 *
 *	v0.1.1:
 *		- Version descriptions
 *		- Removed unused variable in displayClearLine
 *
 *	v0.1.2:
 *		- Adapted to board version
 *
 *	v0.1.3:
 *		- Additional casting to get rid of warnings
 *
 *	v0.1.4:
 *		- Created macros to define the enable pulse width.
 *		- Created macros to define the delay for data and
 *		instruction writes.
 *
 *	v0.1.5:
 *		- Updated lib for the new gpio driver.
 *		- Updated code formatting.
 *
 *	v0.1.6:
 *		- Updated lib to use a better delay.
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

/*
 * TODO: estudar sobre os modos do display (1 ou 2 linhas, 11x7 ou 7x5)
 * TODO: verificar os delays que est�o sendo gerados e otimizar de acordo com o datasheet
 * TODO: Mudar os pinos de dados requer mudar a fun��o writeData/Instruction, pois �
 * necess�rio alinhar os dados com os pinos do microcontrolador. Deixar isso automatico
 * TODO: inicializar display por macro ao inves de manualmente
 *
 */


//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
/* Standard */
#include <stdint.h>

/* Device */
#include "stm32f10x.h"

/* Drivers */
#include "gpio.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/* For now, not a good idea to change these */
#define	configDISPLAY_RS		GPIO_P3
#define configDISPLAY_EN		GPIO_P4
#define	configDISPLAY_D4		GPIO_P5
#define configDISPLAY_D5		GPIO_P6
#define configDISPLAY_D6		GPIO_P7
#define configDISPLAY_D7		GPIO_P8

/* Display control options */
#define DISPLAY_CONTROL_D_ON	(1U << 2)
#define DISPLAY_CONTROL_D_OFF	(0)
#define DISPLAY_CONTROL_C_ON	(1U << 1)
#define DISPLAY_CONTROL_C_OFF	(0)
#define DISPLAY_CONTROL_B_ON	(1U)
#define DISPLAY_CONTROL_B_OFF	(0)
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void displayInitialize(void);
void displayWrite(uint8_t *buffer, uint8_t nbytes);
void displayWriteString(uint8_t *s);
void displaySetCursor(uint8_t line, uint8_t column);
void displayClear(void);
void displayClearLine(uint8_t line);
void displaySetCursorHome(void);
void displayEntryModeSet(uint8_t mode);
void displayControl(uint8_t mode);
//===========================================================================

#endif /* DISPLAY_H_ */
