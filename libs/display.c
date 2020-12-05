/*
 * display.c
 *
 *  Created on: May 5, 2018
 *      Author: Marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "display.h"

/* Libs */
#include "delays.h"
//===========================================================================

//===========================================================================
/*------------------------------ Prototypes -------------------------------*/
//===========================================================================
static void displayEnable(void);
static void displayWriteInstruction(uint8_t data);
static void displayWriteData(uint8_t data);
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/* Display pins */
#define RS			configDISPLAY_RS
#define EN			configDISPLAY_EN
#define	D4			configDISPLAY_D4
#define D5			configDISPLAY_D5
#define D6			configDISPLAY_D6
#define D7			configDISPLAY_D7

#define MASK		(RS | EN | D4 | D5 | D6 | D7)
#define DATA_MASK	(D4 | D5 | D6 | D7)

/* Entry mode set options */
#define	DISPLAY_MODESET_ID_INC	(1U << 1)
#define DISPLAY_MODESET_ID_DEC	(0)
#define DISPLAY_MODESET_SHIFT	(1U << 0)

/* Enable pulse width (~1us) */
#define configDISPLAY_EN_DELAY					0x0A

/* Data and instruction write delays (~50 us) */
#define configDISPLAY_DATA_WRITE_DELAY			0x202
#define configDISPLAY_INSTRUCTION_WRITE_DELAY	0x202

/* Set cursor delay (~50us for set, ~2ms for set home)*/
#define configDISPLAY_SET_CURSOR_DELAY			0x202
#define configDISPLAY_SET_CURSOR_HOME_DELAY		0x505B

/* Display clear delay (~2ms) */
#define configDISPLAY_CLEAR_DELAY				0x505B

/* Display control delay (~50 us) */
#define configDISPLAY_CONTROL_DELAY				0x202

/* Entry mode set delay (~50 us) */
#define configDISPLAY_ENTRY_MODE_SET_DELAY		0x202
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void displayWrite(uint8_t *buffer, uint8_t nbytes){

	while(nbytes--){
		displayWriteData(*buffer++);
	}
}
//---------------------------------------------------------------------------
void displayWriteString(uint8_t *string){

	while(*string){
		displayWriteData(*string++);
	}
}
//---------------------------------------------------------------------------
void displaySetCursor(uint8_t line, uint8_t column){

	uint8_t address;

	switch(line){

	case 0x00:
		address = column;
		break;

	case 0x01:
		address = (uint8_t)(0x40 | column);
		break;

	case 0x02:
		address = (uint8_t)(0x14 + column);
		break;

	case 0x03:
		address = (uint8_t)(0x40 | (0x14 + column));
		break;

	default:
		address = column;
		break;

	}

	displayWriteInstruction(address | 0x80);
	delaysSub(configDISPLAY_SET_CURSOR_DELAY);
}
//---------------------------------------------------------------------------
void displayClear(void){

	displayWriteInstruction(0x01);
	delaysSub(configDISPLAY_CLEAR_DELAY);
}
//---------------------------------------------------------------------------
void displayClearLine(uint8_t line){

	uint8_t blank[] = "                    ";

	displaySetCursor(line, 0);
	displayWrite(blank, 20);
	displaySetCursor(line, 0);
}
//---------------------------------------------------------------------------
void displaySetCursorHome(void){

	displayWriteInstruction(0x02);
	delaysSub(configDISPLAY_SET_CURSOR_HOME_DELAY);
}
//---------------------------------------------------------------------------
void displayEntryModeSet(uint8_t mode){

	displayWriteInstruction(mode | 0x04);
	delaysSub(configDISPLAY_ENTRY_MODE_SET_DELAY);
}
//---------------------------------------------------------------------------
void displayControl(uint8_t mode){

	displayWriteInstruction(mode | 0x08);
	delaysSub(configDISPLAY_CONTROL_DELAY);
}
//---------------------------------------------------------------------------
void displayInitialize(void){

	gpioPortEnable(GPIOB);
	gpioConfig(GPIOB, MASK, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);

	/* ~50ms delay and command (first 0x03) */
	delaysSub(0x7D8ED);
	gpioOutputWrite(GPIOB, MASK, D5 | D4);
	displayEnable();
	delaysSub(0x7D8ED);

	/* Command and ~5ms delay (second 0x03) */
	gpioOutputWrite(GPIOB, MASK, D5 | D4);
	displayEnable();
	delaysSub(0xC8E4);

	/*Command and ~100us delay (third 0x03) */
	gpioOutputWrite(GPIOB, MASK, D5 | D4);
	displayEnable();
	delaysSub(0x1416);

	/* Command and ~100us delay */
	gpioOutputWrite(GPIOB, MASK, D5);
	displayEnable();
	delaysSub(0x1416);

	/* Function set */
	gpioOutputWrite(GPIOB, MASK, D5);
	displayEnable();
	gpioOutputWrite(GPIOB, MASK, D7);
	displayEnable();
	delaysSub(0x1416 << 2);

	/* Display on/off */
	gpioOutputWrite(GPIOB, MASK, 0);
	displayEnable();
	gpioOutputWrite(GPIOB, MASK, D7);
	displayEnable();
	delaysSub(0x1416);

	/* Entry mode set */
	gpioOutputWrite(GPIOB, MASK, 0);
	displayEnable();
	gpioOutputWrite(GPIOB, MASK, D5 | D6);
	displayEnable();
	delaysSub(0x1416);

	/* Display clear */
	gpioOutputWrite(GPIOB, MASK, 0);
	displayEnable();
	gpioOutputWrite(GPIOB, MASK, D4);
	displayEnable();
	delaysSub(0x7D8ED);

	/* Display on */
	//gpioOutputWrite(GPIOB, MASK, D6 | D7);
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static void displayEnable(void){

	gpioOutputSet(GPIOB, EN);
	delaysSub(configDISPLAY_EN_DELAY);

	gpioOutputReset(GPIOB, EN);
	delaysSub(configDISPLAY_EN_DELAY);
}
//---------------------------------------------------------------------------
static void displayWriteInstruction(uint8_t data){

	gpioOutputWrite(GPIOB, MASK, (uint16_t)((data << 1) & DATA_MASK));
	displayEnable();

	gpioOutputWrite(GPIOB, MASK, (uint16_t)((data << 5) & DATA_MASK));
	displayEnable();

	delaysSub(configDISPLAY_INSTRUCTION_WRITE_DELAY);
}
//---------------------------------------------------------------------------
static void displayWriteData(uint8_t data){

	gpioOutputWrite(GPIOB, MASK, (uint16_t)(RS | ((data << 1) & DATA_MASK)));
	displayEnable();

	gpioOutputWrite(GPIOB, MASK, (uint16_t)(RS | ((data << 5) & DATA_MASK)));
	displayEnable();

	delaysSub(configDISPLAY_DATA_WRITE_DELAY);
}
//---------------------------------------------------------------------------
//===========================================================================
