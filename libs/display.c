/*
 * display.c
 *
 *  Created on: May 5, 2018
 *      Author: Marco
 */

#include "display.h"

//=============================
/*-------- Prototypes -------*/
//=============================
static void displayInefficientDelay(uint32_t cycles);
static void displayEnable(void);
static void displayWriteInstruction(uint8_t data);
static void displayWriteData(uint8_t data);
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define RS			configDISPLAY_RS
#define EN			configDISPLAY_EN
#define	D4			configDISPLAY_D4
#define D5			configDISPLAY_D5
#define D6			configDISPLAY_D6
#define D7			configDISPLAY_D7
//#define MASK		(RS | RW | EN | D4 | D5 | D6 | D7)
#define MASK		(RS | EN | D4 | D5 | D6 | D7)
#define DATA_MASK	(D4 | D5 | D6 | D7)
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void displayWrite(uint8_t *buffer, uint8_t nbytes){

	while(nbytes--){
		displayWriteData(*buffer++);
	}
}
//-----------------------------
void displayWriteString(uint8_t *string){

	while(*string){
		displayWriteData(*string++);
	}
}
//-----------------------------
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
	displayInefficientDelay(0xE10);
}
//-----------------------------
void displayClear(void){

	displayWriteInstruction(0x01);
	displayInefficientDelay(0xE10 << 3);
}
//-----------------------------
void displayClearLine(uint8_t line){

	//uint8_t k  = 20;
	uint8_t blank[] = "                    ";

	displaySetCursor(line, 0);
	//while(k--) displayWrite(blank, 1);
	displayWrite(blank, 20);
	displaySetCursor(line, 0);
}
//-----------------------------
void displaySetCursorHome(void){

	displayWriteInstruction(0x02);
	displayInefficientDelay(0xE10 << 3);
}
//-----------------------------
void displayEntryModeSet(uint8_t mode){

	displayWriteInstruction(mode | 0x04);
	displayInefficientDelay(0xE10);
}
//-----------------------------
void displayControl(uint8_t mode){

	displayWriteInstruction(mode | 0x08);
	displayInefficientDelay(0xE10);
}
//-----------------------------
void displayInitialize(void){

	gpioPortEnable(GPIO_PB);
	gpioMode(GPIO_PB, GPIO_MODE_OUTPUT_10MHZ, MASK);
	gpioConfig(GPIO_PB, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL, MASK);

	/* ~50ms delay and command (first 0x03) */
	displayInefficientDelay(0x1B7740);
	gpioOutputWrite(GPIO_PB, MASK, D5 | D4);
	displayEnable();
	displayInefficientDelay(0x1B7740);

	/* Command and ~5ms delay (second 0x03) */
	gpioOutputWrite(GPIO_PB, MASK, D5 | D4);
	displayEnable();
	displayInefficientDelay(0x2BF20);

	/*Command and ~100us delay (third 0x03) */
	gpioOutputWrite(GPIO_PB, MASK, D5 | D4);
	displayEnable();
	displayInefficientDelay(0xE10);

	/* Command and ~100us delay */
	gpioOutputWrite(GPIO_PB, MASK, D5);
	displayEnable();
	displayInefficientDelay(0xE10);

	/* Function set */
	gpioOutputWrite(GPIO_PB, MASK, D5);
	displayEnable();
	gpioOutputWrite(GPIO_PB, MASK, D7);
	displayEnable();
	displayInefficientDelay(0xE10);

	/* Display on/off */
	gpioOutputWrite(GPIO_PB, MASK, 0);
	displayEnable();
	//gpioOutputWrite(GPIO_PB, MASK, D6 | D7);
	gpioOutputWrite(GPIO_PB, MASK, D7);
	displayEnable();
	displayInefficientDelay(0xE10);

	/* Entry mode set */
	gpioOutputWrite(GPIO_PB, MASK, 0);
	displayEnable();
	gpioOutputWrite(GPIO_PB, MASK, D5 | D6);
	displayEnable();
	displayInefficientDelay(0xE10);

	/* Display clear */
	gpioOutputWrite(GPIO_PB, MASK, 0);
	displayEnable();
	gpioOutputWrite(GPIO_PB, MASK, D4);
	displayEnable();
	displayInefficientDelay(0x1B7740);

	/* Display on */
	//gpioOutputWrite(GPIO_PB, MASK, D6 | D7);
}
//-----------------------------
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void displayInefficientDelay(uint32_t cycles){

	while(cycles--);
}
//-----------------------------
static void displayEnable(void){

	gpioOutputSet(GPIO_PB, EN);
	displayInefficientDelay(configDISPLAY_EN_DELAY);
	gpioOutputReset(GPIO_PB, EN);
	displayInefficientDelay(configDISPLAY_EN_DELAY);
}
//-----------------------------
static void displayWriteInstruction(uint8_t data){

	gpioOutputWrite(GPIO_PB, MASK, (uint16_t)((data << 1) & DATA_MASK));
	displayEnable();
	displayInefficientDelay(0xE10 >> 6);
	gpioOutputWrite(GPIO_PB, MASK, (uint16_t)((data << 5) & DATA_MASK));
	displayEnable();
	displayInefficientDelay(configDISPLAY_INSTRUCTION_WRITE_DELAY);
}
//-----------------------------
static void displayWriteData(uint8_t data){

	gpioOutputWrite(GPIO_PB, MASK, (uint16_t)(RS | ((data << 1) & DATA_MASK)));
	displayEnable();
	gpioOutputWrite(GPIO_PB, MASK, (uint16_t)(RS | ((data << 5) & DATA_MASK)));
	displayEnable();
	displayInefficientDelay(configDISPLAY_DATA_WRITE_DELAY);
}
//-----------------------------
//=============================

