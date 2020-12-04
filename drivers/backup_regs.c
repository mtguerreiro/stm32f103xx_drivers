/*
 * backup_regs.c
 *
 *  Created on: Jun 21, 2018
 *      Author: Marco
 */

#include "backup_regs.h"

//uint32_t *backupRegsAddress[] = {&BKP->DR1, &BKP->DR2, &BKP->DR3, &BKP->DR4};

//=============================
/*-------- Prototypes -------*/
//=============================
static uint32_t backupRegsGetRegAddress(uint8_t reg);
//=============================
//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void backupRegsEnableAccess(void){
	/* Enables power and backup clocks */
	RCC->APB1ENR |= (1 << 28) | (1 << 27);

	/* Enables access to RTC and backup registers */
	PWR->CR |= (1 << 8);
}
//-----------------------------
uint16_t backupRegsRead(uint8_t reg){

	uint16_t regData;
	uint32_t regAddressVal;
	uint32_t *regAddress;

	/*
	 * We use intermediate variables because the
	 * compiler complains that (1) we cannot cast
	 * from function return and (2) we have to cast
	 * before returning the data.
	 * Future implementations can investigate this
	 * issue to use less processing to do the same job.
	 */

	regAddressVal = backupRegsGetRegAddress(reg);
	regAddress = (uint32_t *)regAddressVal;
	regData = (uint16_t)(*regAddress);

	return regData;
}
//-----------------------------
void backupRegsWrite(uint8_t reg, uint16_t data){

	uint32_t regAddressVal;
	uint32_t *regAddress;

	/*
	 * We use intermediate variables because the
	 * compiler complains that (1) we cannot cast
	 * from function return and (2) we have to cast
	 * before returning the data.
	 * Future implementations can investigate this
	 * issue to use less processing to do the same job.
	 */

	regAddressVal = backupRegsGetRegAddress(reg);
	regAddress = (uint32_t *)regAddressVal;

	*regAddress = (uint32_t)data;
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static uint32_t backupRegsGetRegAddress(uint8_t reg){

	uint32_t address;

	if(reg <= 10){
		address = ((uint32_t)BKP_BASE) + ((uint32_t)(0x04*reg));
	}
	else{
		address = ((uint32_t)BKP_BASE) + (0x40U) + ((uint32_t)(0x04*(reg - 11)));
	}

	return address;
}
//-----------------------------
//=============================
