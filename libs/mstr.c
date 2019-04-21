/*
 * mstr.c
 *
 *  Created on: Feb 4, 2018
 *      Author: Marco
 */

#include "mstr.h"

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
uint8_t mstrCompare(char *str1, char *str2){

	while(*str1){
		if(*str1++ != *str2++) return 1;
	}

	return 0;

}
//-----------------------------
uint8_t mstrIntToStr(uint32_t x, uint8_t *buffer){

	uint8_t auxBuffer[20];
	uint8_t *auxBufferptr = auxBuffer;
	uint8_t k = 0;
	uint8_t strlen;

	if( !x ){
		*buffer = 0x30;
		strlen = 1;
		return strlen;
	}
	/*
	 * Saves each number of the integer to a different position
	 * in the buffer. For instance, if the integer is 1234, it will save
	 * 4, 3, 2 and 1 to the auxBuffer.
	 */
	while(x){
		*auxBufferptr++ = (uint8_t)(x % 10);
		x /= 10;
		k++;
	}

	/*
	 * Now flip the auxBuffer will be converted to ASCII characters and
	 * saved to the buffer. For instance, if the auxBuffer is 4, 3, 2 and 1,
	 * it will save 0x31, 0x32, 0x33, 0x34 to the buffer.
	 */
	strlen = k;
	while(k--){
		*buffer++ = (uint8_t)(*(--auxBufferptr) + 0x30);
	}

	return strlen;
}
//-----------------------------
void mstrIntToStrFixedDigits(uint32_t x, uint8_t *buffer, uint8_t digits){

	uint8_t auxBuffer[20];
	uint8_t *auxBufferptr = auxBuffer;
	uint8_t k = 0;

	/*
	 * Saves each number of the integer to a different position
	 * in the buffer. For instance, if the integer is 1234, it will save
	 * 4, 3, 2 and 1 to the auxBuffer.
	 */
	while(digits--){
		*auxBufferptr++ = (uint8_t)(x % 10);
		x /= 10;
		k++;
	}

	/*
	 * Now flip the auxBuffer will be converted to ASCII characters and
	 * saved to the buffer. For instance, if the auxBuffer is 4, 3, 2 and 1,
	 * it will save 0x31, 0x32, 0x33, 0x34 to the buffer.
	 */
	while(k--){
		*buffer++ = (uint8_t)(*(--auxBufferptr) + 0x30);
	}
}
//-----------------------------
uint16_t mstrFloatToStr(float x, uint8_t *buffer, uint8_t digits){

	uint32_t xInt;
	uint32_t xDec;
	uint32_t multiplier;
	uint8_t xIntMsgSize;
	uint8_t xDecDigits;

	xInt = (uint32_t)x;

	xDecDigits = digits;
	multiplier = 1;
	while(xDecDigits--) multiplier = 10*multiplier;
	xDec = (uint32_t)((float)multiplier*((x - (float)xInt)) );

	xIntMsgSize = mstrIntToStr(xInt, buffer);
	buffer[xIntMsgSize] = '.';
	mstrIntToStrFixedDigits(xDec, &buffer[xIntMsgSize + 1], digits);

	return (uint16_t)(xIntMsgSize + digits + 1);
}
//-----------------------------
void mstrCopy(uint8_t *src, uint8_t *dst, uint32_t len){

	while(len--){
		*dst++ = *src++;
	}
}
//-----------------------------
//=============================
