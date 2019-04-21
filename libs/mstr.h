/*
 * mstr.h
 *
 *  Created on: Feb 4, 2018
 *      Author: Marco
 *
 * Current version: v0.1.3
 *
 * 	-v0.1.0:
 * 		- Initial version
 *
 * 	v0.1.1:
 * 		- Corrected bug in mstrIntToStr when x is zero. Now returns
 * 		strlen = 1 and saves '0' to buffer.
 *
 * 	v0.1.2:
 * 		- Added mstrCopy.
 *
 * 	v0.1.3:
 * 		- Additional casting to get rid of warnings.
 */
#ifndef LIBS_MSTR_H_
#define LIBS_MSTR_H_

//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>
//=============================

//=============================
/*-------- Functions --------*/
//=============================
uint8_t mstrCompare(char *str1, char *str2);
uint8_t mstrIntToStr(uint32_t x, uint8_t *buffer);
void mstrIntToStrFixedDigits(uint32_t x, uint8_t *buffer, uint8_t digits);
uint16_t mstrFloatToStr(float x, uint8_t *buffer, uint8_t digits);
void mstrCopy(uint8_t *src, uint8_t *dst, uint32_t len);
//=============================

#endif /* LIBS_MSTR_H_ */
