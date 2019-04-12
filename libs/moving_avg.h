/*
 * moving_avg.h
 *
 *  Created on: Apr 7, 2019
 *      Author: marco
 */

#ifndef MOVING_AVG_H_
#define MOVING_AVG_H_

//=============================
/*--------- Includes --------*/
//=============================
#include <stdint.h>
//=============================

//=============================
/*--------- Defines ---------*/
//=============================
#define configMOVING_AVG_N		3

#define configMOVING_AVG_Q_BASE	10
//=============================


//=============================
/*--------- Structs ---------*/
//=============================
//-----------------------------
typedef struct{

	uint16_t samples[1 << configMOVING_AVG_N];
	uint16_t head;
	uint16_t tail;
	uint32_t psum;
}movingAvg_t;
//-----------------------------
//=============================

//=============================
/*-------- Functions --------*/
//=============================
void movingAvgDataInitialize(movingAvg_t *data);
uint16_t movingAvgCompute(uint16_t sample, movingAvg_t *data);
//=============================

#endif /* MOVING_AVG_H_ */
