/*
 * moving_avg.c
 *
 *  Created on: Apr 7, 2019
 *      Author: marco
 */
//=============================
/*--------- Includes --------*/
//=============================
#include "moving_avg.h"

//=============================


//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void movingAvgDataInitialize(movingAvg_t *data){

	uint16_t k;

	k = 1 << configMOVING_AVG_N;
	while(k--){
		data->samples[k] = 0;
	}
	data->head = (1 << configMOVING_AVG_N) - 1;
	data->tail = 0;
	data->psum = 0;
}
//-----------------------------
uint16_t movingAvgCompute(uint16_t sample, movingAvg_t *data){

	uint32_t diff;
//	uint16_t n;
//	uint16_t n2;

	uint32_t sampleIn;
	uint32_t sampleOut;

//	n = 1 << configMOVING_AVG_N;
//	n2 = n >> 1;

	sampleIn = (uint32_t)(sample << configMOVING_AVG_Q_BASE);
	sampleOut = (uint32_t)(data->samples[data->tail] << configMOVING_AVG_Q_BASE);

	if(sampleIn > sampleOut){
		diff = (sampleIn - sampleOut);
		data->psum = data->psum + (diff >> configMOVING_AVG_N);
	}
	else{
		diff = sampleOut - sampleIn;
		data->psum = data->psum - (diff >> configMOVING_AVG_N);
	}

	data->head++;
	if(data->head >= (1 << configMOVING_AVG_N)) data->head = 0;
	data->samples[data->tail] = sample;

	data->tail++;
	if(data->tail >= (1 << configMOVING_AVG_N)) data->tail = 0;

	return (data->psum >> configMOVING_AVG_Q_BASE);
}
//-----------------------------
//=============================
