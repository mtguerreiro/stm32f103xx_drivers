/*
 * cqueue.c
 *
 *  Created on: March 13, 2021
 *      Author: Marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "cqueue.h"

#include "stm32f10x.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void cqueueInitialize(cqueue_t *queue, uint8_t *buffer, uint16_t size){

	queue->buffer = buffer;
	queue->bufferEnd = &buffer[size];

	queue->head = buffer;
	queue->tail = buffer;

	queue->size = size;
	queue->space = size;
}
//---------------------------------------------------------------------------
uint8_t cqueueAdd(cqueue_t *queue, uint8_t *data){

	CQUEUE_CRITICAL_ENTER;

	if( queue->space == 0 ){
		CQUEUE_CRITICAL_EXIT;
		return 1;
	}
	queue->space--;

	*queue->tail = *data;
	queue->tail++;
	if( queue->tail == queue->bufferEnd ) queue->tail = queue->buffer;

	CQUEUE_CRITICAL_EXIT;

	return 0;
}
//---------------------------------------------------------------------------
uint8_t cqueueRemove(cqueue_t *queue, uint8_t *data){

	CQUEUE_CRITICAL_ENTER;

	/* If space == size, there are no items in the queue */
	if( queue->space == queue->size ){
		CQUEUE_CRITICAL_EXIT;
		return 1;
	}
	queue->space++;

	*data = *queue->head;
	queue->head++;
	if( queue->head == queue->bufferEnd ) queue->head = queue->buffer;

	CQUEUE_CRITICAL_EXIT;

	return 0;
}
//---------------------------------------------------------------------------
uint16_t cqueueSpace(cqueue_t *queue){

	return queue->space;
}
//---------------------------------------------------------------------------
void cqueueReset(cqueue_t *queue){

	CQUEUE_CRITICAL_ENTER;

	queue->head = queue->buffer;
	queue->tail = queue->buffer;
	queue->space = queue->size;

	CQUEUE_CRITICAL_EXIT;
}
//---------------------------------------------------------------------------
//===========================================================================
