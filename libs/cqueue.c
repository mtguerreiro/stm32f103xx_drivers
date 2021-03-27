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

	if( queue->space == 0 ) return 1;

	*queue->tail = *data;
	queue->space--;
	queue->tail++;
	if( queue->tail == queue->bufferEnd ) queue->tail = queue->buffer;

	return 0;
}
//---------------------------------------------------------------------------
uint8_t cqueueRemove(cqueue_t *queue, uint8_t *data){

	/* If space == size, there are no items in the queue */
	if( queue->space == queue->size ) return 1;

	*data = *queue->head;
	queue->space++;
	queue->head++;
	if( queue->head == queue->bufferEnd ) queue->head = queue->buffer;

	return 0;
}
//---------------------------------------------------------------------------
uint16_t cqueueSpace(cqueue_t *queue){

	return queue->space;
}
//---------------------------------------------------------------------------
//===========================================================================
