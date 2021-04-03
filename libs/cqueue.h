/**
 * @file cqueue.h
 * @brief Provides a simple FIFO queue implementation in C.
 *
 * The queue only supports bytes, and it is mainly intended to use with low-
 * level drivers for communication.
 *
 *  Created on: March 13, 2021
 *      Author: Marco
 */

#ifndef CQUEUE_H_
#define CQUEUE_H_


//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>
//===========================================================================

//===========================================================================
/*---------------------------- Critical section ---------------------------*/
//===========================================================================
#include "stm32f10x.h"

#define CQUEUE_CRITICAL_ENTER	__disable_irq() /**< Enters critical section. */
#define CQUEUE_CRITICAL_EXIT	__enable_irq()	/**< Exits critical section. */

//===========================================================================


//===========================================================================
/*------------------------------- Structs ---------------------------------*/
//===========================================================================
/**
 * Context of the queue.
 */
typedef struct{
	uint8_t *buffer; 	/**< Buffer to hold the data. */
	uint8_t *head;		/**< Newest element. */
	uint8_t *tail;		/**< Oldest element. */
	uint8_t *bufferEnd;	/**< Last position of the buffer. */
	uint16_t size;		/**< Size of buffer holding the data. */
	uint16_t space;		/**< Free space on the queue. */
}cqueue_t;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
/**
 * @brief Initializes the queue.
 *
 * @param queue Pointer to queue struct.
 * @param buffer Pointer to buffer that will hold the data.
 * @param size Size of buffer that will hold the data.
 */
void cqueueInitialize(cqueue_t *queue, uint8_t *buffer, uint16_t size);
//---------------------------------------------------------------------------
/**
 * @brief Adds a byte to the queue.
 *
 * @param queue Pointer to queue struct.
 * @param buffer Pointer to buffer holding the data.
 * @return 0 if the data was added successfully, 1 otherwise.
 */
uint8_t cqueueAdd(cqueue_t *queue, uint8_t *data);
//---------------------------------------------------------------------------
/**
 * @brief Removes a byte from the queue.
 *
 * @param queue Pointer to queue struct.
 * @param buffer Pointer to buffer to save the data.
 * @return 0 if the data was removed successfully, 1 otherwise.
 */
uint8_t cqueueRemove(cqueue_t *queue, uint8_t *data);
//---------------------------------------------------------------------------
/**
 * @brief Space available on the queue.
 *
 * @param queue Pointer to queue struct.
 * @return Space available, in bytes.
 */
uint16_t cqueueSpace(cqueue_t *queue);
//---------------------------------------------------------------------------
//===========================================================================

#endif /* CQUEUE_H_ */
