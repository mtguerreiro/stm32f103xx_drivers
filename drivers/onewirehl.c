/*
 * onewirehl.c
 *
 *  Created on: 23 de abr de 2021
 *      Author: marco
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "onewirehl.h"

/* Device and drivers */
#include "stm32f10x.h"
#include "gpio.h"

#ifdef OWHL_CONFIG_FREERTOS_EN
/* Kernel */
#include "FreeRTOS.h"
#include "semphr.h"
#endif
//===========================================================================


//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	OWHL_STATE_IDLE,
	OWHL_STATE_RESET,
	OWHL_STATE_RESET_WAITING_CLEAR,
	OWHL_STATE_RESET_WAITING_SET,
	OWHL_STATE_WRITE,
	OWHL_STATE_WRITE_RECOVER,
	OWHL_STATE_READ,
	OWHL_STATE_READ_RECOVER,
}owhlStates_t;

typedef enum{
	OWHL_STATUS_IDLE,
	OWHL_STATUS_BUSY,
	OWHL_STATUS_RESET_OK,
	OWHL_STATUS_RESET_FAIL,
	OWHL_STATUS_WRITE_DONE,
	OWHL_STATUS_READ_DONE,
}owhlStatus_t;
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{
	uint8_t bits;
	uint8_t byte;
	owhlStates_t state;
	owhlStatus_t status;
#ifdef OWHL_CONFIG_FREERTOS_EN
	SemaphoreHandle_t semaphore;
#endif
}owhlControl_t;
//===========================================================================


//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
owhlControl_t owhlControl = {.bits = 0, .byte = 0,
		.status = OWHL_STATUS_IDLE, .state = OWHL_STATE_IDLE};
//volatile owhlControl_t owhlControl = {.bits = 0, .byte = 0,
//		.status = OWHL_STATUS_IDLE, .state = OWHL_STATE_IDLE};
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static void onewirehlInitializeTimer(void);
//---------------------------------------------------------------------------
#ifdef OWHL_CONFIG_FREERTOS_EN
static int32_t onewirehlInitializeSW(void);
#endif
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigInit(void);
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigOD(void);
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigInput(void);
//---------------------------------------------------------------------------
static void onewirehlGPIOSet(void);
//---------------------------------------------------------------------------
static void onewirehlGPIOClear(void);
//---------------------------------------------------------------------------
static uint8_t onewirehlGPIORead(void);
//---------------------------------------------------------------------------
static int32_t __attribute__((optimize("O0"))) onewirehlWaitWhileBusy(uint32_t to);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
#define OWHL_CONFIG_GPIO_INIT		onewirehlGPIOConfigInit()
#define OWHL_CONFIG_GPIO_CONFIG_OD	onewirehlGPIOConfigOD()
#define OWHL_CONFIG_GPIO_CONFIG_IN	onewirehlGPIOConfigInput()
#define OWHL_CONFIG_GPIO_OD_SET		onewirehlGPIOSet()
#define OWHL_CONFIG_GPIO_OD_CLEAR	onewirehlGPIOClear()
#define OWHL_CONFIG_GPIO_IN_READ	onewirehlGPIORead()

#define OWHL_CONFIG_SEMAPHORE_TO	1000 / portTICK_PERIOD_MS
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t onewirehlInitialize(void){

	OWHL_CONFIG_GPIO_INIT;

	onewirehlInitializeTimer();

#ifdef OWHL_CONFIG_FREERTOS_EN
	if( onewirehlInitializeSW() != 0 ) return 1;
#endif

	return 0;
}
//---------------------------------------------------------------------------
int32_t onewirehlReset(uint32_t to){

	int32_t ret;

	/* Sets state/status */
	owhlControl.state = OWHL_STATE_RESET;
	owhlControl.status = OWHL_STATUS_BUSY;

#ifdef OWHL_CONFIG_FREERTOS_EN
	/*
	 * Takes the semaphore so we can assume that we can only take it if the
	 * IRQ released it.
	 */
	xSemaphoreTake(owhlControl.semaphore, 0);
#endif

	/* Sets timer to generate a ~800 us delay */
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->ARR = 0xFFFF;

	/* Write 0 to the line and waits */
	OWHL_CONFIG_GPIO_CONFIG_OD;
	OWHL_CONFIG_GPIO_OD_CLEAR;

	/* Runs timer */
	TIM2->CR1 |= TIM_CR1_CEN;

	/* Wait until reset is finished */
	ret =  onewirehlWaitWhileBusy(to);
	owhlControl.state = OWHL_STATE_IDLE;

	if( ret != 0 ) return OWHL_ERR_RESET_TO;

	if( owhlControl.status == OWHL_STATUS_RESET_FAIL ) return OWHL_ERR_RESET_ERR;

	return 0;
}
//---------------------------------------------------------------------------
int32_t onewirehlWrite(uint8_t data, uint32_t to){

	int32_t ret;

	/* Sets state/status */
	owhlControl.state = OWHL_STATE_WRITE;
	owhlControl.status = OWHL_STATUS_BUSY;
	owhlControl.byte = data;
	owhlControl.bits = 0;

#ifdef OWHL_CONFIG_FREERTOS_EN
	/*
	 * Takes the semaphore so we can assume that we can only take it if the
	 * IRQ released it.
	 */
	xSemaphoreTake(owhlControl.semaphore, 0);
#endif

	/*
	 * Sets timer to generate a ~60 us delay for the write time slot. Also
	 * sets the CCR1 to generate a ~1us delay, required to start the writing
	 * process.
	 */
	TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
	TIM2->ARR = 4320 - 1;
	TIM2->CCR1 = 72 - 1;

	/* Write 0 to the line and waits */
	OWHL_CONFIG_GPIO_CONFIG_OD;
	OWHL_CONFIG_GPIO_OD_CLEAR;

	/* Runs timer */
	TIM2->CR1 |= TIM_CR1_CEN;

	/* Wait until writing is finished */
	ret =  onewirehlWaitWhileBusy(to);
	owhlControl.state = OWHL_STATE_IDLE;

	if( ret != 0 ) return OWHL_ERR_WRITE_TO;

	if( owhlControl.status != OWHL_STATUS_WRITE_DONE ) return OWHL_ERR_WRITE_ERR;

	return 0;
}
//---------------------------------------------------------------------------
int32_t onewirehlRead(uint8_t *data, uint32_t to){

	int32_t ret;

	/* Sets state/status */
	owhlControl.state = OWHL_STATE_READ;
	owhlControl.status = OWHL_STATUS_BUSY;
	owhlControl.byte = 0;
	owhlControl.bits = 0;

#ifdef OWHL_CONFIG_FREERTOS_EN
	/*
	 * Takes the semaphore so we can assume that we can only take it if the
	 * IRQ released it.
	 */
	xSemaphoreTake(owhlControl.semaphore, 0);
#endif

	/*
	 * Sets timer to generate a ~60 us delay for the read time slot. Also
	 * sets the CCR1 to generate a ~1 us delay, required to start the reading
	 * process, and sets CCR2 to generate a ~13 us delay, to sample the line.
	 */
	TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
	TIM2->ARR = 4320 - 1;
	TIM2->CCR1 = 72 - 1;
	TIM2->CCR2 = 936 - 1;

	/* Write 0 to the line and waits */
	OWHL_CONFIG_GPIO_CONFIG_OD;
	OWHL_CONFIG_GPIO_OD_CLEAR;

	/* Runs timer */
	TIM2->CR1 |= TIM_CR1_CEN;

	/* Wait until reading is finished */
	ret =  onewirehlWaitWhileBusy(to);
	owhlControl.state = OWHL_STATE_IDLE;

	if( ret != 0 ) return OWHL_ERR_READ_TO;

	if( owhlControl.status != OWHL_STATUS_READ_DONE ) return OWHL_ERR_READ_ERR;

	*data = owhlControl.byte;

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================


//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
static void onewirehlInitializeTimer(void){

	/* Sets interrupt in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 6);
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Sets timer */
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
	TIM2->DIER = TIM_DIER_UIE;

}
//---------------------------------------------------------------------------
#ifdef OWHL_CONFIG_FREERTOS_EN
static int32_t onewirehlInitializeSW(void){

	owhlControl.semaphore = xSemaphoreCreateBinary();

	if( owhlControl.semaphore == NULL ) return OWHL_ERR_SW_INIT;
	xSemaphoreTake(owhlControl.semaphore, 0);

	return 0;
}
#endif
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigInit(void){

	gpioPortEnable(GPIOB);
	gpioConfig(GPIOB, GPIO_P0, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_FLOAT);
}
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigOD(void){
	GPIOB->CRL = (1 << 0) | (1 << 2);
}
//---------------------------------------------------------------------------
static void onewirehlGPIOConfigInput(void){
	GPIOB->CRL = (1 << 2);
}
//---------------------------------------------------------------------------
static void onewirehlGPIOSet(void){
	GPIOB->BSRR = 1;
}
//---------------------------------------------------------------------------
static void onewirehlGPIOClear(void){
	GPIOB->BRR = 1;
}
//---------------------------------------------------------------------------
static uint8_t onewirehlGPIORead(void){

	return (uint8_t)(GPIOB->IDR & 1);
}
//---------------------------------------------------------------------------
static int32_t onewirehlWaitWhileBusy(uint32_t to){

#ifdef OWHL_CONFIG_FREERTOS_EN
	if( xSemaphoreTake(owhlControl.semaphore, to) != pdTRUE) return 1;
#else
	while( (owhlControl.status ==  OWHL_STATUS_BUSY) && (to != 0) ) to--;
	if( to == 0 ) return 1;
#endif

	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*-----------------------------  IRQ Handlers -----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void TIM2_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void TIM2_IRQHandler(void){

	uint32_t status;

	status = TIM2->SR;
	TIM2->SR = 0;

	if( owhlControl.state == OWHL_STATE_RESET ){
		/* Releases the line and sets it as input*/
		OWHL_CONFIG_GPIO_OD_SET;
		OWHL_CONFIG_GPIO_CONFIG_IN;

		owhlControl.state = OWHL_STATE_RESET_WAITING_CLEAR;

		/* Sets the timer to generate an interrupt at ~10 us*/
		TIM2->CCR1 = 720;
		TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
		TIM2->CR1 |= TIM_CR1_CEN;
	} // if( owhlControl.state == OWHL_STATE_RESET )

	else if( owhlControl.state == OWHL_STATE_RESET_WAITING_CLEAR ){
		if( status & TIM_SR_CC1IF ){
			if( OWHL_CONFIG_GPIO_IN_READ == 0 ){
				/* Detected response from sensor. Wait until line is set again */
				TIM2->CNT = 0;
				TIM2->CCR1 = 720;
				owhlControl.state = OWHL_STATE_RESET_WAITING_SET;
			}
			else{
				/* Still waiting for response */
				TIM2->CCR1 = (uint16_t)(TIM2->CNT + 720U);
			}
		} // if( status & TIM_SR_CC1IF )
		else if( status & TIM_SR_UIF ){
			/* Timed-out without response from sensor */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_RESET_FAIL;
#ifdef OWHL_CONFIG_FREERTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		} // else if( status & TIM_SR_UIF )
	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_CLEAR )

	else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET ){
		if( status & TIM_SR_CC1IF ){
			if( OWHL_CONFIG_GPIO_IN_READ == 1 ){
				/* Sensor released the line, reset is successful */
				owhlControl.state = OWHL_STATE_IDLE;
				owhlControl.status = OWHL_STATUS_RESET_OK;
				TIM2->CR1 &= (uint16_t)(~TIM_CR1_CEN);
				/* Clears counter so next cmd can assume counter is zero */
				TIM2->CNT = 0;
#ifdef OWHL_CONFIG_FREERTOS_EN
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
				if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
			}
			else{
				/* Still waiting for response */
				TIM2->CCR1 = (uint16_t)(TIM2->CNT + 720U);
			}
		} // if( status & TIM_SR_CC1IF )
		else if( status & TIM_SR_UIF ){
			/* Timed-out and line was not released */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_RESET_FAIL;
#ifdef OWHL_CONFIG_FREERTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		} // else if( status & TIM_SR_UIF )
	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET )

	else if( owhlControl.state == OWHL_STATE_WRITE ){
		if( status & TIM_SR_CC1IF ){
			/* Releases or clears the line according to the next bit */
			if( owhlControl.byte & 1 ) OWHL_CONFIG_GPIO_OD_SET;
			else OWHL_CONFIG_GPIO_OD_CLEAR;
		}
		else if( status & TIM_SR_UIF ){
			/* Bit written, recover for ~1 us */
			OWHL_CONFIG_GPIO_OD_SET;
			owhlControl.state = OWHL_STATE_WRITE_RECOVER;
			TIM2->ARR = 72 - 1;
			TIM2->DIER = TIM_DIER_UIE;
			TIM2->CR1 |= TIM_CR1_CEN;
		}
	} // else if( owhlControl.state == OWHL_STATE_WRITE )

	else if( owhlControl.state == OWHL_STATE_WRITE_RECOVER ){
		owhlControl.bits++;
		if( owhlControl.bits == 8 ){
			/* Wrote all bits */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_WRITE_DONE;
#ifdef OWHL_CONFIG_FREERTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Writes next bit */
			owhlControl.state = OWHL_STATE_WRITE;
			owhlControl.byte = owhlControl.byte >> 1;
			TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
			TIM2->ARR = 4320 - 1;
			OWHL_CONFIG_GPIO_OD_CLEAR;
			TIM2->CR1 |= TIM_CR1_CEN;
		}
	} // else if( owhlControl.state == OWHL_STATE_WRITE )

	else if( owhlControl.state == OWHL_STATE_READ ){
		if( status & TIM_SR_CC1IF ){
			/* Releases the line and sets it as input */
			OWHL_CONFIG_GPIO_OD_SET;
			OWHL_CONFIG_GPIO_CONFIG_IN;
		}
		else if( status & TIM_SR_CC2IF ){
			/* Samples the line */
			owhlControl.byte |= (uint8_t)(OWHL_CONFIG_GPIO_IN_READ << 7);
		}
		else if( status & TIM_SR_UIF ){
			/* Bit read, recover for ~1 us */
			OWHL_CONFIG_GPIO_CONFIG_OD;
			OWHL_CONFIG_GPIO_OD_SET;
			owhlControl.state = OWHL_STATE_READ_RECOVER;
			TIM2->ARR = 72 - 1;
			TIM2->DIER = TIM_DIER_UIE;
			TIM2->CR1 |= TIM_CR1_CEN;
		}
	} // else if( owhlControl.state == OWHL_STATE_READ

	else if( owhlControl.state == OWHL_STATE_READ_RECOVER ){
		owhlControl.bits++;
		if( owhlControl.bits == 8 ){
			/* Read all bits */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_READ_DONE;
#ifdef OWHL_CONFIG_FREERTOS_EN
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
		}
		else{
			/* Reads next bit */
			owhlControl.state = OWHL_STATE_READ;
			owhlControl.byte = (uint8_t)(owhlControl.byte >> 1);
			TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
			TIM2->ARR = 4320 - 1;
			OWHL_CONFIG_GPIO_OD_CLEAR;
			TIM2->CR1 |= TIM_CR1_CEN;
		}
	} // else if( owhlControl.state == OWHL_STATE_READ_RECOVER )
}
//---------------------------------------------------------------------------
//===========================================================================
