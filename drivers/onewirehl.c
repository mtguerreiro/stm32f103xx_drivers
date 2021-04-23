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


#include "gpio.h"
//===========================================================================


//===========================================================================
/*--------------------------------- Enums ---------------------------------*/
//===========================================================================
typedef enum{
	OWHL_STATE_IDLE,
	OWHL_STATE_RESET_INIT,
	OWHL_STATE_RESET_WAITING_CLEAR,
	OWHL_STATE_RESET_WAITING_SET,
}owhlStates_t;

typedef enum{
	OWHL_STATUS_FREE,
	OWHL_STATUS_BUSY,
	OWHL_STATUS_RESET_OK,
	OWHL_STATUS_RESET_FAIL
}owhlStatus_t;
//===========================================================================

//===========================================================================
/*-------------------------------- Structs --------------------------------*/
//===========================================================================
typedef struct{

	uint8_t bits;
	uint8_t byte;
	uint8_t state;
	uint8_t status;
}owhlControl_t;
//===========================================================================


//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
owhlControl_t owhlControl = {.bits = 0, .byte = 0,
		.status = OWHL_STATUS_FREE, .state = OWHL_STATE_IDLE};
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void onewirehlInitializeTimer(void);
//---------------------------------------------------------------------------
void onewirehlGPIOConfigOD(void);
//---------------------------------------------------------------------------
void onewirehlGPIOConfigInput(void);
//---------------------------------------------------------------------------
void onewirehlGPIOSet(void);
//---------------------------------------------------------------------------
void onewirehlGPIOClear(void);
//---------------------------------------------------------------------------
uint8_t onewirehlGPIORead(void);
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------ Definitions ------------------------------*/
//===========================================================================
#define OWHL_CONFIG_GPIO_CONFIG_OD	onewirehlGPIOConfigOD()
#define OWHL_CONFIG_GPIO_CONFIG_IN	onewirehlGPIOConfigInput()
#define OWHL_CONFIG_GPIO_OD_SET		onewirehlGPIOSet()
#define OWHL_CONFIG_GPIO_OD_CLEAR	onewirehlGPIOClear()
#define OWHL_CONFIG_GPIO_IN_READ	onewirehlGPIORead()

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t onewirehlInitialize(void){

	onewirehlInitializeTimer();

	return 0;
}
//---------------------------------------------------------------------------
int32_t onewirehlReset(void){

	/* Sets state/status */
	owhlControl.state = OWHL_STATE_RESET_INIT;
	owhlControl.status = OWHL_STATUS_BUSY;

	/* Sets timer to generate a ~800 us delay */
	TIM2->ARR = 0xFFFF;

	/* Write 0 to the line and waits */
	OWHL_CONFIG_GPIO_CONFIG_OD;
	OWHL_CONFIG_GPIO_OD_CLEAR;

	/* Runs timer */

	/* Wait until reset is finished */
	while( owhlControl.status ==  OWHL_STATUS_BUSY );

	if( owhlControl.status == OWHL_STATUS_RESET_FAIL ) return 1;

	owhlControl.status = OWHL_STATUS_FREE;
	return 0;
}
//---------------------------------------------------------------------------
//===========================================================================


//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void onewirehlInitializeTimer(void){

	/* Sets interrupt in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 6);
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Sets timer */
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
	TIM2->DIER = TIM_DIER_UIE;

}
//---------------------------------------------------------------------------
void onewirehlGPIOConfigOD(void){
	GPIOB->CRL = (1 << 0) | (1 << 2);
}
//---------------------------------------------------------------------------
void onewirehlGPIOConfigInput(void){
	GPIOB->CRL = (1 << 2);
}
//---------------------------------------------------------------------------
void onewirehlGPIOSet(void){
	GPIOB->BSRR = 1;
}
//---------------------------------------------------------------------------
void onewirehlGPIOClear(void){
	GPIOB->BRR = 1;
}
//---------------------------------------------------------------------------
uint8_t onewirehlGPIORead(void){

	return (uint8_t)(GPIOB->IDR & 1);
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

	gpioOutputSet(GPIOA, GPIO_P6);

	status = TIM2->SR;
	TIM2->SR = 0;

	if( owhlControl.state == OWHL_STATE_RESET_INIT ){
		/* Releases the line and sets it as input*/
		OWHL_CONFIG_GPIO_OD_SET;
		OWHL_CONFIG_GPIO_CONFIG_IN;

		owhlControl.state = OWHL_STATE_RESET_WAITING_CLEAR;

		/* Sets the timer to generate an interrupt at ~10 us*/
		TIM2->CCR1 = 720;
		TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
		TIM2->CR1 |= TIM_CR1_CEN;
	} // if( owhlControl.state == OWHL_STATE_RESET_INIT )

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
				TIM2->CCR1 = (uint16_t)(TIM2->CCR1 + 720U);
			}
		} // if( status & TIM_SR_CC1IF )
		else if( status & TIM_SR_UIF ){
			/* Timed-out without response from sensor */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_RESET_FAIL;
		} // else if( status & TIM_SR_UIF )
	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_CLEAR )

	else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET ){
		if( status & TIM_SR_CC1IF ){
			if( OWHL_CONFIG_GPIO_IN_READ == 1 ){
				/* Sensor released the line, reset is successful */
				TIM2->CNT = 0;
				owhlControl.state = OWHL_STATE_IDLE;
				owhlControl.status = OWHL_STATUS_RESET_OK;
			}
			else{
				/* Still waiting for response */
				TIM2->CCR1 = (uint16_t)(TIM2->CCR1 + 720U);
			}
		} // if( status & TIM_SR_CC1IF )
		else if( status & TIM_SR_UIF ){
			/* Timed-out and line was not released */
			owhlControl.state = OWHL_STATE_IDLE;
			owhlControl.status = OWHL_STATUS_RESET_FAIL;
		} // else if( status & TIM_SR_UIF )
	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET )


	gpioOutputReset(GPIOA, GPIO_P6);
}
//---------------------------------------------------------------------------
//===========================================================================
