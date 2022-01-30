///*
// * onewirehl.c
// *
// *  Created on: 23 de abr de 2021
// *      Author: marco
// */
//
////===========================================================================
///*------------------------------- Includes --------------------------------*/
////===========================================================================
//#include "onewirehl.h"
//
///* Device and drivers */
//#include "stm32f10x.h"
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
///* Kernel */
//#include "FreeRTOS.h"
//#include "semphr.h"
//#endif
////===========================================================================
//
//
////===========================================================================
///*--------------------------------- Enums ---------------------------------*/
////===========================================================================
//typedef enum{
//	OWHL_STATE_IDLE,
//	OWHL_STATE_RESET,
//	OWHL_STATE_RESET_WAITING_CLEAR,
//	OWHL_STATE_RESET_WAITING_SET,
//	OWHL_STATE_WRITE,
//	OWHL_STATE_WRITE_RECOVER,
//	OWHL_STATE_READ,
//	OWHL_STATE_READ_RECOVER,
//}owhlStates_t;
//
//typedef enum{
//	OWHL_STATUS_IDLE,
//	OWHL_STATUS_BUSY,
//	OWHL_STATUS_RESET_OK,
//	OWHL_STATUS_RESET_FAIL,
//	OWHL_STATUS_WRITE_DONE,
//	OWHL_STATUS_READ_DONE,
//}owhlStatus_t;
////===========================================================================
//
////===========================================================================
///*-------------------------------- Structs --------------------------------*/
////===========================================================================
//typedef struct{
//	GPIO_TypeDef *port;
//	uint16_t pin;
//	uint16_t pinBit;
//	uint16_t confBit;
//}owhlGPIO_t;
//
//typedef struct{
//	uint8_t bits;
//	uint8_t byte;
//	owhlStates_t state;
//	owhlStatus_t status;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	SemaphoreHandle_t semaphore;
//#endif
//}owhlControl_t;
////===========================================================================
//
//
////===========================================================================
///*-------------------------------- Globals --------------------------------*/
////===========================================================================
//owhlControl_t owhlControl = {.bits = 0, .byte = 0,
//		.status = OWHL_STATUS_IDLE, .state = OWHL_STATE_IDLE};
//owhlGPIO_t owhlgpio;
////===========================================================================
//
////===========================================================================
///*------------------------------- Prototypes ------------------------------*/
////===========================================================================
////---------------------------------------------------------------------------
//static void onewirehlInitializeTimer(void);
////---------------------------------------------------------------------------
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//static int32_t onewirehlInitializeSW(void);
//#endif
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigInit(GPIO_TypeDef *gpio, uint8_t pin);
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigOD(void);
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigInput(void);
////---------------------------------------------------------------------------
//static void onewirehlGPIOSet(void);
////---------------------------------------------------------------------------
//static void onewirehlGPIOClear(void);
////---------------------------------------------------------------------------
//static uint8_t onewirehlGPIORead(void);
////---------------------------------------------------------------------------
//static int32_t __attribute__((optimize("O0"))) onewirehlWaitWhileBusy(uint32_t to);
////---------------------------------------------------------------------------
////===========================================================================
//
////===========================================================================
///*------------------------------- Functions -------------------------------*/
////===========================================================================
////---------------------------------------------------------------------------
//int32_t onewirehlInitialize(void *gpio, uint8_t pin){
//
//	onewirehlGPIOConfigInit((GPIO_TypeDef *)gpio, pin);
//
//	onewirehlInitializeTimer();
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	if( onewirehlInitializeSW() != 0 ) return 1;
//#endif
//
//	return 0;
//}
////---------------------------------------------------------------------------
//int32_t onewirehlReset(uint32_t to){
//
//	int32_t ret;
//
//	/* Sets state/status */
//	owhlControl.state = OWHL_STATE_RESET;
//	owhlControl.status = OWHL_STATUS_BUSY;
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	/*
//	 * Takes the semaphore so we can assume that we can only take it if the
//	 * IRQ released it.
//	 */
//	xSemaphoreTake(owhlControl.semaphore, 0);
//#endif
//
//	/* Sets timer to generate a ~800 us delay */
//	TIM2->DIER = TIM_DIER_UIE;
//	TIM2->ARR = 0xFFFF;
//
//	/* Write 0 to the line and waits */
//	onewirehlGPIOConfigOD();
//	onewirehlGPIOClear();
//
//	/* Runs timer */
//	TIM2->CR1 |= TIM_CR1_CEN;
//
//	/* Wait until reset is finished */
//	ret =  onewirehlWaitWhileBusy(to);
//	owhlControl.state = OWHL_STATE_IDLE;
//
//	if( ret != 0 ) return OWHL_ERR_RESET_TO;
//
//	if( owhlControl.status == OWHL_STATUS_RESET_FAIL ) return OWHL_ERR_RESET_ERR;
//
//	return 0;
//}
////---------------------------------------------------------------------------
//int32_t onewirehlWrite(uint8_t data, uint32_t to){
//
//	int32_t ret;
//
//	/* Sets state/status */
//	owhlControl.state = OWHL_STATE_WRITE;
//	owhlControl.status = OWHL_STATUS_BUSY;
//	owhlControl.byte = data;
//	owhlControl.bits = 0;
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	/*
//	 * Takes the semaphore so we can assume that we can only take it if the
//	 * IRQ released it.
//	 */
//	xSemaphoreTake(owhlControl.semaphore, 0);
//#endif
//
//	/*
//	 * Sets timer to generate a ~60 us delay for the write time slot. Also
//	 * sets the CCR1 to generate a ~1us delay, required to start the writing
//	 * process.
//	 */
//	TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
//	TIM2->ARR = 4320 - 1;
//	TIM2->CCR1 = 72 - 1;
//
//	/* Write 0 to the line and waits */
//	onewirehlGPIOConfigOD();
//	onewirehlGPIOClear();
//
//	/* Runs timer */
//	TIM2->CR1 |= TIM_CR1_CEN;
//
//	/* Wait until writing is finished */
//	ret =  onewirehlWaitWhileBusy(to);
//	owhlControl.state = OWHL_STATE_IDLE;
//
//	if( ret != 0 ) return OWHL_ERR_WRITE_TO;
//
//	if( owhlControl.status != OWHL_STATUS_WRITE_DONE ) return OWHL_ERR_WRITE_ERR;
//
//	return 0;
//}
////---------------------------------------------------------------------------
//int32_t onewirehlRead(uint8_t *data, uint32_t to){
//
//	int32_t ret;
//
//	/* Sets state/status */
//	owhlControl.state = OWHL_STATE_READ;
//	owhlControl.status = OWHL_STATUS_BUSY;
//	owhlControl.byte = 0;
//	owhlControl.bits = 0;
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	/*
//	 * Takes the semaphore so we can assume that we can only take it if the
//	 * IRQ released it.
//	 */
//	xSemaphoreTake(owhlControl.semaphore, 0);
//#endif
//
//	/*
//	 * Sets timer to generate a ~60 us delay for the read time slot. Also
//	 * sets the CCR1 to generate a ~1 us delay, required to start the reading
//	 * process, and sets CCR2 to generate a ~13 us delay, to sample the line.
//	 */
//	TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
//	TIM2->ARR = 4320 - 1;
//	TIM2->CCR1 = 72 - 1;
//	TIM2->CCR2 = 936 - 1;
//
//	/* Write 0 to the line and waits */
//	onewirehlGPIOConfigOD();
//	onewirehlGPIOClear();
//
//	/* Runs timer */
//	TIM2->CR1 |= TIM_CR1_CEN;
//
//	/* Wait until reading is finished */
//	ret =  onewirehlWaitWhileBusy(to);
//	owhlControl.state = OWHL_STATE_IDLE;
//
//	if( ret != 0 ) return OWHL_ERR_READ_TO;
//
//	if( owhlControl.status != OWHL_STATUS_READ_DONE ) return OWHL_ERR_READ_ERR;
//
//	*data = owhlControl.byte;
//
//	return 0;
//}
////---------------------------------------------------------------------------
////===========================================================================
//
//
////===========================================================================
///*--------------------------- Static functions ----------------------------*/
////===========================================================================
////---------------------------------------------------------------------------
//static void onewirehlInitializeTimer(void){
//
//	/* Sets interrupt in NVIC */
//	NVIC_SetPriority(TIM2_IRQn, OWHL_CONFIG_TIM_IRQ_PRIO);
//	NVIC_EnableIRQ(TIM2_IRQn);
//
//	/* Sets timer */
//	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//	TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
//	TIM2->DIER = TIM_DIER_UIE;
//}
////---------------------------------------------------------------------------
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//static int32_t onewirehlInitializeSW(void){
//
//	owhlControl.semaphore = xSemaphoreCreateBinary();
//
//	if( owhlControl.semaphore == NULL ) return OWHL_ERR_SW_INIT;
//	xSemaphoreTake(owhlControl.semaphore, 0);
//
//	return 0;
//}
//#endif
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigInit(GPIO_TypeDef *gpio, uint8_t pin){
//
//	uint32_t port;
//	uint32_t bit;
//
//	uint16_t confClear;
//	uint16_t confSet;
//
//	owhlgpio.port = gpio;
//	owhlgpio.pin = pin;
//
//	/*
//	 * First, enables clock to corresponding port, by computing the bit to
//	 * set in the RCC register.
//	 */
//	port = (uint32_t)owhlgpio.port;
//	bit = 1U << (((port - GPIOA_BASE) >> 10) + 2);
//	RCC->APB2ENR |= bit;
//
//	/*
//	 * By happenstance, we can change a STM GPIO from floating input to
//	 * general-purpose open drain output simply by first setting the pin as
//	 * floating input and later on setting bit 0 of the corresponding MODE
//	 * field. So, we'll set the GPIO as floating input and pre compute this
//	 * bit for later on.
//	 */
//	if( owhlgpio.pin < 8 ){
//		owhlgpio.confBit = (uint16_t)( 1 << (owhlgpio.pin * 4) );
//		confClear = (uint16_t)( 0x0B << (owhlgpio.pin * 4) );
//		confSet = (uint16_t)( 0x04 << (owhlgpio.pin * 4) );
//		owhlgpio.port->CRL = ( ( owhlgpio.port->CRL & ((uint16_t)~confClear) ) | confSet );
//	}
//	else{
//		owhlgpio.confBit = (uint16_t)( 1 << ((owhlgpio.pin - 8) * 4) );
//		confClear = (uint16_t)( 0x0B << ((owhlgpio.pin - 8) * 4) );
//		confSet = (uint16_t)( 0x04 << ((owhlgpio.pin - 8) * 4) );
//		owhlgpio.port->CRH = ( ( owhlgpio.port->CRH & ((uint16_t)~confClear) ) | confSet );
//	}
//
//	/*
//	 * Convert gpioPin from the range 0-15 to the actual bit we need to set
//	 * on the BRR/BSRR registers.
//	 */
//	owhlgpio.pinBit = (uint16_t)( 1 << owhlgpio.pin );
//}
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigOD(void){
//
//	if( owhlgpio.pin < 8 ) owhlgpio.port->CRL |= owhlgpio.confBit;
//	else owhlgpio.port->CRH |= owhlgpio.confBit;
//}
////---------------------------------------------------------------------------
//static void onewirehlGPIOConfigInput(void){
//
//	if( owhlgpio.pin < 8 ) owhlgpio.port->CRL &= ( (uint32_t)~owhlgpio.confBit );
//	else owhlgpio.port->CRH &= ( (uint32_t)~owhlgpio.confBit );
//}
////---------------------------------------------------------------------------
//static void onewirehlGPIOSet(void){
//
//	owhlgpio.port->BSRR = owhlgpio.pinBit;
//}
////---------------------------------------------------------------------------
//static void onewirehlGPIOClear(void){
//
//	owhlgpio.port->BRR = owhlgpio.pinBit;
//}
////---------------------------------------------------------------------------
//static uint8_t onewirehlGPIORead(void){
//
//	return (uint8_t)( (owhlgpio.port->IDR & owhlgpio.pinBit) >> owhlgpio.pin);
//}
////---------------------------------------------------------------------------
//static int32_t onewirehlWaitWhileBusy(uint32_t to){
//
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//	if( xSemaphoreTake(owhlControl.semaphore, to) != pdTRUE) return 1;
//#else
//	while( (owhlControl.status ==  OWHL_STATUS_BUSY) && (to != 0) ) to--;
//	if( to == 0 ) return 1;
//#endif
//
//	return 0;
//}
////---------------------------------------------------------------------------
////===========================================================================
//
////===========================================================================
///*-----------------------------  IRQ Handlers -----------------------------*/
////===========================================================================
////---------------------------------------------------------------------------
//void TIM2_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
//void TIM2_IRQHandler(void){
//
//	uint32_t status;
//
//	status = TIM2->SR;
//	TIM2->SR = 0;
//
//	if( owhlControl.state == OWHL_STATE_RESET ){
//		/* Releases the line and sets it as input*/
//		onewirehlGPIOSet();
//		onewirehlGPIOConfigInput();
//
//		owhlControl.state = OWHL_STATE_RESET_WAITING_CLEAR;
//
//		/* Sets the timer to generate an interrupt at ~10 us*/
//		TIM2->CCR1 = 720;
//		TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
//		TIM2->CR1 |= TIM_CR1_CEN;
//	} // if( owhlControl.state == OWHL_STATE_RESET )
//
//	else if( owhlControl.state == OWHL_STATE_RESET_WAITING_CLEAR ){
//		if( status & TIM_SR_CC1IF ){
//			if( onewirehlGPIORead() == 0 ){
//				/* Detected response from sensor. Wait until line is set again */
//				TIM2->CNT = 0;
//				TIM2->CCR1 = 720;
//				owhlControl.state = OWHL_STATE_RESET_WAITING_SET;
//			}
//			else{
//				/* Still waiting for response */
//				TIM2->CCR1 = (uint16_t)(TIM2->CNT + 720U);
//			}
//		} // if( status & TIM_SR_CC1IF )
//		else if( status & TIM_SR_UIF ){
//			/* Timed-out without response from sensor */
//			owhlControl.state = OWHL_STATE_IDLE;
//			owhlControl.status = OWHL_STATUS_RESET_FAIL;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
//			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//		} // else if( status & TIM_SR_UIF )
//	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_CLEAR )
//
//	else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET ){
//		if( status & TIM_SR_CC1IF ){
//			if( onewirehlGPIORead() == 1){
//				/* Sensor released the line, reset is successful */
//				owhlControl.state = OWHL_STATE_IDLE;
//				owhlControl.status = OWHL_STATUS_RESET_OK;
//				TIM2->CR1 &= (uint16_t)(~TIM_CR1_CEN);
//				/* Clears counter so next cmd can assume counter is zero */
//				TIM2->CNT = 0;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//				xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
//				if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//			}
//			else{
//				/* Still waiting for response */
//				TIM2->CCR1 = (uint16_t)(TIM2->CNT + 720U);
//			}
//		} // if( status & TIM_SR_CC1IF )
//		else if( status & TIM_SR_UIF ){
//			/* Timed-out and line was not released */
//			owhlControl.state = OWHL_STATE_IDLE;
//			owhlControl.status = OWHL_STATUS_RESET_FAIL;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
//			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//		} // else if( status & TIM_SR_UIF )
//	} // else if( owhlControl.state == OWHL_STATE_RESET_WAITING_SET )
//
//	else if( owhlControl.state == OWHL_STATE_WRITE ){
//		if( status & TIM_SR_CC1IF ){
//			/* Releases or clears the line according to the next bit */
//			if( owhlControl.byte & 1 ) onewirehlGPIOSet();
//			else onewirehlGPIOClear();
//		}
//		else if( status & TIM_SR_UIF ){
//			/* Bit written, recover for ~1 us */
//			onewirehlGPIOSet();
//			owhlControl.state = OWHL_STATE_WRITE_RECOVER;
//			TIM2->ARR = 72 - 1;
//			TIM2->DIER = TIM_DIER_UIE;
//			TIM2->CR1 |= TIM_CR1_CEN;
//		}
//	} // else if( owhlControl.state == OWHL_STATE_WRITE )
//
//	else if( owhlControl.state == OWHL_STATE_WRITE_RECOVER ){
//		owhlControl.bits++;
//		if( owhlControl.bits == 8 ){
//			/* Wrote all bits */
//			owhlControl.state = OWHL_STATE_IDLE;
//			owhlControl.status = OWHL_STATUS_WRITE_DONE;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
//			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//		}
//		else{
//			/* Writes next bit */
//			owhlControl.state = OWHL_STATE_WRITE;
//			owhlControl.byte = owhlControl.byte >> 1;
//			TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
//			TIM2->ARR = 4320 - 1;
//			onewirehlGPIOClear();
//			TIM2->CR1 |= TIM_CR1_CEN;
//		}
//	} // else if( owhlControl.state == OWHL_STATE_WRITE )
//
//	else if( owhlControl.state == OWHL_STATE_READ ){
//		if( status & TIM_SR_CC1IF ){
//			/* Releases the line and sets it as input */
//			onewirehlGPIOSet();
//			onewirehlGPIOConfigInput();
//		}
//		else if( status & TIM_SR_CC2IF ){
//			/* Samples the line */
//			owhlControl.byte |= (uint8_t)(onewirehlGPIORead() << 7);
//		}
//		else if( status & TIM_SR_UIF ){
//			/* Bit read, recover for ~1 us */
//			onewirehlGPIOConfigOD();
//			onewirehlGPIOSet();
//			owhlControl.state = OWHL_STATE_READ_RECOVER;
//			TIM2->ARR = 72 - 1;
//			TIM2->DIER = TIM_DIER_UIE;
//			TIM2->CR1 |= TIM_CR1_CEN;
//		}
//	} // else if( owhlControl.state == OWHL_STATE_READ
//
//	else if( owhlControl.state == OWHL_STATE_READ_RECOVER ){
//		owhlControl.bits++;
//		if( owhlControl.bits == 8 ){
//			/* Read all bits */
//			owhlControl.state = OWHL_STATE_IDLE;
//			owhlControl.status = OWHL_STATUS_READ_DONE;
//#if (OWHL_CONFIG_FREERTOS_EN == 1)
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			xSemaphoreGiveFromISR(owhlControl.semaphore, &xHigherPriorityTaskWoken);
//			if( xHigherPriorityTaskWoken == pdTRUE ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//#endif
//		}
//		else{
//			/* Reads next bit */
//			owhlControl.state = OWHL_STATE_READ;
//			owhlControl.byte = (uint8_t)(owhlControl.byte >> 1);
//			TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
//			TIM2->ARR = 4320 - 1;
//			onewirehlGPIOClear();
//			TIM2->CR1 |= TIM_CR1_CEN;
//		}
//	} // else if( owhlControl.state == OWHL_STATE_READ_RECOVER )
//}
////---------------------------------------------------------------------------
////===========================================================================
