/*
 * startup.c
 *
 *  Created on: Dec 7, 2019
 *      Author: marco
 */

//=============================
/*--------- Includes --------*/
//=============================
#include "startup.h"

/* Device */
#include "stm32f10x.h"

/* Vector table */
#include "isr_vector.h"
//=============================

//=============================
/*---------- Extern ---------*/
//=============================
/* Main */
extern void main(void);

/* .data and .bss segments */
extern uint32_t __sdata_ram, __edata_ram, __sdata_flash;
extern uint32_t __sbss, __ebss;
//=============================

//=============================
/*-------- Prototypes -------*/
//=============================
//-----------------------------
/** @brief Sets hardware.
 *
 * Here, the system clock, flash wait-states and JTAG pins are set.
 *
 * The system clock is set as PLL / 2. The PLL clock is set to run from an
 * HSE crystal of 8 MHz, with a 9x multiplier. The clocks are set as follows
 * 	- AHB: 72 MHz
 * 	- APB1: 36 MHz
 * 	- APB2: 72 MHz
 * 	- ADC: 14 MHz
 * 	and the flash wait-states is set to 2.
 *
 * 	If initialization of either the HSE or PLL fails, the clock falls back to
 * 	the default state, which is HSI @ 8 MHz. In this case, the AHB, APB1 and
 * 	APB2 clocks are set to 8 MHz, while the ADC clock is set to 4 MHz.
 *
 * 	In addition, the JTAG pins are freed for used (PB3, PB4 and PA15).
 */
void startupHW(void);
//-----------------------------
uint32_t startupClockHSE(void);
//-----------------------------
void startupDisableJTAG(void);
//-----------------------------
/**
 * @brief Copies from source to destiny.
 *
 * The pointer type is uint32_t (word), so 4 bytes are copied at time. The
 * pointers are incremented and their initial values are lost.
 *
 * @param dst Pointer to destiny.
 * @param src Pointer to source.
 * @param size Number of words to copy.
 */
void startMemCopy(uint32_t *dst, uint32_t *src, uint32_t size);
//-----------------------------
/**
 * @brief Sets the destiny with a single value.
 *
 * The pointer is uin32_t (word), so 4 bytes are set at the same time. The
 * pointer is incremented and its initial value is lost
 *
 * @param dst Pointer to destiny.
 * @param val Value to set destiny.
 * @param size Number of words to set with the same value.
 */
void startMemSet(uint32_t *dst, uint32_t val, uint32_t size);
//-----------------------------
//=============================

#define configSTARTUP_CLK_TIMEOUT	0x1FFFFF

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void startup(void){

	/* Sets hardware */
    startupHW();

    /* Initializes RAM memory */
    startMemCopy((uint32_t *)&__sdata_ram, (uint32_t *)&__sdata_flash, (uint32_t)(&__edata_ram - &__sdata_ram));
    startMemSet((uint32_t *)&__sbss, 0x00, (uint32_t)(&__ebss - &__sbss));

    /* Jumps to main */
    main();
}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
void startupHW(void){

	uint32_t status;

	status = startupClockHSE();

	startupDisableJTAG();
}
//-----------------------------
uint32_t startupClockHSE(void){

	uint32_t k;
    /*
     * Here, we attempt to set the system clock to 72 MHz.
     *
     * We consider that an 8 MHz ressonator is connected to the HSE pins.
     * This 8 MHz clock is fed to the PLL, which is set to multiply its
     * input by 9, thus providing an output of 72 MHz. Then, the PLL's
     * output is set as system clock.
     *
     * Important considerations:
     *  - Maximum clock for AHB: 72 MHz
     *  - Maximum clock for APB1: 36 MHz
     *  - Maximum clock for APB2: 72 MHz
     *  - Maximum clock for ADC: 14 MHz
     *
     * The AHB and APB2 can run on 72 MHz, so their prescaler are set to 1.
     * However, the APB1 can run only at 36 MHz, while the ADC can oly run
     * at 14 MHz. Thus, the APB1 prescaler is set to 2, while the ADC
     * prescaler is set to 6.
     *
     * In addition, the reference manual specifies that, for clocks higher
     * than 48 MHz (and up to 72 MHz), flash wait states should be set to 2.
     */

    /* Turns HSE on and waits until it is stable */
	k = configSTARTUP_CLK_TIMEOUT;
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY) && --k);
    if(k == 0) {
    	/* If timed out, turns HSE off */
    	RCC->CR &= (uint32_t)(~RCC_CR_HSEON);
    	return 1U;
    }

    /*
     * Now, sets the RCC config register.
     *  - PLL input: HSE clock (8 MHz)
     *  - PLL multiplier: 9 (PLL output: 8 MHz * 9 = 72 MHz)
     *  - ADC prescaler: 6 (ADC clock: 72 MHz / 6 = 14 MHz)
     *  - APB1 prescaler: 2 (APB1 clock: 72 MHz / 2 = 36 MHz)
     *  - APB2 prescaler: 1 (APB2 clock: 72 MHz)
     *  - AHB prescaler: 1 (AHB clock: 72 MHz)
     */
    RCC->CFGR |= (RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE1_DIV2);

    /* Turns PLL on and waits until it is stable */
    k = configSTARTUP_CLK_TIMEOUT;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY) && --k);
    if(k == 0){
    	/* If timed out, turns HSE and PLL off */
    	RCC->CR &= (uint32_t)(~(RCC_CR_HSEON | RCC_CR_PLLON));
    	return 2U;
    }

    /* Enables prefetch buffer (should be enabled on reset) */
    FLASH->ACR |= (1U << 4);

    /* Sets wait states to two (since the system clock will be > 48 MHz) */
    FLASH->ACR |= 2U;

    /* Sets system clock as PLL's output and waits until it is set */
    k = configSTARTUP_CLK_TIMEOUT;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(((RCC->CFGR & RCC_CFGR_SWS_PLL) != 0x08U) && --k);
    if(k == 0){
    	/*
    	 * If timed out, turns HSE and PLL off, sets wait-states back to its
    	 * reset value and clears the RCC configuration register (CFGR).
    	 * By default, the system clock should already be HSI @ 8 MHz.
    	 */
    	FLASH->ACR &= (uint32_t)(~2U);
    	RCC->CR &= (uint32_t)(~(RCC_CR_HSEON | RCC_CR_PLLON));
    	RCC->CFGR = 0;
    	return 3U;
    }

	return 0;
}
//-----------------------------
void startupDisableJTAG(void){

	/* Disable JTAG to free pins PB_3, PB_4 and PA_15 */
	RCC->APB2ENR |= 1;
	AFIO->MAPR = (0x02 << 24);
}
//-----------------------------
void startMemCopy(uint32_t *dst, uint32_t *src, uint32_t size){

    while(size--){
        *dst++ = *src++;
    }
}
//-----------------------------
void startMemSet(uint32_t *dst, uint32_t val, uint32_t size){

    while(size--){
        *dst++ = val;
    }
}
//-----------------------------
//=============================
