/*
 * startup.c
 *
 *  Created on: Dec 7, 2019
 *      Author: marco
 */

#include "startup.h"
#include "stm32f10x.h"

/* Main */
extern void main(void);

/* .data and .bss segments */
extern uint32_t _sdata, _edata, _data_addr;//,_sbss, _ebss;
extern uint32_t _sbss, _ebss;

void startupHW(void);
void startMemCopy(uint32_t *dst, uint32_t *src, uint32_t size);
void startMemSet(uint32_t *dst, uint32_t val, uint32_t size);

void startup(void){

    startupHW();

    startMemCopy((uint32_t *)&_sdata, (uint32_t *)&_data_addr, &_edata - &_sdata);
    startMemSet((uint32_t *)&_sbss, 0x00, &_ebss - &_sbss);

    main();
}

void startupHW(void){

    /*
     * Here, we set the system clock to 72 MHz.
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

    /* Enables prefetch buffer (should be enabled on reset) */
    FLASH->ACR |= (1U << 4);

    /* Sets wait states to two (since the system clock will be > 48 MHz) */
    FLASH->ACR |= 2U;

    /* Turns HSE on and waits until it is stable */
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

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
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    /* Sets system clock as PLL's output and waits until it is set */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS_PLL) != 0x08U);

}

void startMemCopy(uint32_t *dst, uint32_t *src, uint32_t size){

    while(size--){
        *dst++ = *src++;
    }
}

void startMemSet(uint32_t *dst, uint32_t val, uint32_t size){

    while(size--){
        *dst++ = val;
    }
}
