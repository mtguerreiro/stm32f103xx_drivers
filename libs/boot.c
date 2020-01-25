/*
 * boot.c
 *
 *  Created on: Jan 25, 2020
 *      Author: marco
 */

#include <stdint.h>
#include "stm32f10x.h"

static void bootInit(void);
static void bootBoot(void);
void boot(void){

    uint32_t k;
    /* Initializes UART */
    bootInit();

    /* Sends out a '?' to wait for boot data */
    USART1->DR = ((uint8_t)'?');
    USART1->CR1 |= USART_CR1_TE;
    while(!(USART1->SR & USART_SR_TC));

    /* Waits for a response */

    while(!(USART1->SR & USART_SR_RXNE));
    //if(k == 0) return;

    k = (uint32_t)USART1->DR;
    bootBoot();

}

static void bootBoot(void){

    uint8_t buffer[5000];
    uint8_t *pbuffer;
    uint32_t k;
    uint32_t size;

    k = 4;
    size = 0;
    while(k--){
        size = size << 8;
        while(!(USART1->SR & USART_SR_RXNE));
        size |= (uint32_t)USART1->DR;
    }

    pbuffer = buffer;
    while(size--){
        while(!(USART1->SR & USART_SR_RXNE));
        *pbuffer++ = (uint8_t)USART1->DR;
    }

}
static void bootInit(void){

    /* Enables clock to USART peripheral and GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /*
     * Sets GPIOA pins as UART TX/RX. TX is pin 9 and RX is pin 10.
     * GPIOA pin 9:
     *  - Mode: output @ 10 MHz
     *  - Conf: Alternate push pull
     * GPIOA pin 10:
     *  - Mode: Input
     *  - Conf: Input floating
     *
     */
    /* Clears mode and conf for pins 9 and 10 */
    GPIOA->CRH &= ~((uint32_t)0x00000FF0);
    /* Sets pin 9 */
    GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_CNF9_1; //(1U << 4) | (1U << 7);
    /* Sets pin 10 */
    GPIOA->CRH |= GPIO_CRH_CNF10_1;
    GPIOA->ODR |= GPIO_ODR_ODR10;

    /* Sets BRR to 52.1 to generate ~9600 bps @ 8 MHz*/
    USART1->BRR = (uint32_t)(52U << 4) | (uint32_t)(1U);
    USART1->CR1 = USART_CR1_RE | USART_CR1_UE;
}
