/*
 * boot.c
 *
 *  Created on: Jan 25, 2020
 *      Author: marco
 */

#include <stdint.h>
#include "stm32f10x.h"
//#include "stm32f10x_flash.h"
#include "boot.h"

//=============================
/*---------- Extern ---------*/
//=============================
extern uint32_t __stext_flash, __etext_flash;
//=============================

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
#define ACR_LATENCY_Mask         ((uint32_t)0x00000038)

#define FLASH_Latency_0                ((uint32_t)0x00000000)  /*!< FLASH Zero Latency cycle */
#define FLASH_Latency_1                ((uint32_t)0x00000001)  /*!< FLASH One Latency cycle */
#define FLASH_Latency_2                ((uint32_t)0x00000002)  /*!< FLASH Two Latency cycles */

#define FLASH_FLAG_BSY                 ((uint32_t)0x00000001)  /*!< FLASH Busy flag */
#define FLASH_FLAG_BANK1_BSY                 FLASH_FLAG_BSY

#define FLASH_FLAG_PGERR               ((uint32_t)0x00000004)  /*!< FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR            ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag */

#define FLASH_FLAG_BANK1_PGERR               FLASH_FLAG_PGERR
#define FLASH_FLAG_BANK1_WRPRTERR            FLASH_FLAG_WRPRTERR

/* Flash Control Register bits */
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE)
#define CR_PER_Set               ((uint32_t)0x00000002)
#define CR_PER_Reset             ((uint32_t)0x00001FFD)
#define CR_MER_Set               ((uint32_t)0x00000004)
#define CR_MER_Reset             ((uint32_t)0x00001FFB)
#define CR_OPTPG_Set             ((uint32_t)0x00000010)
#define CR_OPTPG_Reset           ((uint32_t)0x00001FEF)
#define CR_OPTER_Set             ((uint32_t)0x00000020)
#define CR_OPTER_Reset           ((uint32_t)0x00001FDF)
#define CR_STRT_Set              ((uint32_t)0x00000040)
#define CR_LOCK_Set              ((uint32_t)0x00000080)

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

#define EraseTimeout          ((uint32_t)0x000B0000)
#define ProgramTimeout        ((uint32_t)0x00002000)

//=============================
/*-------- Prototypes -------*/
//=============================
static void __attribute__ ((section(".boot"))) bootInit(void);
static void __attribute__ ((section(".boot"))) bootBoot(void);
static void __attribute__ ((section(".boot"))) bootFlashUnlock(void);
static void __attribute__ ((section(".boot"))) bootFlashSetLatency(uint32_t FLASH_Latency);
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashWaitForLastOperation(uint32_t Timeout);
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashGetBank1Status(void);
static void __attribute__ ((section(".boot"))) bootFlashErase(void);
static void __attribute__ ((section(".boot"))) bootFlashWrite(uint32_t address, uint8_t *buffer, uint32_t size);
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashProgramWord(uint32_t address, uint32_t data);
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashErasePage(uint32_t address);
//=============================

//=============================
/*-------- Functions --------*/
//=============================
//-----------------------------
void __attribute__ ((section(".boot"))) boot(void){

    uint32_t k;
    /* Initializes UART */
    bootInit();

    /* Sends out a '?' to wait for boot data */
    USART1->DR = ((uint8_t)'?');
    USART1->CR1 |= USART_CR1_TE;
    while(!(USART1->SR & USART_SR_TC));

    /* Waits for a response */

    while(!(USART1->SR & USART_SR_RXNE));

    k = (uint32_t)USART1->DR;
    bootBoot();

}
//-----------------------------
//=============================

//=============================
/*----- Static functions ----*/
//=============================
//-----------------------------
static void __attribute__ ((section(".boot"))) bootBoot(void){

    uint8_t buffer[5000];
    uint8_t *pbuffer;
    uint32_t k;
    uint32_t size;
    uint32_t address;


    k = 4;
    size = 0;
    while(k--){
        size = size << 8;
        while(!(USART1->SR & USART_SR_RXNE));
        size |= (uint32_t)USART1->DR;
    }

    pbuffer = buffer;
    k = size;
    while(k--){
        while(!(USART1->SR & USART_SR_RXNE));
        *pbuffer++ = (uint8_t)USART1->DR;
    }

    bootFlashErase();

    address = 0x08000000 + (5U << 10);
    bootFlashWrite(address, buffer, size >> 2);

}
//-----------------------------
static void __attribute__ ((section(".boot"))) bootInit(void){

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

    bootFlashUnlock();
    bootFlashSetLatency(FLASH_Latency_2);
}
//-----------------------------
static void __attribute__ ((section(".boot"))) bootFlashUnlock(void){
    /* Authorize the FPEC of Bank1 Access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}
//-----------------------------
static void __attribute__ ((section(".boot"))) bootFlashSetLatency(uint32_t FLASH_Latency){
    uint32_t tmpreg = 0;

    /* Check the parameters */
    //assert_param(IS_FLASH_LATENCY(FLASH_Latency));

    /* Read the ACR register */
    tmpreg = FLASH->ACR;

    /* Sets the Latency value */
    tmpreg &= ACR_LATENCY_Mask;
    tmpreg |= FLASH_Latency;

    /* Write the ACR register */
    FLASH->ACR = tmpreg;
}
//-----------------------------
static void __attribute__ ((section(".boot"))) bootFlashErase(void){

    uint32_t flashInit = 0x08000000 + (5U << 10);
    uint32_t pages = 5;
    uint32_t address;
    uint32_t status = 0;

    __disable_irq();
    while(pages--){
        address = flashInit + (pages << 10);
        //status = FLASH_ErasePage(address);
        status = bootFlashErasePage(address);
    }
    __enable_irq();
}
//-----------------------------
static void __attribute__ ((section(".boot"))) bootFlashWrite(uint32_t address, uint8_t *buffer, uint32_t size){

    uint32_t data;

    __disable_irq();
    while(size--){
        data = (uint32_t)*buffer++;
        data |= ((uint32_t)(*buffer++ << 8));
        data |= ((uint32_t)(*buffer++ << 16));
        data |= ((uint32_t)(*buffer++ << 24));
        bootFlashProgramWord(address, data);
        address += 4;
    }
    __enable_irq();
}
//-----------------------------
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashProgramWord(uint32_t address, uint32_t data){
    FLASH_Status status = FLASH_COMPLETE;

    uint32_t word;

    word = data;

    /* Wait for last operation to be completed */
    status = bootFlashWaitForLastOperation(ProgramTimeout);

    if(status != FLASH_COMPLETE) return status;

    /* Programs the first 16 bits */
    FLASH->CR |= CR_PG_Set;
    *(__IO uint16_t*)address = (uint16_t)word;

    /* Wait for programming to be completed */
    status = bootFlashWaitForLastOperation(ProgramTimeout);

    if(status != FLASH_COMPLETE) return status;

    /* Programs the remaining 16 bits */
    address += 2;
    word = data >> 16;
    *(__IO uint16_t*)address = (uint16_t)word;

    /* Waits for programming to be completed */
    status = bootFlashWaitForLastOperation(ProgramTimeout);

    /* Disable the PG Bit */
    FLASH->CR &= CR_PG_Reset;

    /* Return the Program Status */
    return status;
}
//-----------------------------
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashErasePage(uint32_t address){

    FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = bootFlashWaitForLastOperation(EraseTimeout);

  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase the page */
    FLASH->CR|= CR_PER_Set;
    FLASH->AR = address;
    FLASH->CR|= CR_STRT_Set;

    /* Wait for last operation to be completed */
    status = bootFlashWaitForLastOperation(EraseTimeout);

    /* Disable the PER Bit */
    FLASH->CR &= CR_PER_Reset;
  }

  /* Return the Erase Status */
  return status;
}
//-----------------------------
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashWaitForLastOperation(uint32_t Timeout){
    FLASH_Status status = FLASH_COMPLETE;

    /* Check for the Flash Status */
    status = bootFlashGetBank1Status();
    /* Wait for a Flash operation to complete or a TIMEOUT to occur */
    while((status == FLASH_BUSY) && (Timeout != 0x00))
    {
      status = bootFlashGetBank1Status();
      Timeout--;
    }
    if(Timeout == 0x00 )
    {
      status = FLASH_TIMEOUT;
    }
    /* Return the operation status */
    return status;
}
//-----------------------------
static FLASH_Status __attribute__ ((section(".boot"))) bootFlashGetBank1Status(void){
    FLASH_Status flashstatus = FLASH_COMPLETE;

    if((FLASH->SR & FLASH_FLAG_BANK1_BSY) == FLASH_FLAG_BSY)
    {
      flashstatus = FLASH_BUSY;
    }
    else
    {
      if((FLASH->SR & FLASH_FLAG_BANK1_PGERR) != 0)
      {
        flashstatus = FLASH_ERROR_PG;
      }
      else
      {
        if((FLASH->SR & FLASH_FLAG_BANK1_WRPRTERR) != 0 )
        {
          flashstatus = FLASH_ERROR_WRP;
        }
        else
        {
          flashstatus = FLASH_COMPLETE;
        }
      }
    }
    /* Return the Flash Status */
    return flashstatus;
}
//=============================
