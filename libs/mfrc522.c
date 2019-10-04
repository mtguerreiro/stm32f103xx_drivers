///*
// * mfrc522.c
// *
// *  Created on: June 15, 2018
// *      Author: Marco
// */
//
////=============================
///*--------- Includes --------*/
////=============================
//#include <mfrc522.h>
//#include <stdint.h>
//
///* Kernel */
//#include "FreeRTOS.h"
//#include "semphr.h"
//
///* Drivers */
//#include "spi.h"
//#include "gpio.h"
////=============================
//
////=============================
///*--------- Defines ---------*/
////=============================
///* SPI driver */
////-----------------------------
//#define MFRC522_SPI					configMFRC522_SPI
//
//#define MFRC522_SPI_CLK_DIV		 	configMFRC522_SPI_CLK_DIV
////-----------------------------
//
//#define MFRC522_NSS_PORT			configMFRC522_NSS_PORT
//#define MFRC522_NSS_PIN				configMFRC522_NSS_PIN
//
//#define MFRC522_RESET_PORT			configMFRC522_RESET_PORT
//#define MFRC522_RESET_PIN			configMFRC522_RESET_PIN
//
//
//#define MFRC522_IRQ_PORT			configMFRC522_IRQ_PORT
//#define MFRC522_IRQ_PIN				configMFRC522_IRQ_PIN
//
//#define MFRC522_NSS_SET				gpioOutputSet(MFRC522_NSS_PORT, MFRC522_NSS_PIN)
//#define MFRC522_NSS_RESET			gpioOutputReset(MFRC522_NSS_PORT, MFRC522_NSS_PIN)
//
//#define MFRC522_RESET_SET			gpioOutputSet(MFRC522_RESET_PORT, MFRC522_RESET_PIN)
//#define MFRC522_RESET_RESET			gpioOutputReset(MFRC522_RESET_PORT, MFRC522_RESET_PIN)
//
///* MFRC522 */
////-----------------------------
///* Registers */
//#define MFRC522_REG_COMMAND		0x01
//#define MFRC522_REG_COMM_IRQ	0x04
//#define MFRC522_REG_ERROR		0x06
//#define MFRC522_REG_STATUS1		0x07
//#define MFRC522_REG_STATUS2		0x08
//#define MFRC522_REG_FIFO_DATA	0x09
//#define MFRC522_REG_FIFO_LEVEL	0x0A
//#define MFRC522_REG_MODE		0x11
//#define MFRC522_REG_TX_ASK		0x15
//#define MFRC522_REG_RFC_FG		0x26
//#define MFRC522_REG_TMODE		0x2A
//#define MFRC522_REG_TPRESCALER	0x2B
//#define MFRC522_REG_TRELOAD_H	0x2C
//#define MFRC522_REG_TRELOAD_L	0x2D
//#define MFRC522_REG_TEST_SEL1	0x31
//#define MFRC522_REG_AUTO_TEST	0x36
//
///* Commands */
//#define MFRC522_CMD_IDLE		0x00
//#define MFRC522_CMD_MEM			0x01
//#define MFRC522_CMD_CALC_CRC	0x03
//#define MFRC522_CMD_NO_CHANGE	0x07
//#define MFRC522_CMD_SOFT_RESET	0x0F
////-----------------------------
////=============================
//
////=============================
///*-------- Prototypes -------*/
////=============================
////-----------------------------
///** @brief Initializes the RESET, IRQ and NSS pins. */
//static void mfrc522PortInitialize(void);
////-----------------------------
///** @brief Sets the IRQ pin to generate interrupts. */
//static void mfrc522EXTIInitialize(void);
////-----------------------------
///** @brief Compares buffer with version.
// *
// * @param buffer Buffer containing bytes read from MFRC522 after the self-test.
// * @param version Buffer containing the expected bytes from a MFRC522  version.
// *
// * @return 0 if buffer and version matches, 1 otherwise.
// */
//uint8_t mfrc522CompareVersion(uint8_t *buffer, uint8_t *version);
////-----------------------------
////=============================
//
////=============================
///*--------- Globals ---------*/
////=============================
//SemaphoreHandle_t mfrc522Semaphore;
//
///* Versions */
////-----------------------------
//uint8_t mfcr522Version;
//uint8_t mfrc522v1Data[] = {
//		0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
//		0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
//		0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
//		0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
//		0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
//		0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
//		0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
//		0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
//};
//
//uint8_t mfrc522v2Data[] = {
//		0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
//		0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
//		0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
//		0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
//		0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
//		0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
//		0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
//		0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
//};
////-----------------------------
////=============================
//
////=============================
///*-------- Functions --------*/
////=============================
////-----------------------------
//uint8_t mfrc522Initialize(void){
//
//	uint32_t k;
//
//	mfrc522PortInitialize();
//
//	/* Hard-reset */
//	MFRC522_NSS_SET;
//	MFRC522_RESET_RESET;
//	k = 0x63FFA;
//	while(k--);
//	MFRC522_RESET_SET;
//	k = 0x63FFA;
//	while(k--);
//
//	if( spiInitialize(MFRC522_SPI, MFRC522_SPI_CLK_DIV) ) return 1;
//
//	mfrc522Semaphore = xSemaphoreCreateBinary();
//	if(mfrc522Semaphore == NULL) return 2;
//	xSemaphoreTake(mfrc522Semaphore, 0);
//
//	/* Performs self-test to verify version */
//	if( mfrc522SelfTest(10) ) return 1;
//
//	/* Settings for operation */
//	mfrc522WriteRegister(MFRC522_REG_TMODE, 0x80);
//	mfrc522WriteRegister(MFRC522_REG_TPRESCALER, 0xA9);
//	mfrc522WriteRegister(MFRC522_REG_TRELOAD_H, 0x03);
//	mfrc522WriteRegister(MFRC522_REG_TRELOAD_L, 0xE8);
//	mfrc522WriteRegister(MFRC522_REG_TX_ASK, 0x40);
//	mfrc522WriteRegister(MFRC522_REG_MODE, 0x3D);
//	mfrc522WriteRegister(MFRC522_REG_RFC_FG	, 0x07U << 4);
//
//	return 0;
//}
////-----------------------------
//uint8_t mfrc522Read(uint8_t *buffer, uint8_t size, uint32_t pendTicks){
//
//
//    return 0;
//}
////-----------------------------
//uint8_t mfrc522Write(uint8_t *buffer, uint8_t size, uint32_t pendTicks){
//
//
//    return 0;
//}
////-----------------------------
//uint8_t mfrc522ReadRegister(uint8_t reg, uint8_t *buffer){
//
//	uint8_t command[2];
//
//	/* Formats register for SPI, read command always has the MSB set */
//	command[0] = (uint8_t)(0x80 | ((reg & 0x3FU) << 1));
//	command[1] = 0x00;
//
//	MFRC522_NSS_RESET;
//
//	/* Sends the command twice to provide clock for slave's transmission */
//	spiWrite(configMFRC522_SPI, command, 2);
//	spiWaitTX(configMFRC522_SPI, 0xFFFF);
//
//	MFRC522_NSS_SET;
//
//	/* Received two bytes, discards the first one and saves the second */
//	if( spiRead(configMFRC522_SPI, buffer, 0) ) return 2;
//	if( spiRead(configMFRC522_SPI, buffer, 0) ) return 2;
//
//	return 0;
//}
////-----------------------------
//uint8_t mfrc522WriteRegister(uint8_t reg, uint8_t data){
//
//	uint8_t command[2];
//
//	/* Formats register for SPI, write command always has the MSB cleared */
//	command[0] = (uint8_t)((reg & 0x3FU) << 1);
//	command[1] = data;
//
//	MFRC522_NSS_RESET;
//
//	/* Sends the command and waits until transmission is over */
//	spiWrite(configMFRC522_SPI, command, 2);
//	if( spiWaitTX(configMFRC522_SPI, 0xFFFF) ) {
//		MFRC522_NSS_SET;
//		return 1;
//	}
//
//	MFRC522_NSS_SET;
//
//	/* Discards the received bytes */
//	if( spiRead(configMFRC522_SPI, command, 0) ) return 1;
//	if( spiRead(configMFRC522_SPI, command, 0) ) return 1;
//
//	return 0;
//}
////-----------------------------
//void mfrc522SoftReset(void){
//
//	uint32_t delay;
//
//	/* Sends reset command */
//	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_SOFT_RESET);
//
//	/* Waits for reset */
//	delay = 0x63FFA;
//	while(delay--);
//}
////-----------------------------
//uint8_t mfrc522SelfTest(uint32_t timeout){
//
//	uint32_t k;
//	uint8_t buffer[64];
//	uint8_t fifoSize;
//
//	/* Initializes version to 0 */
//	mfcr522Version = 0x00;
//
//	k = 0;
//	while(k < 64){
//		buffer[k] = 0x00;
//		k++;
//	}
//
//	/* Sends idle command */
//	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_IDLE);
//	mfrc522ReadRegister(MFRC522_REG_COMMAND, buffer);
//
//	/* Step 1: perform a soft-reset */
//	mfrc522SoftReset();
//
//	/*
//	 * Step 2: clear the internal buffer
//	 * Clears the internal buffer by writing 25 bytes of 0x00 to it.
//	 * Note that the bytes must first be written to the FIFO buffer and then
//	 * transferred to the internal buffer with the "Mem" command
//	 */
//	mfrc522FIFOFlush();
//	mfrc522FIFOWrite(buffer, 25);
//	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_MEM);
//
//	/* Step 3: Enable the self test */
//	mfrc522WriteRegister(MFRC522_REG_AUTO_TEST, 0x09);
//
//	/* Step 4: Write 0x00 to the FIFO buffer */
//	mfrc522FIFOWrite(buffer, 1);
//
//	/* Step 5: Enable CALC_CRC command */
//	mfrc522WriteRegister(MFRC522_REG_COMMAND, MFRC522_CMD_CALC_CRC);
//
//	/* Step 6: wait until FIFO buffer contains 64 bytes */
//	fifoSize = 0;
//	while( (fifoSize != 64) && (timeout > 0) ){
//		k = 0x63FFA;
//		while(k--);
//		mfrc522ReadRegister(MFRC522_REG_FIFO_LEVEL, &fifoSize);
//		timeout--;
//	}
//
//	if(!timeout) return 1;
//
//	/* Reads data from FIFO */
//	mfrc522FIFORead(buffer, 64);
//
//	/* Verifies MFRC522 version */
//	if( !mfrc522CompareVersion(buffer, mfrc522v1Data) ) mfcr522Version = 0x01;
//	if( !mfrc522CompareVersion(buffer, mfrc522v2Data) ) mfcr522Version = 0x02;
//
//
//	return 0;
//}
////-----------------------------
//void mfrc522FIFOFlush(void){
//
//	mfrc522WriteRegister(MFRC522_REG_FIFO_LEVEL, 0x80);
//}
////-----------------------------
//void mfrc522FIFOWrite(uint8_t* buffer, uint8_t nbytes){
//
//	uint8_t *ptr;
//
//	ptr = buffer;
//	while(nbytes--){
//		mfrc522WriteRegister(MFRC522_REG_FIFO_DATA, *ptr++);
//	}
//}
////-----------------------------
//void mfrc522FIFORead(uint8_t* buffer, uint8_t nbytes){
//
//	uint8_t *ptr;
//
//	ptr = buffer;
//	while(nbytes--){
//		mfrc522ReadRegister(MFRC522_REG_FIFO_DATA, ptr++);
//	}
//}
////-----------------------------
//uint8_t mfrc522FIFOLevel(void){
//
//	uint8_t level;
//
//	level = mfrc522ReadRegister(MFRC522_REG_FIFO_LEVEL, &level);
//
//	return level;
//}
////-----------------------------
//uint8_t mfrc522GetVersion(void){
//
//	return mfcr522Version;
//}
////-----------------------------
////=============================
//
////=============================
///*----- Static functions ----*/
////=============================
////-----------------------------
//static void mfrc522PortInitialize(void){
//
//	gpioPortEnable(MFRC522_NSS_PORT);
//	gpioConfig(MFRC522_NSS_PORT, MFRC522_NSS_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);
//
//	gpioPortEnable(MFRC522_RESET_PORT);
//	gpioConfig(MFRC522_RESET_PORT, MFRC522_RESET_PIN, GPIO_MODE_OUTPUT_10MHZ, GPIO_CONFIG_OUTPUT_GP_PUSH_PULL);
//
//	gpioPortEnable(MFRC522_IRQ_PORT);
//	gpioConfig(MFRC522_IRQ_PORT, MFRC522_IRQ_PIN, GPIO_MODE_INPUT, GPIO_CONFIG_INPUT_PULL_UP);
//
//	/* Configures EXTI line 3 as external source */
//	mfrc522EXTIInitialize();
//}
////-----------------------------
//static void mfrc522EXTIInitialize(void){
//
//	/* Selects GPIOB pin 3 as external source for line 3 */
//	AFIO->EXTICR[0] = (1U << 12);
//	/* Interrupt for line 3 is not masked */
//	EXTI->IMR |= (1U << 3);
//	/* Sets falling edge as trigger for line 2 */
//	EXTI->FTSR |= (1U << 3);
//	/* Clears pending register */
//	EXTI->PR |= (1U << 3);
//
//	/* Sets NVIC priority and enables interrupt */
//	NVIC_SetPriority(EXTI3_IRQn, 6);
//	NVIC_EnableIRQ(EXTI3_IRQn);
//}
////-----------------------------
//uint8_t mfrc522CompareVersion(uint8_t *buffer, uint8_t *version){
//
//	uint16_t k;
//	uint8_t *b, *v;
//
//	b = buffer;
//	v = version;
//	k = 64;
//	while(k--){
//		if( *b++ != *v++ ) return 1;
//	}
//
//	return 0;
//}
////-----------------------------
////=============================
//
////=============================
///*------- IRQ Handlers ------*/
////=============================
////-----------------------------
//void EXTI3_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
//void EXTI3_IRQHandler(void){
//
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//	EXTI->PR |= (1U << 3);
//
//	xSemaphoreGiveFromISR(mfrc522Semaphore, &xHigherPriorityTaskWoken);
//	if(xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//}
////-----------------------------
////=============================
