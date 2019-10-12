/*
 * hsdescr.h
 *
 *  Created on: Oct 12, 2019
 *      Author: marco
 */

#ifndef HSDESCR_H_
#define HSDESCR_H_

#include "garmosys.h"
#include "radio.h"


#define configHSDESCR_RPROT_CMD_SIZE        configRADIO_CMD_SIZE
#define configHSDESCR_RPROT_DATA_SIZE       configRADIO_DATA_SIZE
#define configHSDESCR_RPROT_PAYLOAD_SIZE    configRADIO_PAYLOAD_SIZE

#define configHSDESCR_RPROT_CMD             0
#define configHSDESCR_RPROT_REG             1


#define configHSDESCR_GARMO_REG_STATUS      0
#define configHSDESCR_GARMO_REG_ADC         1
#define configHSDESCR_GARMO_REG_DO          2

#define configHSDESCR_GARMO_REG_DO_RESET    0x00
#define configHSDESCR_GARMO_REG_DO_SET      0x01

#define configHSDESCR_GARMO_CMD_READ        0x00
#define configHSDESCR_GARMO_CMD_WRITE       0x01

#define configHSDRESCR_GARMO_ADC_CHANNELS         4
#define configHSDRESCR_GARMO_ADC_INPUTS           ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))

#endif /* HSDESCR_H_ */
