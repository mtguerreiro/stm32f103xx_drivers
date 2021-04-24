/*
 * onewirehl.h
 *
 *  Created on: 23 de abr de 2021
 *      Author: marco
 */

#ifndef DRIVERS_ONEWIREHL_H_
#define DRIVERS_ONEWIREHL_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>


//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
int32_t onewirehlInitialize(void);
//---------------------------------------------------------------------------
int32_t onewirehlReset(void);
//---------------------------------------------------------------------------
int32_t onewirehlWrite(uint8_t data);
//---------------------------------------------------------------------------
//===========================================================================




#endif /* DRIVERS_ONEWIREHL_H_ */