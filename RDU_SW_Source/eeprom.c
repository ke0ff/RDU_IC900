/*********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   **************
 *
 *  File name: eeprom.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This file encapsulates the EEPROM driver Fns.
 *
 *  Project scope revision history:
 *    03-23-15 jmh:  creation date
 *
 *******************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "tiva_init.h"
#include "init.h"
#include "eeprom.h"

//=============================================================================
// local registers

//=============================================================================
// local Fn declarations



//*****************************************************************************
// eeprom_init()
//  initializes the eeprom memory
//*****************************************************************************
#define	EEREG_WAIT_DLY 200	// 200ms wait limit for register action

U16 eeprom_init(void){
	volatile uint32_t tempt;
	U16 ipl = 0;

	SYSCTL_RCGCEEPROM_R = SYSCTL_RCGCEEPROM_R0;			// enable clock to eeprom
	tempt += 1;											// delay
	tempt -= 1;
	if(wait_reg0(&EEPROM_EEDONE_R, EEPROM_EEDONE_WORKING, EEREG_WAIT_DLY)){
		ipl |= IPL_REGWERR;
	}
	if(EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY)){
		ipl = IPL_EEPERR;								// return error
	}else{
		SYSCTL_SREEPROM_R = SYSCTL_SREEPROM_R0;
		tempt += 1;										// do delay
		tempt -= 1;
		SYSCTL_SREEPROM_R &= ~SYSCTL_SREEPROM_R0;		// clear reset
		tempt += 1;										// do delay
		tempt -= 1;
		if(wait_reg1(&SYSCTL_PREEPROM_R, SYSCTL_PREEPROM_R0, EEREG_WAIT_DLY)){	// wait for EEPROM ready
			ipl |= IPL_REGWERR;
		}
		if(wait_reg0(&EEPROM_EEDONE_R, EEPROM_EEDONE_WORKING, EEREG_WAIT_DLY)){
			ipl |= IPL_REGWERR;
		}
		if(EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY)){
			ipl = IPL_EEPERR;							// return error
		}
	}
	return ipl;
}

//*****************************************************************************
// eerd() reads a 32 bit word from the EEPROM array
//*****************************************************************************
U32 eerd(U16 addr){

	if(addr < (EEPROM_EESIZE_R & 0xffff)){
		EEPROM_EEBLOCK_R = addr >> 4;					// set block addr
		EEPROM_EEOFFSET_R = addr & 0x0f;				// set word addr
		return EEPROM_EERDWR_R;							// read back word
	}else{
		return 0;
	}
}

//*****************************************************************************
// eewr() writes a 32 bit word to the EEPROM array
//*****************************************************************************
U8 eewr(U16 addr, U32 data){
	U8	eflag = EE_OK;

	if(addr < (EEPROM_EESIZE_R & 0xffff)){
		EEPROM_EEBLOCK_R = addr >> 4;					// set block addr
		EEPROM_EEOFFSET_R = addr & 0x0f;				// set word addr
		EEPROM_EERDWR_R = data;							// write word
		if(wait_reg0(&EEPROM_EEDONE_R, EEPROM_EEDONE_WORKING, EEREG_WAIT_DLY)){
			eflag |= EE_FAIL;
		}
		if(EEPROM_EESUPP_R){
			EEPROM_EESUPP_R = EEPROM_EESUPP_START;		// start erase
			if(wait_reg0(&EEPROM_EEDONE_R, EEPROM_EEDONE_WORKING, EEREG_WAIT_DLY)){
				eflag |= EE_FAIL;
			}
		}
	}else{
		eflag |= EE_FAIL;
	}
	return eflag;
}
