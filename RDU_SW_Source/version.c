/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the serial I/O module for the FF-PGMR11
 *             protocol converter.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#define VERSOURCE
#include "version.h"
#include "stdio.h"
#include "serial.h"
#include "init.h"

//------------------------------------------------------------------------------
// Define Statements
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Variable Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// dispSWvers() displays SW version
//-----------------------------------------------------------------------------
void dispSWvers(void){
	char buf[30];
	
	puts1("\n\nKE0FF IC-900 RDU");
    sprintf(buf,"Vers: %s, Date: %s",version_number,date_code);
    puts1(buf);
   	sprintf(buf,"IPL: %04x\n",getipl());
   	puts1(buf);
}

//-----------------------------------------------------------------------------
// ccmdSWvers() returns SW version for ccmds
//-----------------------------------------------------------------------------
void ccmdSWvers(char* sbuf){

    sprintf(sbuf,"RDU,IPL:%04x,Vers:%s,%s",getipl(),version_number,date_code);
}

//-----------------------------------------------------------------------------
// nvram_sn() returns NVRAM sernum
//	this number is used to validate NVRAM revision.  If this number differs
//	from that stored on the NVRAM, the system must execute nvram_fix() to update
//	or re-init the SRAM contents.
//-----------------------------------------------------------------------------
U16 nvram_sn(void){

    return 0x0001;
}
