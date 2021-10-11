/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: dpl_fn.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  dpl support functions
 *  
 *******************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
//#include <intrins.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions

#include "dpl_fn.h"						// header for this c-file
#include "cmd_fn.h"						// fn protos, bit defines, rtn codes, etc.
#include "serial.h"

//=============================================================================
// local registers

// local data structures
//
// DPL code list (112 codes, in octal) from Connect Systems supported list
U16 dpl_list[] = {
	0006, 0007, 0015, 0017, 0021, 0023, 0025, 0026,
	0031, 0032, 0036, 0043, 0047, 0050, 0051, 0053,
	0054, 0065, 0071, 0072, 0073, 0074, 0114, 0115,
	0116, 0122, 0125, 0131, 0132, 0134, 0141, 0143,
	0145, 0152, 0155, 0156, 0162, 0165, 0172, 0174,
	0205, 0212, 0214, 0223, 0225, 0226, 0243, 0244,
	0245, 0246, 0251, 0252, 0255, 0261, 0263, 0265,
	0266, 0271, 0274, 0306, 0311, 0315, 0325, 0331,
	0332, 0343, 0346, 0351, 0356, 0364, 0365, 0371,
	0411, 0412, 0413, 0423, 0431, 0432, 0445, 0446,
	0452, 0454, 0455, 0462, 0464, 0465, 0466, 0503,
	0506, 0516, 0523, 0526, 0532, 0546, 0565, 0606,
	0612, 0624, 0627, 0631, 0632, 0654, 0662, 0664,
	0703, 0712, 0723, 0731, 0732, 0734, 0743, 0754
};
// dpl_calc() LUTs
#define DPL_RBITS	(11)		// #bits in the parity part of the bitstream
#define DPL_CBITS	(9)			// #bits in the code part of the bitstream
#define	DPL_FBITS	(0x800L)	// 3 fixed bits in the bitstream

// parity calc arrays in P11 .. P1 bit order
// padd[] is a mask of the code[8:0] bits that are to be added-up for the P[11:1] results
U16	padd[DPL_RBITS] = { 0x4f, 0x168, 0x1b4, 0x1da, 0x1ed, 0xb9, 0x113, 0x1c6, 0xe3, 0x13e, 0x9f };
// pnot[] defines if the summed result is to be inverted (1 = invert)
U8	pnot[DPL_RBITS] = { 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0 };

//=============================================================================
// local Fn declarations


//=============================================================================
// dpl_cmd() is a CLI debug command to produce the DPL bitstream from the DPL
//	octal code pointed to by the params pointer.  Run get_Dargs() prior to
//	calling and pass pointer to params[0] in param list.  if ps == true ("-s"
//	in cmd line), the command lists the 112 Connect Systems codes to the
//	terminal.
//=============================================================================
void dpl_cmd(char* obuf, U16* params, char ps){

	U8		i;				// temp
	U16		k;				// U16 temp

	k = sizeof(dpl_list)/sizeof(U16);
	if(ps){
		for(i=0; i<k; i++){
			sprintf(obuf,"%3d DPL code: %03o, bitstream: 0x%06x", i, dpl_list[i], dpl_calc(dpl_list[i]));
			putsQ(obuf);
		}
		putsQ(" ");
	}else{
		sprintf(obuf,"DPL list len = %d\n", k);
		putsQ(obuf);
		sprintf(obuf,"DPL code: %03o, bitstream: 0x%06x\n", *params, dpl_calc(*params));
		putsQ(obuf);
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// dpl_calc() calculates DPL bitstream from code# (binary from octal)
//		{ Estimates suggest that this lashup (the function code and LUTs) occupies }
//		{ less than 350 bytes of program memory.  RAM usage is likely less than    }
//		{ 20 bytes between variables and stack.  A direct LUT method would take    }
//		{ about this much PLUS an additional 448 bytes for the bitstream LUT.      }
//-----------------------------------------------------------------------------
U32 dpl_calc(U16 dplcode){
	U8	i = 0;		// temp
	U32	r = 0;		// result

	for(i=0; i<DPL_RBITS; i++){					// start at "C11" and calc parity
		r <<= 1;								// left shift new parity bit into result
		r |= (U32)cadd(dplcode, i);
	}
	r <<= DPL_CBITS + 3;						// shift parity to correct bit location
	r |= DPL_FBITS | (U32)dplcode;				// combine "F" bits and code
	return r;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// cadd() calculates parity bits using Lynn Richardson algorithm (1995)
//	index specifies code bit# (0 = lsb, 8 = msb of 3-digit octal code#).
//-----------------------------------------------------------------------------
U8 cadd(U16 dplcode, U8 index){
	U8	i;			// bit index
	U16	c;			// temp
	U8	r = 0;		// result

	c = dplcode & padd[index];					// mask bits to add for this index (bit#)
	for(i=0; i<DPL_RBITS; i++){					// count the 1's
		if(c & 1) r++;
		c >>= 1;
	}
	if(pnot[index]) r = ~r;						// invert if specified
	return (r & 1);								// return lsb of result
}
