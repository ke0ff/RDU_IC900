/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: ccmd.h
 *
 *  Module:    Control
 *
 *  Summary:
 *  CCMD Command Interpreter header
 *
 *******************************************************************/

#ifndef CCMD_H_
#define CCMD_H_

//=============================================================================
// Global defines
// ccmd IDs
//#define	SWRESET_CCMD	00			// soft restart
#define	CCMD_IPL	0xff				// IPL command

//=============================================================================
// public command fn declarations

void process_CCMD(U8 cmd);
float get_temp(U8 chan, U8 res);

#endif /* CCMD_H_ */
