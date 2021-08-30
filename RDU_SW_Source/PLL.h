/********************************************************************
 ************ COPYRIGHT (c) 2014 by ke0ff, Taylor, TX   *************
 *
 *  File name: PLL.h
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the header file for PLL.c.
 *
 *	Adapted from file from J. Valvano:
 *		This example accompanies the book:
 *		"Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
 *		ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
 *		Program 2.10, Figure 2.37
 *
 * Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
 *    You may use, edit, run or distribute this file
 *    as long as the above copyright notice remains
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 * For more information about my classes, my research, and my books, see
 * http://users.ece.utexas.edu/~valvano/
 *
 *  File scope revision history:
 *    03-21-15 jmh:  reformatted file
 *    10-03-13 dv:   Creation Date
 *******************************************************************/

#include "typedef.h"

// public Fn declarations
void PLL_Init(U32 sysfreq);
