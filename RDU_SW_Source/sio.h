/********************************************************************
 ************ COPYRIGHT (c) 2015 by KE0FF, Taylor, TX   *************
 *
 *  File name: spi.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for spi.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>
#ifndef SIO_H_
#define SIO_H_


//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
// SOUT data word defines
// masks & data bit defn's
#define SOUT_ADDR	0x38000000L	// bits [29:27]

#define SOUT_CNTL	0x07f00000L	// bits [26:20]
#define SOUT_DATA	0x000fffffL	// bits [19:0]
// addr bits
#define SOUT_UX19	0x08000000L
#define SOUT_UX59	0x10000000L
#define SOUT_UX29	0x18000000L
#define SOUT_UX39	0x20000000L
#define SOUT_UX49	0x28000000L
#define SOUT_UX129	0x30000000L

// Front Unit control signals
#define SOUT_MAIN	0x04000000L
#define SOUT_SUB	0x02000000L
#define SOUT_PON	0x01000000L
#define SOUT_LOHI	0x00800000L
#define SOUT_BAND	0x00400000L
#define SOUT_PTT	0x00200000L	// band unit PTT
#define	SOUT_NU0	0x00100000L	// not used bit
#define	SOUT_PLLM	0x000fffffL	// mask - 20 bits of PLL data

// addr == 0x7
#define SOUT_ADDR7	0x38000000L	// B-unit addr
#define SOUT_OPT12	0x04000000L	// if "1" MAIN connects to OPT1
								// if "0" MAIN connects to OPT2
#define SOUT_VOL	0x02000000L	// lower 18 data bits active
#define SOUT_SQL	0x01000000L	// lower 18 data bits active
#define SOUT_TONEA	0x00800000L	// lower 8 data bits active
#define SOUT_DSQL	0x00400000L	// if "1", mutes audio
#define SOUT_SEL12	0x00200000L	// if TONEA == 1 && SEL12 == 1, OPT1 is addressed
								// if TONEA == 1 && SEL12 == 0, OPT2 is addressed

// SIN data word defines
#define SIN_START	0x80000000L
#define SIN_ADDR	0x40000000L
#define SIN_STOP	0x0000fff0L	// mask - stop validate bits
//SIN_ADDR = 0
#define SIN_BUSY	0x20000000L	// module selected feedback
#define SIN_SQSA	0x10000000L	// main COS
#define SIN_SQSB	0x08000000L	// sub COS
#define SIN_SRFA	0x07800000L	// mask (main)
#define SIN_SRFB	0x00780000L	// mask (sub)
#define SIN0_DATA	(SIN_BUSY|SIN_SQSA|SIN_SQSB|SIN_SRFA|SIN_SRFB)	// mask
// SIN_ADDR = 1
#define SIN_SEND	0x20000000L	// /PTT active
#define SIN_DSQA	0x10000000L	// "1" = tone detected
#define SIN_DSQB	0x08000000L
#define SIN_MCK		0x04000000L	// MIC u/d button pressed
#define SIN_MUP		0x02000000L	// MIC u/d button state, "1" = UP
#define SIN_SEL11	0x00800000L	// OPT1 detect: 00 = no OPT unit, 10 = UT-28
#define SIN_SEL12	0x00400000L	//	01 = UT-29
#define SIN_SEL21	0x00200000L	// OPT2 detect: same as for OPT1
#define SIN_SEL22	0x00100000L
#define SIN1_DATA	(SIN_SEND|SIN_DSQA|SIN_DSQB|SIN_MCK|SIN_MUP|SIN_SEL11|SIN_SEL12|SIN_SEL21|SIN_SEL22)	// mask

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

U32 init_sio(void);
void send_so(uint32_t data);

U32 get_sin(void);
char got_sin(void);
void gpiof_isr(void);
void Timer2A_ISR(void);

#endif /* SPI_H_ */
