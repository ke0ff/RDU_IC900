/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: sio.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the source file for the IC-900 RDU Clone application
 *  async serial I/O.
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    06-29-21 jmh:  creation date
 *
 *******************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "sio.h"
#include "tiva_init.h"
#include "serial.h"
#include "radio.h"

//-----------------------------------------------------------------------------
// local declarations
//-----------------------------------------------------------------------------

U32	sin_perr;
U8	sin_error;
U32	sin_mask;
U8	sin_hptrm1;
U8	sin_hptr;
U8	sin_tptr;
#define	SIN_MAX	10
U32	sin_buf[SIN_MAX];
U32	sin_dr;
U32 pttm;			// ptt edge mem

//-----------------------------------------------------------------------------
// ***** START OF CODE *****
//-----------------------------------------------------------------------------

/****************
 * init_sio is called at IPL and initializes the data structures for the sio Fns
 */
U32 init_sio(void)
{
	U32				j;				// temp U32
	U32				iplr = 0;
	volatile U32	ui32Loop;

	// init local variables
	sin_error = 0;
	sin_perr = 0;
	sin_mask = 0;
	sin_hptrm1 = 0;
	sin_hptr = 0;
	sin_tptr = 0;
	sin_buf[sin_hptr] = 0;
	pttm = 0xff;

	// init ssi1 (4800 baud, 32b, async serial out)
	// with 32 bits (2, 16bit values written into the FIFO one after the other), the SO bitmap is as follows
	//	BIT#		DESCRIPTION
	//	31			start, always cleared to 0
	//	[30:28]		addr
	//	[27:21]		control
	//	[20:01]		data
	//	00			stop, always set to 1 -- really need 2 stop bits, but this will have to be enforced at the next level up

	// SOUT config
	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;
    ui32Loop = SYSCTL_RCGCSSI_R;
	GPIO_PORTF_AFSEL_R |= SOUT_TTL;										// enable alt fn
	GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M);
	GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF1_SSI1TX);
	SSI1_CR1_R = 0;														// disable SSI before configuring
	// Set bit rate & bit width:
	// CPSDVR = 248, SCR = 41 for 50MHz SYSCLK gives 4800.3 baud
	// BR = SYSCLK/(CPSDVSR * (1 + SCR))
	// SCR = (SYSCLK/(BR * CPSDVSR)) - 1
	// see SSICLK_calc.xls for minimum error values for CPSDVSR and SCR
	SSI1_CPSR_R = SSI1_CPSDVSR;
	SSI1_CR0_R = (SSI1_SCR << 8) | SSI_CR0_DSS_16 | SSI_CR0_SPO | SSI_CR0_FRF_TI;		// bit rate, clock ph/pol, #bits
	SSI1_CC_R = 0;														// SYSCLK is the clk reference for SSI1
	SSI1_CR1_R = SSI_CR1_SSE;											// enable SSI
	// SIN config
	j = GPIO_PORTF_IM_R;												// disable SIN edge intr
	GPIO_PORTF_IM_R = 0x00;
	GPIO_PORTF_IEV_R &= ~SIN_TTL;										// falling edge
	GPIO_PORTF_IBE_R &= ~SIN_TTL;										// one edge
	GPIO_PORTF_IS_R &= ~SIN_TTL;										// edge ints
	GPIO_PORTF_ICR_R = 0xff;											// clear int flags
	j |= (SIN_TTL);														// enable SIN edge intr
	GPIO_PORTF_IM_R = j;
	GPIO_PORTF_IM_R |= (SIN_TTL);										// enable SIN edge intr

	// init Timer2A (Count dn, periodic -- inputs IC-900 async data)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
	ui32Loop = SYSCTL_RCGCGPIO_R;
	TIMER2_CTL_R &= ~(TIMER_CTL_TAEN);									// disable timer
	TIMER2_CFG_R = TIMER_CFG_16_BIT; //0x4; //0;
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
	TIMER2_TAPR_R = TIMER2A_PS;
	TIMER2_TAILR_R = (uint16_t)(SIN_BIT_TIME);
	TIMER2_IMR_R = TIMER_IMR_TATOIM;									// enable timer intr
//	TIMER2_CTL_R |= (TIMER_CTL_TAEN);									// enable timer
	TIMER2_ICR_R = TIMER2_MIS_R;										// clear any flagged ints
	NVIC_EN0_R = NVIC_EN0_TIMER2A;										// enable timer2A intr in the NVIC_EN regs
	NVIC_EN0_R = NVIC_EN0_GPIOF;										// enable GPIOF edge intr in the NVIC_EN regs
	iplr = IPL_ASIOINIT;

	return iplr;
}

//**********************************************************************************************//
//																								//
//	Serial Out via SSI1TX to create a 4800 baud, 32b data path (1 start bit, 1 stop bit)		//
//	Requires an inverter to get it to work right since the TIVA SPI returns to zero between		//
//	transfers.																					//
//																								//
//**********************************************************************************************//

/****************
 * send_so sends 4800 baud ASYNC data using SSI1 tx output
 * 	data is 30 bits, right justified. send_so() adds the start and stop bits, and then stores
 * 	the transfer as two 16 bit values to the SPI data register/FIFO
 */
void send_so(uint32_t data)
{
	uint32_t i2;
	uint16_t ii;
	uint16_t jj;

	i2 = ((data << 1) | 0x0001L) & 0x7fffffffL;				// add start and stop bits
	if(SSI1_SR_R & SSI_SR_BSY){								// wait for previous msg to clear
		wait(!(SSI1_SR_R & SSI_SR_BSY));
//		wait(2);											// add extra time to complete 2nd stop bit
	}
	ii = ~(uint16_t)(i2 >> 16);								// invert & break 32 bit data into 2 16 bit..
	jj = ~(uint16_t)(i2 & 0xffff);							// .. pieces to feed the 16 bit SSI
	SSI1_DR_R = ii;
	SSI1_DR_R = jj;
	return;
}

//**********************************************************************************************//
//																								//
//	Serial IN via PF4 & timer2a to create a 4800 baud, 31b data path (1 start bit, 16 stop bit)	//
//																								//
//**********************************************************************************************//

//-----------------------------------------------------------------------------
// get_sin looks for chr in input buffer.  If none, return '\0'
//	uses circular buffer sin_buf[] which is filled in the TIMER2A interrupt
//-----------------------------------------------------------------------------
U32 get_sin(void){
	U32 c = 0L;

	if(sin_tptr != sin_hptr){						// if head != tail,
		c = sin_buf[sin_tptr++];					// get chr from circ-buff and update tail ptr
		if(sin_tptr >= SIN_MAX){					// process tail wrap-around
			sin_tptr = 0;
		}
	}
	return c;
}

//-----------------------------------------------------------------------------
// got_sin checks if there are any sin chrs in the buffer.
//-----------------------------------------------------------------------------
char got_sin(void){
	char c = FALSE;			// return val, default to "no chr"

    if(sin_tptr != sin_hptr){						// if (head != tail) && there is at least 1 msg..
    	c = TRUE;									// .. set chr ready to get flag
	}
	return c;
}

//-----------------------------------------------------------------------------
// flush_sin empties the input buffer.
//-----------------------------------------------------------------------------
void flush_sin(void){
	U8	i;	// temp

	GPIO_PORTF_IM_R &= ~(SIN_TTL);					// disable SIN edge and timer intr
	TIMER2_CTL_R &= ~(TIMER_CTL_TAEN);
	for(i=0; i<SIN_MAX; i++){						// clear buffer entries
		sin_buf[i] = 0;
	}
	sin_hptr = 0;
	sin_hptrm1 = 0;
	sin_tptr = sin_hptr;							// make tail == head,
	GPIO_PORTF_ICR_R = (SIN_TTL);					// clear int flags
	GPIO_PORTF_IM_R |= (SIN_TTL);					// enable SIN edge intr
	return;
}

//-----------------------------------------------------------------------------
// get_error() returns framing error count
//-----------------------------------------------------------------------------
U32 get_error(void){

	return sin_perr;
}

//-----------------------------------------------------------------------------
// print_ptr() prints t/h ptrs
//-----------------------------------------------------------------------------
void print_ptr(void){
	char dbuf[25];

	sprintf(dbuf,"h%d,t%d", sin_hptr, sin_tptr);
	putsQ(dbuf);
	return;
}

//-----------------------------------------------------------------------------
// gpiof_isr
//-----------------------------------------------------------------------------
//
// GPIO_PORTf isr, processes the SIN start bit detect
//		starts timer2 at half bit time, and inits SIN process flags
//

void gpiof_isr(void){
	if(GPIO_PORTF_MIS_R & (SIN_TTL)){
		GPIO_PORTF_ICR_R = (SIN_TTL);						// clear int flags
		GPIO_PORTF_IM_R &= ~(SIN_TTL);						// disable SIN edge intr
		sin_dr = 0x00000000L;								// init data reg and mask
		sin_mask = SIN_START;
		TIMER2_TAILR_R = (uint16_t)(SIN_HALF_BIT_TIME);
		TIMER2_CTL_R |= (TIMER_CTL_TAEN);					// enable timer
	}else{
		GPIO_PORTF_ICR_R = 0xff;
	}
	return;
}

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// Called when timer2A overflows:
//	This ISR times one bit time at 4800 baud (1/2 bit time for start bit, this
//	aligns the ISR to the center of the bit time).  Bits are shifted in from PF4
//	until the mask == 0, at which point the data is transfered to the input buffer,
//	and the GPIO isr is enabled to arm for the next start bit.
//
//-----------------------------------------------------------------------------

void Timer2A_ISR(void)
{
	U32	i;		// temps
	U32	j;

	if(sin_hptr >= SIN_MAX){									// failsafe countermeasure
		sin_hptr = 0;
		sin_hptrm1 = 0;
		sin_buf[sin_hptr] = 0;
	}
	if(TIMER2_MIS_R & TIMER_MIS_TATOMIS){
		TIMER2_TAILR_R = (uint16_t)(SIN_BIT_TIME);
		if(sin_mask == SIN_START){
			if(GPIO_PORTF_DATA_R & SIN_TTL){
				// framing error if data == 1 here
				TIMER2_CTL_R &= ~(TIMER_CTL_TAEN);				// disable timer
				GPIO_PORTF_ICR_R = (SIN_TTL);					// clear int flags
				GPIO_PORTF_IM_R |= (SIN_TTL);					// enable SIN edge intr
				if(sin_error < 250) sin_error += 1;				// increment framing error count
			}else{
				sin_mask >>= 1;									// signal start of data capture
			}
		}else{
			if(GPIO_PORTF_DATA_R & SIN_TTL){
				sin_dr |= sin_mask;								// capture a 1
			}
			sin_mask >>= 1;
			if(sin_mask == 0x2000L){							// data capture complete
				i = sin_dr | 0x3fffL;							// set data for storage
//				if(sin_buf[sin_hptrm1] != i){					// if new data is different from last word, store it
					if(i&SIN_ADDR){
						pass_ud(i);
						j = i&SIN_SEND;
						sin_buf[sin_hptr++] = i & ~(SIN_SEND|SIN_MUP|SIN_MCK);
						if(j != pttm){
							pttm = j;
							EN_PROC_SOUT;						// Process changes to SOUT data state
							if(j){ //!!!
								GPIO_PORTD_DATA_R &= ~sparePD4;
							}else{
								GPIO_PORTD_DATA_R |= sparePD4;
							}
						}
					}else{
						sin_buf[sin_hptr++] = i;
					}

/*					if((i&(SIN_ADDR|SIN_MCK|SIN_MUP))==(SIN_ADDR|SIN_MCK|SIN_MUP)){
						putsQ("mk+");
					}
					if((i&(SIN_ADDR|SIN_MCK|SIN_MUP))==(SIN_ADDR|SIN_MCK)){
						putsQ("mk-");
					}*/
					if(sin_hptr >= SIN_MAX) sin_hptr = 0;
					if(sin_hptr == sin_tptr) sin_perr++;
//					sin_hptrm1++;
//					if(sin_hptrm1 >= SIN_MAX) sin_hptrm1 = 0;
//				}
				sin_time(SIN_ACTIVITY);							// reset activity timer
				TIMER2_CTL_R &= ~(TIMER_CTL_TAEN);				// disable timer
				GPIO_PORTF_ICR_R = (SIN_TTL);					// clear int flags
				GPIO_PORTF_IM_R |= (SIN_TTL);					// enable SIN edge intr
			}
		}
	}
	TIMER2_ICR_R = TIMERA_MIS_MASK;								// clear all A-intr
	return;
}
