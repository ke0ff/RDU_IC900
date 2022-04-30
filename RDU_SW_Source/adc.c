/********************************************************************
 ************ COPYRIGHT (c) 2022 by ke0ff, Taylor, TX   *************
 *
 *  File name: adc.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  Tiva adc support functions
 *
 *******************************************************************/

#include <stdint.h>
#include <ctype.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "adc.h"

//=============================================================================
// local registers


//=============================================================================
// local Fn declarations


//*****************************************************************************
// adc_init()
//  initializes the processor ADC peripheral
//	returns bitmapped initialize result status as U16
//
//*****************************************************************************
U16 adc_init(void)
{
	volatile uint32_t	ui32Loop;

    // ADC init
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;			// enable ADC clock
	ui32Loop = SYSCTL_RCGCADC_R;
	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;   			// activate ADC0 (legacy code)
	ui32Loop = SYSCTL_RCGC0_R;

	GPIO_PORTD_AFSEL_R |= sparePD2;					// enable alt fn, PD2
	GPIO_PORTD_AMSEL_R |= sparePD2;
	ADC0_CC_R = ADC_CC_CS_PIOSC;					// use PIOSC
	ADC0_PC_R = ADC_PC_SR_125K;						// 125KHz samp rate
	// ADC sequencer init
	ADC0_SSPRI_R = 0x1023;							// Sequencer 2 is highest priority
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN2;				// disable sample sequencer 2
	ADC0_EMUX_R &= ~ADC_EMUX_EM2_M;					// seq2 is software trigger
//	ADC0_SSMUX3_R &= ~0x000F;						// clear SS2 field low nyb
	ADC0_SSMUX2_R = 0x55555555;						// set channels, PD2 = ain5
//	ADC0_SSCTL2_R = (ADC_SSCTL0_TS1|ADC_SSCTL0_IE1|ADC_SSCTL0_END1);
//													// TS0 is 2nd sample, IE0 enabled after 2nd samp
//	ADC0_SSCTL2_R = (ADC_SSCTL0_TS0|ADC_SSCTL0_IE0|ADC_SSCTL0_END0);
	ADC0_SSCTL2_R = (ADC_SSCTL0_TS3|ADC_SSCTL0_TS2|ADC_SSCTL0_TS1|ADC_SSCTL0_IE3|ADC_SSCTL0_END3);
													// do 3 samples of Tj per eratta ADC#09
													// TS is 4th sample, IE enabled after 4th samp
													// ignore 1st 2 TS samples, only use TS3 reading
	ADC0_SAC_R = ADC_SAC_AVG_64X;					// set 64x averaging
	ADC0_IM_R &= ~ADC_IM_MASK2;						// disable SS2 interrupts
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN2;				// enable sample sequencer 2
	return 0;
}

//*****************************************************************************
// adc_in()
//  starts SS2 and waits for it to finish.  Returns ADC results as U16 placed
//	at pointer.  In the array, even offsets are status, odd offsets are data.
//	offset 1 = Light sense, offset 3 & 5 = TJ (ignore), offset 7 = TJ
//	float voltage = rawADC * Vref / maxADC ; Vref = 3.3V, maxADC = 0x1000
//	Vtj = 2.7 - ((TJ + 55) / 75)
//	Vtj = rawADC * 3.3 / 4096
//	TJ = (-75 * ((rawADC * 3.3 / 4096) - 2.7)) - 55
//	TJ = 147.5 - (75 * (rawADC * 3.3 / 4096))
//
//*****************************************************************************
U8 adc_in(U16* p)
{
#define NUM_SAMPS	4		// number of samples in sequence
	U8	i;

	ADC0_PSSI_R = ADC_PSSI_SS2;						// initiate SS2
	while((ADC0_RIS_R & ADC_RIS_INR2) == 0);		// wait for conversion done
	ADC0_ISC_R = ADC_ISC_IN2;						// acknowledge completion (clr flag)
	for(i=0; i<NUM_SAMPS; i++){
		*p++ = ADC0_SSFSTAT2_R & 0xffff;			// get fifo status in 1st buffer word
		*p++ = ADC0_SSFIFO2_R & 0x0fff;				// read result in 2nd buffer word
	}
	return i;
}
