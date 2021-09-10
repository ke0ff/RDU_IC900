/********************************************************************
 ************ COPYRIGHT (c) 2021 by KE0FF, Taylor, TX   *************
 *
 *  File name: spi.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  spi for RDU LCD controller IC
 *
 *******************************************************************/

//#include <stdint.h>
//#include <string.h>
//#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "version.h"
#include "spi.h"
#include "lcd.h"

// ******************************************************************
// defines

	volatile	U8	ssiflag;			// clock edge trigger (if == 1), set by Timer1B ISR

// declarations
U8 shift_spi(U8 dato);
void open_nvr(void);
void close_nvr(void);

// ******************************************************************
// ***** START OF CODE ***** //
/****************
 * init_spi3 initializes bit-bang QSSI
 */
void init_spi3(void)
{

	ssiflag = 0;									// init volatiles
//	TIMER1_IMR_R |= TIMER_IMR_TBTOIM;				// enable timer intr
	return;
}

/****************
 * send_spi3 does bit-bang SPI for port1 <<< with the addition of the SPI NVRAM, the QSPI implementation is now deprecated -- !! DO NOT USE !! >>>
 */
uint8_t send_spi3(uint8_t data)
{

#if (USE_QSPI == 1)

//	uint8_t di = 0;

    wait_reg1(&GPIO_PORTE_DATA_R, BUSY_N, BUSY_WAT);	// wait for LCD busy to set
	while((SSI3_CR1_R & SSI_CR1_EOT) == 1);				// wait for eot and not busy
//	SSI3_ICR_R = SSI_ICR_EOTIC;							// pre-clear EOT flag
	SSI3_DR_R = ~data;									// invert data to compensate for 74HCT04
//	di = SSI3_DR_R;
//	SSI3_ICR_R = SSI_ICR_EOTIC;							// clear EOT flag
	return 0;

#else
	// use bit-bang SSI (for LCD -- checks for LCD BUSY == 1 before sending)
	//

    wait_reg1(&GPIO_PORTE_DATA_R, BUSY_N, BUSY_WAT);	// wait for LCD busy to set
    shift_spi(~data);									// invert LCD data because of the inverting level shifter driving the LCD chips (NVRAM doesn't get inverted)
	return 0;
#endif
}

//-----------------------------------------------------------------------------
// shift_spi() drives shift engine for bit-bang SSI
//	returns receive data (requires NVRAM CS to be low to receive NVRAM data,
//	else all you get is the lock switch status)
//	Uses ssiflag trigger from Timer1B_ISR to apply the clock period to each edge
//-----------------------------------------------------------------------------
U8 shift_spi(U8 dato){
	U8	i;
	U8	datain = 0;

	ssiflag = 0;
	TIMER1_CTL_R |= (TIMER_CTL_TBEN);					// enable timer
	while(!ssiflag);									// sync sync to timer ISR
	ssiflag = 0;
	for(i=0x80;i;i >>= 1){
		if(i & dato) GPIO_PORTD_DATA_R |= MOSI_N;		// set MOSI
		else GPIO_PORTD_DATA_R &= ~MOSI_N;				// clear MOSI
		GPIO_PORTD_DATA_R |= SCK;						// clr SCK (it is inverted before reaching the slave devices)
		while(!ssiflag);								// delay 1/2 bit time
		ssiflag = 0;
		if(GPIO_PORTB_DATA_R & MISO_LOCK) datain |= i;	// capture MISO == 1
		GPIO_PORTD_DATA_R &= ~SCK;						// set SCK
		while(!ssiflag);								// delay 1/2 bit time
		ssiflag = 0;
	}
	TIMER1_CTL_R &= ~(TIMER_CTL_TBEN);					// disable timer
	return datain;
}

//-----------------------------------------------------------------------------
// put_spi() does puts to the SPI.  The first byte of the string specifies the
//	length, CS, and C/D settings per the bitmap:
//	CS2_MASK	0x80	= CS2
//	CS1_MASK	0x40	= CS1
//	DA_CM_MASK	0x20	= data/cmd
//	LEN_MASK	0x1f	= string length
//-----------------------------------------------------------------------------
int put_spi(const U8 *string, U8 mode){
	U8	i;
	U8	k;

	if(mode & CS_OPEN){
		open_spi((*string) & CS2_MASK);			// activate CS
	}
//	wait2(100);
	lcd_cmd((*string) & DA_CM_MASK);			// set cmd/data
	k = (*string++) & LEN_MASK;
	for(i=0; i<k; i++){							// send data
		send_spi3(*string++);
	}
	if(mode & CS_CLOSE){
		close_spi();							// close CS
	}
	return 0;
}

/****************
 * spi3_clean clears out the rx fifo.
 */
void spi3_clean(void)
{
	volatile uint8_t di = 0;

#if (USE_QSPI == 1)
	while(SSI1_SR_R & SSI_SR_RNE){						// repeat until FIFO is empty
	di = SSI1_DR_R;
	}
#endif
	return;
}

/****************
 * open_spi starts an spi message by lowering CS to the addressed device (1 or 0)
 */
void open_spi(uint8_t addr)
{
	GPIO_PORTE_ICR_R = (BUSY_N);						// pre-clear edge flag
	if(addr){
		GPIO_PORTD_DATA_R |= (CS2);						// open IC1 /CS
	}else{
		GPIO_PORTD_DATA_R |= (CS1);						// open IC2 /CS
	}
	// wait for busy == 0
	wait_busy0(3);										// wait 3m (max) for busy to release
	return;
}

/****************
 * close_spi closes an spi message by raising CS
 */
void close_spi(void)
{
	wait_busy1(3);										// wait 3m (max) for busy to release
//	wait(3);											// delay busy
//    wait_reg0(&GPIO_PORTE_DATA_R, BUSY_N, BUSY_WAT);	// wait up for not busy
	GPIO_PORTD_DATA_R &= ~(CS1 | CS2);					// close all SPI /CS
	return;
}

/****************
 * lcd_cmd activates updates the DA_CM GPIO
 * if cdata != 0, DA_CM = 1; else DA_CM = 0
 */
void lcd_cmd(U8 cdata)
{
	if(cdata){
		GPIO_PORTE_DATA_R |= DA_CM;						// set data
	}else{
		GPIO_PORTE_DATA_R &= ~DA_CM;					// set cmd
	}
	wait(1);
	return;
}

//************************************************************************
// NVRAM support Fns
//************************************************************************

/****************
 * open_nvr starts an spi message by lowering RAMCS_N
 */
void open_nvr(void)
{

	GPIO_PORTD_DATA_R &= ~RAMCS_N;						// open NVRAM
	return;
}

/****************
 * close_nvr closes an spi message by raising RAMCS_N
 */
void close_nvr(void)
{

	GPIO_PORTD_DATA_R |= RAMCS_N;						// close NVRAM
	return;
}

/****************
 * wen_nvr sends write enable cmd to the NVRAM
 */
void wen_nvr(void)
{

	open_nvr();
	shift_spi(WREN);
	close_nvr();
	return;
}

//-----------------------------------------------------------------------------
// Timer1B_ISR() drives bit-bang SPI
//-----------------------------------------------------------------------------
//
// Called when timer 1B overflows (NORM mode):
//	Timer1_B runs as 16 bit reload timer set to 5us interrupt rate
//	This gives a 100KHz SPI clk
//
//-----------------------------------------------------------------------------

void Timer1B_ISR(void)
{

	// set flag to trigger bit
	ssiflag = 1;
	return;
}
