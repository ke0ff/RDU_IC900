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

// ***** START OF CODE ***** //

/****************
 * send_spi3 does bit-bang SPI for port1 <<< place holder for QSPI implementation >>>
 */
uint8_t send_spi3(uint8_t data)
{
#define	SPI_DLY	2

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
	// use bit-bang SSI
	//
	uint8_t	i;
//	uint8_t di = 0;

    wait_reg1(&GPIO_PORTE_DATA_R, BUSY_N, BUSY_WAT);	// wait LCD busy to set
//	while(!(GPIO_PORTE_DATA_R & BUSY_N));	// wait for not busy
//	wait(3);									// delay busy
	for(i=0x80;i;i >>= 1){
		if(i & data) GPIO_PORTD_DATA_R &= ~(MOSI_N);	// set MOSI
		else GPIO_PORTD_DATA_R |= MOSI_N;
		GPIO_PORTD_DATA_R |= SCK;						// clr SCK
		wait2(SPI_DLY);									// delay bit_time/2
		GPIO_PORTD_DATA_R &= ~SCK;						// set SCK
		wait2(SPI_DLY);									// delay bit_time/2
	}
//	wait2(SPI_DLY*2);									// delay bit_time/2
	return 0;
#endif
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
