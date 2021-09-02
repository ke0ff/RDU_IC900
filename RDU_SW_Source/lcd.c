/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the LCD driver source file for the IC-900 LCD
 *  			Functions pertaining to the IC-900 Remote Controller LCD
 *  			interface are the focus of this source file.
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    06-19-21 jmh:  creation date
 *
 *******************************************************************/

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"
#include "stdio.h"
#include "serial.h"
#include "spi.h"
#include "cmd_fn.h"
#include "lcd.h"
#include "radio.h"
#include "uxpll.h"
#include "sio.h"
#include "spi.h"

//------------------------------------------------------------------------------
// Define Statements
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Variable Declarations
//-----------------------------------------------------------------------------
U8	lcd_init_1[] = { 0x45, 0x49, CLR_DMEM, CLR_BMEM, DISP_ON, BLINK_FAST };	// CS1 chip init
U8	lcd_init_2[] = { 0x85, 0x49, CLR_DMEM, CLR_BMEM, DISP_ON, BLINK_FAST};	// CS2 chip init
U8	lcd_mfreq_1[] = { 0x43, 0xe9, OR_DMEM, WITH_DECODE };					// set pointer, "5"00 hz = no change, WITH seg decoder
U8	lcd_mfreq_2[] = { 0x41, 0xd4 };											// MDP
U8	lcd_mfreq_3[] = { 0x41, 0xd0 };											// no MDP
U8	lcd_mraw[] = { CS1_MASK | 3, LOAD_PTR | 9, OR_DMEM, WITHOUT_DECODE };	// set pointer, "5"00 hz = no change, WITHOUT seg decoder
U8	lcd_sfreq_1[] = { 0x83, 0xe7, OR_DMEM, WITH_DECODE };					// set pointer, "5"00 hz = no change, WITH seg decoder
U8	lcd_sfreq_2[] = { 0x81, 0xd4 };											// SDP
U8	lcd_sfreq_3[] = { 0x81, 0xd0 };											// no MDP
U8	lcd_sraw[] = { CS2_MASK | 3, LOAD_PTR | 7, OR_DMEM, WITHOUT_DECODE };	// set pointer, "5"00 hz = no change, WITHOUT seg decoder
/*U8	lcd_qv_lsd[5][3] = {													// list of 5 7-seg ordinal patterns
					 { 0, 0, 0 },											// [ordinal][seg addr]
					 { 0, 4, 0 },
					 { 0, 6, 0 },
					 { 0, 7, 0 },
					 { 2, 7, 2 }
};*/
U8	lcd_qv_lsd[5][3] = {													// list of 5 7-seg ordinal patterns
					// [ordinal][seg addr]
					{ 0, 4, 0 },
					{ 0, 6, 0 },
					{ 0, 6, 1 },
					{ 0, 7, 1 },
					{ 1, 7, 1 }
};

U16	tone_list[] = {  670,  719,  744,  770,  797,  825,  854,  885,
					 915,  948,  974, 1000, 1035, 1072, 1109, 1148,
					1188, 1230, 1273, 1318, 1365, 1413, 1462, 1514,
					1567, 1622, 1679, 1738, 1799, 1862, 1928, 2035,
					2107, 2181, 2257, 2336, 2418, 2503 };

#define	LCDMEM_LEN	32
U8	lcd_mem[LCDMEM_LEN];			// LCD segment map (may yet be deprecated, not yet used)
#define	LCD_BUFLEN 23
U8	lcd_buf[LCD_BUFLEN];			// LCD comm message buffer
U8	maddr;							// mhz digit mode composite digit address and mode flags register
U8	vfo_display;					// display update signal.  Is band, MAIN/SUB, to update or'd with 0x80
U8	xmode;							// xmode flags track the adjust modes (vol, squ, tone, offs)
U8	chkmode;						// indicates check mode is in effect
U8	chksqu;							// save reg for check squelch

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------
U8	asc27(char c);
void clear_lcd_buf(void);
void alock(U8 tf);
U32 bin32_bcdp(U32 bin32);
void lamp_test(U8 tf);
void update_lcd(void);
U8 process_MS(U8 cmd);
U8 test_for_cancel(U8 key);
void process_DIAL(U8 focus);
U8 process_MEM(U8 cmd);
U8 process_SET(U8 cmd);
void ats(U8 tf);
void mset_500hz(U8 cmd);
void sset_500hz(U8 cmd);
void mon_500hz(U8 tf);
void son_500hz(U8 tf);
void mblink_500hz(U8 tf);
void sblink_500hz(U8 tf);
void mmute_action(U8* mute_mode, U8* save_vol);
void smute_action(U8* mute_mode, U8* save_vol);

//-----------------------------------------------------------------------------
// init_lcd() initializes lcd resources
//	The LCD chips (uPD7225) use an SPO=1/SPH=1 transfer format (clock idles high,
//	and data changes on clk FE, stable at clk RE).  However, the clock and data
//	are inverted for this design, so we need SPO=0/SPH=1 transfer format.
//
// The max SSI clock rate per the uPD7225 datasheet is about 1.1 MHz.
//-----------------------------------------------------------------------------
void init_lcd(void){
	U8	i;
	volatile uint32_t ui32Loop;
//	U8	test_str[10];

	reset_lcd();														// reset the LCD chipset
#if	(USE_QSPI == 1)
	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R3;
    ui32Loop = SYSCTL_RCGCSSI_R;
	GPIO_PORTD_AFSEL_R |= (SCK | MOSI_N);								// enable alt fn
	GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD0_M | GPIO_PCTL_PD3_M);
	GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_SSI3CLK | GPIO_PCTL_PD3_SSI3TX);
	SSI3_CR1_R = 0;														// disable SSI before configuring
	// Set bit rate & bit width:
	// CPSDVR = 2, SCR = 499 for 50MHz SYSCLK gives 50,000 baud
	// BR = SYSCLK/(CPSDVSR * (1 + SCR))
	// SCR = (SYSCLK/(BR * CPSDVSR)) - 1
	// see SSICLK_calc.xls for minimum error values for CPSDVSR and SCR
	SSI3_CPSR_R = SSI3_CPSDVSR;
	SSI3_CR0_R = (SSI3_SCR << 8) | SSI_CR0_DSS_8 | SSI_CR0_SPH | SSI_CR0_FRF_MOTO;	// bit rate, clock ph/pol, #bits
//	SSI3_CR0_R = (SSI3_SCR << 8) | SSI_CR0_DSS_8 | SSI_CR0_SPO | SSI_CR0_SPH | SSI_CR0_FRF_MOTO;	// bit rate, clock ph/pol, #bits
//	SSI3_CR0_R = (SSI3_SCR << 8) | SSI_CR0_DSS_8 | SSI_CR0_FRF_MOTO;	// bit rate, clock ph/pol, #bits
	SSI3_CC_R = 0;														// SYSCLK is the clk reference for SSI1
	SSI3_CR1_R = SSI_CR1_SSE;											// enable SSI
#endif
	wait(3);
	put_spi(lcd_init_1, CS_OPENCLOSE);									// send init and display mem clear cmds
	wait(2);
	put_spi(lcd_init_2, CS_OPENCLOSE);
	wait(2);
	for(i=0; i<LCDMEM_LEN; i++){
		lcd_mem[i] = 0;
	}
	mhz_time(0xff);														// clear MHZ timer

/*	lamp_test(1);
	wait(1000);
	lamp_test(0);
	reset_lcd();														// reset the LCD chipset
	wait2(20);
	put_spi(lcd_init_1, CS_OPENCLOSE);									// send init and display mem clear cmds
	wait2(20);
	put_spi(lcd_init_2, CS_OPENCLOSE);
	wait2(20);*/
	return;
}

/****************
 * reset_lcd activates (1) or clears (0) LCD reset
 */
void reset_lcd(void)
{
	wait(3);
	GPIO_PORTD_DATA_R &= ~(CS1 | CS2);									// close all SPI /CS
	wait(3);
	GPIO_PORTB_DATA_R |= LCDRST;										// activate reset
	wait(20);
	GPIO_PORTB_DATA_R &= ~LCDRST;										// de-activate reset
	wait(20);
	return;
}

//-----------------------------------------------------------------------------
// process_UI() updates LCD based on SIN change flags
//	processes key inputs, dial changes, and CCMD inputs
//-----------------------------------------------------------------------------
void process_UI(U8 cmd){
	U8	i;					// temp
	U8	mode_rtn;
	static U8	sw_stat;	// sliding switches memory reg
	static U8	mode;		// mode

	if(cmd == 0xff){													// IPL (Initial Program Load) init branch
		sw_stat = ~GPIO_PORTB_DATA_R & (DIM | LOCK);					// force update of slide switch status
		mode = MAIN_MODE;												// init process variables
		chkmode = 0;
		vfo_display = 0;
		update_lcd();
		process_MS(cmd);												// trigger main/sub process IPL init
	}else{
		// process the UI for each of the different modes
		switch(mode){
		default:
			mode = MAIN_MODE;											// fault trap... force mode to MAIN_MODE
		// MAIN/SUB "normal" process
		case MAIN_MODE:
		case SUB_MODE:
 			mode_rtn = process_MS(mode);
			break;

		// SET configuration loop process
		case SET_MODE:
//			mode_rtn = process_SET(mode);
			break;

		// Memory mode process
		case MEM_MODE:
//			mode_rtn = process_MEM(mode);
			break;
		}
		// update the display when there are mode changes
		if(mode_rtn != mode){
			switch(mode_rtn){
			case MAIN_MODE:
//				init_lcd_main();
				break;

			case SUB_MODE:
//				init_lcd_sub();
				break;

			case SET_MODE:
//				init_lcd_set();
				break;

			case MEM_MODE:
//				init_lcd_mem();
				break;
			}
			mode = mode_rtn;
		}
		// process slide switches (DIM and LOCK)
		i = GPIO_PORTB_DATA_R & (DIM | LOCK);							// capture DIM/LOCK switch settings
		if(i ^ sw_stat){												// if changes..
			if(i & DIM){
		        set_pwm(5, 90);											// process bright settings (!!! these need to be configurable in SET loop)
		        set_pwm(6, 90);
			}else{
		        set_pwm(5, 30);											// process DIM settings (!!! these need to be configurable in SET loop)
		        set_pwm(6, 30);
			}
			if(i & LOCK){
				alock(1);												// process lock mode
			}else{
				alock(0);												// process unlock mode
			}
			sw_stat = (sw_stat & ~(DIM | LOCK)) | i;					// update GPIO memory (used to trap changes)
		}
	}
}

//-----------------------------------------------------------------------------
// update_lcd() forces update of LCD values
//-----------------------------------------------------------------------------
void update_lcd(void){
	U8	i;

	i = get_lohi(MAIN, 0xff);											// update low/hi power display
	if(i){
		alow(1);
	}else{
		alow(0);
	}
	switch(read_dplx(MAIN) & (DPLX_MASK)){								// update Duplex display, main
	default:
	case DPLX_S:
		mdupa('S');
		break;

	case DPLX_P:
		mdupa('+');
		break;

	case DPLX_M:
		mdupa('-');
		break;
	}
	switch(read_dplx(SUB) & (DPLX_MASK)){								// update Duplex display, sub
	default:
	case DPLX_S:
		sdupa('S');
		break;

	case DPLX_P:
		sdupa('+');
		break;

	case DPLX_M:
		sdupa('-');
		break;
	}
	ats(read_dplx(MAIN) & TSA_F);										// TS (f-step) display
	mtonea(adjust_toneon(MAIN, 0xff));									// TONE display
	stonea(adjust_toneon(SUB, 0xff));
	msmet(0xff, 0);														// init SRF/MEM
	ssmet(0xff, 0);														// init SRF/MEM
	mfreq(get_freq(MAIN), 0, 0);										// Frequency display
	sfreq(get_freq(SUB), 0, 0);
	// update radios
	update_radio_all();													// trigger update of radios
	return;
}

//-----------------------------------------------------------------------------
// process_MS() MAIN/SUB mode: updates LCD based on SIN change flags
//	processes key inputs, dial changes, and CCMD inputs
//	returns mode changes
//-----------------------------------------------------------------------------
U8 process_MS(U8 mode){
	static	 U32	iflags;
	U8	i;					// temp
	U32	ii;
	static   U8		subvol;
	static   U8		mainvol;
	static   U8		smute_mode;
	volatile U32	sin_a0;
	volatile U32	sin_a1;
	U8	band_focus = mode;	// band focus of keys/dial
	S32	sii;
	char tbuf[8];			// ascii freq field buffer

	char dgbuf[30];	// !!!!debug

	if(mode == 0xff){													// IPL init
		maddr = MHZ_OFF;												// MHz/thumbwheel mode init
		xmode = 0;														// x-modes
		smute_mode = 0;													// smute = off
		iflags = 0;														// SIN change flags storage init
		subvol = 0;														// smute temp
		mainvol = 0;													// smute temp
		vfo_display = MAIN|SUB_D;										// force update of main/sub freq
	}else{
		// process SIN changes
		iflags |= read_sin_flags(0);									// merge with radio.c variable
		if(iflags){
			// got changes...
			sin_a0 = fetch_sin(0);										// update data
			// check if squelch adjust
			if(!(xmode & SQU_XFLAG)){
				if(iflags & SIN_MSRF_F){
					// update SRF
					ii = sin_a0 >> 23;									// isolate main and sub SRF
					i = (U8)(ii & 0x0f);
					msmet(i>>1, 0);										// update glass
					read_sin_flags(SIN_MSRF_F);							// clear changes flag
				}
			}
			// check if vol adjust
			if(!(xmode & VOL_XFLAG)){
				if(iflags & SIN_SSRF_F){
					// update SRF
					ii = sin_a0 >> 19;									// isolate main and sub SRF
					i = (U8)(ii & 0x0f);
					ssmet(i>>1, 0);										// update glass
					read_sin_flags(SIN_SSRF_F);							// clear changes flag
				}
			}
			if(iflags & SIN_SQS_F){										// LED updates (MRX, MTX, SRX)
				// update RX LEDs
				if(sin_a0 & SIN_SQSA){
					// main led
					GPIO_PORTC_DATA_R |= MRX_N;
				}else{
					GPIO_PORTC_DATA_R &= ~MRX_N;
				}
				if(sin_a0 & SIN_SQSB){
					// sub led
					GPIO_PORTC_DATA_R |= SRX_N;
				}else{
					GPIO_PORTC_DATA_R &= ~SRX_N;
				}
				read_sin_flags(SIN_SQS_F);								// clear changes flag
			}
/*			if(iflags & SIN_SEND_F){
				sin_a1 = fetch_sin(1);
				// update TX LED
				if(sin_a1 & SIN_SEND){
					// main led
					GPIO_PORTD_DATA_R |= MTX_N;
				}else{
					GPIO_PORTD_DATA_R &= ~MTX_N;
				}
				read_sin_flags(SIN_SEND_F);								// clear changes flag
			}*/
		}
		// process vfo display update signal
		if(vfo_display){
			if(xmode & (TONE_XFLAG|OFFS_XFLAG)){
				if(xmode & TONE_XFLAG){
					// TONE adjust mode
					if(vfo_display & VMODE_TDISP){						// if triggered
						i = adjust_tone(band_focus, 0) & CTCSS_MASK;
						if((i > TONE_MAX) || (i == 0)){
							i = 1;										// error trap
						}
						ii = tone_list[i - 1];							// tone list is "0" origin, tones values are "1" origin
						sprintf(tbuf,"%4d  ", ii);						// convert number to display string
						if(band_focus == MAIN){
							mputs_lcd(tbuf, 1);
						}else{
							sputs_lcd(tbuf, 1);
						}
						vfo_display &= ~VMODE_TDISP;					// clear update trigger
						force_push();									// force update to NVRAM
					}
				}else{
					// offset adjust mode

				}
			}else{
				// normal VFO display
				sprintf(dgbuf,"vfodisp: %02x",vfo_display); //!!!
				putsQ(dgbuf);
				if(vfo_display & MAIN){
					if(vfo_display & VMODE_ISTX){
						mfreq(get_freq(MAIN | VMODE_ISTX), 0, 0);		// update main freq display from vfotr
						sprintf(dgbuf,"vfofrqT: %d",get_freq(band_focus | 0x80)); //!!!
						putsQ(dgbuf);
					}else{
						mfreq(get_freq(MAIN), 0, 0);					// update main freq display
						sprintf(dgbuf,"vfofrqR: %d",get_freq(band_focus)); //!!!
						putsQ(dgbuf);
					}
					vfo_display &= ~(VMODE_ISTX | MAIN);
				}else{
					if(vfo_display & SUB_D){
						sfreq(get_freq(SUB), 0, 0);						// update sub freq display
						vfo_display &= ~(SUB_D);
					}
				}
			}
//			vfo_display = 0;
		}
		// process dial
		process_DIAL(band_focus);										// process dial and mic up/dn changes
		// process x-flag timeouts
		if((xmode & VOL_XFLAG) && (!v_time(0))){						// if vol x-mode timeout
			xmode &= ~VOL_XFLAG;
			iflags |= SIN_SSRF_F;										// update sub SRFs
			ssmet(0xff, 0);												// update glass
		}
		if((xmode & SQU_XFLAG) && (!q_time(0))){						// if vol x-mode timeout
			xmode &= ~SQU_XFLAG;
			iflags |= SIN_MSRF_F;										// update main SRFs
			msmet(0xff, 0);												// update glass
		}
		if((xmode & OFFS_XFLAG) && (!offs_time(0))){					// if offs x-mode timeout
			if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0);
			else digblink(maddr,0);
			mhz_time(0xff);												// clear timers
			offs_time(0xff);
			maddr = MHZ_OFF;
			xmode &= ~OFFS_XFLAG;
			if(band_focus == MAIN) vfo_display |= MAIN;					// update main VFO
			else vfo_display |= SUB_D;									// update sub VFO
			force_push();												// force update to NVRAM
		}
/*		if((xmode & OFFS_XFLAG) && (vfo_display & VMODE_ISTX)){	// if offs+tx force x-mode timeout
			xmode &= ~OFFS_XFLAG;								// clear xmode
			offs_time(0xff);									// clear timer
		}*/
		// process keys
		if(got_key()){													// only run through this branch if there are keys to input
			i = get_key();												// pull key in from buffer space
			i = test_for_cancel(i);										// check to see if keycode qualifies for "cancel" (substitutes "cancel" key in place of the actual key)
			switch(i){													// dispatch to key-specific code segments...

			case DUPchr:												// duplex button, initial press
				if(band_focus == MAIN_MODE){
					switch(inc_dplx(MAIN) & (DPLX_MASK)){				// advance the duplex (function returns changed status)
					default:
					case DPLX_S:
						mdupa('S');
						break;

					case DPLX_P:
						mdupa('+');
						break;

					case DPLX_M:
						mdupa('-');
						break;
					}
				}else{
					switch(inc_dplx(SUB) & (DPLX_MASK)){
					default:
					case DPLX_S:
						sdupa('S');
						break;

					case DPLX_P:
						sdupa('+');
						break;

					case DPLX_M:
						sdupa('-');
						break;
					}
				}
				force_push();											// force update to NVRAM
				break;

			case HILOchr:												// HILO button, initial press
				i = get_lohi(band_focus, 0xff);
				if(i){
					i = 0;
					alow(0);
				}else{
					i = 1;
					alow(1);
				}
				get_lohi(band_focus, i);
				if(band_focus == MAIN){
					add_vfo(band_focus, 0, MHZ_ONE);
				}
				force_push();											// force update to NVRAM
				break;

			case SUBchr:												// SUB button, initial press
				if(band_focus == MAIN_MODE){
					band_focus = SUB_MODE;
					asub(1);
					ats(read_dplx(SUB) & TSA_F);						// TS flag updates based on focus
				}else{
					band_focus = MAIN_MODE;
					asub(0);
					ats(read_dplx(MAIN) & TSA_F);
				}
				i = get_lohi(band_focus, 0xff);
				if(i){													// HILO updates based on focus
					alow(1);
				}else{
					alow(0);
				}
				break;

			case MHZchr:												// MHZ button 1st press
				if(!mhz_time(0)){										// if timer is not zero, one of the MHz modes is active
					if(maddr == MHZ_OFF){								// this means that the thumbwheel mode isn't active and MHz mode is off
						maddr = MHZ_ONE;								// set MHz mode
						amhz(1);										// turn on mhz icon
					}else{
						maddr = MHZ_OFF;								// if any other MHz mode active, turn it off
						amhz(0);										// turn off mhz icon
					}
				}else{
					sii = 1;
					for(i=0; i<maddr ; i++){							// construct the multiplier for the currently selected digit
						sii *= 10;
					}
					if(maddr == 0) sii = 5;								// lowest digit can only be 0 or 5 (these are all 5KHz stepped radios, except for the UX129 which is a 10 KHz step)
					set_mhz_step(sii);									// store the step mulitplier
//					else set_mhz_step(sii / 10L);
//					i = set_mhz_addr(0xff);
					if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,0); // un-blink the old digit (m/s)
					else digblink(maddr,0);
					if(--maddr == 0xff){								// move the digit and process roll-under
						if(get_band_index(band_focus) == ID1200_IDX){
							maddr = 5;
							set_mhz_step(100000L);
						}else{
							maddr = 4;
							set_mhz_step(10000L);
						}
					}
					if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,1); // blink the new digit (m/s)
					else digblink(maddr,1);
				}
				break;

			case MHZchr_H:												// MHZ button, hold (this enters thumbwheel mode)
				amhz(0);												// turn off mhz icon
//				set_mhz_addr(0);
				if(!mhz_time(0)){										// not in thumbwheel mode
					mhz_time(1);										// start timer
					set_mhz_step(100000L);								// set start step
					if(get_band_index(band_focus) == ID1200_IDX){
						maddr = 5;
					}else{
						maddr = 4;
					}
					if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,1); // blink the 1st digit (m/s)
					else digblink(maddr,1);
					temp_vfo(band_focus);								// copy vfo -> vfot
				}else{													// already in thumbwheel mode (this will cancel the thumbwheel mode)
//					i = set_mhz_addr(0xff);
					if(xmode & OFFS_XFLAG){								// offset is differentiated from VFO frequency
						if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0); // blkin digit off
						else digblink(maddr&(~MHZ_OFFS),0);
						mhz_time(0xff);									// clear timers
						offs_time(0xff);
						maddr = MHZ_OFF;								// turn off thumbwheel mode
					}else{
						if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0);
						else digblink(maddr&(~MHZ_OFFS),0);
						copy_vfot(band_focus);							// copy updated temp vfo to normal vfo
						set_mhz_step(5L);
						mhz_time(0xff);									// clear timers
						offs_time(0xff);
						maddr = MHZ_OFF;
					}
					force_push();										// force update to NVRAM
				}
				break;

			case Vupchr:												// volume increase, initial press... VOL/SQU commandeer the SRF meters and mem ch digit to display level graphics
				v_time(1);
				xmode |= VOL_XFLAG;
				adjust_vol(band_focus, 1);
				ssmet(0x80 | adjust_vol(band_focus, 0), 0);				// signal a volume update
				force_push();											// force update to NVRAM
				break;

			case Vdnchr_H:												// VOL-, hold mutes the audio
				adjust_vol(band_focus, 0x81);							// set vol = 1 (then to 0 below...)
			case Vupchr_H:												// VOL+ hold displays the current level without change
			case Vdnchr:												// VOL-, initial press decreases the VOL level 1 step
				v_time(1);
				xmode |= VOL_XFLAG;
				adjust_vol(band_focus, -1);
				ssmet(0x80 | adjust_vol(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case Qupchr_H:												// SQU+, hold sets max squelch
				adjust_squ(band_focus, 0x80 | (LEVEL_MAX - 1));			// set squ = max-1 (then to max below...)
			case Qdnchr_H:												// SQU- hold displays the current level without change
			case Qupchr:												// SQU+, initial press increases the SQU level 1 step
				q_time(1);
				xmode |= SQU_XFLAG;
				adjust_squ(band_focus, 1);
				msmet(0x80 | adjust_squ(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case Qdnchr:												// SQU-, initial press decreases the SQU level 1 step
				q_time(1);
				xmode |= SQU_XFLAG;
				adjust_squ(band_focus, -1);
				msmet(0x80 | adjust_squ(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case SMUTEchr_H:											// SMUTE hold press toggles mute of main-band audio
				mmute_action(&smute_mode, &mainvol);
				break;

			case SMUTEchr:												// SMUTE initial press toggles mute of sub-band audio (unmutes main if previously muted)
				if(smute_mode & MS_MUTE) mmute_action(&smute_mode, &mainvol);
				smute_action(&smute_mode, &subvol);
				break;

			case TONEchr:												// TONE, initial: toggle tone on/off or cancel adj mode (uses xmode to display tone freq in VFO space)
				if(xmode & TONE_XFLAG){									// tone mode already on, cancel it
					if(band_focus == MAIN){
						vfo_display |= MAIN;							// update main VFO
					}else{
						vfo_display |= SUB_D;							// update sub VFO
					}
					xmode &= ~TONE_XFLAG;
				}else{
					i = adjust_toneon(band_focus, 0xff);				// tone mode off, turn it on...
					if(i) i = 0;
					else i = 1;
					adjust_toneon(band_focus, i);
					if(band_focus == MAIN){								// display tone freq
						mtonea(i);
					}else{
						stonea(i);
					}
					force_push();										// force update to NVRAM
				}
				break;

			case TONEchr_H:												// TONE hold, adjust tone
				vfo_display |= VMODE_TDISP;								// force tone freq to display
				xmode |= TONE_XFLAG;									// enable tone xmode
				i = adjust_toneon(band_focus, 0xff);					// undo tone toggle that got us here...
				if(i) i = 0;
				else i = 1;
				adjust_toneon(band_focus, i);
				if(band_focus == MAIN){
					mtonea(i);
				}else{
					stonea(i);
				}
				force_push();											// force update to NVRAM
				break;

			case TSchr:													// TS, initial press: freq step mode toggle
				i = read_dplx(band_focus);
				if(i & TSA_F){
					set_ab(band_focus, 0);
					ats(0);
				}else{
					set_ab(band_focus, 1);
					ats(1);
				}
				force_push();											// force update to NVRAM
				break;

			case DUPchr_H:												// duplex button HOLD (offset adjust)
				// undo the duplex press
				if(band_focus == MAIN_MODE){
					inc_dplx(MAIN);
					inc_dplx(MAIN);
					mfreq((U32)get_offs(band_focus), LEAD0_BLINK, 0);
				}else{
					inc_dplx(SUB);
					inc_dplx(SUB);
					sfreq((U32)get_offs(band_focus), LEAD0_BLINK, 0);
				}
				mhz_time(1);
				offs_time(1);
				set_mhz_step(100000L);
				if(get_band_index(band_focus) == ID1200_IDX){
					maddr = MHZ_OFFS|5;
				}else{
					maddr = MHZ_OFFS|4;
				}
				if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),1);
				else digblink(maddr&(~MHZ_OFFS),1);
				xmode |= OFFS_XFLAG;
//				update_lcd();
				break;

			case VFOchr:												// VFO, initial press: cycles selected band modules (this is the BAND button on the IC-901)
				set_next_band(band_focus);
				update_lcd();
				force_push();											// force update to NVRAM
				break;

			case MSchr:
				set_swap_band();
				update_lcd();
				force_push();											// force update to NVRAM
				break;

			case CHKchr:												// CHECK, initial press: this is the reverse mode (no squ change)
				if(!chkmode){
					// if chkmode == 0, then initiate chk/rev
					rev_vfo(band_focus);
					chkmode = REV_FLAG;
					if(band_focus == MAIN) mset_500hz(2);				// blink chk/rev flasher
					else sset_500hz(2);
				}else{
					// if chkmode == 1, then cancel chk/rev
					rev_vfo(band_focus);
					chkmode = 0;
					if(band_focus == MAIN) mset_500hz(0);				// un-blink chk/rev flasher
					else sset_500hz(0);
				}
				break;

			case CHKchr_H:												// CHECK hold: this opens the SQU
				// if chkmode active, this is the hold from that initiation, so go to SQU open state (else, ignore)
				if(chkmode){
					chkmode |= REV_SQU_FLAG;
					// !!! open squelch
					chksqu = adjust_squ(band_focus, 0);
					adjust_squ(band_focus, 0x80);						// open up squ (set to zero)
				}
				break;

			case CHKchr_R:												// CHECK, release: closes out chk/rev mode
				if(chkmode & REV_SQU_FLAG){
					// if squ mode: cancel check/rev
					adjust_squ(band_focus, chksqu | 0x80);				// restore squelch
					rev_vfo(band_focus);								// return VFO to normal
					chkmode = 0;										// clear check flag
					if(band_focus == MAIN) mset_500hz(0);				// un-blink chk/rev flasher
					else sset_500hz(0);
				}else{
					// if chkmode (not squ) && duplex == S: cancel check/rev
					if(chkmode && ((read_dplx(band_focus) & DPLX_MASK) == DPLX_S)){
						rev_vfo(band_focus);
						chkmode = 0;
						if(band_focus == MAIN) mset_500hz(0);			// if simplex, cancel reverse mode
						else sset_500hz(0);
					}
				}
				break;
			}
		}
	}
	return band_focus;
}

//-----------------------------------------------------------------------------
// test_for_cancel() looks for cancel keys based on mode registers
//	called from top of key dispatch tree
//-----------------------------------------------------------------------------
U8 test_for_cancel(U8 key){
	U8	i = key;		// temp

	if((maddr & ~MHZ_OFFS) < MHZ_OFF){					//	force the MHZ-hold key-code (this clears by-digit mode)
		switch(key){									// process thumbwheel key-cancel
		case Vupchr:									// don't cancel on these codes:
		case Vdnchr:									// VOLu/d, SQUu/d, MHz, DUP(release)
		case Vupchr_H:
		case Vdnchr_H:
		case Vdnchr_R:
		case Vupchr_R:
		case Qupchr:
		case Qdnchr:
		case Qupchr_H:
		case Qdnchr_H:
		case Qupchr_R:
		case Qdnchr_R:
		case MHZchr:
		case MHZchr_H:
		case MHZchr_R:
		case DUPchr_R:
			break;

		default:										// for all other keys, modify if abort MHz digit mode active to
			i = MHZchr_H;
			break;
		}
	}else{
		if(chkmode){									// for check mode, CHKchr is cancel
			switch(key){								// process thumbwheel key-cancel
			case Vupchr:								// don't cancel on these codes:
			case Vdnchr:								// VOLu/d, SQUu/d, CHECK
			case Vupchr_H:
			case Vdnchr_H:
			case Vdnchr_R:
			case Vupchr_R:
			case Qupchr:
			case Qdnchr:
			case Qupchr_H:
			case Qdnchr_H:
			case Qupchr_R:
			case Qdnchr_R:
			case CHKchr:
			case CHKchr_H:
			case CHKchr_R:
				break;

			default:									// for all other keys, modify if abort MHz digit mode active to
				i = CHKchr;
				break;
			}
		}
	}
	return i;
}

//-----------------------------------------------------------------------------
// smute_action() mutes the sub-band
// mmute_action() mutes the main-band
//-----------------------------------------------------------------------------
void smute_action(U8* mute_mode, U8* save_vol){
	U8	i;		// temp

	if(*mute_mode & SUB_MUTE){
		*mute_mode &= ~SUB_MUTE;
		adjust_vol(SUB, *save_vol | 0x80);				// reset vol
		// unblink sub VFO
		lcd_buf[0] = (CS2_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | S0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<23; i++){
			lcd_buf[i] = WR_BMEM | 0x00;				// clear blink bits
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}else{
		*mute_mode |= SUB_MUTE;
		*save_vol = adjust_vol(SUB, 0);					// save vol
		adjust_vol(SUB, 0x80);							// mute vol
		// blink sub VFO
		lcd_buf[0] = (CS2_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | S0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<22; i++){
			lcd_buf[i] = WR_BMEM | 0x07;				// set blink bits
		}
		lcd_buf[i] = WR_BMEM | 0x05;					// set 1G/DP blink bits
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}
	return;
}

void mmute_action(U8* mute_mode, U8* save_vol){
	U8	i;		// temp

	if(*mute_mode & MS_MUTE){
		*mute_mode &= ~MS_MUTE;
		adjust_vol(MAIN, *save_vol | 0x80);				// reset vol
		// unblink sub VFO
		lcd_buf[0] = (CS1_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | M0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<23; i++){
			lcd_buf[i] = WR_BMEM | 0x00;				// clear blink bits
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}else{
		*mute_mode |= MS_MUTE;
		*save_vol = adjust_vol(MAIN, 0);				// save vol
		adjust_vol(MAIN, 0x80);							// mute vol
		// blink sub VFO
		lcd_buf[0] = (CS1_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | M0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<22; i++){
			lcd_buf[i] = WR_BMEM | 0x07;				// set blink bits
		}
		lcd_buf[i] = WR_BMEM | 0x05;					// set 1G/DP blink bits
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}
	return;
}

//-----------------------------------------------------------------------------
// process_DIAL() handles dial changes
//-----------------------------------------------------------------------------
void process_DIAL(U8 focus){
	U8	i;					// temp
	U8	j;
	U8	k;

	// process mhz digit timeout
	if(!mhz_time(0) && (maddr < MHZ_OFF)){
		if(focus == MAIN_MODE) digblink(MAIN_CS|maddr,0);
		else digblink(maddr,0);
		maddr = MHZ_OFF;
		copy_vfot(focus);
		vfo_change(focus);
	}
	// process dial
	j = is_mic_updn() + get_dial(1);
	if(xmode & TONE_XFLAG){
		if(j){
			adjust_tone(focus, j);
			vfo_display |= VMODE_TDISP;
		}
	}else{
		if(j){
			if((maddr == MHZ_OFF) || (maddr == MHZ_ONE)){
				i = add_vfo(focus, j, maddr);			// update vfo
				if(i && (focus == MAIN)){
					mfreq(get_freq(MAIN), 0, 0);		// update display
				}
				if(i && (focus == SUB)){
					sfreq(get_freq(SUB), 0, 0);
				}
				vfo_change(focus);
			}else{										// digit-by-digit (thumbwheel) mode
				k = maddr & (~MHZ_OFFS);				// mask offset mode flag
				if(k == 0) j = (j & 0x01) * 5;			// pick the odd 5KHz
				i = add_vfo(focus, j, maddr);			// update vfo
				if(mhz_time(0)){
					mhz_time(1);						// reset timer
					if(maddr & MHZ_OFFS){
						offs_time(1);
					}
				}
				else{
					mhz_time(0xff);						// exit MHz mode
					offs_time(0xff);
				}
				if(i && (focus == MAIN)){
					mfreq(get_vfot(), LEAD0_BLINK, 0);	// update main display
				}
				if(i && (focus == SUB)){
					sfreq(get_vfot(), LEAD0_BLINK, 0);	// update sub display
				}
				vfo_change(focus);
			}
		}
	}
	return;
}

//-----------------------------------------------------------------------------
// digfreq() sets the frequency by digit addr.
//	if tr = TRUE, turn on blink
//	if tf = FALSE, turn off blink
//
//	digaddr is a homogenized address index:
//	1234.567
//	 ^     ^-- addr 0 (main addr 0x0a, sub addr 0x08)
//	 |-------- addr 5 (main addr 0x19, sub addr 0x17)
//
//	hi-bit set = main band
//-----------------------------------------------------------------------------
void digblink(U8 digaddr, U8 tf){
	U8	daddr = (digaddr & 0x3f) * 3;		// convert to LCD index

	if(digaddr & MAIN_CS){
		daddr += 0x0a;									// set main addr
		lcd_buf[0] = (CS1_MASK);						// set data string preamble
	}else{
		daddr += 0x08;									// sub addr
		lcd_buf[0] = (CS2_MASK);						// set data string preamble
	}
	lcd_buf[1] = LOAD_PTR | daddr;						// set lcd pointer
	lcd_buf[2] = WITHOUT_DECODE;						// set lcd pointer
	if(tf){
		lcd_buf[3] = WR_BMEM | 0x03;					// set blink bits
		lcd_buf[4] = WR_BMEM | 0x07;
		lcd_buf[5] = WR_BMEM | 0x03;
		lcd_buf[0] |= 5;								// set length
	}else{
		lcd_buf[3] = WR_BMEM;							// clear blink bits
		lcd_buf[4] = WR_BMEM;
		lcd_buf[5] = WR_BMEM;
		lcd_buf[0] |= 5;								// set length
	}
	put_spi(lcd_buf, CS_OPENCLOSE);						// send string to set display address and decode on
	return;
}

//-----------------------------------------------------------------------------
// mfreq() sets the main frequency.  binfreq = binary KHz
//	set blink to display leading zeros
//	!!! maybe test for (maddr & MHZ_OFFS) to supress leading zeros ???
//-----------------------------------------------------------------------------
void mfreq(U32 binfreq, U8 blink, U8 off_err){
	U8	i;
	U8	k = 0;
	U32	ii;
	U32	jj;

	if(off_err){										// trap "off" and "off-line" states
		if(off_err == NO_UX_PRSNT){
			// disp "OFF"
			mputs_lcd("-OFF-", 0);
		}
		if(off_err == NO_B_PRSNT){
			// disp "----" (off-line)
			mputs_lcd("------", 0);
		}
	}else{
		ii = bin32_bcdp(binfreq);						// convert binary to BCD
		// suppress leading zeros (if blink == 0)
		if(!(blink == LEAD0_BLINK)){					// if no leading zeros, skip this
			jj = 0x0f000000L;							// start at GHz
			do{
				if((ii & jj) == 0){						// suppress leading zeros (set nybble to 0xf) until non-zero digit
					ii |= jj;
					jj >>= 4;
				}else{
					k = 1;								// set exit trap
				}
			}while(!k && jj);
		}
		for(i=1; i<7; i++){								// place BCD data into LCD tx buffer, lsdigit first
			lcd_buf[i] = (U8)(ii & 0xfL);
			ii >>= 4;
		}
		lcd_buf[0] = (CS1_MASK|DA_CM_MASK) | 6;			// set data string preamble
		put_spi(lcd_mfreq_1, CS_OPEN);					// send string to set display address and decode on
		put_spi(lcd_buf, CS_IDLE);						// send BCD data
		put_spi(lcd_mfreq_2, CS_CLOSE);					// set DP & close transaction
		//set/clear "1" GHz digit
		lcd_buf[0] = CS1_MASK | 2;						// isolate GHz and enable or disable 1 GHz digit
		lcd_buf[1] = LOAD_PTR | 0x1c;
		if(ii == 0x1L) lcd_buf[2] = WR_DMEM | 0x05;
		else lcd_buf[2] = WR_DMEM | 0x04;
		put_spi(lcd_buf, CS_OPENCLOSE);
	}
	return;
}

//-----------------------------------------------------------------------------
// sfreq() sets the main frequency.  dfreq = BCD KHz
//-----------------------------------------------------------------------------
void sfreq(U32 binfreq, U8 blink, U8 off_err){
	U8	i;
	U8	k = 0;
	U32	ii;
	U32	jj;

	if(off_err){										// trap "off" and "off-line" states
		if(off_err == NO_UX_PRSNT){
			// disp "OFF"
			sputs_lcd("-OFF-", 0);
		}
		if(off_err == NO_B_PRSNT){
			// disp "----" (off-line)
			sputs_lcd("------", 0);
		}
	}else{
		// supress leading zeros
		ii = bin32_bcdp(binfreq);						// convert binary to BCD
		// suppress leading zeros (if blink == 0)
		if(!(blink == LEAD0_BLINK)){					// if no leading zeros, skip this
			jj = 0x0f000000L;							// start at GHz
			do{
				if((ii & jj) == 0){						// suppress leading zeros (set nybble to 0xf) until non-zero digit
					ii |= jj;
					jj >>= 4;
				}else{
					k = 1;								// set exit trap
				}
			}while(!k && jj);
		}
		for(i=1; i<7; i++){								// place BCD data into LCD tx buffer, lsdigit first
			lcd_buf[i] = (U8)(ii & 0xfL);
			ii >>= 4;
		}
		lcd_buf[0] = (CS2_MASK|DA_CM_MASK) | 6;			// set data string preamble
		put_spi(lcd_sfreq_1, CS_OPEN);					// send string to set display address and decode on
		put_spi(lcd_buf, CS_IDLE);						// send BCD data
		put_spi(lcd_sfreq_2, CS_CLOSE);					// set DP & close transaction
		//set/clear "1" GHz digit
		lcd_buf[0] = CS2_MASK | 2;						// isolate GHz and enable or disable 1 GHz digit
		lcd_buf[1] = LOAD_PTR | 0x1a;
		if(ii == 0x1L) lcd_buf[2] = WR_DMEM | 0x05;		// set 1G plus DP
		else lcd_buf[2] = WR_DMEM | 0x04;				// just DP
		put_spi(lcd_buf, CS_OPENCLOSE);
	}
	return;
}

//-----------------------------------------------------------------------------
// sets the S-meter: srf = 0-7 or VOL/SQU: srf = 0-34 (with hi-bit set)
// msmet() MAIN
// ssmet() SUB
//			seg		seg
//	seg		addr	data
//	6		0x06	0x4
//	3		0x07	0x1
//	4		0x07	0x2
//	5		0x07	0x4
//	0		0x08	0x1
//	1		0x08	0x2
//	2		0x08	0x4
//
// if hi-bit of srf set, disp v/q level (affects mem-ch digit also).
//	SRF met forms most significant slider of level.  Each segment is 5 units.
//		All 7 segments plus the "4" indication in the MEM digit = MAX level.
//	MEM digit forms least sig slider (modulo 5):
//		blank =				"0"
//		bot bar =			"1"
//		bot+mid bar =		"2"
//		bot+mid+top bar =	"3"
//		"o" + top bar =		"4"
//
//	Total level = (number of SRF segments * 5) + mem value
//
// if srf == 0xff, the Fn clears the SRF, MEM, and MEM blink.
//
//-----------------------------------------------------------------------------
void msmet(U8 srf, U8 blink){
	U8	i;
	U8	j = srf & 0x7f;

	if(srf == 0xff){
		lcd_buf[0] = CS1_MASK | 10;						// clear vol artifacts
		lcd_buf[1] = LOAD_PTR | MMEM_ADDR;
		lcd_buf[2] = WR_DMEM;
		lcd_buf[3] = WR_DMEM;
		lcd_buf[4] = WR_DMEM;
		lcd_buf[5] = LOAD_PTR | MMEM_ADDR;
		lcd_buf[6] = WR_BMEM;							// no flash
		lcd_buf[7] = WR_BMEM;
		lcd_buf[8] = WR_BMEM;
		lcd_buf[9] = LOAD_PTR | MSRF012_ADDR;
		lcd_buf[10] = WR_BMEM;							// no flash
		put_spi(lcd_buf, CS_OPENCLOSE);					// send DU message
		srf = 0;
		mmem(get_memnum(MAIN));							// restore mem#
	}
	if(srf & 0x80){										// do q/v level
		srf &= 0x7f;
		if(srf > LEVEL_MAX) srf = LEVEL_MAX;
		i = srf % ATTEN_FINE_LIM;						// calculate fine pattern
		if(srf == LEVEL_MAX) srf = MAX_SRF;				// saturate coarse pattern
		else srf = srf / ATTEN_FINE_LIM;				// calculate coarse pattern
		if(j == 0){
			lcd_buf[0] = CS1_MASK | 8;					// set CS1 with 8 bytes to send
			lcd_buf[1] = LOAD_PTR | MMEM_ADDR;			// for "0" level:
			lcd_buf[2] = WR_DMEM | lcd_qv_lsd[4][0];	// set "4" level
			lcd_buf[3] = WR_DMEM | lcd_qv_lsd[4][1];
			lcd_buf[4] = WR_DMEM | lcd_qv_lsd[4][2];
			lcd_buf[5] = LOAD_PTR | MMEM_ADDR;
			lcd_buf[6] = WR_BMEM | 3;					// and make it flash
			lcd_buf[7] = WR_BMEM | 7;
			lcd_buf[8] = WR_BMEM | 3;
		}else{
			lcd_buf[0] = CS1_MASK | 10;					// set CS1 with 8 bytes to send
			lcd_buf[1] = LOAD_PTR | MMEM_ADDR;
			lcd_buf[2] = WR_DMEM | lcd_qv_lsd[i][0];
			lcd_buf[3] = WR_DMEM | lcd_qv_lsd[i][1];
			lcd_buf[4] = WR_DMEM | lcd_qv_lsd[i][2];
			lcd_buf[5] = LOAD_PTR | MMEM_ADDR;
			lcd_buf[6] = WR_BMEM;						// no flash
			lcd_buf[7] = WR_BMEM;
			lcd_buf[8] = WR_BMEM;
			lcd_buf[9] = LOAD_PTR | MSRF012_ADDR;
			lcd_buf[10] = WR_BMEM | MSRF0;				// flash lowest SRF segment
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send DU message
	}
	i = 0;												// construct ordinal bar, lsb = lowest bar
	if(srf <= MAX_SRF){
		if(srf > 0){
			for(j=0; j<srf; j++){
				i <<= 1;
				i |= 1;
			}
		}
	}
	lcd_buf[0] = CS1_MASK | 4;							// set CS2 with 5 bytes to send
	lcd_buf[1] = LOAD_PTR | MSMET_ADDR;
	lcd_buf[4] = WR_DMEM | (i & 0x07);					// set lower three bars
	i >>= 3;
	lcd_buf[3] = WR_DMEM | (i & 0x07);					// set next 3 bars
	if(i & 0x08) lcd_buf[2] = WR_DMEM | (0x04);			// set highest bar
	else lcd_buf[2] = WR_DMEM;
	put_spi(lcd_buf, CS_OPENCLOSE);						// send DU message
	return;
}

//-----------------------------------------------------------------------------
// ssmet()
//-----------------------------------------------------------------------------
void ssmet(U8 srf, U8 blink){
	U8	i = 0;
	U8	j = srf & 0x7f;

	if(srf == 0xff){
		lcd_buf[0] = CS2_MASK | 10;						// clear vol artifacts
		lcd_buf[1] = LOAD_PTR | SMEM_ADDR;
		lcd_buf[2] = WR_DMEM;
		lcd_buf[3] = WR_DMEM;
		lcd_buf[4] = WR_DMEM;
		lcd_buf[5] = LOAD_PTR | SMEM_ADDR;
		lcd_buf[6] = WR_BMEM;							// no flash
		lcd_buf[7] = WR_BMEM;
		lcd_buf[8] = WR_BMEM;
		lcd_buf[9] = LOAD_PTR | SSRF012_ADDR;
		lcd_buf[10] = WR_BMEM;							// no flash
		put_spi(lcd_buf, CS_OPENCLOSE);					// send DU message
		srf = 0;
		smem(get_memnum(SUB));							// restore mem#
	}
	if(srf & 0x80){										// do q/v level
		srf &= 0x7f;
		if(srf > LEVEL_MAX) srf = LEVEL_MAX;
		i = srf % ATTEN_FINE_LIM;						// calculate fine pattern
		if(srf == LEVEL_MAX) srf = MAX_SRF;				// saturate coarse pattern
		else srf = srf / ATTEN_FINE_LIM;				// calculate coarse pattern
		if(j == 0){
			lcd_buf[0] = CS2_MASK | 8;					// set CS1 with 8 bytes to send
			lcd_buf[1] = LOAD_PTR | SMEM_ADDR;			// for "0" level:
			lcd_buf[2] = WR_DMEM | lcd_qv_lsd[4][0];	// set "4" level
			lcd_buf[3] = WR_DMEM | lcd_qv_lsd[4][1];
			lcd_buf[4] = WR_DMEM | lcd_qv_lsd[4][2];
			lcd_buf[5] = LOAD_PTR | SMEM_ADDR;
			lcd_buf[6] = WR_BMEM | 3;					// and make it flash
			lcd_buf[7] = WR_BMEM | 7;
			lcd_buf[8] = WR_BMEM | 3;
		}else{
			lcd_buf[0] = CS2_MASK | 10;					// set CS1 with 8 bytes to send
			lcd_buf[1] = LOAD_PTR | SMEM_ADDR;
			lcd_buf[2] = WR_DMEM | lcd_qv_lsd[i][0];
			lcd_buf[3] = WR_DMEM | lcd_qv_lsd[i][1];
			lcd_buf[4] = WR_DMEM | lcd_qv_lsd[i][2];
			lcd_buf[5] = LOAD_PTR | SMEM_ADDR;
			lcd_buf[6] = WR_BMEM;						// no flash
			lcd_buf[7] = WR_BMEM;
			lcd_buf[8] = WR_BMEM;
			lcd_buf[9] = LOAD_PTR | SSRF012_ADDR;
			lcd_buf[10] = WR_BMEM | SSRF0;				// flash lowest SRF segment
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send DU message
	}
	i = 0;												// construct ordinal bar, lsb = lowest bar
	if(srf <= MAX_SRF){
		if(srf > 0){
			for(j=0; j<srf; j++){
				i <<= 1;
				i |= 1;
			}
		}
	}
	lcd_buf[0] = CS2_MASK | 4;							// set CS2 with 5 bytes to send
	lcd_buf[1] = LOAD_PTR | SSMET_ADDR;
	lcd_buf[4] = WR_DMEM | (i & 0x07);					// set lower three bars
	i >>= 3;
	lcd_buf[3] = WR_DMEM | (i & 0x07);					// set next 3 bars
	if(i & 0x08) lcd_buf[2] = WR_DMEM | (0x04);			// set highest bar
	else lcd_buf[2] = WR_DMEM;
	put_spi(lcd_buf, CS_OPENCLOSE);						// send DU message
	return;
}

//-----------------------------------------------------------------------------
// asc27() takes the ASCII param and returns the uPD7225 7-seg representation
//-----------------------------------------------------------------------------

// LUT to convert (ASCII - 0x20) to uPD7225 packed 7segment code
//	starts at ASCII 0x20 (space).  0x38 is no-corresponding-code representation
//	(3 horiz bars)

U8 asc7seg[] = {
	0x00, 0x38, 0x41, 0x38, 0x38, 0x91, 0x38, 0x40,		// <spc>, !, ", #, $, %, &, ',
	0xE8, 0x2B, 0x38, 0xD0, 0x38, 0x10, 0x38, 0x91,		// (, ), *, +, ,, -, ., /,
	0xEB, 0x03, 0xB9, 0x3B, 0x53, 0x7A, 0xFA, 0x0B,		// 0, 1, 2, 3, 4, 5, 6, 7,
	0xFB, 0x7B, 0x38, 0x38, 0xB0, 0x30, 0x32, 0x99,		// 8, 9, :, ;, <, =, >, ?,
	0xBB, 0xDB, 0xF2, 0xB0, 0xB3, 0xF8, 0xD8, 0xEA,		// @, A, B, C, D, E, F, G,
	0xD2, 0x02, 0xA3, 0xD0, 0xE0, 0x9A, 0x92, 0xB2,		// H, I, J, K, L, M, N, O,
	0xD9, 0x5B, 0x90, 0x6A, 0xF0, 0xA2, 0xE3, 0xAA,		// P, Q, R, S, T, U, V, W,
	0xD3, 0x73, 0xA9, 0xE8, 0x52, 0x2B, 0x49, 0x20		// X, Y, Z, [, \, ], ^, _
	};

U8	asc27(char c){
	U8	i;			// temps
	U8	rtn;

	if(c >= ' ') i = (U8)(c - ' ');						// convert char to 7-seg code LUT index
	else i = 0xff;										// set invalid char
	if(i > sizeof(asc7seg)){
		rtn = 0x09;										// invalid char
	}else{
		rtn = asc7seg[i];								// return seg code
	}
	return rtn;
}

//-----------------------------------------------------------------------------
// clear_lcd_buf() clears the LCD message buffer
//-----------------------------------------------------------------------------
void clear_lcd_buf(void){
	U8	i;		// temp

	for(i=0; i<LCD_BUFLEN;i++){
		lcd_buf[i] = 0;
	}
	return;
}

//-----------------------------------------------------------------------------
// mputs_lcd() translates an ASCII string to 7-seg for main digits
//	returns #chars written to display
//-----------------------------------------------------------------------------
U8	mputs_lcd(char *s, U8 dp_tf){
	U8	i;		// temp

	clear_lcd_buf();
	lcd_buf[0] = CS1_MASK|DA_CM_MASK | 0x06;
	i = 6;
	while(*s && (i < 7)){
		lcd_buf[i--] = asc27(*s++);
	}
	put_spi(lcd_mraw, CS_OPEN);							// init for raw seg data
	put_spi(lcd_buf, CS_IDLE);
	if(dp_tf) put_spi(lcd_mfreq_2, CS_CLOSE);			// DP
	else put_spi(lcd_mfreq_3, CS_CLOSE);				// no DP
	return (i-1);
}

//-----------------------------------------------------------------------------
// sputs_lcd() translates an ASCII string to 7-seg for sub digits
//	returns #chars written to display
//-----------------------------------------------------------------------------
U8	sputs_lcd(char *s, U8 dp_tf){
	U8	i;		// temp

	clear_lcd_buf();
	lcd_buf[0] = CS2_MASK|DA_CM_MASK | 0x06;	//0x66;
	i = 6;												// only 6 chars per band
	while(*s && (i < 7)){
		lcd_buf[i--] = asc27(*s++);
	}
	put_spi(lcd_sraw, CS_OPEN);							// init for raw seg data
	put_spi(lcd_buf, CS_IDLE);
	if(dp_tf) put_spi(lcd_sfreq_2, CS_CLOSE);			// DP
	else put_spi(lcd_sfreq_3, CS_CLOSE);				// no DP
	return (i-1);
}

//-----------------------------------------------------------------------------
// mmem() displays main mem#.  Input is ASCII char.
//-----------------------------------------------------------------------------
void mmem(char c){
	U8	i;		// temp

	lcd_buf[0] = CS1_MASK | 0x04;
	lcd_buf[1] = LOAD_PTR | MMEM_ADDR;
	i = asc27(c);										// get seg code
	lcd_buf[2] = WR_DMEM | (i & 0x03);
	i >>= 3;
	lcd_buf[3] = WR_DMEM | (i & 0x07);
	lcd_buf[4] = WR_DMEM | ((i >> 3) & 0x03);
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// smem() displays sub mem#.  Input is ASCII.
//-----------------------------------------------------------------------------
void smem(char c){
	U8	i;		// temp

	lcd_buf[0] = CS2_MASK | 0x04;
	lcd_buf[1] = LOAD_PTR | SMEM_ADDR;
	i = asc27(c);										// get seg code
	lcd_buf[2] = WR_DMEM | (i & 0x03);
	i >>= 3;
	lcd_buf[3] = WR_DMEM | (i & 0x07);
	lcd_buf[4] = WR_DMEM | ((i >> 3) & 0x03);
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mtonea() sets/clears MAIN TONE annunc.  Input is T/F
// stonea() sets/clears SUB TONE annunc.  Input is T/F
//-----------------------------------------------------------------------------
void mtonea(U8 tf){

	lcd_buf[0] = CS1_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | MTNE_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | MTNE;
	}else{
		lcd_buf[2] = AND_DMEM | MM;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void stonea(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | STNE_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | STNE;
	}else{
		lcd_buf[2] = AND_DMEM | SM;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mmema() sets/clears MAIN mem annunc.  Input is T/F
// smema() sets/clears SUB mem annunc.  Input is T/F
//-----------------------------------------------------------------------------
void mmema(U8 tf){

	lcd_buf[0] = CS1_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | MM_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | MM;
	}else{
		lcd_buf[2] = AND_DMEM | MTNE;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void smema(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | SM_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | SM;
	}else{
		lcd_buf[2] = AND_DMEM | STNE;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mdupa() sets/clears MAIN DUP annunc.  Input is ASCII, S, +, -
// sdupa() sets/clears SUB DUP annunc.  Input is ASCII, S, +, -
//-----------------------------------------------------------------------------
void mdupa(char dplx){
	U8	i = 0x02;

	lcd_buf[1] = LOAD_PTR | MDUP_ADDR;
	switch(dplx){
	default:
	case 'S':
		lcd_buf[2] = AND_DMEM | MSKP;					// turn off "DUP" & "-"
		break;

	case '+':
		lcd_buf[2] = AND_DMEM | MSKP | MDUP;			// turn off "-"
		lcd_buf[3] = LOAD_PTR | MDUP_ADDR;
		lcd_buf[4] = OR_DMEM | MDUP;					// turn on "DUP"
		i += 2;
		break;

	case '-':
		lcd_buf[2] = OR_DMEM | MDUP | MMIN;				// turn on "DUP" & "-"
		break;
	}
	lcd_buf[0] = CS1_MASK | i;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void sdupa(char dplx){
	U8	i = 0x02;

	lcd_buf[1] = LOAD_PTR | SDUP_ADDR;
	switch(dplx){
	default:
	case 'S':
		lcd_buf[2] = AND_DMEM | SSKP;					// turn off "DUP" & "-"
		break;

	case '+':
		lcd_buf[2] = AND_DMEM | SSKP | SDUP;			// turn off "-"
		lcd_buf[3] = LOAD_PTR | SDUP_ADDR;
		lcd_buf[4] = OR_DMEM | SDUP;					// turn on "DUP"
		i += 2;
		break;

	case '-':
		lcd_buf[2] = OR_DMEM | SDUP | SMIN;				// turn on "DUP" & "-"
		break;
	}
	lcd_buf[0] = CS2_MASK | i;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mdupa_blink() blinks/unblinks MAIN DUP annunc.  Input is tf
// sdupa_blink() blinks/unblinks SUB DUP annunc.  Input is tf
//-----------------------------------------------------------------------------
void mdupa_blink(U8 tf){

	lcd_buf[1] = LOAD_PTR | MDUP_ADDR;
	if(tf){
		lcd_buf[2] = OR_BMEM | MDUP | MMIN;				// blink "DUP" & "-"
	}else{
		lcd_buf[2] = AND_BMEM | MSKP;					// unblink "DUP" & "-"
	}
	lcd_buf[0] = CS1_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void sdupa_blink(U8 tf){

	lcd_buf[1] = LOAD_PTR | SDUP_ADDR;
	if(tf){
		lcd_buf[2] = OR_BMEM | SDUP | SMIN;				// blink "DUP" & "-"
	}else{
		lcd_buf[2] = AND_BMEM | SSKP;					// unblink "DUP" & "-"
	}
	lcd_buf[0] = CS2_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mset_500hz() turns on 500 hz icon.  cmd = 0 is off, 1 is on, 2 is blink
// sset_500hz()
//-----------------------------------------------------------------------------
void mset_500hz(U8 cmd){

	switch(cmd){
	default:
	case 0:
		mblink_500hz(0);
		mon_500hz(0);
		break;

	case 1:
		mon_500hz(1);
		break;

	case 2:
		mon_500hz(1);
		mblink_500hz(1);
		break;
	}
	return;
}

void sset_500hz(U8 cmd){

	switch(cmd){
	default:
	case 0:
		sblink_500hz(0);
		son_500hz(0);
		break;

	case 1:
		son_500hz(1);
		break;

	case 2:
		son_500hz(1);
		sblink_500hz(1);
		break;
	}
	return;
}

//-----------------------------------------------------------------------------
// mon_500hz() turns on 500 hz icon.  Input is tf
// son_500hz()
//-----------------------------------------------------------------------------
void mon_500hz(U8 tf){

	lcd_buf[1] = LOAD_PTR | M00_ADDR;
	if(tf){
		lcd_buf[2] = WR_DMEM | M00;						// set 500hz icon
	}else{
		lcd_buf[2] = WR_DMEM;							// clear...
	}
	lcd_buf[0] = CS1_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void son_500hz(U8 tf){

	lcd_buf[1] = LOAD_PTR | S00_ADDR;
	if(tf){
		lcd_buf[2] = WR_DMEM | S00;						// set 500hz icon
	}else{
		lcd_buf[2] = WR_DMEM;							// clear...
	}
	lcd_buf[0] = CS2_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// mblink_500hz() turns on 500 hz icon.  Input is tf
// sblink_500hz()
//-----------------------------------------------------------------------------
void mblink_500hz(U8 tf){

	lcd_buf[1] = LOAD_PTR | M00_ADDR;
	if(tf){
		lcd_buf[2] = WR_BMEM | M00;						// blink 500hz icon
	}else{
		lcd_buf[2] = WR_BMEM;							// clear...
	}
	lcd_buf[0] = CS1_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

void sblink_500hz(U8 tf){

	lcd_buf[1] = LOAD_PTR | S00_ADDR;
	if(tf){
		lcd_buf[2] = WR_BMEM | S00;						// blink 500hz icon
	}else{
		lcd_buf[2] = WR_BMEM;							// clear...
	}
	lcd_buf[0] = CS2_MASK | 2;
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// alow() sets/clears LOW annunc.  Input is TF
//-----------------------------------------------------------------------------
void alow(U8 tf){

	lcd_buf[0] = CS1_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | ALOW_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | ALOW;
	}else{
		lcd_buf[2] = AND_DMEM | AOW;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// alock() sets/clears LOCK annunc.  Input is TF
//-----------------------------------------------------------------------------
void alock(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | ALCK_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | ALCK;
	}else{
		lcd_buf[2] = AND_DMEM | ASUB;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// amhz() sets/clears MHZ annunc.  Input is TF
//-----------------------------------------------------------------------------
void amhz(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | AMHZ_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | AMHZ;
	}else{
		lcd_buf[2] = AND_DMEM | APRG | ABND;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// abnd() sets/clears BAND annunc.  Input is TF
//-----------------------------------------------------------------------------
void abnd(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | ABND_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | ABND;
	}else{
		lcd_buf[2] = AND_DMEM | APRG | AMHZ;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// asub() sets/clears LOW annunc.  Input is TF
//-----------------------------------------------------------------------------
void asub(U8 tf){

	lcd_buf[0] = CS2_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | ASUB_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | ASUB;
	}else{
		lcd_buf[2] = AND_DMEM | ALCK;
	}
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// asub() sets/clears TS annunc.  Input is TF
//-----------------------------------------------------------------------------
void ats(U8 tf){

	lcd_buf[0] = CS1_MASK | 0x02;						// set chip CS id and message length
	lcd_buf[1] = LOAD_PTR | ATS_ADDR;					// store set addr command
	if(tf){
		lcd_buf[2] = OR_DMEM | ATS;						// if on, or the TS annunc
	}else{
		lcd_buf[2] = AND_DMEM | AVXO | ARIT;			// if off, and the other two annunc at this addr
	}													// so the TS is off, and they are un-changed
	put_spi(lcd_buf, CS_OPENCLOSE);
	return;
}

//-----------------------------------------------------------------------------
// bin32_bcdp() converts a U32 to a packed BCD value.
//-----------------------------------------------------------------------------
U32 bin32_bcdp(U32 bin32){
	U32	jj = 0;		// return temp
	U32	ii;			// temps
	U32	kk;
	U32	ll = bin32;

	for(ii=1000000L; ii!=0; ii/=10){
		jj <<= 4;
		kk = ll/ii;
		if(kk != 0){
			jj |= kk;
			ll = ll - (kk * ii);
		}
	}
	return jj;
}

//-----------------------------------------------------------------------------
// bin32_bcds() converts a U32 to an un-packed BCD string.
//-----------------------------------------------------------------------------
void bin32_bcds(U32 bin32, U8* sptr){
	U32	ii;			// temps
	U32	kk;
	U32	ll = bin32;

	for(ii=1000000L; ii!=0; ii/=10){
		kk = ll/ii;
		*sptr++ = kk;
		if(kk != 0){
			ll = ll - (kk * ii);
		}
	}
	return;
}

//-----------------------------------------------------------------------------
// bcds_bin32() converts an un-packed BCD string to U32 bin.
//-----------------------------------------------------------------------------
U32 bcds_bin32(U8* sptr){
	U32	ii;			// temps
	U32	kk = 0;
//	U32	ll = bin32;

	for(ii=1000000L; ii!=0; ii/=10){
		kk += (U32)(*sptr++) * ii;
	}
	return kk;
}

//-----------------------------------------------------------------------------
// add_bcds() adds n to unpacked BCD string at display addr
//-----------------------------------------------------------------------------
void add_bcds(U8* sptr, S8 adder, U8 addr, U8 max, U8 min){
	U8	i;			// string index

	if(adder > 9) adder = 0;							// error trap
	i = 6 - addr;
	sptr[i] += adder;
	if((sptr[i] > 0x7f) || (sptr[i] < min)){
		sptr[i] = sptr[i] + (max - min + 1);			// roll-under
	}
	if(sptr[i] > max){
		sptr[i] = sptr[i] - (max - min + 1);			// roll-over
	}
	return;
}

//*****************************************************************************
//
//		LED ANNUCIATOR Fns
//		While not LCD related, this is the best place for these FNs
//
//-----------------------------------------------------------------------------
// amtx() sets/clears the main TX LED.  Input is TF
//-----------------------------------------------------------------------------
void amtx(U8 tf){

	if(tf) GPIO_PORTD_DATA_R &= ~MTX_N;
	else GPIO_PORTD_DATA_R |= MTX_N;
	return;
}

//-----------------------------------------------------------------------------
// amrx() sets/clears the main RX LED.  Input is TF
//-----------------------------------------------------------------------------
void amrx(U8 tf){

	if(tf) GPIO_PORTC_DATA_R &= ~MRX_N;
	else GPIO_PORTC_DATA_R |= MRX_N;
	return;
}

//-----------------------------------------------------------------------------
// asrx() sets/clears the sub RX LED.  Input is TF
//-----------------------------------------------------------------------------
void asrx(U8 tf){

	if(tf) GPIO_PORTC_DATA_R &= ~SRX_N;
	else GPIO_PORTC_DATA_R |= SRX_N;
	return;
}

//-----------------------------------------------------------------------------
// lamp_test() writes all "1"s to LCD segment memory and activates RX/TX LEDs
//	True-param sets test
//	false-param clears test
//-----------------------------------------------------------------------------
void lamp_test(U8 tf){
	U8	i;
	U8	j;
	U8	k;

	if(tf){
		j = WR_DMEM | 0xf;
		GPIO_PORTC_DATA_R |= MRX_N | SRX_N;
		GPIO_PORTD_DATA_R |= MTX_N;
	}else{
		j = WR_DMEM;
		GPIO_PORTC_DATA_R &= ~(MRX_N | SRX_N);
		GPIO_PORTD_DATA_R &= ~MTX_N;
	}
	for(k=0; k<2; k++){
		lcd_cmd(0);										// set cmd
		open_spi(k);
		send_spi3(LOAD_PTR);							// set ptr to 0
		wait2(2);
		for(i=0; i<32; i++){
			send_spi3(j);								// fill mem
			wait2(2);
		}
		close_spi();
		wait2(10);
		open_spi(k);
		send_spi3(LOAD_PTR | 0x1f);						// set ptr to end
		wait2(2);
		send_spi3(j);									// fill last addr
		wait2(2);
		close_spi();
		wait2(10);
	}
	return;
}

//-----------------------------------------------------------------------------
// set_vfo_display() sets the vfo display signal
//-----------------------------------------------------------------------------
void set_vfo_display(U8	sig){

	vfo_display |= sig;
	return;
}

//-----------------------------------------------------------------------------
// rev_vfo() reverses the T/R on focused VFO
//-----------------------------------------------------------------------------
void rev_vfo(U8	focus){
	U8	i;		// temp

	if(inv_vfo(focus)){
		i = inv_duplex(focus);
		switch(i){
		default:
		case DPLX_S:
			i = 'S';
			break;

		case DPLX_M:
			i = '-';
			break;

		case DPLX_P:
			i = '+';
			break;
		}
		if(focus == MAIN) mdupa(i);
		else sdupa(i);
		vfo_change(focus);
		if(focus == MAIN) vfo_display |= MAIN;
		else vfo_display |= SUB_D;
	}
	return;
}

//-----------------------------------------------------------------------------
// force_push() sets system to push the VFO structure to HIB RAM if not check mode
//-----------------------------------------------------------------------------
void  force_push(void){

	if(!chkmode) force_push_radio();
	return;
}
