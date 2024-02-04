/********************************************************************
 ************ COPYRIGHT (c) 2022 by ke0ff, Taylor, TX   *************
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
// Hard-coded LCD comm messages:
U8	lcd_init_1[] = { 0x45, 0x49, CLR_DMEM, CLR_BMEM, DISP_ON, BLINK_FAST };	// init: {CS1, cmd, 5bytes}; 0x49 = {1/3 bias, /3 timefreq, 1/(2^8) feq div}
U8	lcd_init_2[] = { 0x85, 0x49, CLR_DMEM, CLR_BMEM, DISP_ON, BLINK_FAST};	// CS2 chip init
U8	lcd_mfreq_1[] = { 0x43, 0xe9, OR_DMEM, WITH_DECODE };					// set pointer, "5"00 hz = no change, WITH seg decoder
//U8	lcd_mfreq_2[] = { 0x41, 0xd4 };											// MDP
//U8	lcd_mfreq_3[] = { 0x41, 0xd0 };											// no MDP
U8	lcd_mraw[] = { CS1_MASK | 3, LOAD_PTR | 9, OR_DMEM, WITHOUT_DECODE };	// set pointer, "5"00 hz = no change, WITHOUT seg decoder
U8	lcd_sfreq_1[] = { 0x83, 0xe7, OR_DMEM, WITH_DECODE };					// set pointer, "5"00 hz = no change, WITH seg decoder
//U8	lcd_sfreq_2[] = { 0x81, 0xd4 };											// SDP
//U8	lcd_sfreq_3[] = { 0x81, 0xd0 };											// no SDP
U8	lcd_sraw[] = { CS2_MASK | 3, LOAD_PTR | 7, OR_DMEM, WITHOUT_DECODE };	// set pointer, "5"00 hz = no change, WITHOUT seg decoder
// VOL/SQU low-order ordinal sequence.  A list of segment patterns for the MEM digit during VOL/SQU adjust
U8	lcd_qv_lsd[5][3] = {													// list of 5 7-seg ordinal patterns
					// [ordinal][seg addr]
					{ 0, 4, 0 },
					{ 0, 6, 0 },
					{ 0, 6, 1 },
					{ 0, 7, 1 },
					{ 1, 7, 1 }
};
// TS ordinal sequence.  A list of step sizes [B:A]
U8	ts_order[3] = { 0x21, 0x51, 0x52 };

// List of supported CTCSS tones, in Hz * 10.  1st tone is #1, last is #38:
//					  00	01    02    03    04    05    06    07
U16	tone_list[] = {  670,  719,  744,  770,  797,  825,  854,  885,
//					  08	09    0a    0b    0c    0d    0e    0f
					 915,  948,  974, 1000, 1035, 1072, 1109, 1148,
//					  10	11    12    13    14    15    16    17
					1188, 1230, 1273, 1318, 1365, 1413, 1462, 1514,
//					  18	19    1a    1b    1c    1d    1e    1f
					1567, 1622, 1679, 1738, 1799, 1862, 1928, 2035,
//					  20	21    22    23    24    25
					2107, 2181, 2257, 2336, 2418, 2503, 0xffff };

// Mem ordinals         0         0123456789012345678901234
char mem_ordinal[] = { "0123456789ABDEFGHJKLMNPRSTUWYZ()[]" };

#define	LCD_BUFLEN 23
U8	lcd_buf[LCD_BUFLEN];			// LCD comm message buffer
U8	maddr;							// mhz digit mode composite digit address and mode flags register
U8	vfo_display;					// display update signal.  This is band (MAIN/SUB) to update or'd with 0x80 to trigger
U8	uimode;							// Current DU mode, M/S
U8	xmode[ID1200];					// xmode flags track mem/call modes
U8	xmodeq;							// xmodeq tracks transient adjust modes (vol, squ, tone, offs)
U8	xmodez;							// xmodez tracks first-scan enabled flags
U8	chkmode;						// indicates check/rev mode is in effect
U8	chksqu;							// save reg for check squelch
U8	mute_mode;						// main/sub mute status
U8	tsdisplay;						// ts display flag
U8	sys_err;						// system error flags.  0x00 = no errors
U8	ptt_change;						// PTT changed flag (from radio.c)
char mfbuf[16];						// ascii text (freq field) buffer, main
char sfbuf[16];						// ... sub
U32	mscan[ID1200];					// mem scan enable bits (gathered by get_mscan())
U32	mscan_maskm;					// scan enable mask (m/s)
U32	mscan_masks;
U8	scan_switch;					// scan switch event flag
U8	last_cosm[MAX_COS_MEM];			// last cos ban/mem# circ buffers
U8	last_coss[MAX_COS_MEM];			// .. hi-nyb is band, low-nyb is mem#
U8	last_cosm_tp;					// head/tail pointers
U8	last_cosm_hp;
U8	last_coss_tp;
U8	last_coss_hp;
U32	dfe_vfo;						// direct freq entry temp vfo
char stat_str[STAT_BUF_LEN];		// status display string
char* stat_ptr;						// status display pointer
U8	sw_stat;						// X-sliding switches memory reg (these switches are now a single PBSW processed along with the other PBSWs)
U8	sw_temp;						// X-sliding switches I/O memory reg

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------
U8	asc27(char c);
void clear_lcd_buf(void);
void alock(U8 tf);
U32 bin32_bcdp(U32 bin32);
void lamp_test(U8 tf);
U8 process_MS(U8 cmd);
void disp_duplex(U8 focus, U8 duplex);
void process_VFODISP(U8 focus);
U8 test_for_cancel(U8 key);
U32 process_DIAL(U8 focus);
void freq_update(U8 focus, S8 step);
U8 process_MEM(U8 cmd);
U8 process_SET(U8 cmd);
void ats(U8 tf);
void mset_500hz(U8 cmd);
void sset_500hz(U8 cmd);
void mon_500hz(U8 tf);
void son_500hz(U8 tf);
void mm6(U8 tf);
void sm6(U8 tf);
void mdp(U8 tf);
void sdp(U8 tf);

void mblink_500hz(U8 tf);
void sblink_500hz(U8 tf);
void dp_blink(U8 tf);
void mdp2_blink(U8 tf);
void sdp_blink(U8 tf);
void sdp2_blink(U8 tf);

void mmute_action(U8* mute_flag);
void smute_action(U8* mute_flag);
U8 puts_lcd(U8 focus, char *s, U8 dp_tf);
void togg_tsab(U8 focus);
U8 copy6str(char* sptr, char* dptr, U8 cidx);
U8 nxtscan(U8 focus, U8 adder);
U32 mem2bit(U8 memnum);
void get_mscan(U8 focus);
void set_last_cos(U8 focus);
U8 get_last_cos(U8 focus);
U8 process_DFE(U8 focus, U8 keychr);
void set_dfe(U8 focus, U8 bandsw, U8 curband);
void ipl_cancel(void);
U8 switch_band(U8 focus);
U8 process_STAT(U8 focus, U8 keychr);

//-----------------------------------------------------------------------------
// init_lcd() initializes lcd resources
//	The LCD chips (uPD7225) use an SPO=1/SPH=1 transfer format (clock idles high,
//	and data changes on clk FE, stable at clk RE).  However, the clock and data
//	are inverted for this design, so we need SPO=0/SPH=1 transfer format.
//
// The max SSI clock rate per the uPD7225 datasheet is about 1.1 MHz.
//-----------------------------------------------------------------------------
void init_lcd(void){
	volatile uint32_t ui32Loop;
//	U8	test_str[10];	// !!! debug

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

//****************
// reset_lcd performs a hardware LCD reset
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

	//**************************************
	// process IPL (Initial Program Load) init
	if(cmd == PROC_INIT){
#ifdef IC900F
		sw_stat = 0;													// init slide switch status to off (!!!add to NVRAM to fix this later)
#else
		GPIO_PORTD_DATA_R |= LOCK_SELECT;
		wait(10);
		sw_stat = ~GPIO_PORTB_DATA_R & (DIM | MISO_LOCK);				// force update of slide switch status
#endif
		backl_adj(BRT_DIM);												// init dim setting
		uimode = MAIN_MODE;												// init process variables
		chkmode = 0;
		vfo_display = 0;
		update_lcd(MAIN, MAIN);
		update_lcd(MAIN, SUB);
		process_MS(0xff);												// trigger main/sub process IPL init
		is_mic_updn(1, 0, 0);											// init mic u/d repeat
		ptt_change = 0;
		wait(100);
		mute_radio(0);
		dfe_vfo = 0;
	}else{
		//**************************************
		// process the UI for each of the different modes:
		switch(uimode){
		default:
			uimode = MAIN_MODE;											// fault trap... force mode to MAIN_MODE
		// MAIN/SUB "normal" process
		case MAIN_MODE:
		case SUB_MODE:
 			mode_rtn = process_MS(uimode);
			break;

		// SET configuration loop process
		case SET_MODE:
//			mode_rtn = process_SET(uimode);
			break;

		// Memory mode process
//		case MEM_MODE:
//			mode_rtn = process_MEM(uimode);
//			break;
		}
		//**************************************
		// update the display when there are mode changes
		if(mode_rtn != uimode){
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

//			case MEM_MODE:
//				init_lcd_mem();
//				break;
			}
			uimode = mode_rtn;
		}
		//**************************************
		// process slide switches (DIM and LOCK)
		if((GPIO_PORTD_DATA_R & RAMCS_N) == LOCK_SELECT){
//			wait(5);														// guaranteed settling time !!! need better solution here
#ifdef IC900F
			i = sw_temp & (DIM | LOCK);										// capture DIM/LOCK switch settings
#else
			i = GPIO_PORTB_DATA_R & (DIM | LOCK);							// capture DIM/LOCK switch settings
#endif
			if(i ^ sw_stat){												// if changes..
				if(i & DIM){
					// process bright settings (!!! these need to be configurable in SET loop)
					backl_adj(BRT_BRT);
/*					set_pwm(5, 99);											// LEDs
			        set_pwm(6, 90);											// LCDBL*/
				}else{
					// process DIM settings (!!! these need to be configurable in SET loop)
					backl_adj(BRT_DIM);
/*					set_pwm(5, 60);
			        set_pwm(6, 30);*/
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
	return;
}	// end process_UI()

//-----------------------------------------------------------------------------
// update_lcd() forces update of LCD values
//	focus = current focus (maintained in uimode)
//	forced_focus = focus of band to be updated
//-----------------------------------------------------------------------------
void update_lcd(U8 focus, U8 forced_focus){
	U8	i;

	//**************************************
	// update low/hi power display icon
	i = get_lohi(focus, 0xff);
	if(i){
		alow(1);
	}else{
		alow(0);
	}
	if(forced_focus == MAIN){
		//**************************************
		// update Duplex display, main
		switch(read_dplx(MAIN) & (DPLX_MASK)){
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
		// update flag cluster
		ats(read_dplx(MAIN) & TSA_F);										// TS (f-step) display
		mtonea(adjust_toneon(MAIN, 0xff));									// TONE display
		if((xmode[get_band_index(MAIN)] & MC_XFLAG) == MEM_XFLAG){			// main band...
			mema(MAIN, 1);													// turn on "M"
			mskpa(MAIN, get_scanmem(MAIN));									// update "skp" annunc.
		}else{
			msmet(0xff, 0);													// init SRF/MEM
			mema(MAIN, 0);													// turn off "M"
			mskpa(MAIN, 1);													// "skp" annunc = off
		}
//		mfreq(get_freq(MAIN), 0);											// Frequency display
		if(ptt_change & PTT_KEYED){
			i = MAIN | VMODE_ISTX;
		}
		else{
			i = MAIN;
		}
		mfreq(get_freq(i), 0);							// update main freq display from vfo or vfotr
	}else{
		//**************************************
		// update Duplex display, sub
		switch(read_dplx(SUB) & (DPLX_MASK)){
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
		// update flag cluster
		stonea(adjust_toneon(SUB, 0xff));
		ssmet(0xff, 0);														// init SRF/MEM
		if((xmode[get_band_index(SUB)] & MC_XFLAG) == MEM_XFLAG){			// sub band...
			mema(SUB, 1);													// turn on "M"
			mskpa(SUB, get_scanmem(SUB));									// update "skp" annunc.
		}else{
			mema(SUB, 0);													// turn off "M"
			mskpa(SUB, 1);													// "skp" annunc = off
		}
		if(!(xmodeq & TEXTS_SLIDE)){
			sfreq(get_freq(SUB), 0);
		}
	}
	return;
}	// end update_lcd()

//-----------------------------------------------------------------------------
// process_MS() MAIN/SUB mode: updates LCD based on SIN change flags
//	processes key inputs, dial changes, and CCMD inputs
//	returns mode changes
//-----------------------------------------------------------------------------
U8 process_MS(U8 mode){
	static	U32		iflags;
	static	U8		hflag;
	static	U8		ssubfl;
			U8		b;					// key beep counter
	static	U8		bb;					// key beep counter (lock/dim)
			U8		i;					// temps
			U8		j;
			U8		k;					// band index
			U8		l;
			U8		m;
	static	U8		sw_arm;				// DIM switch arm flag
			U32		ii;
	volatile U32	sin_a0;
	volatile U32	sin_a1;
			U8		band_focus = mode;	// band focus of keys/dial
			S32		sii;
	static	char	rsbuf[16];			// CAT update message buffers

//	char dgbuf[30];	// !!!!debug

	//**************************************
	// process IPL init
	if(mode == PROC_INIT){
		maddr = MHZ_OFF;												// MHz/thumbwheel mode init
		xmodeq = 0;														// x-modes
		mute_mode = 0;													// smute = off
		iflags = 0;														// SIN change flags storage init
		hflag = 0;														// hold key flag
		tsdisplay = 0;													// clear TS adj display mode
		vfo_display = MAIN|SUB_D;										// force update of main/sub freq
		ssubfl = MAIN;
		xmodez |= IPL_BOOT;												// set bootflag
		bb = 0;
		sw_arm = 0;
	}else{
		//**************************************
		// process SIN changes
		k = get_band_index(band_focus);
		iflags |= read_sin_flags(0);									// merge with radio.c variable
		if(iflags){
			// got changes...
			// trigger status msg
			if(iflags & (SIN_MSRF_F|SIN_SSRF_F|SIN_SEND_F|SIN_DSQA_F|SIN_DSQB_F)){
				send_stat(MAIN, 'R', rsbuf);
			}
			if(ptt_change & PTT_EDGE){
				iflags |= SIN_MSRF_F;									// force update of MSRF
				ptt_change &= ~PTT_EDGE;
//				read_sin_flags(SIN_SEND_F);								// clear changes flag
			}
			sin_a0 = fetch_sin(0);										// update data
			// check if squelch adjust
			if(!(xmodeq & SQU_XFLAG)){
				if(iflags & SIN_MSRF_F){
					// update SRF
					ii = sin_a0 >> 23;									// isolate main and sub SRF
					i = (U8)(ii & 0x0f);
					msmet(i>>1, 0);										// update glass
					read_sin_flags(SIN_MSRF_F);							// clear changes flag
					if(xmode[get_band_index(MAIN)] & CALL_XFLAG){
						mmem(get_callnum(MAIN, 0));						// update call#
					}else{
						mmem(get_memnum(MAIN, 0));						// update mem#
					}
				}
			}
			// check if vol adjust
			if(!(xmodeq & VOL_XFLAG)){
				if(iflags & SIN_SSRF_F){
					// update SRF
					ii = sin_a0 >> 19;									// isolate main and sub SRF
					i = (U8)(ii & 0x0f);
					ssmet(i>>1, 0);										// update glass
					read_sin_flags(SIN_SSRF_F);							// clear changes flag
					if(xmode[get_band_index(SUB)] & CALL_XFLAG){
						smem(get_callnum(SUB, 0));						// update call#
					}else{
						smem(get_memnum(SUB, 0));						// update mem#
					}
				}
			}
			if(iflags & (SIN_SQSM_F|SIN_SQSS_F)){						// LED updates (MRX, MTX, SRX)
				// update RX LEDs
				if((sin_a0 & SIN_SQSA) && !(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT))){
					GPIO_PORTC_DATA_R |= MRX_N;							// main led = on
					scan_time(MAIN, 2);									// reset scan timer
				}else{
					GPIO_PORTC_DATA_R &= ~MRX_N;						// main led = off
				}
				if(sin_a0 & SIN_SQSB && !(sys_err & (NO_B_PRSNT|NO_SUX_PRSNT))){
					// sub led
					GPIO_PORTC_DATA_R |= SRX_N;							// sub led = on
					if(scan_switch){
						if(!scan_time(SUB, 0)){
//							scan_time(SUB, 1);							// reset scan timer after band edge (don't advance mem#)
							scan_switch = 0;							// disable edge flag
						}
					}else{
						scan_time(SUB, 2);								// reset scan timer
					}
				}else{
					GPIO_PORTC_DATA_R &= ~SRX_N;						// sub led = off
					set_last_cos(SUB);									// store last COS
				}
				read_sin_flags(SIN_SQSM_F|SIN_SQSS_F);					// clear changes flag
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
		//**************************************
		// process vfo display update signal
		process_VFODISP(band_focus);
		//**************************************
		// process dial and mic up/dn changes
		iflags |= process_DIAL(band_focus);
		//**************************************
		// process timeouts (MHz, xflag and SUB)
		if(!mhz_time(0) && (maddr < MHZ_OFF)){							// process mhz digit timeout:
			do_2beep();													// timeoute alert beep (Morse "I")
			if(band_focus == MAIN) digblink(MAIN_CS|maddr,0);
			else digblink(maddr,0);
			maddr = MHZ_OFF;
			copy_vfot(band_focus);
			vfo_change(band_focus);
		}
		if((xmodeq & VOL_XFLAG) && (!v_time(0))){						// if vol x-mode timeout:
			xmodeq &= ~VOL_XFLAG;
			iflags |= SIN_SSRF_F;										// update sub SRFs
			ssmet(0xff, 0);												// update glass
		}
		if((xmodeq & SQU_XFLAG) && (!q_time(0))){						// if squ x-mode timeout:
			xmodeq &= ~SQU_XFLAG;
			iflags |= SIN_MSRF_F;										// update main SRFs
			msmet(0xff, 0);												// update glass
		}
		if((xmodeq & OFFS_XFLAG) && (!offs_time(0))){					// if offs x-mode timeout:
			do_2beep();													// timeoute alert beep (Morse "I")
			if(band_focus == MAIN) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0);
			else digblink(maddr,0);
			mhz_time(0xff);												// clear timers
			offs_time(0xff);
			maddr = MHZ_OFF;
			xmodeq &= ~OFFS_XFLAG;
			if(band_focus == MAIN) vfo_display |= MAIN;					// update main VFO
			else vfo_display |= SUB_D;									// update sub VFO
			// if tx
			force_push();												// force update to NVRAM
		}
		if(band_focus == SUB){
			if(!sub_time(0)){											// if SUB timeout, return to main focus
				do_3beep();												// timeoute alert beep (Morse "S")
				band_focus = MAIN;
				asub(0);
				ats(read_dplx(MAIN) & TSA_F);
				i = get_lohi(band_focus, 0xff);
				if(i){													// HILO updates based on focus
					alow(1);
				}else{
					alow(0);
				}
				k = get_band_index(band_focus);							// reset focus pointer
			}
		}
		if(tsdisplay){
			if(!ts_time(0)){											// if SUB timeout, return to main focus
				do_2beep();												// timeoute alert beep (Morse "S")
				tsdisplay = 0;
				if(band_focus == MAIN) vfo_display |= MAIN;				// update main VFO
				else vfo_display |= SUB_D;								// update sub VFO
			}
		}
		//**************************************
		// process keys
		b = 0;
		if(got_key() || got_hm_asc()){									// only run through this branch if there are keys to input
			if(band_focus == SUB){
				sub_time(1);											// reset timeout
			}
			i = get_key();												// pull key in from buffer space
			// if locked, discard key (unless a lock key action)
			if(sw_stat & LOCK){
				if(i != LOCKDIMchr_H){
					i = 0;												// discard keysw
				}
			}
			// scan cancel
			switch(i){													// process scan cancel key presses
			case VFOchr:
			case MRchr:
			case MSchr:
			case CALLchr:
			case TONEchr:
			case DUPchr:
			case SETchr:
				set_slide(band_focus, 0);
			case MHZchr:
				doscan(band_focus, 0);
				break;

#ifdef IC900F
			case LOCKDIMchr:											// process arm DIM select
				sw_arm = 1;
				bb = 1;
				break;

			case LOCKDIMchr_H:											// process lock, disarm DIM select
				sw_temp ^= LOCK;
				sw_arm = 0;
				bb = 3;
				break;

			case LOCKDIMchr_R:											// process dim
				if(sw_arm){
					sw_temp ^= DIM;
					bb = 1;
				}
				sw_arm = 0;
				break;
#endif

			default:
				break;
			}
			i = test_for_cancel(i);										// check to see if keycode qualifies for "cancel" (substitutes "cancel" key in place of the actual key)
			b = 1;
			// ============ start of key parsing switch
			switch(i){													// dispatch to key-specific code segments...

			case SETchr_H:
	  			// advance the bank and display the result
	  			i = nvbank_nxt(1);
	  			b = 3;
			case SETchr:
	  			// bflags hold the pttsub action status.  read the current bflags
	  			//	and advance the status
	  			i = nvbank_nxt(0);
				if(b!=3) b = 1;
	  			// display status
	  			switch(i){
	  			case 0:
	  				put_stat(MAIN, "BANK0  BANK0");
					break;

	  			case 1:
	  				put_stat(MAIN, "BANK1  BANK1");
					break;

	  			case 2:
	  				put_stat(MAIN, "BANK2  BANK2");
					break;

	  			case 3:
	  				put_stat(MAIN, "BANK3  BANK3");
					break;

	  			case 4:
	  				put_stat(MAIN, "BANK4  BANK4");
					break;

	  			case 5:
	  				put_stat(MAIN, "BANK5  BANK5");
					break;

	  			case 6:
	  				put_stat(MAIN, "BANK6  BANK6");
					break;

	  			case 7:
	  				put_stat(MAIN, "BANK7  BANK7");
					break;

	  			case 8:
	  				put_stat(MAIN, "BANK8  BANK8");
					break;

	  			case 9:
	  				put_stat(MAIN, "BANK9  BANK9");
					break;

	 			default:
	  				put_stat(MAIN, "BANK ERROR");
	  				b = 4;
	 				break;
	 			}
	  			break;

			case DUPchr:												// duplex button, initial press
				if(offs_time(0)){
					offs_time(0xff);									// cancel duplex adjust
					send_stat(band_focus, 'S', rsbuf);					// send duplex CAT update
				}else{
					if(band_focus == MAIN_MODE){
						i = inc_dplx(MAIN);								// advance the duplex (function returns changed status)
					}else{
						i = inc_dplx(SUB);
					}
					disp_duplex(band_focus, i);							// update LCD
				}
				// if tx
				force_push();											// force update to NVRAM
				cntxt_stat(band_focus);									// status update
				break;

			case DUPchrS:												// force duplex button, initial press
				if(offs_time(0)){
					offs_time(0xff);									// cancel duplex adjust
				}else{
					write_dplx(band_focus, 0);							// set simplex
					i = read_dplx(band_focus);							// get new duplex setting
					disp_duplex(band_focus, i);							// update LCD
				}
				// if tx
				force_push();											// force update to NVRAM
				break;

			case DUPchrM:												// force duplex button, initial press
				if(offs_time(0)){
					offs_time(0xff);									// cancel duplex adjust
				}else{
					write_dplx(band_focus, DPLX_M);						// set minus
					i = read_dplx(band_focus);							// get new duplex setting
					disp_duplex(band_focus, i);							// update LCD
				}
				// if tx
				force_push();											// force update to NVRAM
				break;

			case DUPchrP:												// force duplex button, initial press
				if(offs_time(0)){
					offs_time(0xff);									// cancel duplex adjust
				}else{
					write_dplx(band_focus, DPLX_P);						// set plus
					i = read_dplx(band_focus);							// get new duplex setting
					disp_duplex(band_focus, i);							// update LCD
				}
				// if tx
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
				// if tx
				force_push();											// force update to NVRAM
				break;

			case RMAINchr:												// restore force-MAIN button, initial press if already SUB, nop
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					if(ssubfl != MAIN_MODE){
						band_focus = SUB_MODE;
						asub(1);
						ats(read_dplx(SUB) & TSA_F);					// TS flag updates based on focus
						sub_time(1);									// set sub timer
						k = get_band_index(band_focus);					// reset focus pointer
						i = get_lohi(band_focus, 0xff);
						update_lcd(band_focus, SUB);
						if(xmodeq & TEXTS_SLIDE){
							update_radio_all(SUB_VQ);					// don't update freq if slider operating
						}else{
							update_radio_all(SUB_ALL);					// update all
						}
						b = 0;											// no beep
					}
				}
				break;

			case MMAINchr:												// force-MAIN button, initial press if already SUB, nop
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					ssubfl = band_focus;								// save previous focus
					if(band_focus != MAIN_MODE){
						band_focus = MAIN_MODE;
						asub(0);
						ats(read_dplx(MAIN) & TSA_F);
						sub_time(0xff);									// clear sub timer
						k = get_band_index(band_focus);					// reset focus pointer
						i = get_lohi(band_focus, 0xff);
						update_lcd(band_focus, SUB);
						if(xmodeq & TEXTS_SLIDE){
							update_radio_all(SUB_VQ);					// don't update freq if slider operating
						}else{
							update_radio_all(SUB_ALL);					// update all
						}
						b = 0;											// no beep
					}
				}
				break;

			case RSUBchr:												// restore force-SUB button, initial press if already SUB, nop
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					if(ssubfl == MAIN_MODE){
						band_focus = MAIN_MODE;
						asub(0);
						ats(read_dplx(MAIN) & TSA_F);
						sub_time(0xff);									// clear sub timer
						k = get_band_index(band_focus);					// reset focus pointer
						i = get_lohi(band_focus, 0xff);
						update_lcd(band_focus, SUB);
						if(xmodeq & TEXTS_SLIDE){
							update_radio_all(SUB_VQ);					// don't update freq if slider operating
						}else{
							update_radio_all(SUB_ALL);					// update all
						}
						b = 0;											// no beep
					}
				}
				break;

			case SSUBchr:												// force-SUB button, initial press if already SUB, nop
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					ssubfl = band_focus;								// save previous focus
					if(band_focus == MAIN_MODE){
						band_focus = SUB_MODE;
						asub(1);
						ats(read_dplx(SUB) & TSA_F);					// TS flag updates based on focus
						sub_time(1);									// set sub timer
						k = get_band_index(band_focus);					// reset focus pointer
						i = get_lohi(band_focus, 0xff);
						update_lcd(band_focus, SUB);
						if(xmodeq & TEXTS_SLIDE){
							update_radio_all(SUB_VQ);					// don't update freq if slider operating
						}else{
							update_radio_all(SUB_ALL);					// update all
						}
						b = 0;											// no beep
					}
				}
				break;

			case SUBchr:												// SUB button, initial press
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					if(band_focus == MAIN_MODE){
						band_focus = SUB_MODE;
						asub(1);
						ats(read_dplx(SUB) & TSA_F);					// TS flag updates based on focus
						sub_time(1);									// set sub timer
					}else{
						band_focus = MAIN_MODE;
						asub(0);
						ats(read_dplx(MAIN) & TSA_F);
						sub_time(0xff);									// clear sub timer
					}
					k = get_band_index(band_focus);						// reset focus pointer
					i = get_lohi(band_focus, 0xff);
					update_lcd(band_focus, SUB);
					if(xmodeq & TEXTS_SLIDE){
						update_radio_all(SUB_VQ);						// don't update freq if slider operating
					}else{
						update_radio_all(SUB_ALL);						// update all
					}
/*					if(i){												// HILO updates based on focus
						alow(1);
					}else{
						alow(0);
					}*/
				}else{
					b = 0;	// no beep
				}
				break;

			case MHZchr:												// MHZ button 1st press
				if(!(xmode[k] & MC_XFLAG) || (xmodeq & OFFS_XFLAG)){	// if in mem/call mode, go to string slide mode check...
					if(!mhz_time(0) && !offs_time(0)){					// if timers are not zero, one of the MHz modes is active
						if(maddr == MHZ_OFF){							// this means that the thumbwheel mode isn't active and MHz mode is off
							maddr = MHZ_ONE;							// set MHz mode
							amhz(1);									// turn on mhz icon
						}else{
							maddr = MHZ_OFF;							// if any other MHz mode active, turn it off
							amhz(0);									// turn off mhz icon
						}
					}else{
						sii = 1;
						for(i=0; i<maddr ; i++){						// construct the multiplier for the currently selected digit
							sii *= 10;
						}
						if((maddr & MHZ_MASK) == 0) sii = 5;			// lowest digit can only be 0 or 5 (these are all 5KHz stepped radios, except for the UX129 which is a 10 KHz step)
						set_mhz_step(sii);								// store the step multiplier
//						else set_mhz_step(sii / 10L);
//						i = set_mhz_addr(0xff);
						if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,0); // un-blink the old digit (m/s)
						else digblink(maddr,0);
						i = maddr;
						if(((--maddr) & MHZ_MASK) == MHZ_MASK){			// move the digit and process roll-under
							if(get_band_index(band_focus) == ID1200_IDX){
								if((maddr & ~MHZ_MASK) == ~MHZ_MASK){
									maddr = 5;
								}else{
									maddr = (i & ~MHZ_MASK) | 5;
								}
								set_mhz_step(100000L);
							}else{
								if((maddr & ~MHZ_MASK) == ~MHZ_MASK){
									maddr = 4;
								}else{
									maddr = (i & ~MHZ_MASK) | 4;
								}
								set_mhz_step(10000L);
							}
						}
						if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,1); // blink the new digit (m/s)
						else digblink(maddr,1);
					}
				}else{
					// toggle mem string disp mode
					set_slide(band_focus, 0xff);
/*					if(band_focus == MAIN){
						if(xmodeq & TEXTM_SLIDE) xmodeq &= ~TEXTM_SLIDE;
						else xmodeq |= TEXTM_SLIDE;
						vfo_display |= MAIN;
					}else{
						if(xmodeq & TEXTS_SLIDE) xmodeq &= ~TEXTS_SLIDE;
						else xmodeq |= TEXTS_SLIDE;
						vfo_display |= SUB_D;
					}*/
				}
				break;

			case MHZchr_H:												// MHZ button, hold (this enters thumbwheel mode)
				if(!(xmode[k] & MC_XFLAG)){								// mem/call, ignore
					amhz(0);											// turn off mhz icon
//					set_mhz_addr(0);
					if(!mhz_time(0)){									// not in thumbwheel mode
						mhz_time(1);									// start timer
						set_mhz_step(100000L);							// set start step
						if(get_band_index(band_focus) == ID1200_IDX){
							maddr = 5;
						}else{
							maddr = 4;
						}
						if(band_focus == MAIN_MODE) digblink(MAIN_CS|maddr,1); // blink the 1st digit (m/s)
						else digblink(maddr,1);
						temp_vfo(band_focus);							// copy vfo -> vfot
					}else{												// already in thumbwheel mode (this will cancel the thumbwheel mode)
//						i = set_mhz_addr(0xff);
						if(xmodeq & OFFS_XFLAG){						// offset is differentiated from VFO frequency
							if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0); // blkin digit off
							else digblink(maddr&(~MHZ_OFFS),0);
							mhz_time(0xff);								// clear timers
							offs_time(0xff);
							maddr = MHZ_OFF;							// turn off thumbwheel mode
							send_stat(band_focus, 'O', rsbuf);			// send CAT update msg
						}else{
							if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),0);
							else digblink(maddr&(~MHZ_OFFS),0);
							copy_vfot(band_focus);						// copy updated temp vfo to normal vfo
							set_mhz_step(5L);
							mhz_time(0xff);								// clear timers
							offs_time(0xff);
							maddr = MHZ_OFF;
							send_stat(band_focus, 'F', rsbuf);			// send CAT update msg
						}
						force_push();									// force update to NVRAM
					}
					b = 2;	// 2beeps
				}
				break;

			case Vupchr:												// volume increase, initial press... VOL/SQU commandeer the SRF meters and mem ch digit to display level graphics
				v_time(1);
				xmodeq |= VOL_XFLAG;
				adjust_vol(band_focus, 1);
				ssmet(0x80 | adjust_vol(band_focus, 0), 0);				// signal a volume update
				force_push();											// force update to NVRAM
				break;

			case Vdnchr_H:												// VOL-, hold mutes the audio
				adjust_vol(band_focus, 0x81);							// set vol = 1 (then to 0 below...)
			case Vupchr_H:												// VOL+ hold displays the current level without change
				b = 2;	// 2beeps
			case Vdnchr:												// VOL-, initial press decreases the VOL level 1 step
				v_time(1);
				xmodeq |= VOL_XFLAG;
				adjust_vol(band_focus, -1);
				ssmet(0x80 | adjust_vol(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case Qupchr_H:												// SQU+, hold sets max squelch
				adjust_squ(band_focus, 0x80 | (LEVEL_MAX - 1));			// set squ = max-1 (then to max below...)
			case Qdnchr_H:												// SQU- hold displays the current level without change
				b = 2;	// 2beeps
			case Qupchr:												// SQU+, initial press increases the SQU level 1 step
				q_time(1);
				xmodeq |= SQU_XFLAG;
				adjust_squ(band_focus, 1);
				msmet(0x80 | adjust_squ(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case Qdnchr:												// SQU-, initial press decreases the SQU level 1 step
				q_time(1);
				xmodeq |= SQU_XFLAG;
				adjust_squ(band_focus, -1);
				msmet(0x80 | adjust_squ(band_focus, 0), 0);
				force_push();											// force update to NVRAM
				break;

			case SMUTEchr_H:											// SMUTE hold sets mute of main-band audio if sub muted (or no action)
				mmute_action(&mute_mode);
				if(!(mute_mode & SUB_MUTE)){
					smute_action(&mute_mode);
				}
				b = 2;	// 2beeps
				break;

			case SMUTEchr:												// SMUTE initial press toggles mute of sub-band audio (unmutes main if previously muted)
				if(mute_mode & MS_MUTE){
					mmute_action(&mute_mode);
				}
				smute_action(&mute_mode);
				break;

			case TONEchr:												// TONE, initial: toggle tone on/off or cancel adj mode (uses xmode to display tone freq in VFO space)
				if(xmodeq & TONE_XFLAG){									// tone mode already on, cancel it
					if(band_focus == MAIN){
						vfo_display |= MAIN;							// update main VFO
					}else{
						vfo_display |= SUB_D;							// update sub VFO
					}
					xmodeq &= ~TONE_XFLAG;
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
				send_stat(band_focus, 'S', rsbuf);
				break;

			case TONEchr_H:												// TONE hold, adjust tone
				vfo_display |= VMODE_TDISP;								// force tone freq to display
				xmodeq |= TONE_XFLAG;									// enable tone xmode
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
				b = 2;	// 2beeps
				break;

			case TSchr_H:												// TS hold, set-mode for TS_A/B (this is preliminary, uses beeps as feedback)
				if(read_tsab(band_focus, TSB_SEL) == TS_10){
//					set_tsab(band_focus, TSB_SEL, TS_25);
					tsdisplay = 1;
				}else{
					if(read_tsab(band_focus, TSA_SEL) == TS_5){
//						set_tsab(band_focus, TSA_SEL, TS_10);
						tsdisplay = 2;
					}else{
//						set_tsab(band_focus, TSA_SEL, TS_5);
//						set_tsab(band_focus, TSB_SEL, TS_10);
						tsdisplay = 3;
					}
				}
				ts_time(1);												// set timeout
				vfo_display |= VMODE_TSDISP;							// set display
				togg_tsab(band_focus);									// undo the toogle from initial press
				b = 2;	// 2beeps
				break;

			case TSchr:													// TS, initial press: freq step mode toggle
				if(tsdisplay){
					ts_time(0xff);										// turn off TS adj mode
				}else{
					togg_tsab(band_focus);
				}
				break;

			case DUPchr_H:												// duplex button HOLD (offset adjust)
				// undo the duplex press
				if(band_focus == MAIN_MODE){
					inc_dplx(MAIN);
					i = inc_dplx(MAIN);
					mfreq((U32)get_offs(band_focus), LEAD0);
				}else{
					inc_dplx(SUB);
					i = inc_dplx(SUB);
					sfreq((U32)get_offs(band_focus), LEAD0);
				}
				disp_duplex(band_focus, i);								// update LCD
				mhz_time(1);
				offs_time(1);
				set_mhz_step(100000L);
				if(get_band_index(band_focus) == ID1200_IDX){
					maddr = MHZ_OFFS|4;
				}else{
					maddr = MHZ_OFFS|3;
				}
				if(band_focus == MAIN_MODE) digblink(MAIN_CS|(maddr&(~MHZ_OFFS)),1);
				else digblink(maddr&(~MHZ_OFFS),1);
				xmodeq |= OFFS_XFLAG;
//				update_lcd(band_focus, band_focus);
				b = 2;	// 2beeps
				break;

			case VFOchr:												// VFO, initial press: cycles selected band modules (this is the BAND button on the IC-901)
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
					set_next_band(band_focus);
					update_lcd(band_focus, band_focus);
					if(band_focus == MAIN) i = MAIN_ALL;
					else i = SUB_ALL;
					update_radio_all(i);
					force_push();										// force update to NVRAM
					set_bandnv();
				}else{
					b = 0;	// no beep
				}
				cntxt_stat(band_focus);									// status update
				break;

			case MSchr:
				if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
//					mute_radio(0);										// mute radio during band-swap
					set_swap_band();
					if(mute_mode & SUB_MUTE){
						adjust_vol(MAIN, 0);							// restore main vol
					}
					update_lcd(band_focus, MAIN);
					update_lcd(band_focus, SUB);
					update_radio_all(UPDATE_ALL);
					force_push();										// force update to NVRAM
				}else{
					b = 0;	// no beep
				}
				cntxt_stat(MAIN);										// status update, both bands
				cntxt_stat(SUB);
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
					cntxt_stat(band_focus);										// send a context update
				}
				break;

			case CHKchr_H:												// CHECK hold: this opens the SQU
				// if chkmode active, this is the hold from that initiation, so go to SQU open state (else, ignore)
				if(chkmode){
					chkmode |= REV_SQU_FLAG;
					// !!! open squelch
					chksqu = adjust_squ(band_focus, 0);
					adjust_squ(band_focus, 0x80);						// open up squ (set to zero)
					b = 2;	// 2beeps
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
				cntxt_stat(band_focus);										// send a context update
				b = 0;	// no beeps
				break;

			case MRchr_H:												// Mem-mode recall last cos active band/mem
				// if !mem mode, abort
				if(!(xmode[k] & MEM_XFLAG)){							// enter mem mode if off
					if((xmode[k] & (MC_XFLAG)) == 0){					// copy vfo to temp
						copy_vfo2temp(band_focus);
					}
					mema(band_focus, 1);								// turn on "M"
					mskpa(band_focus, get_scanmem(band_focus));			// update "skp" annunc.
					xmode[k] |= MEM_XFLAG;
				}
				i = get_last_cos(band_focus);
//				j = i >> 5;
				i &= 0x1f;
				read_mem(band_focus, get_memnum(band_focus, 0));
				write_xmode(band_focus);
				update_lcd(band_focus, band_focus);
				if(band_focus == MAIN) i = MAIN_ALL;
				else i = SUB_ALL;
				update_radio_all(i);
				cntxt_stat(band_focus);										// send a context update
				break;

			case MRchr:													// Mem-mode toggle: this toggles between mem and vfo mode
				// if mem mode, turn off
//				if(xmode[k] & CALL_XFLAG) xmode[k] &= ~(MC_XFLAG);		// turn off call, force M on
				if((xmode[k] & MC_XFLAG) == MEM_XFLAG){					// if mem mode, turn off and go to VFO
					mema(band_focus, 0);								// turn off "M"
					xmode[k] &= ~MC_XFLAG;								// turn off mem/call mode
//					if((xmode[k] & (MC_XFLAG)) == 0){
						copy_temp2vfo(band_focus);
//					}
				}else{
					if((xmode[k] & (MC_XFLAG)) == 0){					// if not call mode, copy vfo to temp
						copy_vfo2temp(band_focus);
					}
					mema(band_focus, 1);								// turn on "M"
					mskpa(band_focus, get_scanmem(band_focus));			// update "skp" annunc.
					xmode[k] |= MEM_XFLAG;
					xmode[k] &= ~CALL_XFLAG;
					read_mem(band_focus, get_memnum(band_focus, 0));
				}
				if(band_focus == MAIN){
					iflags |= SIN_MSRF_F;								// force update of MSRF
				}else{
					iflags |= SIN_SSRF_F;								// force update of SSRF
				}
				write_xmode(band_focus);
				update_lcd(band_focus, band_focus);
				if(band_focus == MAIN) i = MAIN_ALL;
				else i = SUB_ALL;
				update_radio_all(i);
				cntxt_stat(band_focus);									// status update
				send_stat(band_focus, 'T', stat_str);
				break;

				// CALL/MR/VFO operation:
				//		  __________________________________
				//       v                                  v
				//		[MEM]  <==s/r==> [VFO] <==s/r==>  [CALL]		s/r = save VFO moving to CALL/MEM, restore VFO moving from CALL/MEM
				//		 ^                                  ^				No VFO save/restore moving between CALL and MEM
				//		 |__________________________________|
				//

			case CALLchr:												// call-mode toggle: this toggles between call mode
				if((xmode[k] & MC_XFLAG) == CALL_XFLAG){				// if call mode...
					xmode[k] &= ~MC_XFLAG;								// turn off mem/call mode
					copy_temp2vfo(band_focus);							// recall VFO
				}else{
					// call mode coming on...
					mema(band_focus, 0);								// turn off "M" indicator
					if((xmode[k] & (MC_XFLAG)) == 0){					// copy vfo state to temp only if not done already
						copy_vfo2temp(band_focus);
					}
					xmode[k] |= CALL_XFLAG;								// turn on call mode
					xmode[k] &= ~MEM_XFLAG;								// turn on call mode
					read_mem(band_focus, get_callnum(band_focus, 0));	// recall VFO state
				}
				if(band_focus == MAIN){
					iflags |= SIN_MSRF_F;								// force update of MSRF
				}else{
					iflags |= SIN_SSRF_F;								// force update of SSRF
				}
				write_xmode(band_focus);
				update_lcd(band_focus, band_focus);						// update display & radio
				if(band_focus == MAIN) i = MAIN_ALL;
				else i = SUB_ALL;
				update_radio_all(i);
				cntxt_stat(band_focus);										// send a context update
				break;

			case CALLchr_H:												// call-mode hold: write VFO to call mem
				if(xmode[k] & CALL_XFLAG){
					copy_temp2vfo(band_focus);
					write_mem(band_focus, get_callnum(band_focus, 0));
					if(band_focus == MAIN){
						iflags |= SIN_MSRF_F;							// force update of MSRF
					}else{
						iflags |= SIN_SSRF_F;							// force update of SSRF
					}
					update_lcd(band_focus, band_focus);
					if(band_focus == MAIN) i = MAIN_ALL;
					else i = SUB_ALL;
					update_radio_all(i);
					b = 2;	// 2beeps
					cntxt_stat(band_focus);										// send a context update
				}
				break;

			case MWchr:													// Mem-write: arm for skip select
				if(xmode[k] & (MEM_XFLAG)){
					b = 1;	// 1beeps
					hflag = MWchr;
				}
				break;

			case MWchr_R:												// Mem-write: if armed for skip select, toggle skip status
				if(hflag == MWchr){
					i = togg_scanmem(band_focus);
					mskpa(band_focus, i);								// update "skp" annunc.
					if(xmodeq & (MSCANM_XFLAG|MSCANS_XFLAG)){
						get_mscan(band_focus);
					}
					b = 1;	// 1beeps
					hflag = 0;
				}
				break;

			case MWchr_H:												// Mem-write: write VFO to mem in VFO mode; if mem mode, exits with mem in VFO (copy mem to VFO)
				if(xmode[k] & (MC_XFLAG)){
					xmode[k] &= ~(MC_XFLAG);							// turn off call/mem, no VFO coppy-back
					mema(band_focus, 0);								// turn off "M"
					if(band_focus == MAIN){
						iflags |= SIN_MSRF_F;							// force update of MSRF
					}else{
						iflags |= SIN_SSRF_F;							// force update of SSRF
					}
					update_lcd(band_focus, band_focus);
				}else{
					write_mem(band_focus, get_memnum(band_focus, 0));
				}
				b = 2;	// 2beeps
				hflag = 0;
				break;

			case ZEROchr:												// DFE mode processing
			case ONEchr:
			case TWOchr:
			case THREEchr:
			case FOURchr:
			case FIVEchr:
			case SIXchr:
			case SEVENchr:
			case EIGHTchr:
			case NINEchr:
				if(!(xmodez & DFE_MODE)){
					xmodez |= DFE_MODE;
					dfe_vfo = 0;
				}
				if(band_focus){
//					mset_500hz(2);
					mdp2_blink(1);
				}else{
					sdp2_blink(1);
				}
				dfe_time(1);											// set/reset TO timer
				// !- do not place a break here -!
			case DOTchr:
				if(xmodez & DFE_MODE){
					b = process_DFE(band_focus, i);
				}else{
					b = 0;
				}
				if(i == DOTchr){
					if(band_focus){
						mdp2_blink(0);
					}else{
						sdp2_blink(0);
					}
				}
				break;

			case FDchr_H:
  				// bflags hold the pttsub action status.  read the current bflags
  				//	and clear the status
  				i = get_bflag(MAIN, 0, 0);
  				i = i & ~PTTSUB_M;				// clear the bits with the mask define
  				get_bflag(MAIN, 1, i);			// store the new setting
				put_stat(MAIN, "PTTSUB OFF");
  				// 1 response beep
  				b = 2;
  				break;

			case FDchr:
  				// only act on press cmd
  				// bflags hold the pttsub action status.  read the current bflags
  				//	and advance the status
  				i = get_bflag(MAIN, 0, 0);
  				j = i & PTTSUB_M;
  				j += PTTSUB_MOD1;
  				i = i & ~PTTSUB_M;
  				get_bflag(MAIN, 1, i | j);
  				// response beeps indicate status
  				switch(j){
  				case PTTSUB_MOD0:
  					put_stat(MAIN, "PTTSUB OFF");
  					b = 1;
					break;

  				case PTTSUB_MOD1:
  					put_stat(MAIN, "PTTSUB SMUTE");
  					b = 2;
  					break;

 				case PTTSUB_MOD2:
  					put_stat(MAIN, "PTTSUB SCALL");
  					b = 3;
					break;

 				default:
  				case PTTSUB_MOD3:
  					put_stat(MAIN, "PTTSUB MCALL");
  					b = 4;
 					break;
 				}
				b = 1;
				break;

			case ENTchr:
				if(band_focus){
					mdp2_blink(0);
				}else{
					sdp2_blink(0);
				}
				if(xmodez & DFE_MODE){
					if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
						j = get_modulid(dfe_vfo / 1000L);
						if(band_focus == MAIN){
							// process for MAIN
							m = get_modulid(get_vfo(MAIN) / 1000L);
							if(j != m){
								l = get_modulid(get_vfo(SUB) / 1000L);
								if(j == l){
									// swap
									set_swap_band();
									if(mute_mode & SUB_MUTE){
										adjust_vol(MAIN, 0);							// restore main vol
									}
									update_lcd(band_focus, MAIN);
									update_lcd(band_focus, SUB);
									update_radio_all(UPDATE_ALL);
									force_push();										// force update to NVRAM
									// keep same band
									set_dfe(band_focus, 0, k);
									b = 2;
								}else{
									// call up new band (if installed)
									if(switch_band(band_focus)){
										set_boff(band_focus, m-1);
										set_dfe(band_focus, 0, k);
										update_lcd(band_focus, MAIN);
										update_lcd(band_focus, SUB);
										update_radio_all(UPDATE_ALL);
										force_push();										// force update to NVRAM
										b = 2;
									}
									// error
									else b = 4;
								}
							}else{
								// keep same band
								set_dfe(band_focus, 0, k);
								update_lcd(band_focus, MAIN);
								update_lcd(band_focus, SUB);
								update_radio_all(UPDATE_ALL);
								force_push();										// force update to NVRAM
								b = 2;
							}
						}else{
							// process for SUB
							m = get_modulid(get_vfo(SUB) / 1000L);
							if(j != m){
								l = get_modulid(get_vfo(MAIN) / 1000L);
								if(j == l){
									// swap
									set_swap_band();									// swap the bids between main and sub
									if(mute_mode & SUB_MUTE){
										adjust_vol(MAIN, 0);							// restore main vol
									}
									update_lcd(band_focus, MAIN);
									update_lcd(band_focus, SUB);
									update_radio_all(UPDATE_ALL);
									force_push();										// force update to NVRAM
									// keep same band
									set_dfe(band_focus, 0, k);
									b = 2;
								}else{
									// call up new band (if installed)
									if(switch_band(band_focus)){
										set_boff(band_focus, m-1);
										set_dfe(band_focus, 0, k);
										update_lcd(band_focus, MAIN);
										update_lcd(band_focus, SUB);
										update_radio_all(UPDATE_ALL);
										force_push();										// force update to NVRAM
										b = 2;
									}
									// error
									else b = 4;
								}
							}else{
								// keep same band
								set_dfe(band_focus, 0, k);
								update_lcd(band_focus, MAIN);
								update_lcd(band_focus, SUB);
								update_radio_all(UPDATE_ALL);
								force_push();										// force update to NVRAM
								b = 2;
							}
						}
					}
					cntxt_stat(band_focus);												// send a context update
				}else{
					b = 0;
				}

				process_DFE(band_focus, ENTchr);									// exit DFE mode
				break;

			default:
				b = 0;	// no beeps
				break;
			}
			if(b){
				if(xmodez & IPL_BOOT){
					ipl_cancel();
//					xmodez &= ~IPL_BOOT;											// clear bootflag
//					vfo_display |= MAIN | SUB_D;									// force vfo update on m/s
				}
			}
			// process beeps
			if(bb){
				b = bb;				// bb takes priority
				bb = 0;
			}
			switch(b){
			case 1:
				do_1beep();
				break;

			case 2:
				do_2beep();
				break;

			case 3:
				do_3beep();
				break;

			case 4:
				do_4beep();
				break;

			default:
				break;
			}
		}
	}
	// if DFE mode, intercept timeout
	if((xmodez & DFE_MODE) && !dfe_time(0)){
		// cancel dfe mode
		process_DFE(band_focus, DOTchr);
		// undo blink
		if(band_focus){
//			mset_500hz(0);
			mdp2_blink(0);
		}else{
			sdp2_blink(0);
		}
		// set cancel beeps
		do_3beep();
	}
	// if STAT mode, intercept timeout
	if((xmodez & (STATM_MODE|STATS_MODE)) && !dfe_time(0)){
		// cancel dfe mode
		process_STAT(band_focus, DOTchr);
		// set cancel beeps
		do_3beep();
	}
	// check for IPL timeout
	if(xmodez & IPL_BOOT){
		if(!ipl_time(0)){
			ipl_cancel();
		}
	}
	return band_focus;
}	// end process_MS()


//-----------------------------------------------------------------------------
// switch_band() performs band swap
//	get bandid of new freq (or error)
//	is band present (error if not)
//	do the swap
//	return BANDID or BANDOFF (error)
//-----------------------------------------------------------------------------
U8 switch_band(U8 focus){
	U8	i;		// temps
	U8	j;
	U8	k;

	i = get_modulid(dfe_vfo/1000L) - 1;			// calc bandid of dfe vfo
	k = set_bit(i);								// convert to bitmap
	j = get_present();							// get bandids present
	// if the new vfo belongs to an installed band module, set up the swap
	if(k & j){
		// freq is valid and band is present
		if(!(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT|NO_SUX_PRSNT))){ // only allow sub button if there are no errors
			set_bandid(focus, i);
			update_lcd(focus, focus);
			if(focus == MAIN) j = MAIN_ALL;
			else j = SUB_ALL;
			update_radio_all(j);
			force_push();						// force update to NVRAM
			set_bandnv();
			set_dfe(focus, 1, i);
		}else{
			i = BANDOFF;						// err
		}
	}else{
		// not valid
		i = BANDOFF;
	}
	return i;
}

//-----------------------------------------------------------------------------
// ipl_cancel() clears IPL mode
//-----------------------------------------------------------------------------
void ipl_cancel(void){

	xmodez &= ~IPL_BOOT;										// clear bootflag
	vfo_display |= MAIN | SUB_D;								// force vfo update on m/s
	return;
}

//-----------------------------------------------------------------------------
// process_STAT processes ASCII status mode
//-----------------------------------------------------------------------------
U8 process_STAT(U8 focus, U8 keychr){
//	U32	d;			// digit temp
	U8	b = 1;		// beep count

	switch(keychr){
	case ENTchr:								// enter, process to VFO
		// determine band of new freq and test to see if band module present.  if so, switch to the new band and xfr the VFO
		xmodez &= ~(STATM_MODE|STATS_MODE);		// exit
		b = 2;
		break;

	case DOTchr:
		xmodez &= ~(STATM_MODE|STATS_MODE);		// cancel
		b = 3;
		break;

	case 0xff:									// nop, just trigger vfo update
		break;

	default:
/*		d = keychr - '0';						// convert digit to numeral
		dfe_vfo *= 10;							// slide new digit into temp
		dfe_vfo += d;
		if(dfe_vfo > MAX_VFO) dfe_vfo = 0;		// range limit*/
		break;
	}
	if(focus == MAIN){							// trigger DU update
		vfo_display |= MAIN;
	}else{
		vfo_display |= SUB_D;
	}
	return b;
}

//-----------------------------------------------------------------------------
// set_dfe() sets radio to dfe vfo
// !!! so far... this works if in vfo, mem, & call modes.  However, there seems to be a snafu
//	when jumping from mem to/fr call whereby the call freq ends up in the VFO.  Not sure if this
//	is related to DFE... !!!
//-----------------------------------------------------------------------------
void set_dfe(U8 focus, U8 bandsw, U8 curband){
	U8	i;		// temp

	// if mem mode, turn off
	i = xmode[curband];
	if(i & CALL_XFLAG) xmode[curband] &= ~(MC_XFLAG); // turn off call
	if(i & MEM_XFLAG){
		mema(focus, 0);						// turn off "M"
		xmode[curband] &= ~MEM_XFLAG;
	}
	copy_2vfo(focus, dfe_vfo);
	write_xmode(focus);
	update_lcd(focus, focus);
	if(focus == MAIN) i = MAIN_ALL;
	else i = SUB_ALL;
	update_radio_all(i);

/*	if(bandsw){								// band switch
		force_push();						// force update to NVRAM
		set_bandnv();
	}*/
	// xfr dfe to vfo
//	rflags |= SIN_SSRF_F;					// force update of SSRF
//	save_mc(focus);							// save changes to NVRAM
//	update_lcd(focus, SUB);
//	update_radio_all(SUB_ALL);
	return;
}

//-----------------------------------------------------------------------------
// process_DFE processes MFmic numeric keys and updates dfe_vfo
//	freq entry starts with msdig and proceeds until the KHz digit is entered.
//	MFmic "#" button is "ENTER" and "*" button is "CANCEL"
//-----------------------------------------------------------------------------
U8 process_DFE(U8 focus, U8 keychr){
	U32	d;		// digit temp
	U8	b = 1;		// beep count

	switch(keychr){
	case ENTchr:								// enter, process to VFO
		// determine band of new freq and test to see if band module present.  if so, switch to the new band and xfr the VFO
		xmodez &= ~DFE_MODE;					// exit
		b = 2;
		break;

	case DOTchr:
		xmodez &= ~DFE_MODE;					// cancel
		b = 3;
		break;

	case 0xff:									// nop, just trigger vfo update
		break;

	default:
		d = keychr - '0';						// convert digit to numeral
		dfe_vfo *= 10;							// slide new digit into temp
		dfe_vfo += d;
		if(dfe_vfo > MAX_VFO) dfe_vfo = 0;		// range limit
		break;
	}
	if(focus == MAIN){							// trigger DU update
		vfo_display |= MAIN;
	}else{
		vfo_display |= SUB_D;
	}
	return b;
}

//-----------------------------------------------------------------------------
// disp_duplex() updates duplex LCD icons based on duplex and focus
//	duplex is the return value of the last call to inc_dplx()
//-----------------------------------------------------------------------------
void disp_duplex(U8 focus, U8 duplex){
	if(focus == MAIN_MODE){
		switch(duplex & (DPLX_MASK)){
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
		switch(duplex & (DPLX_MASK)){
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
	return;
}

//-----------------------------------------------------------------------------
// process_VFODISP() process freq display options
//-----------------------------------------------------------------------------
void process_VFODISP(U8 focus){
	U8	i;			// temps
	U8	j = 0;
	U16	ii;
	char tbuf[8];	// ascii freq field buffer

	if(vfo_display){
		// calculate display mode for switch() frame:				// This makes it easier to maintain the VFO display modes (now and future)
		if(xmodez & (DFE_MODE | STATM_MODE | STATS_MODE)){
			if(xmodez & DFE_MODE){
				j = DFE_DISP;
			}else{
				j = STAT_DISP;
			}
		}else{
			if(xmodeq & (TONE_XFLAG|OFFS_XFLAG)){
				j = TONE_DISP;
			}else{
				if(tsdisplay){
					j = TS_DISP;
				}else{
					if(focus == MAIN) i = TEXTM_SLIDE;
					else i = TEXTS_SLIDE;
					if(!(xmodeq & i)) j = NORM_DISP;
				}
			}
		}
		// dispatch switch() - each "case" is a display mode
		switch(j){
		case TONE_DISP:
			if(xmodeq & TONE_XFLAG){
				// TONE adjust mode
				if(vfo_display & VMODE_TDISP){						// if triggered
					i = adjust_tone(focus, 0) & CTCSS_MASK;
					if((i > TONE_MAX) || (i == 0)){
						i = 1;										// error trap
					}
					ii = tone_list[i - 1];							// tone list is "0" origin, tones values are "1" origin
					sprintf(tbuf,"%4d  ", ii);						// convert number to display string
					puts_lcd(focus, tbuf, 1);
					vfo_display &= ~VMODE_TDISP;					// clear update trigger
					force_push();									// force update to NVRAM
				}
//			}else{
				// offset adjust mode

			}
			break;

		case TS_DISP:
			switch(tsdisplay){
			case 1:
				puts_lcd(focus, "A 5B10", 1);
				break;

			case 2:
				puts_lcd(focus, "A 5B25", 1);
				break;

			case 3:
				puts_lcd(focus, "A10B25", 1);
				break;

			default:
				puts_lcd(focus, "A--B--", 1);
				break;
			}
			vfo_display &= ~(VMODE_TSDISP);
			break;

		case DFE_DISP:
			// direct freq entry VFO display
			if(vfo_display & (MAIN|SUB_D)){
				if(focus == MAIN){
					mfreq(dfe_vfo, 0);							// update main freq display from vfo or vfotr
					vfo_display &= ~(VMODE_ISTX | MAIN);
				}else{
					if(!(xmodeq & TEXTS_SLIDE)){
						sfreq(dfe_vfo, 0);
					}
					vfo_display &= ~(SUB_D);
				}
			}
			break;

		case STAT_DISP:
			// status display
			break;

		case NORM_DISP:
			// normal VFO display
//				sprintf(dgbuf,"vfodisp: %02x",vfo_display); //!!!
//				putsQ(dgbuf);
			if(vfo_display & MAIN){
				if(ptt_change & PTT_KEYED) i = MAIN | VMODE_ISTX;
				else i = MAIN;
				mfreq(get_freq(i), 0);							// update main freq display from vfo or vfotr
				vfo_display &= ~(VMODE_ISTX | MAIN);

/*				if(vfo_display & VMODE_ISTX){
					mfreq(get_freq(MAIN | VMODE_ISTX), 0);		// update main freq display from vfotr
//						sprintf(dgbuf,"vfofrqT: %d",get_freq(focus | 0x80)); //!!!
//						putsQ(dgbuf);
				}else{
					mfreq(get_freq(MAIN), 0);					// update main freq display
//						sprintf(dgbuf,"vfofrqR: %d",get_freq(focus)); //!!!
//						putsQ(dgbuf);
				}
				vfo_display &= ~(VMODE_ISTX | MAIN);*/
			}else{
				if(vfo_display & SUB_D){
					if(!(xmodeq & TEXTS_SLIDE)){
						sfreq(get_freq(SUB), 0);
					}
//					sfreq(get_freq(SUB), 0);					// update sub freq display
					vfo_display &= ~(SUB_D);
				}
			}
			break;

		case TXTSLID_DISP:
			break;

		default:
			break;
		}
	}
	if(xmodeq & (TEXTM_SLIDE | TEXTS_SLIDE)){					// update main/sub slider displays
		// text slider display mode - always runs
		if(!slide_time(0)){										// slide_time advances the string 1 chr
			if(xmodeq & TEXTM_SLIDE){
				i = puts_slide(MAIN, get_nameptr(MAIN), SLIDE_RUN);
			}
			if(xmodeq & TEXTS_SLIDE){
				i = puts_slide(SUB, get_nameptr(SUB), SLIDE_RUN);
			}
			slide_time(1);
		}
	}else{
		if(xmodez & (STATM_MODE | STATS_MODE)){
			if(!slide_time(0)){
				if(xmodez & STATM_MODE){
					i = puts_slide(MAIN, stat_ptr, SLIDE_RUN);
				}
				if(xmodeq & STATS_MODE){
					i = puts_slide(SUB, stat_ptr, SLIDE_RUN);
				}
				slide_time(1);
			}
		}
	}
	return;
} // end process_VFODISP()

//-----------------------------------------------------------------------------
// test_for_cancel() looks for cancel keys based on mode registers
//	called from top of key dispatch tree
//-----------------------------------------------------------------------------
U8 test_for_cancel(U8 key){
	U8	i = key;		// temps
	U8	m = 0;
	U8	vector;

	if((maddr & ~MHZ_OFFS) < MHZ_OFF) m |= MHZ_CANCEL;		//	force the MHZ-hold key-code (this clears by-digit mode)
	if(chkmode) m |= CHK_CANCEL;							// for check mode, CHKchr is cancel
	if(xmodeq & TONE_XFLAG) m |= TONE_CANCEL;
	if(xmodeq & OFFS_XFLAG) m |= OFFS_CANCEL;
	if(xmodeq & (VOL_XFLAG|SQU_XFLAG)) m |= VQ_CANCEL;
	if(tsdisplay) m |= TS_CANCEL;
	if(m){
		vector = 0x80;
		while(!(m & vector)){
			vector >>= 1;
		}
		if(vector == MHZ_CANCEL){							//	force the MHZ-hold key-code (this clears by-digit mode)
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

			default:										// for all other keys, modify to abort MHz digit mode cancel
				i = MHZchr_H;
				break;
			}
		}
		if(vector == CHK_CANCEL){							// for check mode, CHKchr is cancel
			switch(key){									// process thumbwheel key-cancel
			case Vupchr:									// don't cancel on these codes:
			case Vdnchr:									// VOLu/d, SQUu/d, CHECK
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

			default:										// for all other keys, modify to chk/rev cancel
				i = CHKchr;
				break;
			}
		}
		if(vector == TONE_CANCEL){
			switch(key){									// process TONE adjust key-cancel
			case Vupchr:									// don't cancel on these codes:
			case Vdnchr:									// VOLu/d, SQUu/d, TONE
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
//			case TONEchr:
			case TONEchr_H:
			case TONEchr_R:
				break;

			default:										// for all other keys, modify to tone cancel
				i = TONEchr;
				break;
			}
		}
		if(vector == OFFS_CANCEL){
			switch(key){									// process OFFS adjust key-cancel
			case Vupchr:									// don't cancel on these codes:
			case Vdnchr:									// VOLu/d, SQUu/d, DUP
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
//			case DUPchr:
			case DUPchr_H:
			case DUPchr_R:
			case MHZchr:
			case MHZchr_H:
			case MHZchr_R:
				break;

			default:										// for all other keys, modify to tone cancel
				i = DUPchr;
				break;
			}
		}
		if(vector == VQ_CANCEL){
			switch(key){									// process V/Q adjust key-cancel
			case SUBchr:									// these key codes cancel VQ adj mode
			case MSchr:
			case VFOchr:
			case MRchr:
			case CALLchr:
			case SUBchr_H:
			case MSchr_H:
			case VFOchr_H:
			case MRchr_H:
			case CALLchr_H:
			case SUBchr_R:
			case MSchr_R:
			case VFOchr_R:
			case MRchr_R:
			case CALLchr_R:
				v_time(0xff);								// cancel vol/squ
				q_time(0xff);
				break;

			default:										// for all other keys, no V/Q cancel
				break;
			}
		}
		if(vector == TS_CANCEL){
			switch(key){									// process TS adjust key-cancel
			case SUBchr:									// these key codes cancel TS adj mode
			case MSchr:
			case VFOchr:
			case MRchr:
			case CALLchr:
			case TONEchr_H:
			case MHZchr:
			case SETchr:
			case DUPchr_H:
				ts_time(0xff);								// cancel TS adj
				break;

			default:										// for all other keys, no TS cancel
				break;
			}
		}
	}
	return i;
}	// test_for_cancel()

//-----------------------------------------------------------------------------
// mmute_action() toggles mute of the main-band
// smute_action() toggles mute of the sub-band
//-----------------------------------------------------------------------------
void mmute_action(U8* mute_flag){
	U8	i;		// temp

	if(*mute_flag & MS_MUTE){
		*mute_flag &= ~MS_MUTE;
		// unblink sub VFO
		lcd_buf[0] = (CS1_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | M0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<23; i++){
			lcd_buf[i] = WR_BMEM | 0x00;				// clear blink bits
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}else{
		*mute_flag |= MS_MUTE;
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
	mute_radio(*mute_flag);
	return;
}	// end mmute_action()

void smute_action(U8* mute_flag){
	U8	i;		// temp

	if(*mute_flag & SUB_MUTE){
		*mute_flag &= ~SUB_MUTE;
		// unblink sub VFO
		lcd_buf[0] = (CS2_MASK | 22);					// set data string preamble
		lcd_buf[1] = LOAD_PTR | S0_ADDR;				// set lcd pointer
		lcd_buf[2] = WITHOUT_DECODE;					// set lcd pointer
		for(i=3; i<23; i++){
			lcd_buf[i] = WR_BMEM | 0x00;				// clear blink bits
		}
		put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and decode on
	}else{
		*mute_flag |= SUB_MUTE;
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
	mute_radio(*mute_flag);
	return;
}	// end smute_action()

//-----------------------------------------------------------------------------
// process_DIAL() handles dial changes
//-----------------------------------------------------------------------------
U32 process_DIAL(U8 focus){
	U8	i;					// temp
	S8	j;					// command step, +/-1 = up/dn, 0 = no step.
	U8	k;					// band index
	U32	rflags = 0;			// return flags

	k = get_band_index(focus);
	// process dial
	j = is_mic_updn(0, focus, xmodeq) + get_dial(1);
	if(j){
		if(xmodez & IPL_BOOT){
			// cancel IPL mode
			ipl_cancel();
			j = 0;
		}else{
			// process dial & up/dn
			if((xmodeq & MSCANM_XFLAG) && (focus == MAIN)){
				doscan(MAIN, 0);
				set_slide(focus, 0);
				return rflags;
			}
			if((xmodeq & MSCANS_XFLAG) && (focus == SUB)){
				doscan(SUB, 0);
				set_slide(focus, 0);
				return rflags;
			}
			if(focus == SUB){
				sub_time(1);								// reset timeout
			}
		}
	}
//	if(xmodeq & (TONE_XFLAG|OFFS_XFLAG)){				// exception display for TONE or OFFS takes priority
	if(xmodeq & TONE_XFLAG){							// exception display for TONE or OFFS takes priority
		if(xmodeq & TONE_XFLAG){						// TONE
			if(j){
				adjust_tone(focus, j);
				vfo_display |= VMODE_TDISP;
			}
			j = 0;										// allow sub-scan to operate
//		}else{
//			freq_update(focus, j);						// perform updates for offset adjust
		}
	}else{
		if(tsdisplay){
			// freq step select mode
			if(j){
				if(read_tsab(focus, TSB_SEL) == TS_10){
					set_tsab(focus, TSB_SEL, TS_25);
					tsdisplay = 2;
				}else{
					if(read_tsab(focus, TSA_SEL) == TS_5){
						set_tsab(focus, TSA_SEL, TS_10);
						tsdisplay = 3;
					}else{
						set_tsab(focus, TSA_SEL, TS_5);
						set_tsab(focus, TSB_SEL, TS_10);
						tsdisplay = 1;
					}
				}
				vfo_display |= VMODE_TSDISP;
				ts_time(1);
			}
		}else{
			if(j){
				// if focus and mem mode == active:
				if((xmode[k] & (MC_XFLAG)) && !(xmodeq & OFFS_XFLAG)){
					// mem/call# inc/dec
					if(focus == MAIN){
						if(xmode[k] & CALL_XFLAG){
							get_callnum(focus, j);
						}else{
							get_memnum(focus, j);
						}
						rflags = SIN_MSRF_F;							// force update of MSRF
					}else{
						if(xmode[k] & CALL_XFLAG){
							get_callnum(focus, j);
						}else{
							get_memnum(focus, j);
						}
						rflags = SIN_SSRF_F;							// force update of MSRF
					}
					if(xmode[k] & CALL_XFLAG){
						read_mem(focus, get_callnum(focus, 0));			// read new call
					}else{
						read_mem(focus, get_memnum(focus, 0));			// read new mem
					}
					rflags |= SIN_SSRF_F;								// force update of SSRF
					save_mc(focus);										// save changes to NVRAM
					update_lcd(focus, focus);
					if(focus == MAIN) i = MAIN_ALL;
					else i = SUB_ALL;
					update_radio_all(i);
					cntxt_stat(focus);									// status update
					send_stat(focus, 'T', stat_str);
				}else{
					freq_update(focus, j);
					cntxt_stat(focus);									// status update
				}
			}
		}
	}
	if(j == 0){
		// main mem scan
		if(xmodeq & MSCANM_XFLAG){
			if(GPIO_PORTD_DATA_R & MTX_N){						// PTT active
				doscan(MAIN, 0);								// shut down scan
				set_slide(MAIN, 0);
				do_1beep();
			}else{
				if(!(GPIO_PORTC_DATA_R & MRX_N) || (xmodez & MSCANM1_XFLAG)){ // main COS inactive or 1st scan
					if(!scan_time(MAIN, 0) || (xmodez & MSCANM1_XFLAG)){
						// process if scan timer == 0
						xmodez &= ~MSCANM1_XFLAG;
						if(!nxtscan(MAIN, 1)){
							doscan(MAIN, 0);					// shut down scan, no mems enabled
							set_slide(MAIN, 0);
							do_3beep();
						}else{
							scan_time(MAIN, 1);					// reset scan timer
							read_mem(MAIN, get_memnum(MAIN, 0)); // read new mem
							rflags |= SIN_SSRF_F;				// force update of SSRF
							save_mc(MAIN);						// save changes to NVRAM
							update_lcd(focus, MAIN);
							update_radio_all(MAIN_ALL);
						}
					}
				}
			}
		}
		// sub mem scan...
		if(xmodeq & MSCANS_XFLAG){
			if(!(GPIO_PORTC_DATA_R & SRX_N) || (xmodez & MSCANS1_XFLAG)){					// sub COS inactive
				if(!scan_time(SUB, 0) || (xmodez & MSCANS1_XFLAG)){
					// process if scan timer == 0
					xmodez &= ~MSCANS1_XFLAG;
					if(scan_switch){
						scan_time(SUB, 4);							// reset scan timer after band edge (don't advance mem#)
						scan_switch = 0;							// disable edge flag
					}else{
						i = nxtscan(SUB, 1);						// advance mem#
						if(!i){
							doscan(SUB, 0);							// shut down scan, no mems enabled
							set_slide(SUB, 0);
							do_3beep();
						}else{
							if(i & 0x80){							// band switch
								force_push();						// force update to NVRAM
								set_bandnv();
								scan_switch = 1;					// set switch event flag
								scan_time(SUB, 3);					// reset scan timer for band edge event
							}else{
								scan_time(SUB, 1);					// reset scan timer
							}
							read_mem(SUB, get_memnum(SUB, 0));		// read new mem
							rflags |= SIN_SSRF_F;					// force update of SSRF
							save_mc(SUB);							// save changes to NVRAM
							update_lcd(focus, SUB);
							update_radio_all(SUB_ALL);
						}
					}
				}
			}
		}
	}
	return rflags;
}	// end process_DIAL()

//-----------------------------------------------------------------------------
// freq_update() handles frequency updates
//-----------------------------------------------------------------------------
void freq_update(U8 focus, S8 step){
	U8	i;			// temp
	U8	m;

	// main freq update
	if((maddr == MHZ_OFF) || (maddr == MHZ_ONE)){
		// MHZ (No thumbwheel) mode:
		i = add_vfo(focus, step, maddr);					// update vfo
		if(i && (focus == MAIN)){
			mfreq(get_freq(MAIN), 0);					// update display
		}
		if(i && (focus == SUB)){
			sfreq(get_freq(SUB), 0);
		}
		vfo_change(focus);
	}else{
		// digit-by-digit (thumbwheel) mode
		m = maddr & (~MHZ_OFFS);						// mask offset mode flag
		if(m == 0) step = (step & 0x01) * 5;					// pick the odd 5KHz
		i = add_vfo(focus, step, maddr);					// update vfo
		if(mhz_time(0)){
			mhz_time(1);								// reset timer
			if(maddr & MHZ_OFFS){
				offs_time(1);
				mhz_time(1);
			}
		}
		else{
			mhz_time(0xff);								// exit MHz mode
			offs_time(0xff);
		}
		if(i && (focus == MAIN)){
			mfreq(get_vfot(), LEAD0);					// update main display
		}
		if(i && (focus == SUB)){
			sfreq(get_vfot(), LEAD0);					// update sub display
		}
		vfo_change(focus);
	}
	send_stat(focus, 'F', stat_str);
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
}	// end digblink()

//-----------------------------------------------------------------------------
// mfreq() sets the main frequency.  binfreq = binary KHz
//	set lzero to display leading zeros
//-----------------------------------------------------------------------------
void mfreq(U32 binfreq, U8 lzero){
	U8	i;
	U8	k = 0;
	U32	ii;
	U32	jj;

	if(xmodez & IPL_BOOT){
		mputs_lcd("IC900F", 0);
		return;
	}
	if(sys_err & (NO_B_PRSNT|NO_MUX_PRSNT)){			// trap "off" and "off-line" error states
		if(sys_err & NO_B_PRSNT){
			// disp "----" (off-line)
			mputs_lcd("NOBASE", 0);
		}else{
			if(sys_err & NO_MUX_PRSNT){
				// disp "OFF"
				mputs_lcd("-OFF-", 0);
			}
		}
	}else{
		ii = bin32_bcdp(binfreq);						// convert binary to BCD
		// suppress leading zeros (if lzero == 0)
		if(lzero != LEAD0){								// if no leading zeros, skip this
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
		mdp(1);
		//set/clear "1" GHz digit
		if(ii == 0x1L) mm6(1);
		else mm6(0);
	}
	return;
}	// end mfreq()

//-----------------------------------------------------------------------------
// sfreq() sets the sub frequency.  binfreq = binary KHz
//	set lzero to display leading zeros
//-----------------------------------------------------------------------------
void sfreq(U32 binfreq, U8 lzero){
	U8	i;
	U8	k = 0;
	U32	ii;
	U32	jj;

	if(xmodez & IPL_BOOT){
		sputs_lcd("KE0FF", 0);
		return;
	}
	if(sys_err & (NO_B_PRSNT|NO_SUX_PRSNT)){										// trap "off" and "off-line" error states
		if(sys_err & NO_B_PRSNT){
			// disp "----" (off-line)
			sputs_lcd("------", 0);
		}else{
			if(sys_err & NO_SUX_PRSNT){
				// disp "OFF"
				sputs_lcd("-OFF-", 0);
			}
		}
	}else{
		// supress leading zeros
		ii = bin32_bcdp(binfreq);						// convert binary to BCD
		// suppress leading zeros (if lzero == 0)
		if(lzero != LEAD0){								// if no leading zeros, skip this
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
		sdp(1);
		//set/clear "1" GHz digit
		if(ii == 0x1L) sm6(1); 							// set 1G plus DP
		else sm6(0); 									// just DP
	}
	return;
}	// end sfreq()

//-----------------------------------------------------------------------------
// mm6() turns main M6 on (tf == TRUE) or off (tf == 0)
//-----------------------------------------------------------------------------
void mm6(U8 tf){

	// set main dp
	lcd_buf[0] = (CS1_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | MDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | M6;				// set M6
	else lcd_buf[2] = AND_DMEM | MDP2 | MDP;		// clear M6
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// sm6() turns sub M6 on (tf == TRUE) or off (tf == 0)
//-----------------------------------------------------------------------------
void sm6(U8 tf){

	// set main dp
	lcd_buf[0] = (CS2_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | SDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | S6;				// set M6
	else lcd_buf[2] = AND_DMEM | SDP2 | SDP;		// clear M6
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// mdp() turns main dp on (tf == TRUE) or off (tf == 0)
//-----------------------------------------------------------------------------
void mdp(U8 tf){

	// set main dp
	lcd_buf[0] = (CS1_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | MDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | MDP;				// set DP blink
	else lcd_buf[2] = AND_DMEM | MDP2 | M6;			// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// sdp() turns sub dp on (tf == TRUE) or off (tf == 0)
//-----------------------------------------------------------------------------
void sdp(U8 tf){

	// set main dp
	lcd_buf[0] = (CS2_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | SDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | SDP;				// set DP blink
	else lcd_buf[2] = AND_DMEM | SDP2 | S6;			// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// dp_blink() blinks (tf == TRUE) or un-blinks (tf == 0) main DP
//-----------------------------------------------------------------------------
void dp_blink(U8 tf){

	// blink dp
	lcd_buf[0] = (CS1_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | MDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_BMEM | MDP;				// set DP blink
	else lcd_buf[2] = AND_BMEM | MDP2 | M6;			// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// mdp2_blink() blinks (tf == TRUE) or un-blinks (tf == 0) main DP2
//-----------------------------------------------------------------------------
void mdp2_blink(U8 tf){

	// turn on dp2
	lcd_buf[0] = (CS1_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | MDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | MDP2;				// set DP
	else lcd_buf[2] = AND_DMEM | MDP | M6;			// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	// blink dp2
//	lcd_buf[1] = LOAD_PTR | MDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_BMEM | MDP2;				// set DP blink
	else lcd_buf[2] = AND_BMEM | MDP | M6;			// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// sdp_blink() blinks (tf == TRUE) or un-blinks (tf == 0) main DP
//-----------------------------------------------------------------------------
void sdp_blink(U8 tf){

	// blink dp
	lcd_buf[0] = (CS2_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | SDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_BMEM | SDP;				// set DP blink
	else lcd_buf[2] = AND_BMEM | SDP2;				// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	return;
}

//-----------------------------------------------------------------------------
// sdp2_blink() blinks (tf == TRUE) or un-blinks (tf == 0) main DP2
//-----------------------------------------------------------------------------
void sdp2_blink(U8 tf){

	// turn on dp2
	lcd_buf[0] = (CS2_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | SDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_DMEM | SDP2;				// set DP blink
	else lcd_buf[2] = AND_DMEM | SDP;				// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
	// blink dp2
	lcd_buf[0] = (CS2_MASK | 0x02);					// set data string preamble
	lcd_buf[1] = LOAD_PTR | SDP_ADDR;				// set lcd pointer
	if(tf) lcd_buf[2] = OR_BMEM | SDP2;				// set DP blink
	else lcd_buf[2] = AND_BMEM | SDP;				// clear DP
	put_spi(lcd_buf, CS_OPENCLOSE);					// send string to set display address and blink
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
		if(xmode[get_band_index(MAIN)] & CALL_XFLAG){
			mmem(get_callnum(MAIN, 0));					// restore call#
		}else{
			mmem(get_memnum(MAIN, 0));					// restore mem#
		}
		srf = 0;
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
	}else{
//		if(fetch_sin(1) & SIN_SEND){					// if ptt == 1
		if(ptt_change & PTT_KEYED){						// if ptt == 1
			if(srf > 2){								// we need *something* from the module...
				if(get_lohi(MAIN, 0xff)) srf = 2;		// ... before we mod SRF to reflect TX power level
				else srf = MAX_SRF;
			}else{
				srf = 0;								// ... or that the TXRF is dead
			}
		}
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
		if(xmode[get_band_index(SUB)] & CALL_XFLAG){
			smem(get_callnum(SUB, 0));					// restore call#
		}else{
			smem(get_memnum(SUB, 0));					// restore mem#
		}
		srf = 0;
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
//	starts at ASCII 0x20 (space).  0x38 ('!') is "no-corresponding-code" representation
//	(3 horiz bars).  Alternate invalid chr is 0x09 (segments "ab",character '#').

U8 asc7seg[] = {
	0x00, 0x38, 0x41, 0x09, 0x38, 0x91, 0x38, 0x40,		// <spc>, !, ", #, $, %, &, ',
	0xB0, 0x32, 0x38, 0xD0, 0x38, 0x10, 0x38, 0x91,		// (, ), *, +, ,, -, ., /,
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
	else i = (U8)'!';									// set invalid char
	if(i > sizeof(asc7seg)){
		i = (U8)'!';									// set invalid char
	}
	rtn = asc7seg[i];									// return seg code
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
	put_spi(lcd_buf, CS_CLOSE); //put_spi(lcd_buf, CS_IDLE);
	if(dp_tf) mdp(1); //put_spi(lcd_mfreq_2, CS_CLOSE);			// DP
	else mdp(0); //put_spi(lcd_mfreq_3, CS_CLOSE);				// no DP
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
	put_spi(lcd_buf, CS_CLOSE);	//put_spi(lcd_buf, CS_IDLE);
	if(dp_tf) sdp(1); //put_spi(lcd_sfreq_2, CS_CLOSE);			// DP
	else sdp(0); //put_spi(lcd_sfreq_3, CS_CLOSE);				// no DP
	return (i-1);
}

//-----------------------------------------------------------------------------
// puts_lcd() translates an ASCII string to 7-seg for sub digits
//	returns #chars written to display
//-----------------------------------------------------------------------------
U8	puts_lcd(U8 focus, char *s, U8 dp_tf){
	U8	i;		// temp

	if(focus == MAIN){
		i = mputs_lcd(s, dp_tf);
	}else{
		i = sputs_lcd(s, dp_tf);
	}
	return (i);
}

//-----------------------------------------------------------------------------
// mmem() displays main mem#.  Input is ASCII char.
//-----------------------------------------------------------------------------
void mmem(U8 mn){
	U8		i;		// temp
	char	mchr;

	if(mn >= NUM_MEMS) mchr = 0;
	else mchr = mem_ordinal[mn];
	lcd_buf[0] = CS1_MASK | 0x04;
	lcd_buf[1] = LOAD_PTR | MMEM_ADDR;
	i = asc27(mchr);									// get seg code
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
void smem(U8 mn){
	U8	i;		// temp
	char	mchr;

	if(mn >= NUM_MEMS) mchr = 0;
	else mchr = mem_ordinal[mn];
	lcd_buf[0] = CS2_MASK | 0x04;
	lcd_buf[1] = LOAD_PTR | SMEM_ADDR;
	i = asc27(mchr);									// get seg code
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
// mema() sets/clears MAIN/SUB mem annunc.  Input is M/S focus, T/F
//-----------------------------------------------------------------------------
void mema(U8 focus, U8 tf){

	if(focus == MAIN){
		lcd_buf[0] = CS1_MASK | 0x02;
		lcd_buf[1] = LOAD_PTR | MM_ADDR;
		if(tf){
			lcd_buf[2] = OR_DMEM | MM;
		}else{
			lcd_buf[2] = AND_DMEM | MTNE;
		}
		put_spi(lcd_buf, CS_OPENCLOSE);
	}else{
		lcd_buf[0] = CS2_MASK | 0x02;
		lcd_buf[1] = LOAD_PTR | SM_ADDR;
		if(tf){
			lcd_buf[2] = OR_DMEM | SM;
		}else{
			lcd_buf[2] = AND_DMEM | STNE;
		}
		put_spi(lcd_buf, CS_OPENCLOSE);
	}
	return;
}

//-----------------------------------------------------------------------------
// mskpa() sets/clears MAIN/SUB skip annunc.  Input is M/S focus, T/F
//-----------------------------------------------------------------------------
void mskpa(U8 focus, U8 tf){

	if(focus == MAIN){
		lcd_buf[1] = LOAD_PTR | MDUP_ADDR;
		if(tf){
			lcd_buf[2] = AND_DMEM | MDUP | MMIN;			// turn off "skp"
		}else{
			lcd_buf[2] = OR_DMEM | MSKP;					// turn on "skp"
		}
		lcd_buf[0] = CS1_MASK | 0x02;
		put_spi(lcd_buf, CS_OPENCLOSE);
	}else{
		lcd_buf[1] = LOAD_PTR | SDUP_ADDR;
		if(tf){
			lcd_buf[2] = AND_DMEM | SDUP | SMIN;			// turn off "skp"
		}else{
			lcd_buf[2] = OR_DMEM | SSKP;					// turn on "skp"
		}
		lcd_buf[0] = CS2_MASK | 0x02;
		put_spi(lcd_buf, CS_OPENCLOSE);
	}
	return;
}

//-----------------------------------------------------------------------------
// mdupa() sets/clears MAIN DUP annunc.  Input is ASCII, S, +, -
// sdupa() sets/clears SUB DUP annunc.  Input is ASCII, S, +, -
//-----------------------------------------------------------------------------
void mdupa(char dplx){
	U8	i = 2; 	// message length

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
// asow() sets/clears OW annunc.  Input is TF
//-----------------------------------------------------------------------------
void asow(U8 tf){

	lcd_buf[0] = CS1_MASK | 0x02;
	lcd_buf[1] = LOAD_PTR | AOW_ADDR;
	if(tf){
		lcd_buf[2] = OR_DMEM | AOW;
	}else{
		lcd_buf[2] = AND_DMEM | ALOW;
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

	if(tf){
		GPIO_PORTD_DATA_R &= ~MTX_N;		// RX
	}
	else{
		GPIO_PORTD_DATA_R |= MTX_N;			// TX
		// process cancel IPL
		if(xmodez & IPL_BOOT){
			ipl_cancel();
		}
	}
	return;
}

//-----------------------------------------------------------------------------
// amrx() sets/clears the main RX LED.  Input is TF
//-----------------------------------------------------------------------------
void amrx(U8 tf){

	if(tf) GPIO_PORTC_DATA_R &= ~MRX_N;		// SQ open
	else GPIO_PORTC_DATA_R |= MRX_N;		// SQ closed
	return;
}

//-----------------------------------------------------------------------------
// asrx() sets/clears the sub RX LED.  Input is TF
//-----------------------------------------------------------------------------
void asrx(U8 tf){

	if(tf) GPIO_PORTC_DATA_R &= ~SRX_N;		// SQ open
	else GPIO_PORTC_DATA_R |= SRX_N;		// SQ closed
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

//-----------------------------------------------------------------------------
// get_mutefl() returns the mute_mode contents
//-----------------------------------------------------------------------------
U8  get_mutefl(void){

	return mute_mode;
}

//-----------------------------------------------------------------------------
// togg_tsab() returns the mute_mode contents
//-----------------------------------------------------------------------------
void togg_tsab(U8 focus){

	if(read_dplx(focus) & TSA_F){
		set_ab(focus, 0);
		ats(0);
	}else{
		set_ab(focus, 1);
		ats(1);
	}
	force_push();											// force update to NVRAM
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_xmode() returns xmode[] value
//-----------------------------------------------------------------------------
U8 get_xmode(U8 b_id){

	return xmode[b_id];
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// write_xmode() writes the xmode state to the nv memory space
//-----------------------------------------------------------------------------
void write_xmode(U8 main){
	U32	addr;		// temps
	U8	i = get_band_index(main);

	addr = nvaddr(XMODET_0 + (i * sizeof(U8)), IDLE_BANK);
	rw8_nvr(addr, xmode[i], CS_WRITE|CS_OPENCLOSE);
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// read_xmode() reads the xmode state array from the nv memory space
//-----------------------------------------------------------------------------
void read_xmode(void){
	U32	addr;		// temps
	U8	i;
	U8	j;

	j = CS_OPEN;
	addr = nvaddr(XMODET_0, IDLE_BANK);
	for(i=0; i<ID1200; i++){
		if(i == (ID1200 - 1)) j = CS_CLOSE;
		xmode[i] = rw8_nvr(addr, 0, CS_READ|j);
		j = CS_READ;
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// clear_xmode() clears the xmode state to the nv memory space
//-----------------------------------------------------------------------------
void clear_xmode(void){
	U32	addr;		// temps
	U8	i;
	U8	j;

	j = CS_WRITE|CS_OPEN;
	addr = nvaddr(XMODET_0, IDLE_BANK);
	for(i=0; i<ID1200; i++){
		if(i == (ID1200 - 1)) j = CS_WRITE|CS_CLOSE;
		xmode[i] = 0;
		rw8_nvr(addr, xmode[i], j);
		j = CS_WRITE;
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_sys_err() transferrs sys err status to LCD module
//-----------------------------------------------------------------------------
void set_sys_err(U8 err){

	sys_err |= err;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// clr_sys_err() transferrs sys err status to LCD module
//-----------------------------------------------------------------------------
void clr_sys_err(U8 err){

	sys_err &= ~err;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// copy_stat() copies the calling string to the status string and sets status_ptr
//-----------------------------------------------------------------------------
/*void copy_stat(char* sptr){
	U8		i = 0;			// chr counter
	char*	s = stat_str;	// copy pointer

	while(*s && (i++ < MAX)){
		*s++ = *sptr++;
	}
	stat_ptr = stat_str;
	if(focus == MAIN){
		xmodez |= STATM_MODE;
	}else{
		xmodez |= STATS_MODE;
	}
	return;
}*/

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// put_stat() sets the status display pointer and starts the text slider
//	if focus == 0xff, disable status slider
//-----------------------------------------------------------------------------
void put_stat(U8 focus, char* sptr){

	if(focus == 0xff){
		xmodez &= ~(STATM_MODE|STATS_MODE);
		return;
	}
	stat_ptr = sptr;
	dfe_time(1);										// set/reset TO timer
	if(focus == MAIN){
		xmodez |= STATM_MODE;
	}else{
		xmodez |= STATS_MODE;
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// puts_slide() handles display of long text strings
//	focus is m/s
//	sptr is pointer to base string
//	cmd is 0xff to init, 0x01 to start, 0x00 to slide string 1 chr (left only)
//	return TRUE if entire slide sequence is complete
//-----------------------------------------------------------------------------
U8 puts_slide(U8 focus, char* sptr, U8 cmd){
	static U8		moffs_idx;		// offset start of display string, main
	static U8		soffs_idx;		// ... sub
		   U8*		offsptr;
		   char*	tptr;
		   U8		i = 0;			// temp
#define	MAX_SLIDE 18

	if(focus == MAIN){									// use pointers to diff main vs. sub
		offsptr = &moffs_idx;
		tptr = mfbuf;
	}else{
		offsptr = &soffs_idx;
		tptr = sfbuf;
	}
	switch(cmd){
	default:
	case SLIDE_IPL:										// IPL init
		moffs_idx = 0;
		soffs_idx = 0;
		slide_time(0xff);
		break;

	case SLIDE_START:									// start slide
		*offsptr = 0;
		copy6str(sptr, tptr, *offsptr);
		puts_lcd(focus, tptr, 0);
		break;

	case SLIDE_RUN:										// run slide
		*offsptr += 1;
		i = copy6str(sptr, tptr, *offsptr);				// if i == true, end of slide is reached
		if(!i){
			puts_lcd(focus, tptr, 0);
		}else{
			i++;
			i--;
		}
		if(*offsptr > MAX_SLIDE) *offsptr = 0;
		break;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// copy6str() copies 6 chrs from sptr to dptr, plus a null at dptr
//	sptr is pointer to source string
//	dptr is pointer to dest string
//	return TRUE if source string is empty
//-----------------------------------------------------------------------------
U8 copy6str(char* sptr, char* dptr, U8 cidx){
	U8	i = 0;
	U8	j = cidx;
	U8	ccnt = 6;

	do{
		if(j++ < 6){
			*dptr++ = ' ';					// copy spaces to start
			*dptr = '\0';
		}else{
			*dptr = *(sptr+cidx-6);			// copy..
			if(!(*dptr++)) i = 1;				// if source empty, set true return
		}
	}while((*(++sptr)) && (--ccnt));		// up to 6 chrs
	dptr = '\0';
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// doscan() set/clear the scan function
//-----------------------------------------------------------------------------
U8 doscan(U8 focus, U8 tf){

	if(tf & ((xmode[get_band_index(focus)] & MC_XFLAG) != MEM_XFLAG)){ // if not mem, exit with error
		return 0;
	}
	if(focus == MAIN){
		if(tf){
			xmodeq |= MSCANM_XFLAG;
			xmodez |= MSCANM1_XFLAG;
			get_mscan(focus);
		}else{
			xmodeq &= ~MSCANM_XFLAG;
		}
		lcd_buf[0] = CS1_MASK | 0x02;
		lcd_buf[1] = LOAD_PTR | MM_ADDR;
		if(tf){
			lcd_buf[2] = WR_BMEM | MM;
		}else{
			lcd_buf[2] = WR_BMEM;
		}
		put_spi(lcd_buf, CS_OPENCLOSE);
	}else{
		if(tf){
			xmodeq |= MSCANS_XFLAG;
			xmodez |= MSCANS1_XFLAG;
			get_mscan(focus);
		}else{
			xmodeq &= ~MSCANS_XFLAG;
		}
		lcd_buf[0] = CS2_MASK | 0x02;
		lcd_buf[1] = LOAD_PTR | SM_ADDR;
		if(tf){
			lcd_buf[2] = WR_BMEM | SM;
		}else{
			lcd_buf[2] = WR_BMEM;
		}
		put_spi(lcd_buf, CS_OPENCLOSE);
	}
	return 1;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// nxtscan() steps to the next scannable mem
// SUB-scan will skip to the next band when it reaches the end of the mem list
//	The algorithm is not terribly simple:
//	  1) the band must be installed.  If not, skip it entirely (set_next_band() accomplishes this)
//	  2) Use the same algorithm as for the main-band until the end of the mem list is reached.
//	  3) If we end up back at the origin band and origin mem#, then there are no scanable memories (abort).
//	  4) the function needs to pass a signal that a band change was performed.
//
//	returns 0 if no enabled mems in band
//	returns 1 if new mem is ready
//	returns 0x81 if band-switch (sub only)
//-----------------------------------------------------------------------------
U8 nxtscan(U8 focus, U8 adder){
	U8	memstrt;
	U8	j;
	U8	breakout = 0;
	U8	scanen = 0;
	U8	bid;			// band id
	U8	bids;			// band id start
	U32	memask;

	memstrt = get_memnum(focus, 0);						// set start mem#
	memask = mem2bit(memstrt);
	bids = get_band_index(focus);						// set start band id
	if(focus == MAIN){
		j = memstrt;
		do{
			j += 1;										// advance mem#
			memask <<= 1;
			if(j >= MAX_MEM){
				j = 0;
				memask = 0x01L;
			}
			if(memask & mscan[bids]){
				scanen = TRUE;							// k is true if scan enabled
				set_memnum(bids, j);
			}

/*			j = get_memnum(focus, adder);				// get next memnum
//			eol = j & 0x80;								// isolate end of list..
			j &= 0x7f;									// isolate mem#
			if(mem2bit(j) & mscan[bids]){
				scanen = TRUE;							// k is true if scan enabled
			}*/
		}while((j != memstrt) && !scanen);
		if(j == memstrt) scanen = 0;					// if 1 or no channels enabled, "no scan"
	}else{
		bid = bids;										// set band id to start
		j = memstrt;									// set running mem# = start mem#
		do{
			j += 1;										// advance mem#
			memask <<= 1;
			if(j >= MAX_MEM){							// if end of list...
				j = 0;									// rollover mem# and mask
				memask = 0x01L;
				bid = set_next_band(focus);				// go to next available band
				if(((bid == bids) && (j == memstrt)) || (bid == BAND_ERROR)){ // if back to starting point or error, set exit
					breakout = 1;						// end of lookup
				}else{
					if(mscan[bid] == 0){				// if no channels enabled,
						j = MAX_MEM;					// no channels enabled, skip to next band
					}
				}
			}
			if(mem2bit(j) & mscan[bid]){
				scanen = 1;								// if scan enabled set end of lookup
			}
		}while(!scanen & !breakout);
		if(breakout) scanen = 0;
		else{
			if(scanen && (bid != bids)) scanen = 0x81; // band change exit
			set_memnum(bid, j);
		}
	}
	return scanen;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_mscan() gathers scan enable bits for all mem locations.
//	main just does main band, sub does all bands
//-----------------------------------------------------------------------------
void get_mscan(U8 focus){
	U8	bid;
	U8	j;
	U8	k;
	U32	mask;

	if(focus == MAIN){								// process main band
		bid = get_band_index(focus);
		mask = 0x20000000L;							// set start at mem#29
		mscan[bid] = 0;
		j = MAX_MEM-1;
		do{
			k = get_scanen(bid, j);					// k is true if scan enabled
			if(k){
				mscan[bid] |= mask;
			}
			j--;
			mask >>= 1;
		}while(mask);
	}else{											// process all bands
		for(bid=0; bid<ID1200; bid++){
			mscan[bid] = 0;
			mask = 0x20000000L;						// set start at mem#30
			j = MAX_MEM-1;
			do{
				k = get_scanen(bid, j);				// k is true if scan enabled
				if(k){
					mscan[bid] |= mask;
				}
				j--;
				mask >>= 1;
			}while(mask);
		}
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// mem2bit() returns bitmask for corresponding mem#
//-----------------------------------------------------------------------------
U32 mem2bit(U8 memnum){
	U32	ii = 0x01L;		// temp
	U8	i = memnum;

	while(i){
		ii <<= 1;
		i--;
	}
	return ii;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_xmodeq() returns xmodeq
//-----------------------------------------------------------------------------
U8 get_xmodeq(void){

	return xmodeq;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_xmodeq() returns xmodeq
//-----------------------------------------------------------------------------
void set_ptt(U8 pttstat){

	ptt_change = pttstat | PTT_EDGE;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_slide() sets slide mode or toggles
//	0 = turn off, 1 = turn on, 0xff = toggle
//-----------------------------------------------------------------------------
void set_slide(U8 focus, U8 tf){

	// toggle mem string disp mode
	if(focus == MAIN){
		switch(tf){
		default:
		case 0:
			xmodeq &= ~TEXTM_SLIDE;
			break;

		case 1:
			xmodeq |= TEXTM_SLIDE;
			break;

		case 0xff:
			if(xmodeq & TEXTM_SLIDE) xmodeq &= ~TEXTM_SLIDE;
			else xmodeq |= TEXTM_SLIDE;
			break;
		}
		vfo_display |= MAIN;
	}else{
		switch(tf){
		default:
		case 0:
			xmodeq &= ~TEXTS_SLIDE;
			break;

		case 1:
			xmodeq |= TEXTS_SLIDE;
			break;

		case 0xff:
			if(xmodeq & TEXTS_SLIDE) xmodeq &= ~TEXTS_SLIDE;
			else xmodeq |= TEXTS_SLIDE;
			break;
		}
		vfo_display |= SUB_D;
	}
}

//-----------------------------------------------------------------------------
// mem2ordinal() returns ordinal of ASCII mem#.
//-----------------------------------------------------------------------------
U8 mem2ordinal(char cm){
	U8	i;		// temp

	i=0;
	while((i<sizeof(mem_ordinal)) && (cm != mem_ordinal[i])){
		i++;
	}
	if(i >= sizeof(mem_ordinal)) i = 0xff;				// err exit
	return i;
}

//-----------------------------------------------------------------------------
// ordinal2mem() returns ASCII mem# of ordinal.
//-----------------------------------------------------------------------------
char ordinal2mem(U8 memnum){

	if(memnum >= NUM_MEMS) return '!';					// err exit
	return mem_ordinal[memnum];
}

//-----------------------------------------------------------------------------
// lookup_pl() returns U8 CTCSS# from U16 frequency (PL tone * 10).
//	truncates 1's digit of both LUT and parameter.  Thus, the decimal portion
//	of the PL tone is not needed to get a match.
//	returns 0xff if tone not found
//-----------------------------------------------------------------------------
U8 lookup_pl(U16 ctcss){
	U8	i;	// temps
	U16	ii;

	ii = (ctcss / 10) * 10;								// truncate param
	i = 0;
	while((ii != (tone_list[i]/10)*10) && (i < sizeof(tone_list))){
		i++;
	}
	if(i >= sizeof(tone_list)) i = 0xff;
	return i;
}

//-----------------------------------------------------------------------------
// pl_lookup() returns U16 CTCSS frequency given the U8 tone#.
//	last tone in array returns 0xffff
//-----------------------------------------------------------------------------
U16 pl_lookup(U8 code){

	return tone_list[code];
}

//-----------------------------------------------------------------------------
// set_last_cos() stores bid and mem# in next location of last_cosx[] array.
//-----------------------------------------------------------------------------
void set_last_cos(U8 focus){
	U8	i;	// temps
	U8	j;

	i = get_memnum(focus, 0);									// construct composite of mem# and bid
	j = get_band_index(focus);
	i |= j << 5;
	if(focus == MAIN){
		last_cosm[last_cosm_hp] = i;							// store to next buffer position
		if(++last_cosm_hp >= MAX_COS_MEM) last_cosm_hp = 0;		// advance head pointer
		if(last_cosm_hp == last_cosm_tp){						// if h == t, advance tail (lose oldest data)
			if(++last_cosm_tp >= MAX_COS_MEM) last_cosm_tp = 0;
		}
	}else{
		last_coss[last_coss_hp] = i;							// store to next buffer position
		if(++last_coss_hp >= MAX_COS_MEM) last_coss_hp = 0;		// advance head pointer
		if(last_coss_hp == last_coss_tp){						// if h == t, advance tail (lose oldest data)
			if(++last_coss_tp >= MAX_COS_MEM) last_coss_tp = 0;
		}
	}
	return;
}

//-----------------------------------------------------------------------------
// get_last_cos() recalls bid and mem# from previous location of last_cosx[] array.
//	returns composite of bid/mem#.
//	A return value of 0xff means that there are no COS entries to recall.
//-----------------------------------------------------------------------------
U8 get_last_cos(U8 focus){
	U8	i = 0xff;	// temps

	if(focus == MAIN){
		if(last_cosm_hp != last_cosm_tp){
			if(--last_cosm_hp == 0xff) last_cosm_hp = MAX_COS_MEM - 1;
			i = last_cosm[last_cosm_hp];
		}
	}else{
		if(last_coss_hp != last_coss_tp){
			if(--last_coss_hp == 0xff) last_coss_hp = MAX_COS_MEM - 1;
			i = last_coss[last_coss_hp];
		}
	}
	return i;
}

//-----------------------------------------------------------------------------
// backl_adj() adjusts the backlight to one of 10 settings
//	if setting is invalid, do nothing.
//	returns current setting
//-----------------------------------------------------------------------------

//					 dim             ...                 brt
U8	brt_table1[] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 99 };
U8	brt_table2[] = { 5, 15, 25, 38, 45, 50, 60, 70, 80, 90 };

U8 backl_adj(U8 setv){
	static	U8	setmem;

	if(setmem > BRT_MAX){
		setmem = BRT_DIM;
	}
	if(setv <= BRT_MAX){
		setmem = setv;
		// process bright settings
		set_pwm(5, brt_table1[setmem]);						// LEDs
		set_pwm(6, brt_table2[setmem]);						// LCDBL
	}
    return setmem;
}

//-----------------------------------------------------------------------------
// xmodez_init() does IPL init of xmodez
//-----------------------------------------------------------------------------
void xmodez_init(void){

	xmodez = IPL_BOOT;
    return;
}

//-----------------------------------------------------------------------------
// xmodez_stat() returns xmodez value
//-----------------------------------------------------------------------------
U8 xmodez_stat(void){

    return xmodez;
}

//-----------------------------------------------------------------------------
// xmode_stat() returns xmode value for m/s
//-----------------------------------------------------------------------------
U8 xmode_stat(U8 focus){

    return xmode[get_band_index(focus)];
}

//-----------------------------------------------------------------------------
// query_tx() returns true if TX led is on
//-----------------------------------------------------------------------------
U8 query_tx(void){

	if(GPIO_PORTD_DATA_R & MTX_N) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// query_mode() returns true if TX led is on
//-----------------------------------------------------------------------------
U8 query_mode(void){

	return uimode;
}

//-----------------------------------------------------------------------------
// set_sw() passes sw state for DIM/LOCK
//	if parm == 0xff, only return current value of sw_temp
//	else, sw_temp = parm
//-----------------------------------------------------------------------------
U8 set_sw(U8 parm){

	if(parm != 0xff){
		sw_temp = parm;
	}
	return sw_temp;
}

// eof
