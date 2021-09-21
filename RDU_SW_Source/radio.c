/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: radio.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the source file for the IC-900 RDU Clone application
 *  radio I/O.
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    08-21-21 jmh:	 Modified VFO data structure to include TSA/B and to store VFO/5 and OFFS/5
 *    					merged with into one U32. Updated CRC and HIB store recall accordingly.
 *    08-10-21 jmh:  creation date
 *
 *******************************************************************/

/********************************************************************
 *  TOO:
 *  radio.c/h drives the UI for the RDU Clone application.  Interrupts are used to move data into
 *  and out of the application, so that it may be easily structured to operate as a polled state
 *  machine.  Core to the UI is the concept of "modes".  The primary mode is the VFO, whereby the
 *  operator may adjust the frequency, VOL, and SQU.  These parameters are focused on main or sub
 *  band based on the <SUB> key (toggles between main and sub).
 *
 *  SET mode enters a loop which allows access to less frequently needed parameters:
 *  	* duplex offset
 *  	* ctcss freq
 *  	* tuning step (A/B) in 5KHz increments up to 25 KHz
 *  	* "dim" brightness level (0 - 49)
 *  	* "bright" brightness level (50 - 99)
 *  	* Auto-dim threshold (0 = off)
 *
 *	MEM mode recalls the selected memory and allows the memory selection to be changed using DIAL/MICu/d.
 *
 *******************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions, main.c defines
#include "sio.h"
#include "radio.h"
#include "lcd.h"
#include "uxpll.h"
#include "serial.h"
#include "spi.h"
#include "version.h"

//-----------------------------------------------------------------------------
// local declarations
//-----------------------------------------------------------------------------

U32	sin_addr0;							// holding registers for SIN data (addr 1 and 0)
U32	sin_addr1;
U32	sin_flags;							// bitmapped activity flags that signal changed data
										// addr1 and 0 are muxed into a single 32 bit flag (only 16 bits of data are ever transferred)
U8	sout_flags;							// signal for SOUT changes
U8	ux_present_flags;					// bitmapped "present" (AKA, "installed") flags.
// **************************************************************
// these data structures are mirrored in a SW shadow NVRAM (HIB RAM)
//	each band module has its own cluster of data for frq, offset, etc...
//
/*
U32	vfo[NUM_VFOS];						// main vfo frequency (RX) in KHz
U16	offs[NUM_VFOS];						// TX offset in KHz

U8	dplx[NUM_VFOS];						// duplex/RFpwr/XIT/MEM - packed flags for each resource
U8	ctcss[NUM_VFOS];						// PL setting and "OFF" control bit
U8	tsa[NUM_VFOS];						// frq step "A" setting
U8	tsb[NUM_VFOS];						// frq step "B" setting
U8	sq[NUM_VFOS];							// SQ setting
U8	vol[NUM_VFOS];						// VOL setting
U8	mem[NUM_VFOS];						// mem# setting - an ordinal offset that maps to an ASCII character
U8	call[NUM_VFOS];						// call-mem# setting - an ordinal offset that maps to an ASCII character
U8	bflags[NUM_VFOS];					// expansion flags
U8	scanflags[NUM_VFOS];				// scan flags (expansion)

U8	ux129_xit;							// XIT setting (UX-129) - 4-bit signed nybble (+7/-8)
U8	ux129_rit;							// RIT setting (UX-129) - 4-bit signed nybble (+7/-8)
U8	ux129_xit_m;						// main temp
U8	ux129_rit_m;
U8	ux129_xit_s;						// sub temp
U8	ux129_rit_s;
*/
struct vfo_struct vfo_p[ID1200*2];		// main and temp-copy VFOs - main are mirrored into NVRAM, temp-copies are not
U8	mem[ID1200];						// mem# setting - an ordinal offset that maps to an ASCII character
U8	call[ID1200];						// call-mem# setting - an ordinal offset that maps to an ASCII character

// XIT/RIT only apply to one module, one per radio, so we don't need a matrix for them, just a temp.
U8	ux129_xit;							// XIT setting (UX-129) - 4-bit signed nybble (+7/-8)
U8	ux129_rit;							// RIT setting (UX-129) - 4-bit signed nybble (+7/-8)
U8	ux129_xit_t;						// temp
U8	ux129_rit_t;

U8	bandid_m;							// bandid for main (index into above data structures)
U8	bandid_s;							// bandid for sub (index into above data structures)

U8	bandoff_m;							// band-off registers
U8	bandoff_s;

U32	vfot;								// temp vfo used for calculations and TX frequency
U32	vfotr;								// tr vfo
U32	mhz_step;							// mhz digit add value - tracks the digit index in the thumbwheel digit adjust mode
U8	mute_band;							// flags to control audio mute function for main/sub (mirror of lcd.c mute_mode)

// TX upper frequency limits for each band
U32	vfo_tulim[ID1200];
// TX lower frequency limits for each band
U32	vfo_tllim[ID1200];

// upper frequency limits for each band
U32	vfo_ulim[] = { 40000L, 60000L, 170000L, 228000L, 470000L, 1310000L };
// lower frequency limits for each band
U32	vfo_llim[] = { 27000L, 45000L, 130000L, 215000L, 420000L, 1200000L };
// upper TX offset frequency limits for each band (lower limit is zero for all bands)
U32	offs_ulim[] = { 13000L, 15000L, 40000L, 13000L, 50000L, 900000L };
// Mem NVRAM base address table
U32 mem_band[] = { ID10M_MEM, ID6M_MEM, ID2M_MEM, ID220_MEM, ID440_MEM, ID1200_MEM };
//U32 mem_band[1] = { ID10M_MEM };
// mem name strings
char memname[NUM_VFOS][MEM_NAME_LEN];

// **************************************************************

#define	PLL_BUF_MAX	14
U32	pll_buf[PLL_BUF_MAX];				// PLL data buffer
U8	pll_ptr;							// pll_buf index

// reset units
#define	SO_INIT_LENA	2
U32	so_inita[] = { 0x00000000, 0x3F000818 };

// sout data to scan for installed UX units
#define	SO_INIT_LENB	7
U32	so_initb[] = { 0x08000000, 0x10000000, 0x18000000, 0x20000000, 0x28000000, 0x30000000, 0x00000000 };

// 2m/440 and base unit init (debug patch)
#define	SO_INIT_LENC	5
U32	so_initc[] = {
				   0x3D220050, 					// squ left (main)
				   0x3D020048,					// squ right
				   0x3E202090,					// vol left	(main)
				   0x3E002088,					// vol right
				   0x3CA000EA					// tone enc
				   };

// **************************************************************
// local Fn declarations

U8 get_bandid(U32 freqMM);
U8 get_busy(void);

//-----------------------------------------------------------------------------
// ***** START OF CODE *****
//-----------------------------------------------------------------------------

/****************
 * init_radio is called at IPL and initializes the data structures and radio hardware for the radio Fns
 */
void init_radio(void){
	U8	i;
	U8	j;
	U8	k;
	U16	ii;
	U8	usnbuf[16];		// User SN buffer
//	char	ibuf[25];	// sprintf/putsQ debug patch !!!

	wait(100);
	// send init array 1 (reset)
	for(i=0; i<SO_INIT_LENA; i++){						// do base module reset
		send_so(so_inita[i]);
		wait(SOUT_PACE_TIME);
	}
	// send init array 2 (UX module query)
	ux_present_flags = 0;								// start with no modules
	flush_sin();										// clear SIN buffer
	for(i=0, j=0x01; i<SO_INIT_LENB; i++, j<<=1){
		send_so(so_initb[i]);							// send a query message to each possible module
		wait(SOUT_PACE_TIME);
		if(get_busy()){
			ux_present_flags |= j;						// and set the present flag if the busy bit is active
		}
	}
	ux_present_flags &= IDALL_B;						// mask known modules
	// !!! need to trap exceptions here if only 1 or no modules present
	//
	// send init array 3 (base init)					// !!! this needs to get deprecated...
/*	for(i=0; i<SO_INIT_LENC; i++){						// send a fixed init string to place base resources
		send_so(so_initc[i]);							// into a known (if not relevant) state
		wait(10);
	}*/
	// recall vfo NV data
	recall_vfo();
	// validate VFOs to validate NVRAM contents
	k = TRUE;
	for(i=ID10M_IDX; i<ID1200; i++){
		if(vfo_p[i].vfo > vfo_ulim[i]) k = FALSE;		// if VFO is out of limit, invalidate the NVRAM
		if(vfo_p[i].vfo < vfo_llim[i]) k = FALSE;
	}
	if(!k) putsQ("NVRAMfail");							// display error msg to console
	rwusn_nvr(usnbuf, 0);
	ii = (U16)usnbuf[14];
	ii |= ((U16)usnbuf[15]) << 8;
	if(ii != nvram_sn()){
		// if version SN incorrect, invalidate NVRAM
		// future versions may call a "fixit" fn that updates an
		// out-of-version NVRAM without deleting the existing config.
		k = FALSE;
		putsQ("USNfail");								// display error msg to console
	}
	if(!k){
		// Validation fail, stored VFO data corrupt: re-initialize...
		// (nvram_fix(ii) is the function we need here.  Maybe someday...)
		putsQ("Initializing VFOS...");					// display status msg to console

		for(i=ID10M_IDX; i<NUM_VFOS; i++){
			vfo_p[i].dplx = DPLX_S | LOHI_F;			// duplex = S & low power
			vfo_p[i].ctcss = 0x0c;						// PL setting = 100.0
			vfo_p[i].sq = LEVEL_MAX-6;					// SQ setting
			vfo_p[i].vol = 20;							// vol setting
			vfo_p[i].tsa = 1;							// tsa/b defaults
			vfo_p[i].tsb = 2;
			if(i<ID1200){
				mem[i] = 0;
				call[i] = CALL_MEM;
			}
			vfo_p[i].bflags = 0xff;						// expansion flags
			vfo_p[i].scanflags = 0xff;					// scan flags (expansion)
			vfo_tulim[i] = vfo_ulim[i];					// copy RX limits to TX
			vfo_tllim[i] = vfo_llim[i];
			memname[i][0] = '\0';						// init mem names
		}
		vfo_p[ID10M_IDX].vfo = 29100L;					// each band has a unique initial freq
		vfo_p[ID6M_IDX].vfo = 52525L;
		vfo_p[ID2M_IDX].vfo = 146520L;
		vfo_p[ID220_IDX].vfo = 223500L;
		vfo_p[ID440_IDX].vfo = 446000L;
		vfo_p[ID1200_IDX].vfo = 1270000L;
		vfo_p[ID10M_IDX].offs = 100;					// each band has a unique initial TX offset
		vfo_p[ID6M_IDX].offs = 1000;
		vfo_p[ID2M_IDX].offs = 600;
		vfo_p[ID220_IDX].offs = 1600;
		vfo_p[ID440_IDX].offs = 5000;
		vfo_p[ID1200_IDX].offs = 20000;
		// init temp vfos
		vfo_p[ID10M_IDX+ID1200].vfo = 29100L;			// each band has a unique initial freq
		vfo_p[ID6M_IDX+ID1200].vfo = 52525L;
		vfo_p[ID2M_IDX+ID1200].vfo = 146520L;
		vfo_p[ID220_IDX+ID1200].vfo = 223500L;
		vfo_p[ID440_IDX+ID1200].vfo = 446000L;
		vfo_p[ID1200_IDX+ID1200].vfo = 1270000L;
		vfo_p[ID10M_IDX+ID1200].offs = 100;				// each band has a unique initial TX offset
		vfo_p[ID6M_IDX+ID1200].offs = 1000;
		vfo_p[ID2M_IDX+ID1200].offs = 600;
		vfo_p[ID220_IDX+ID1200].offs = 1600;
		vfo_p[ID440_IDX+ID1200].offs = 5000;
		vfo_p[ID1200_IDX+ID1200].offs = 20000;
		ux129_xit = 0;									// X/RIT settings
		ux129_rit = 0;
		clear_xmode();									// init xmode to no mem/call
		push_vfo();										// push new data vfo to nvram (hib)
		// init mems
		putsQ("Initializing MEMS...");					// display status msg to console
		// copy default VFOs to memory space
		for(bandid_m=ID10M_IDX; bandid_m<ID1200; bandid_m++){
			for(i=0; i<NUM_MEMS; i++){
				write_mem(MAIN, i);
			}
		}
		bandid_m = BAND_ERROR;							// default to error, figure out assignment below...
		bandid_s = BAND_ERROR;
		ii = nvram_sn();
		usnbuf[14] = (U8)ii;
		usnbuf[15] = (U8)(ii >> 8);
		rwusn_nvr(usnbuf, CS_WRITE);					// set version SN to validate version config

// debug presets !!!
//		ux129_xit = 0xaa;								// debug...XIT setting (UX-129)
//		ux129_rit = 0x55;
//		vfo_p[ID2M_IDX].ctcss = 0x1a;					// PL setting = 162.2
//		vfo_p[ID440_IDX].ctcss = 0x10;					// PL setting = 114.8
//		get_lohi(MAIN, 1);
//		get_lohi(SUB, 1);
	}
	putsQ("Validate selected module...");				// display status msg to console
	// double-check band-ids for validity against installed hardware
	if(bandid_m != BAND_ERROR){
		if(!(set_bit(bandid_m) & ux_present_flags)) bandid_m = BAND_ERROR;			// set main invalid if not present
//		bandid_s = BAND_ERROR);														// if main invalid, so is sub (will get reassigned below)
	}else{
		if(bandid_s != BAND_ERROR){
			if(!(set_bit(bandid_s) & ux_present_flags)) bandid_s = BAND_ERROR;		// set sub invalid if not present
		}
	}
	// re-assign active module if error
	j = 0x01;											// start at lowest module
	i = 0;
	do{
		if(j&ux_present_flags){							// pick first two modules present to be main/sub
			if(bandid_m == BAND_ERROR){					// only try to assign if error
				if(i != bandid_s) bandid_m = i;			// assign if not already taken by sub-band
			}else{
				if(bandid_s == BAND_ERROR){				// only try to assign if error
					if(i != bandid_m) bandid_s = i;		// assign if not already taken by main-band
				}
			}
		}
		i++;
		j <<= 1;
	}while(j<=ID1200_B);
	// if main band is error, copy up sub-band
	if(bandid_m == BAND_ERROR){
		bandid_m = bandid_s;
		bandid_s = BAND_ERROR;
	}

//		bandid_m = ID2M_IDX;	//!!! need to choose lowest 2 present modules
//		bandid_s = ID440_IDX;
//	ux_present_flags = 0x3f; //!!! force all modules on for debug
	i = 0;
	if(bandid_s == BAND_ERROR) i |= NO_SUX_PRSNT;
	if(bandid_m == BAND_ERROR) i |= NO_MUX_PRSNT;
	set_sys_err(i);

	read_xmode();										// pull xmode from NVRAM
	k = get_xmode(bandid_m);							// get main b-id
	if(k & MEM_XFLAG){									// if mem mode,
		read_mem(MAIN, mem[bandid_m]);					// copy mem
	}else{
		if(k & CALL_XFLAG){								// if call mode,
			read_mem(MAIN, call[bandid_m]);				// copy call mem
		}
	}
	k = get_xmode(bandid_s);							// get sub b-id
	if(k & MEM_XFLAG){									// if mem mode,
		read_mem(SUB, mem[bandid_s]);					// copy vfo to temp
	}else{
		if(k & CALL_XFLAG){								// if call mode,
			read_mem(SUB, call[bandid_s]);				// copy call mem
		}
	}
	force_push_radio();
	update_radio_all();									// force radio update
	i = 0;
	while(i < 3){
		if(process_SOUT(0) == 0){
			i++;
		}
	}
	wait(SOUT_PACE_TIME);
	send_so(0x38800021);								// this is what the IC-901 does... times 2 ???
	wait(SOUT_PACE_TIME);
	send_so(0x38800021);
	wait(SOUT_PACE_TIME);
	return;
}

//-----------------------------------------------------------------------------
// process_SIN() processes SIN change flags
//-----------------------------------------------------------------------------
void  process_SIN(U8 cmd){
	U32	sin_data;
	U32	ii;				// temp

	if(cmd == 0xff){								// initial program load (reset) branch
		sin_time(SIN_ACTIVITY);						// start SIN activity timer
		sin_addr0 = 0;								// pre-clear data and activity flags
		sin_addr1 = 0;
		sin_flags = 0;
		init_radio();								// init radio and data structures
	}else{											// normal (run) branch
		sin_data = get_sin();						// get'n check for SIN data
		if((sin_data & SIN_STOP) == SIN_STOP){		// validate extended "stop" bits
			sin_time(SIN_ACTIVITY);					// reset activity timer
			clr_sys_err(NO_B_PRSNT);				// clear LOS error
			if(sin_data & SIN_ADDR){				// process addr == 1 data
				// ADDR 1
				if(sin_data != sin_addr1){			// if data is new (different == new)
					// process new addr1 data ... calculate change flags
					ii = (sin_flags & 0x0000ffffL) | ((sin_data ^ sin_addr1) & SIN1_DATA);
					sin_flags |= ii;
/*					if(ii & SIN_SEND){
						sin_flags |= SIN_SEND_F;
					}
					if(ii & (SIN_DSQA | SIN_DSQB)) sin_flags |= SIN_DSQ_F;
					if(ii & (SIN_MCK)) sin_flags |= SIN_MCK_F;
					if(ii & (SIN_SEL11 | SIN_SEL12 | SIN_SEL21 | SIN_SEL22)) sin_flags |= SIN_SEL_F;*/
					sin_addr1 = sin_data;			// store new data
				}
			}else{
				// process ADDR == 0 data
				if(sin_data != sin_addr0){			// if data is new (different == new)
					// process new addr0 data ... calculate change flags
					ii = (sin_flags & 0xffff0000L) | (((sin_data ^ sin_addr0) & SIN0_DATA) >> 16);
					sin_flags |= ii;
/*					if(ii & (SIN_SQSA | SIN_SQSB)) sin_flags |= SIN_SQS_F;
					if(ii & (SIN_SRFA | SIN_SRFB)) sin_flags |= SIN_SRF_F;*/
					sin_addr0 = sin_data;
				}
			}
		}else{
			// no (valid) data
			if(!sin_time(0)){
				sin_flags |= SIN_SINACTO_F;			// set timeout error if activity timer expires
													// !!! need to trap this condition and post an error message to LCD
				set_sys_err(NO_B_PRSNT);
			}
		}
	}
}

//-----------------------------------------------------------------------------
// process_SOUT() processes SOUT buffer
//	input signals:
//		sin_flags (PTT, srf, cos)
//		sout_flags (vfo freq, T, SQU/D, VOLU/D)
//		uses a U32 buffer array to sequence SOUT data to be sent
//	returns pll_ptr .. if == 0xff, send is idle.
//-----------------------------------------------------------------------------
U8 process_SOUT(U8 cmd){
			U8	i;				// temp
			U8	j;
			U8	k = 0;
	static	U8	xit_count;		// used to mechanize UX-129 XIT/RIT adjustment sequences
	static	U8	xit_dir;
			U32* pptr;			// pointer into SOUT buffer
			char dgbuf[30];		// !!! debug sprintf/putsQ buffer

	if(cmd == 0xff){								// initial program load (reset) branch
		pll_ptr = 0xff;								// set index to "idle" state
		sout_time(0);								// initialize timer and statics
		xit_count = 0;
		xit_dir = 0;
//		sout_flags = 0;
	}else{											// normal (run) branch
		if(pll_ptr == 0xff){
			// no data is being sent branch ...
			j = 0;
			if(bandoff_m){																// turn-off band module flag trap, main
				pll_buf[j++] = bandoff_m << 27;
				pll_ptr = 0;
				bandoff_m = 0;															// clear signal flag
				pll_buf[j] = 0xffffffffL;
				k = 0xff;															// processing...
			}
			if(bandoff_s){																// turn-off band module flag trap, main
				pll_buf[j++] = bandoff_s << 27;
				pll_ptr = 0;
				bandoff_s = 0;															// clear signal flag
				pll_buf[j] = 0xffffffffL;
				k = 0xff;															// processing...
			}
			if(!j){
				if(sin_flags & SIN_SEND_F){													// PTT change detected
					if(sin_addr1 & SIN_SEND){
						i = 1;																// determine new state of PTT
					}
					else{
						i = 0;
					}
					amtx(i^1);																// update TX LED
					setpll(bandid_m, pll_buf, i, MAIN);										// PTT only drives MAIN band, set the module for TX
					set_vfo_display(VMODE_ISTX | MAIN);
					pll_ptr = 0;															// enable data send
					sprintf(dgbuf,"sinf: %08x",sin_flags); //!!!
					putsQ(dgbuf);
					sin_flags &= ~SIN_SEND_F;												// clear the signal
					k = 0xff;															// processing...
				}else{
					// process sout signals
					if(sout_flags){															// if not zero, there are (bit-mapped) signals to process
						pptr = pll_buf;														// preset sout buffer pointer
						// get an ordinal value for the highest priority signal
						if(mute_time(0)){
							i = get_bit(sout_flags & ~(SOUT_MVOL_F|SOUT_SVOL_F));			// if mute delay, mask off vol flags
						}else{
							i = get_bit(sout_flags);
						}
						switch(i){															// this will mechanize the various updates to process in order without collision
						case SOUT_VFOM_N:
							if(sin_addr1 & SIN_SEND) i = 1;									// get current state of PTT
							else i = 0;
							pptr = setpll(bandid_m, pll_buf, i, MAIN);						// update main pll
							pll_ptr = 0;
							sout_flags &= ~SOUT_VFOM_F;										// clear the signal
							set_vfo_display(MAIN);											// send disp update signal
							save_vfo(bandid_m);												// save affected VFO
							break;

						case SOUT_VFOS_N:
							setpll(bandid_s, pptr, 0, SUB);									// update main pll
							pll_ptr = 0;
							sout_flags &= ~SOUT_VFOS_F;
							set_vfo_display(SUB_D);											// send disp update signal
							save_vfo(bandid_s);												// save affected VFO
							break;

						case SOUT_MVOL_N:													// process VOL/SQU triggers...
							if((mute_band & MS_MUTE) || (bandid_m > ID1200_IDX)){
								i = 0;														// 0 if muted
							}else{
								i = vfo_p[bandid_m].vol;
							}
							pll_buf[0] = atten_calc(i) | ATTEN_MAIN | VOL_ADDR;
							pll_ptr = 0;
							sout_flags &= ~SOUT_MVOL_F;										// clear signal flag
							pll_buf[1] = 0xfffffffeL;										// set to confirm mute
							break;

						case SOUT_SVOL_N:
							if((mute_band & SUB_MUTE) || (bandid_s > ID1200_IDX)){
								i = 0;
							}else{
								i = vfo_p[bandid_s].vol;
							}
							pll_buf[0] = atten_calc(i) | ATTEN_SUB | VOL_ADDR;
							pll_ptr = 0;
							sout_flags &= ~SOUT_SVOL_F;										// clear signal flag
							pll_buf[1] = 0xfffffffeL;										// set to confirm mute
							break;

						case SOUT_MSQU_N:
							pll_buf[0] = atten_calc(vfo_p[bandid_m].sq) | ATTEN_MAIN | SQU_ADDR;
							pll_ptr = 0;
							sout_flags &= ~SOUT_MSQU_F;										// clear signal flag
							pll_buf[1] = 0xffffffffL;
							break;

						case SOUT_SSQU_N:
							pll_buf[0] = atten_calc(vfo_p[bandid_s].sq) | ATTEN_SUB | SQU_ADDR;
							pll_ptr = 0;
							sout_flags &= ~SOUT_SSQU_F;										// clear signal flag
							pll_buf[1] = 0xffffffffL;
							break;

						case SOUT_TONE_N:													// send tone message
							pll_buf[0] = (U32)vfo_p[bandid_m].ctcss | TONE_ADDR;
							pll_ptr = 0;
							pll_buf[1] = 0xffffffffL;
							sout_flags &= ~SOUT_TONE_F;										// clear signal flag
							break;

						default:
						case SOUT_VUPD_N:
							push_vfo();														// save VFO
							sout_flags &= ~SOUT_VUPD_F;
							break;
						}
						if(pll_ptr == 0) k = 0xff;											// processing...
					}
				}
			}
		}else{
			// data is being sent branch ...
			// wait for pacing timer
			k = 0xff;																// set "processing" flag
			if(!sout_time(0xff)){														// check for the pacing timer to expire
				// send buffer, 1 word/pass
				if((pll_buf[pll_ptr] & 0xfffffff0) != 0xfffffff0){						// (almost) all "f's" is end of buffer semaphore
					if((pll_buf[pll_ptr] & UX_XIT_MASK) != UX_XIT){						// process non-xit/rit messages
						putsQ("so1");	// !!!debug
						send_so(pll_buf[pll_ptr++]);									// send next word
						sout_time(SOUT_PACE_TIME);
						if(pll_ptr >= PLL_BUF_MAX){										// check for hard-end-of-buffer
							pll_ptr = 0xff;												// set end of tx
						}
					}else{																// process xit/rit as a sequence of PLL message that pulse an up-dn counter on the UX-129
																						// mechanizing this reduces the 32bit buffer length needed to queue SOUT data
						if(xit_count){													// if we are still counting:
							if(xit_count & 0x01){
								putsQ("so2");	// !!!debug
								send_so(UX_XIT_CK0);									// alternate FE clk with..
								sout_time(SOUT_PACE_TIME);
							}else{
								putsQ("so3");	// !!!debug
								if(xit_dir) send_so(UX_XIT_CKUP);						// ...REF clk up,
								else send_so(UX_XIT_CKDN);								// or dn
								sout_time(SOUT_PACE_TIME);
							}
							if(--xit_count == 0) pll_ptr++;								// X/RIT done... next pll buffer
						}else{
							xit_count = (U8)(pll_buf[pll_ptr] & UX_XIT_COUNT) << 1;		// set up the count and direction
							xit_dir = (U8)(pll_buf[pll_ptr] & UX_XIT_UP);
							if(xit_count == 0) pll_ptr++;								// X/RIT done... next pll buffer
						}
					}
				}else{
					if(pll_buf[pll_ptr] == 0xfffffffe){									// look for mute confirm semaphore
						mute_band |= 0x80;												// set muted flag
					}
					pll_ptr = 0xff;														// set end of tx flag
				}
			}
		}
	}
	return k;
}

//-----------------------------------------------------------------------------
// save_vfo() copies VFO/offset to NVRAM
//	call this fn anytime something changes in a VFO
//	b_id == 0xff, saves all
//	else, only save one indexed dataset
//-----------------------------------------------------------------------------
void  save_vfo(U8 b_id){
	U8	i;
	U32	j;
	U8	k;
	U8	startid;
	U8	stopid;

	if(b_id == 0xff){
		startid = 0;
		stopid = NUM_VFOS;
	}else{
		startid = b_id;
		stopid = b_id + 1;
		if((get_xmode(b_id) & (MC_XFLAG)) && (b_id < ID1200)){
			// if call or mem mode, just save mem/call#:
			j = SQ_0 + (VFO_LEN * startid);						// set start @ squ
			rw8_nvr(j, vfo_p[startid].sq, CS_WRITE|CS_OPEN);	// save squ and vol
			rw8_nvr(j, vfo_p[startid].vol, CS_WRITE|CS_CLOSE);
			return;												// exit now...
		}
	}
	// combine VFO/offset into single 32 bit value
	k = CS_WRITE | CS_OPEN;
	for(i=startid, j=VFO_0 + (VFO_LEN * startid); i<stopid; i++){
		rw32_nvr(j, vfo_p[i].vfo, k);							// write each element of the band state in sequence
		k = CS_WRITE;									// the addr (j) only matters for first call in an "OPEN" sequence.  So...
														// we don't update it here (the NVRAM increments it automatically).  Saves
		rw16_nvr(j, vfo_p[i].offs, k);						// a lot of SPI clock cycles.
		rw8_nvr(j, vfo_p[i].dplx, k);
		rw8_nvr(j, vfo_p[i].ctcss, k);
		rw8_nvr(j, vfo_p[i].sq, k);
		rw8_nvr(j, vfo_p[i].vol, k);
		if(i >= ID1200){
			rw8_nvr(j, mem[i-ID1200], k);
			rw8_nvr(j, call[i-ID1200], k);
		}else{
			rw8_nvr(j, mem[i], k);
			rw8_nvr(j, call[i], k);
		}
		rw8_nvr(j, vfo_p[i].bflags, k);
		rw8_nvr(j, vfo_p[i].scanflags, k);
		rw8_nvr(j, vfo_p[i].tsa, k);
		if(i == (stopid - 1)) k |= CS_CLOSE;
		rw8_nvr(j, vfo_p[i].tsb, k);
	}
	rw8_nvr(XIT_0, ux129_xit, CS_WRITE | CS_OPEN);
	rw8_nvr(RIT_0, ux129_rit, CS_WRITE);
	rw8_nvr(BIDM_0, bandid_m, CS_WRITE);
	rw8_nvr(BIDS_0, bandid_s, CS_WRITE | CS_CLOSE);
	return;
}

//-----------------------------------------------------------------------------
// save_mc() copies mem/call to NVRAM
//	call this fn anytime mem or call # changes
//-----------------------------------------------------------------------------
void  save_mc(U8 focus){
	U32	j;
	U8	i;

	if(focus == MAIN) i = bandid_m;
	else i = bandid_s;
	j = MEM_0 + (VFO_LEN * (U32)i);
	rw8_nvr(j, mem[i], CS_WRITE | CS_OPEN);			// mem & call are adjacent in the NV map
	rw8_nvr(j, call[i], CS_WRITE | CS_CLOSE);
	return;
}

//-----------------------------------------------------------------------------
// recall_vfo() copies NVRAM to VFO struct
//	call this fn on power-up
//-----------------------------------------------------------------------------
void  recall_vfo(void){
	U8	i;
	U32	j;
	U8	k;
	U8	startid;
	U8	stopid;

	startid = 0;
	stopid = NUM_VFOS;
	k = CS_OPEN;
	for(i=startid, j=VFO_0; i<stopid; i++){
		vfo_p[i].vfo = rw32_nvr(j, 0, k);
		k = CS_IDLE;
		vfo_p[i].offs = rw16_nvr(j, 0, k);
		vfo_p[i].dplx = rw8_nvr(j, 0, k);
		vfo_p[i].ctcss = rw8_nvr(j, 0, k);
		vfo_p[i].sq = rw8_nvr(j, 0, k);
		vfo_p[i].vol = rw8_nvr(j, 0, k);
		if(i < ID1200){
			mem[i] = rw8_nvr(j, mem[i], k);
			call[i] = rw8_nvr(j, 0, k);
		}else{
			rw8_nvr(j, mem[i], k);						// dummy reads
			rw8_nvr(j, 0, k);
		}
		vfo_p[i].bflags = rw8_nvr(j, 0, k);
		vfo_p[i].scanflags = rw8_nvr(j, 0, k);
		vfo_p[i].tsa = rw8_nvr(j, 0, k);
		if(i == (stopid - 1)) k |= CS_CLOSE;
		vfo_p[i].tsb = rw8_nvr(j, 0, k);
	}
	ux129_xit = rw8_nvr(XIT_0, 0, CS_OPEN);
	ux129_rit = rw8_nvr(RIT_0, 0, CS_IDLE);
	bandid_m = rw8_nvr(BIDM_0, 0, CS_IDLE);
	bandid_s = rw8_nvr(BIDS_0, 0, CS_CLOSE);
	read_xmode();



/*	U8	i;		// band array index & temp
	U8	j;		// ram addr
	U8	k;		// temps
	U32	ii;

	for(i=0, j=0; i<LEN_VFO_ARY; j++, i++){		// expand out VFO/OFFS
//		for(k=0; k<4; k++, j++){
//			ii <<= 4;
//			ii = rdhib_ram(j);
//		}
		ii = rdwhib_ram(j);
		vfo_p[i].vfo = (ii & 0x3ffff) * 5;
		vfo_p[i].offs = (ii >> 18) * 5;
	}
	j *= 4;
	for(i=0; i<LEN_ARY; i++, j++){				// duplex/hilo
		vfo_p[i].dplx = rdhib_ram(j);
	}
	for(i=0; i<LEN_ARY; i++, j++){
		vfo_p[i].ctcss = rdhib_ram(j);				// PL
	}
	for(i=0; i<LEN_ARY; i++, j++){
		vfo_p[i].sq = rdhib_ram(j);					// squ
	}
	for(i=0; i<LEN_ARY; i++, j++){				// tstep a/b
		k = rdhib_ram(j);
		vfo_p[i].tsa = k & 0x0f;
		vfo_p[i].tsb = k >> 4;
	}
	for(i=0; i<LEN_ARY; i++, j++){
		mem[i] = rdhib_ram(j);					// mem#
	}
	k = rdhib_ram(j++);							// vol main
	for(i=0; i<LEN_ARY; i++){
		vfo_p[i].vol = k;								// fill all vol with stored main
	}
	i = rdhib_ram(j++);							// sub vol...
	k = rdhib_ram(j++);							// bandids
	bandid_m = k & 0x0f;						// break out bandids
	bandid_s = k >> 4;
	vfo_p[bandid_s].vol = i;							// ... store nvram sub vol to (new) sub band
	k = rdhib_ram(j);							// bandids
	ux129_xit = k & 0x0f;						// break out xit/rit
	ux129_rit = k >> 4;*/
	return;
}

//-----------------------------------------------------------------------------
// crc_vfo() calculates crc on vfo ram
//-----------------------------------------------------------------------------
/*U16 crc_vfo(void){
	U16 crc = 0;
	U8	i;		// temps
	U8	j;
	U32	ii;

	// VFO/offset
	for(i=0; i<LEN_VFO_ARY; i++){
		ii = vfo_p[i].vfo/5 | ((vfo_p[i].offs/5) << 18);
		for(j=0; j<4; j++){
			crc = calcrc((U8)(ii & 0xffL), crc);
			ii >>= 4;
		}
	}
	for(i=0; i<LEN_ARY; i++){
		crc = calcrc(vfo_p[i].dplx, crc);
	}
	for(i=0; i<LEN_ARY; i++){
		crc = calcrc(vfo_p[i].ctcss, crc);
	}
	for(i=0; i<LEN_ARY; i++){
		crc = calcrc(vfo_p[i].sq, crc);
	}
	for(i=0; i<LEN_ARY; i++){
		crc = calcrc(vfo_p[i].vol, crc);
	}
	for(i=0; i<LEN_ARY; i++){
		crc = calcrc((vfo_p[i].tsa & 0x0f) | (vfo_p[i].tsb >> 4), crc);
	}
	i = bandid_m | (bandid_s << 4);			// combine bandid
	crc = calcrc(i, crc);
	crc = calcrc(ux129_xit, crc);			// save x/rit
	crc = calcrc(ux129_rit, crc);
	return crc;
}*/

//-----------------------------------------------------------------------------
// crc_hib() calculates crc on HIB RAM
//-----------------------------------------------------------------------------
U16 crc_hib(void){
	U16 crc = 0;
	U8	i;
	U8	j;
	U32	ii;

	for(i=0; i<LEN_HIB; i++){				// crc calculation loop
		ii = rdwhib_ram(i);					// using the word read is quicker (more efficient) than using the byte-read version
		for(j=0; j<4; j++){					// we still use bytes to calculate the CRC.  This makes the algorithm device-specific
			crc = calcrc((U8)(ii & 0xffL), crc); // but we don't care since we never use this CRC outside this calculation domain
			ii >>= 4;
		}
	}
	return crc;
}

//-----------------------------------------------------------------------------
// calcrc() calculates incremental crcsum using supplied poly
//	(xmodem poly = 0x1021)
//	oldcrc = 0 at first.  Feeds back on subsequent calls.
//	c is the next byte to crc
//-----------------------------------------------------------------------------
U16 calcrc(U8 c, U16 oldcrc){
	U16 crc;
	U8	i;

	crc = oldcrc ^ ((U16)c << 8);			// this is an old algorithm I found for xmodem a long, long time ago (but in THIS galaxy).
	for (i = 0; i < 8; ++i){
		if (crc & 0x8000) crc = (crc << 1) ^ XPOLY;
		else crc = crc << 1;
	 }
	 return crc;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// push_vfo() copies vfo structures to hib ram, does a crc on the hib, and stores
//	the crc to the hib.
//-----------------------------------------------------------------------------
void push_vfo(void){
//	U16	crctmp;

	save_vfo(0xff);										// save the vfo data structure to NVRAM
//	crctmp = crc_hib();									// calculate the CRC
//	wrhib_ram((U8)(crctmp >> 8), CRC_HIB_ADDR);			// and store to the last 2 bytes of the HIB (NVRAM)
//	wrhib_ram((U8)(crctmp & 0xff), CRC_HIB_ADDR+1);
	return;
}

//-----------------------------------------------------------------------------
// fetch_sin() returns the last valid SIN data based on the address param
//	used to transfer local data to other source domains
//-----------------------------------------------------------------------------
U32  fetch_sin(U8 addr){
	U32	ii;

	if(addr) ii = sin_addr1;
	else ii = sin_addr0;
	return ii;
}

//-----------------------------------------------------------------------------
// read_sin_flags() returns or clears the change flags
//-----------------------------------------------------------------------------
U32  read_sin_flags(U32 flag){

	if(flag) sin_flags &= ~flag;
	return sin_flags;
}

//-----------------------------------------------------------------------------
// vfo_change() sets the VFO flag to trigger a VFO update
//-----------------------------------------------------------------------------
void  vfo_change(U8 band){

	if(band == MAIN) sout_flags |= SOUT_VFOM_F;
	else sout_flags |= SOUT_VFOS_F;
	return;
}

//-----------------------------------------------------------------------------
// update_radio_all() sets the VFO flag to trigger a VFO update
//-----------------------------------------------------------------------------
void  update_radio_all(void){

	sin_flags = SIN_SQS_F|SIN_MSRF_F|SIN_SSRF_F|SIN_SEND_F|SIN_DSQ_F|SIN_SEL_F|SIN_VFOM_F|SIN_VFOS_F;
	sout_flags = SOUT_MSQU_F|SOUT_SSQU_F|SOUT_MVOL_F|SOUT_SVOL_F|SOUT_VFOS_F|SOUT_VFOM_F;
	return;
}

//-----------------------------------------------------------------------------
// force_push_radio() sets system to push the VFO structure to HIB RAM
//-----------------------------------------------------------------------------
void  force_push_radio(void){

//	sout_flags |= SOUT_VUPD_F;
	sout_flags |= (SOUT_MSQU_F | SOUT_SSQU_F | SOUT_MVOL_F | SOUT_SVOL_F | SOUT_VUPD_F | SOUT_VFOS_F | SOUT_VFOM_F);
	return;
}

//-----------------------------------------------------------------------------
// get_busy() looks for band busy == 1 or 20ms timeout
//	used by init_radio() as part of the IPL init
//-----------------------------------------------------------------------------
U8  get_busy(void){
	U8	bzy = 0;
	U8	i = 50;			// timeout, 20ms
	U8	j;
	U32	ii;

	for(j=0; j<3; j++){
		do{												// get 1st SIN, and discard
			wait(1);
			ii = get_sin();
			i--;
		}while(!ii && i);
	}
	i = 50;
	do{
		wait(1);
		ii = get_sin();
		i--;
	}while((ii & SIN_ADDR) || (!ii && i));				// look for 2nd SIN with ADDR == 0...
	if(ii && !(ii & SIN_ADDR) && !(ii & SIN_BUSY)){
		bzy = 1;
	}
	return bzy;
}

//-----------------------------------------------------------------------------
// get_present() returns ux_present_flags
//-----------------------------------------------------------------------------
U8  get_present(void){

	return ux_present_flags;
}

//-----------------------------------------------------------------------------
// clear_sin clears transient flags (eg, busy)
//-----------------------------------------------------------------------------
void clear_sin(void){

	sin_addr0 &= ~SIN_BUSY;
	return;
}
//-----------------------------------------------------------------------------
// atten_calc calculates atten pattern for TC9154 used in B-unit for squelch and volume
//	input is level (0-34, 0 = -68dB, >=34 = 0dB) which is converted to attenuation (34-0, 0 = 0dB, =34 = -68dB)
//-----------------------------------------------------------------------------
U32 atten_calc(U8 alevel){
	U8	i;		// temp
	U32	jj;		// temp and return

	if(alevel > LEVEL_MAX) alevel = LEVEL_MAX;			// enforce max limit
	alevel = (LEVEL_MAX) - alevel;						// flip sense (level => atten)
	i = alevel % ATTEN_FINE_LIM;						// calculate fine pattern
	jj = ATTEN_FINE >> i;
	i = alevel / ATTEN_FINE_LIM;						// calculate coarse pattern
	jj |= ATTEN_COARSE >> i;
	return jj;
}

//-----------------------------------------------------------------------------
// adjust_squ updates squ registers
// adjust_vol updates vol registers
//	hi-bit set, bit 0x40 clear: store masked value [5:0] to register
//	ELSE: add signed value [5:0] to register with ceiling and floor limit enforced
//	return new value
//	--- NOTE: call with value = "0" to read current setting ---
//-----------------------------------------------------------------------------
U8 adjust_squ(U8 mainsub, S8 value){
	S8	i;		// temp
	U8	j;		// return value

	if((value & 0xC0) == 0x80){							// set value branch
		i = value & 0x3f;								// mask flags
		if(i > LEVEL_MAX) i = LEVEL_MAX;
		if(mainsub){
			vfo_p[bandid_m].sq = i;							// store main
			j = vfo_p[bandid_m].sq;
			sout_flags |= SOUT_MSQU_F;					// send signal to update B-unit
		}else{
			vfo_p[bandid_s].sq = i;							// store sub
			j = vfo_p[bandid_s].sq;
			sout_flags |= SOUT_SSQU_F;					// send signal to update B-unit
		}
	}else{
		i = value;										// +/- branch
		if(mainsub){
			vfo_p[bandid_m].sq += i;							// adjust main +/-
			if(vfo_p[bandid_m].sq > 0x7f) vfo_p[bandid_m].sq = 0;
			if(vfo_p[bandid_m].sq > LEVEL_MAX) vfo_p[bandid_m].sq = LEVEL_MAX;
			j = vfo_p[bandid_m].sq;
			if(value != 0) sout_flags |= SOUT_MSQU_F;	// send signal to update B-unit
		}else{
			vfo_p[bandid_s].sq += i;							// adjust sub +/-
			if(vfo_p[bandid_s].sq > 0x7f) vfo_p[bandid_s].sq = 0;
			if(vfo_p[bandid_s].sq > LEVEL_MAX) vfo_p[bandid_s].sq = LEVEL_MAX;
			j = vfo_p[bandid_s].sq;
			if(value != 0) sout_flags |= SOUT_SSQU_F;	// send signal to update B-unit
		}
	}
	return j;											// return current value
}

U8 adjust_vol(U8 mainsub, S8 value){
	S8	i;		// temp
	U8	j;		// return value

	if((value & 0xC0) == 0x80){							// set value branch
		i = value & 0x3f;								// mask flags
		if(i > LEVEL_MAX) i = LEVEL_MAX;
		if(mainsub){
			vfo_p[bandid_m].vol = i;							// store main
			j = vfo_p[bandid_m].vol;
			sout_flags |= SOUT_MVOL_F;					// send signal to update B-unit
		}else{
			vfo_p[bandid_s].vol = i;							// store sub
			j = vfo_p[bandid_s].vol;
			sout_flags |= SOUT_SVOL_F;					// send signal to update B-unit
		}
	}else{
		i = value;										// +/- branch
		if(mainsub){
			vfo_p[bandid_m].vol += i;							// adjust main +/-
			if(vfo_p[bandid_m].vol > 0x7f) vfo_p[bandid_m].vol = 0;
			if(vfo_p[bandid_m].vol > LEVEL_MAX) vfo_p[bandid_m].vol = LEVEL_MAX;
			j = vfo_p[bandid_m].vol;
			if(value != 0) sout_flags |= SOUT_MVOL_F;	// send signal to update B-unit
		}else{
			vfo_p[bandid_s].vol += i;							// adjust sub +/-
			if(vfo_p[bandid_s].vol > 0x7f) vfo_p[bandid_s].vol = 0;
			if(vfo_p[bandid_s].vol > LEVEL_MAX) vfo_p[bandid_s].vol = LEVEL_MAX;
			j = vfo_p[bandid_s].vol;
			if(value != 0) sout_flags |= SOUT_SVOL_F;	// send signal to update B-unit
		}
	}
	return j;											// return current value
}

//-----------------------------------------------------------------------------
// adjust_tone updates ctcss register
//	hi-bit set, bit 0x40 clear: store masked value [5:0] to register
//	ELSE: add signed value [5:0] to register with roll-over/under (AKA, wrap-around) enforced
//-----------------------------------------------------------------------------
U8 adjust_tone(U8 mainsub, S8 value){
	S8	i;		// temp
	U8	j;		// return value
	U8	k;		// temp

	if((value & 0xC0) == 0x80){									// set value branch
		i = value & 0x3f;										// mask flags
		if(i > TONE_MAX) i = TONE_MAX;
		if(mainsub){
			k = vfo_p[bandid_m].ctcss & (~CTCSS_MASK);				// mask control bits
			vfo_p[bandid_m].ctcss = i | k;							// store main
			j = vfo_p[bandid_m].ctcss;
			sout_flags |= SOUT_TONE_F;							// send signal to update B-unit
		}else{
			k = vfo_p[bandid_s].ctcss & (~CTCSS_MASK);				// mask control bits
			vfo_p[bandid_s].ctcss = i | k;							// store sub
			j = vfo_p[bandid_s].ctcss;
//			sout_flags |= SOUT_TONE_F;							// send signal to update B-unit
		}
	}else{
		i = value;												// +/- branch
		if(mainsub){
			k = vfo_p[bandid_m].ctcss & CTCSS_MASK;
			k += i;
			if(k > TONE_MAX) k = 1;								// wrap-around
			if((k > CTCSS_MASK) || (k == 0)) k = TONE_MAX;
			j = (vfo_p[bandid_m].ctcss & (~CTCSS_MASK)) | k;			// adjust main +/-
			vfo_p[bandid_m].ctcss = j;
			if(!value) sout_flags |= SOUT_TONE_F;				// send signal to update B-unit
		}else{
			k = vfo_p[bandid_s].ctcss & CTCSS_MASK;
			k += i;
			if(k > TONE_MAX) k = 1;								// wrap-around
			if((k > CTCSS_MASK) || (k == 0)) k = TONE_MAX;
			j = (vfo_p[bandid_s].ctcss & (~CTCSS_MASK)) | k;			// adjust sub +/-
			vfo_p[bandid_s].ctcss = j;
			if(!value) sout_flags |= SOUT_TONE_F;				// send signal to update B-unit
		}
	}
	return j;													// return current value
}

//-----------------------------------------------------------------------------
// adjust_toneon updates ctcss register tone enable status
//	value == 0xff: return masked "off" bit
//	ELSE: if value == 0, turn off tone (set bit), else turn tone on (clear bit)
//-----------------------------------------------------------------------------
U8 adjust_toneon(U8 mainsub, U8 value){
	U8	j;

	if(value == 0xff){									// read setting
		if(mainsub){
			if(vfo_p[bandid_m].ctcss & CTCSS_OFF) j = 0;
			else j = 1;
		}else{
			if(vfo_p[bandid_s].ctcss & CTCSS_OFF) j = 0;
			else j = 1;
		}
	}else{
		if(mainsub){
			if(value){
				vfo_p[bandid_m].ctcss &= ~CTCSS_OFF;			// turn on
			}else{
				vfo_p[bandid_m].ctcss |= CTCSS_OFF;			// turn off
			}
			sout_flags |= SOUT_TONE_F;					// send signal to update B-unit
		}else{
			if(value){
				vfo_p[bandid_s].ctcss &= ~CTCSS_OFF;			// turn on
			}else{
				vfo_p[bandid_s].ctcss |= CTCSS_OFF;			// turn off
			}
//			sout_flags |= SOUT_TONE_F;					// send signal to update B-unit
		}
		j = value;
	}
	return j;											// return current value
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// setpll() calculates pll data using frequency in KHz
//	returns composite value of wbrx and band_id (0 = invalid band)
//	copied from FFRONT VisualC PC interface program source
// Stores data to U32 pointer location and updates pointer.  Returns new pointer.
//	bid			band ID
//	plldata		a pointer to the U32 data buffer
//	is_tx		a boolean flag to indicate TX (true) or RX (false)
//	is_main		true for main band, false for sub band
//-----------------------------------------------------------------------------
U32* setpll(U8 bid, U32* plldata, U8 is_tx, U8 is_main){
    U8	i = 0;			// pll array index
    U8	band_idr;		// temps
    U32	ux_noptt;
    U32	ux_ptt;
    U32	pll;
    U32	ii;
    U32	jj;
    U32	tt;
    U32	*	pllptr = plldata;
/*
#define SOUT_MAIN	0x04000000L
#define SOUT_SUB	0x02000000L
#define SOUT_PON	0x01000000L
#define SOUT_LOHI	0x00800000L
#define SOUT_BAND	0x00400000L
#define SOUT_PTT	0x00200000L	// band unit PTT
*/
    band_idr = get_bandid(vfo_p[bid].vfo / 1000L);		// use vfo frequency to determine band ID
    ux_noptt = band_idr << 27;							// align band ID in SOUT proto word
    if(is_main){
    	ux_noptt |= SOUT_MAIN | SOUT_PON;				// set module pre-amble bits (main)
    }else{
    	ux_noptt |= SOUT_SUB | SOUT_PON;				// .. (sub)
    }
    if(vfo_p[bid].dplx & LOHI_F){
    	ux_noptt |= SOUT_LOHI;							// set RF power level
    }
    ux_ptt = ux_noptt | SOUT_PTT;						// set PTT in is_tx proto word
    if(is_tx){
    	switch(vfo_p[bid].dplx & (DPLX_P|DPLX_M)){		// calculate TX frequency (if is_tx)
    	default:
    	case DPLX_S:
    		vfotr = vfo_p[bid].vfo;
    		break;

    	case DPLX_M:
    		vfotr = vfo_p[bid].vfo - (U32)vfo_p[bid].offs;
    		break;

    	case DPLX_P:
    		vfotr = vfo_p[bid].vfo + (U32)vfo_p[bid].offs;
    		break;
    	}
    }else{
    	vfotr = vfo_p[bid].vfo;							// no TX, no calc needed
    }
	// !!!
/*	char tgbuf[20];
	sprintf(tgbuf,"frq: %d",vfotr); //!!!
	putsQ(tgbuf);
*/
    //
    if(is_tx) sout_flags |= SOUT_TONE_F;				// trigger tone update if TX
    //
    // use calculated band_id to dispatch to module specific PLL calculations
    //	construct band select bitmap
    //	convert VFO freq to chanelized value for PLL formatting
    switch (band_idr & 0x0f) {
    case ID10M:
        pll = (vfotr - BASE_RX_10M) / 5L;				// convert to 5KHz chan
        pll += PLL_10M;									// add base PLL bitmap
        if (is_tx) {
            pll -= BASE_TX_10M;							// subtract tx offset (for 10M and 6M)
       }
        pll <<= 1;										// align bitmap (only for 10M and 6M)
        pllptr[i++] = ux_noptt | INIT_PLL_6M;			// store init frame
        pllptr[i++] = ux_noptt | pll;					// store PLL plus bit length
        if(is_tx){
            pllptr[i++] = ux_ptt | pll;					// store PLL plus PTT
        }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
        break;

    case ID6M:
        pll = (vfotr - BASE_RX_6M) / 5L;				// convert to 5KHz chan
        pll += PLL_6M;									// add base PLL bitmap
        if (is_tx) {
            pll -= BASE_TX_6M;							// subtract tx offset (for 10M and 6M)
        }
        pll <<= 1;										// align bitmap (only for 10M and 6M)
        pllptr[i++] = ux_noptt | INIT_PLL_6M;			// store init frame
        pllptr[i++] = ux_noptt | pll;					// store PLL plus bit length
        if(is_tx){
            pllptr[i++] = ux_ptt | pll;					// store PLL plus PTT
        }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
        break;

    case ID2M:
        pll = (vfotr - BASE_RX_2M) / 5L;				// convert to 5KHz chan
        pll += PLL_2M;									// add base PLL bitmap
        if (is_tx) {
            pll += BASE_TX_2M;							// add tx offset
        }
        // insert a "0" into bit 7
        pll = ((pll << 1) & 0x3ff80L) | (pll & 0x003fL);
        pllptr[i++] = ux_noptt | pll;					// store PLL plus bit length
        if(is_tx){
            pllptr[i++] = ux_ptt | pll;					// store PLL plus PTT
        }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
        break;

    case ID220:
        pll = (vfotr - BASE_RX_220) / 5L;				// convert to 5KHz chan
        pll += PLL_220;									// add base PLL bitmap
        if (is_tx) {
            pll += BASE_TX_220;							// add tx offset
        }
        // insert a "0" into bit 7
        pll = ((pll << 1) & 0x3ff80L) | (pll & 0x003fL);
        pllptr[i++] = ux_noptt | pll;					// store PLL plus bit length
        if(is_tx){
            pllptr[i++] = ux_ptt | pll;					// store PLL plus PTT
        }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
        break;

    case ID440:
        pll = (vfotr - BASE_RX_440) / 5L;				// convert to 5KHz chan
        pll += PLL_440;									// add base PLL bitmap
        if (is_tx) {
            pll += BASE_TX_440;							// add tx offset
        }
        pllptr[i++] = ux_noptt | pll;					// store PLL plus bit length
        if(is_tx){
            pllptr[i++] = ux_ptt | pll;					// store PLL plus PTT
        }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
        break;

    case ID1200:
        pll = (vfotr - BASE_RX_1200) / 10L;				// convert to 10KHz chan
        pll += PLL_1200;							    // add base PLL bitmap
        if (is_tx) {
            pll += BASE_TX_1200;					    // add tx offset
        }
        // insert a "0" into bit 6 ==> [...ponmlkjihg0fedcba]
        pll = ((pll << 1) & 0x3ff80L) | (pll & 0x003fL);
        // add "code" bits for N-frame
        pll |= PLL_1200_N;
        // bit reverse... tt will contain the new, bit-reversed PLL frame
        for (ii = 0x80000L, jj = 1L, tt = 0L; ii != 0; ii >>= 1, jj <<=1) {
            if (pll & ii) tt |= jj;
        }
        pllptr[i++] = ux_noptt | INIT_PLL_1200;			// init frame
        pllptr[i++] = ux_noptt | tt;					// store PLL plus bit length
        if(is_tx){
        	ux_noptt = ux_ptt;
            pllptr[i++] = ux_ptt | tt;					// store PLL plus PTT
        }
        pllptr[i++] = ux_ptt | INIT_PLL_1201;			// post-init frames
        pllptr[i++] = ux_ptt | INIT_PLL_1202;
        pllptr[i++] = ux_ptt | INIT_PLL_1203;
        if(is_tx){
            pllptr[i] = UX_XIT | (ux129_xit & (REG_XIT_UP | REG_XIT_CNT));
        }else{
            pllptr[i] = UX_XIT | (ux129_rit & (REG_XIT_UP | REG_XIT_CNT));
       }
        pllptr[i] = 0xffffffff;							// set end of PLL update string
       break;

    default:											// error trap, set end of update string
        band_idr = 0;
        pllptr[0] = 0xffffffff;							// set empty PLL update string
        i = 0xff;
        break;
    }
    return &pllptr[i+1];
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_bandid() calculates band id (1-6) from input freq (MHz)
//	and set wide-band bit if outside the ham RX limits
//-----------------------------------------------------------------------------
U8 get_bandid(U32 freqMM){
    U8	j;
    U8	k;
    U8	i = BANDOFF;

    k = (U8)(freqMM / 100);
    switch (k) {
    case 0:										// is 10M or 6M?
        j = (U8)(freqMM);
        if ((j > 39) && (j < 61)) {				// is 6M (MM MM between 0040 and 0061)?
            i = ID6M;
            if ((j < 50) || (j >= 54)) {
                i |= (BNDID_WBIF >> BND_SHIFT);	// enable wide band RX (BAND = 1)
            }
        }
        else {
            if ((j >= 20) && (j < 40)) {		// is 10M (MM MM between 0020 and 0040)?
                i = ID10M;						// is 10M
            }
        }
        break;

    case 1:										// is 2M ?
        j = (U8)(freqMM - 100);
        if ((j > 19) && (j < 71)) {				// is 2M (MM MM between 0120 and 0170)?
            i = ID2M;
            if ((j < 44) || (j >= 48)) {
                i |= (BNDID_WBIF >> BND_SHIFT);	// enable wide band RX (BAND = 1)
            }
        }
        break;

    case 2:										// is 220 ?
        j = (U8)(freqMM - 200);
        if ((j > 19) && (j < 26)) {				// is 2M (MM MM between 0220 and 0225)?
            i = ID220;
        }
        break;

    case 4:										// is 440 ?
        j = (U8)(freqMM - 400);
        if (j < 56) {					        // is UHF (MM MM between 0400 and 0455)?
            i = ID440;
        }
        break;

    case 12:									// is 1200 ?
        i = ID1200;
        break;

    case 13:									// is 1300 - 1320 ?
        j = (U8)(freqMM - 1300);
        if (j < 21) {
            i = ID1200;
        }
        break;

    default:
        i = BANDOFF;							// is error
        break;
    }
    return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// inc_dplx() increments duplex for main or sub
//	sequences between S (simplex), Minus, and Plus with roll-over/under
//-----------------------------------------------------------------------------
U8 inc_dplx(U8 main){
    U8	i;

    if(main) i = bandid_m;						// select duplex to modify
    else i = bandid_s;
    vfo_p[i].dplx += DPLX_P;							// increment to next duplex ordinal
    if((vfo_p[i].dplx & (DPLX_P|DPLX_M)) == (DPLX_P|DPLX_M)){
    	vfo_p[i].dplx &= ~(DPLX_P|DPLX_M);			// roll over duplex
    }
    return vfo_p[i].dplx;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// read_dplx() reads duplex byte (composite) for main or sub
//-----------------------------------------------------------------------------
U8 read_dplx(U8 main){
    U8	i;

    if(main) i = bandid_m;						// select duplex (m/s) to modify
    else i = bandid_s;
    return vfo_p[i].dplx;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_ab() sets tsab index
//	selects from the A or B frequency step setting
//-----------------------------------------------------------------------------
void set_ab(U8 main, U8 tf){
    U8	i;

    if(main) i = bandid_m;						// select register (m/s) to modify
    else i = bandid_s;
    if(tf){
    	vfo_p[i].dplx |= TSA_F;						// set index
    }else{
    	vfo_p[i].dplx &= ~TSA_F;						// clear index
    }
    return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// read_tsab() reads tsa/b.  1 = a, 2 = b
//-----------------------------------------------------------------------------
U8 read_tsab(U8 main, U8 absel){
    U8	i;

	if(main) i = bandid_m;						// select rean Main or Sub
	else i = bandid_s;
	if(absel == TSA_SEL) return vfo_p[i].tsa;
	return vfo_p[i].tsb;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_tsab() sets tsa/b.  1 = a, 2 = b
//-----------------------------------------------------------------------------
void set_tsab(U8 main, U8 absel, U8 value){
    U8	i;

	if(main) i = bandid_m;						// select duplex to modify
	else i = bandid_s;
	if(absel == TSA_SEL) vfo_p[i].tsa = value;
	else vfo_p[i].tsb = value;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_mhz_step() sets/reads mhz_step (no set if sval == 0)
//-----------------------------------------------------------------------------
S32 set_mhz_step(S32 sval){

    if(sval){
    	mhz_step = sval;
    }
   return mhz_step;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// is_mic_updn() returns 0 if no button, -1 if dn, +1 if up
//-----------------------------------------------------------------------------
S8 is_mic_updn(U8 ipl){
	static	S8	i;			// return value
	static	U8	click_mem;	// button pressed memory
			S8	rtn = 0;

	if(ipl){
		i = 0;				// init statics
		click_mem = 0;
		return 0;
	}
	if(sin_flags & SIN_MCK_F){								// if edge flag...
		if(sin_addr1 & SIN_MCK){							// u/d "clock" active
			micdb_time(1);									// set debounce timer
			i = 0;											// clear statics
			click_mem = 0;
		}
		read_sin_flags(SIN_MCK_F);
	}else{
		if(!click_mem){
			if((!micdb_time(0)) && (sin_addr1 & SIN_MCK)){
				mic_time(2);								// set long gap time for first press
		    	if((sin_addr1 & (SIN_MUP | SIN_MCK)) == (SIN_MUP | SIN_MCK)){
		    		i = 1;									// if up button pressed, set +
		    	}
		    	if((sin_addr1 & (SIN_MUP | SIN_MCK)) == (SIN_MCK)){
		    		i = -1;									// if dn button pressed, set -
		    	}
	    		do_dial_beep();								// beep
	    		click_mem = 1;
				rtn = i;
			}
		}else{
			if((!mic_time(0)) && (sin_addr1 & SIN_MCK)){
				mic_time(1);								// set short gap time for hold press
	    		do_dial_beep();								// beep
				rtn = i;
			}
		}
	}
	return rtn;



/*    S8	i = 0;		// return value (no button)

    // repeat u/d if button is held for 1+ sec.  abt 100ms rep rate
	if(sin_addr1 & SIN_MCK){
	    if(sin_flags & SIN_MCK_F){
			mic_time(2);									// set long gap time for first press
	    }
		// hold-down/repeat timeout?
		if(!mic_time(0)){
			// yes => trip change flag & reset timer
			sin_flags |= SIN_MCK_F;
			mic_time(1);
		}
	}
	if(!micdb_time(0)){
	    if(sin_flags & SIN_MCK_F){
	    	if((sin_addr1 & (SIN_MUP | SIN_MCK)) == (SIN_MUP | SIN_MCK)){
	    		i = 1;										// if up button pressed, set +
	    	}
	    	if((sin_addr1 & (SIN_MUP | SIN_MCK)) == (SIN_MCK)){
	    		i = -1;										// if dn button pressed, set -
	    	}
	    	if(i != 0){
	    		micdb_time(1);								// set debounce timer
	    		do_dial_beep();								// beep
	    	}
	    	read_sin_flags(SIN_MCK_F);
	    }
	}else{
    	if((sin_addr1 & (SIN_MUP | SIN_MCK)) == (SIN_MCK)){
	    	read_sin_flags(SIN_MCK_F);						// if dn during debounce, ignore and clear flag
    	}
	}
   return i;*/
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// add_vfo() adds signed multiplier to VFO or OFFS
//	manages destination, MHz, and "thumbwheel" modes using daddr
//	main		selects main/sub
//	adder		delta (+/-) add value
//-----------------------------------------------------------------------------
S8 add_vfo(U8 main, S8 adder, U8 daddr){
	U8	i;
	S32	delta;
	U8	bbuf[7];

	if(main) i = bandid_m;									// get band id for current focus
	else i = bandid_s;
	if(daddr & MHZ_OFFS){									// offset adjust: always uses thumbwheel mode
		vfot = (U32)vfo_p[i].offs;								// copy to vfot
		bin32_bcds(vfot, bbuf);								// convert to bcd in bbuf
		add_bcds(bbuf, adder, daddr & MHZ_MASK, 9, 0);		// add/subtract digit
		vfot = bcds_bin32(bbuf);
		// offs rollover/under
		if(vfot > offs_ulim[i]){
			vfot -=  offs_ulim[i];
		}
		vfo_p[i].offs = (U16)vfot;
	}else{
		if(daddr < MHZ_OFF){								// no mode flags set, this means we are in Thumbwheel mode and daddr selects the digit to be modified
			bin32_bcds(vfot, bbuf);							// convert to BCD string (one BCD nybble per sting byte)
			add_bcds(bbuf, adder, daddr, 9, 0);				// +/- the digit
			vfot = bcds_bin32(bbuf);						// convert back to U32
			if(vfot > vfo_ulim[i]){							// process band edge rollover/under
				vfot -=  vfo_ulim[i] - vfo_llim[i];
			}
			if(vfot < vfo_llim[i]){
				vfot +=  vfo_ulim[i] - vfo_llim[i];
			}
			vfo_p[i].vfo = vfot;									// update target VFO
		}else{
			if(daddr == MHZ_OFF){							// not MHz mode, adjust using freq step (a or b) setting
				if(vfo_p[i].dplx & TSA_F){
					delta = (S32)(vfo_p[i].tsb & 0x07) * 5L;
				}else{
					delta = (S32)(vfo_p[i].tsa & 0x07) * 5L;
				}
				vfo_p[i].vfo += delta * (S32)adder;
			}
			if(daddr == MHZ_ONE){							// MHz mode, adjust +/- MHz increments
				vfo_p[i].vfo += 1000 * (S32)adder;
			}
			if(vfo_p[i].vfo > vfo_ulim[i]){						// same roll-over/under as above
				vfo_p[i].vfo -= vfo_ulim[i] - vfo_llim[i];
			}
			if(vfo_p[i].vfo < vfo_llim[i]){
				vfo_p[i].vfo += vfo_ulim[i] - vfo_llim[i];
			}
		}
	}
	return adder;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_band_index() returns index for main or sub band
//-----------------------------------------------------------------------------
U8 get_band_index(U8 main){
	U8	i;

	if(main == MAIN) i = bandid_m;
	else i = bandid_s;
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_freq() returns vfo of indicated band
//	if hi-bit set, return the tr vfo
//-----------------------------------------------------------------------------
U32 get_freq(U8 main){

	if(main & VMODE_ISTX){
		return vfotr;
	}else{
	    if(main) return vfo_p[bandid_m].vfo;
	    else return vfo_p[bandid_s].vfo;
	}
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// copy_vfot() copies vfot to indicated vfo
//-----------------------------------------------------------------------------
void copy_vfot(U8 main){

    if(main) vfo_p[bandid_m].vfo = vfot;
    else vfo_p[bandid_s].vfo = vfot;
    return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// temp_vfo() copies vfo to vfot
//-----------------------------------------------------------------------------
void temp_vfo(U8 main){

    if(main) vfot = vfo_p[bandid_m].vfo;
    else vfot = vfo_p[bandid_s].vfo;
    return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_vfot() returns vfot
//-----------------------------------------------------------------------------
U32 get_vfot(void){

    return vfot;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_memnum() adds "adder" and then returns mem# for main or sub band
//-----------------------------------------------------------------------------
U8 get_memnum(U8 main, U8 adder){
	U8	k;

	if(main == MAIN){
		k = bandid_m;
	}else{
		k = bandid_s;
	}
	if(mem[k] >= MAX_MEM){
		mem[k] = 0;								// error fix
	}
	if((mem[k] < MAX_MEM) && (adder != 0)){
		mem[k] += adder;
		if(mem[k] > 0x80) mem[k] = MAX_MEM - 1;
		if(mem[k] >= MAX_MEM) mem[k] = 0;
	}
	return mem[k];
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_callnum() adds "adder" and then returns mem# for main or sub band
//-----------------------------------------------------------------------------
U8 get_callnum(U8 main, U8 adder){
	U8	k;

	if(main == MAIN){
		k = bandid_m;
	}else{
		k = bandid_s;
	}
	if(adder != 0){
		if((call[k] < MAX_MEM) || (call[k] >= NUM_MEMS)){
			call[k] = CALL_MEM;
		}else{
			call[k] += adder;
			if(call[k] >= NUM_MEMS) call[k] = CALL_MEM;
			if(call[k] < CALL_MEM) call[k] = NUM_MEMS - 1;
		}
	}
	return call[k];
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_lohi() returns current RF power setting if param is 0xff
//	otherwise, sets or clears LOHI bit in the duplex register
//-----------------------------------------------------------------------------
U8 get_lohi(U8 main, U8 setread){
	U8	i;

	if(main) i = bandid_m;
	else i = bandid_s;
    if(setread != 0xff){
    	if(setread){
    		vfo_p[i].dplx |= LOHI_F;
    	}else{
    		vfo_p[i].dplx &= ~LOHI_F;
    	}
    }
    return vfo_p[i].dplx & LOHI_F;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_next_band() returns bandid for next band on list
//	return 0xff if no band (eg, less than 3 modules present)
//-----------------------------------------------------------------------------
U8 set_next_band(U8 focus){
	U8	i;	// temps
	U8	j;
	U8	k;
	U8	h;

	if((bandid_m == BAND_ERROR) || (bandid_s == BAND_ERROR)){
		i = BAND_ERROR;										// if either bandid is error, abort (less than 2 modules present)
	}else{
		if(focus == MAIN){									// j is focused band, k is other band
			k = set_bit(bandid_s);							// get sub-band bitmap
			j = set_bit(bandid_m);							// get main-band bitmap
			bandoff_m = bandid_m + 1;
		}else{
			j = set_bit(bandid_s);							// get sub-band bitmap
			k = set_bit(bandid_m);							// get main-band bitmap
			bandoff_s = bandid_s + 1;
		}
		i = k | j;
		if(ux_present_flags & (~i)){						// check to see if there are at least 3 modules present
			i = ux_present_flags & (~i);					// remove current bands from list
			for(h=0; h<6; h++){								// slide around to find the next present module...
				j <<= 1;
				if(j == 0x40) j = 0x01;
				if(j & i){
					i = j;									// found it... i = bit,
					h = 6;									// break out of the for loop
				}
			}
			i = get_bit(i);									// convert bitmap to index
			if(focus == MAIN){								// j is focused band, k is other band
				bandid_m = i;								// set new main index
			}else{
				bandid_s = i;								// set new sub index
			}
		}else{
			i = BAND_ERROR;									// 2 or less modules, no next band
		}
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_swap_band() swaps main/sub
//-----------------------------------------------------------------------------
U8 set_swap_band(void){
	U8	i;	// temp

	if((bandid_m == BAND_ERROR) || (bandid_s == BAND_ERROR)){
		i = BAND_ERROR;										// if either bandid is error, abort (less than 2 modules present)
	}else{
		i = bandid_m;										// swap main and sub
		bandid_m = bandid_s;
		bandid_s = i;
		i = 0;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_bit() returns count of the highest set bit in value
//	bit 0x80 will return 7, bit 0x01 returns 0
//	no bit set returns 0xff
//-----------------------------------------------------------------------------
U8 get_bit(U8 value){
	U8	i;	// temps
	U8	j;

	if(!value) return 0xff;							// no bits set, error
	i = 7;											// get an ordinal value for the highest priority signal
	j = 0x80;										// by scanning the value for the highest "1"
	while(!(j & value) && j){
		j >>= 1;
		i--;
	}												// i = 0 to 7
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_bit() takes count of a set bit in value
//	and creates a bit-map return
//-----------------------------------------------------------------------------
U8 set_bit(U8 value){
	U8	i;	// temps
	U8	j;

	if(value > 7) value = 7;
	for(i=0, j=0x01; i<value; i++){
		j<<=1;										// shift bitmap
	}
	return j;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// bit_set() takes bitmap of a (single) set bit in value
//	and creates a count return
//-----------------------------------------------------------------------------
U8 bit_set(U8 value){
	U8	i = 0;	// temps
	U8	j;

	if(!value) return 0xff;							// no bits set, error
	j = 0x01;										// scan value for the first "1"
	while(!(j & value) && j){
		j <<= 1;
		i++;
	}												// i = 0 to 7
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_offs() updates offset
//-----------------------------------------------------------------------------
void set_offs(U8 focus, U16 value){

	if(focus == MAIN){
		vfo_p[bandid_m].offs = value;
	}else{
		vfo_p[bandid_s].offs = value;
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_offs() reads offset
//-----------------------------------------------------------------------------
U16 get_offs(U8 focus){
	U16	i;	// temp

	if(focus == MAIN){
		i = vfo_p[bandid_m].offs;
	}else{
		i = vfo_p[bandid_s].offs;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// inv_duplex() if duplex = +/-, invert.  Otherwise, no change.
//	returns vfo_p[].dplx & DPLX_MASK
//-----------------------------------------------------------------------------
U8 inv_duplex(U8 focus){
	U8	i;	// temps
	U8	j;

	if(focus == MAIN){
		j = bandid_m;
	}else{
		j = bandid_s;
	}
	i = vfo_p[j].dplx & DPLX_MASK;
	if(i){
		vfo_p[j].dplx = vfo_p[j].dplx ^ DPLX_MASK;
	}
	return vfo_p[j].dplx & DPLX_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// inv_vfo() uses duplex to calculate flip of RX/TX.  Call inv_duplex() AFTER
//	performing this Fn.  returns vfo_p[].dplx & DPLX_MASK
//-----------------------------------------------------------------------------
U8 inv_vfo(U8 focus){
	U8	i;	// temp

	if(focus == MAIN){
		i = bandid_m;
	}else{
		i = bandid_s;
	}
	if(vfo_p[i].dplx & DPLX_P){
		vfo_p[i].vfo = vfo_p[i].vfo + (U32)vfo_p[i].offs;
	}
	if(vfo_p[i].dplx & DPLX_M){
		vfo_p[i].vfo = vfo_p[i].vfo - (U32)vfo_p[i].offs;
	}
	return vfo_p[i].dplx & DPLX_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// mute_radio() sets mute_band and triggers SOUT update
//-----------------------------------------------------------------------------
void mute_radio(U8 mutefl){

	mute_band = mutefl;
	sout_flags |= SOUT_MVOL_F | SOUT_SVOL_F;			// set signal flag
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_mute_radio() returns mute_band
//-----------------------------------------------------------------------------
U8 get_mute_radio(void){

	return mute_band;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// write_mem() writes the vfo state to the nv memory space
//-----------------------------------------------------------------------------
void write_mem(U8 focus, U8 memnum){
	// mem structure follows this format:
	// VFO + OFFS + DPLX + CTCSS + SQ + VOL + XIT + RIT + BID + MEM_NAME_LEN
	U8	band;

	if(focus == MAIN) band = bandid_m;
	else band = bandid_s;
	write_nvmem(band, memnum);
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// read_mem() reads the vfo state from the nv memory space
//-----------------------------------------------------------------------------
void read_mem(U8 focus, U8 memnum){
	U8	band;

	if(focus == MAIN) band = bandid_m;
	else band = bandid_s;
	read_nvmem(band, memnum);
	return;
}
///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// write_nvmem() writes the vfo state to the nv memory space
//-----------------------------------------------------------------------------
void write_nvmem(U8 band, U8 memnum){
	// mem structure follows this format:
	// VFO + OFFS + DPLX + CTCSS + SQ + VOL + XIT + RIT + BID + MEM_NAME_LEN
	U32	addr;		// temps
	U8	i;
	U8	j;
	char* cptr;

	addr = mem_band[band] + (memnum * MEM_LEN);
	j = CS_WRITE;
	rw32_nvr(addr, vfo_p[band].vfo, j|CS_OPEN);
	rw16_nvr(addr, vfo_p[band].offs, j);
	rw8_nvr(addr, vfo_p[band].dplx, j);
	rw8_nvr(addr, vfo_p[band].ctcss, j);
	rw8_nvr(addr, vfo_p[band].sq, j);
	rw8_nvr(addr, vfo_p[band].vol, j);
	rw8_nvr(addr, ux129_xit, j);
	rw8_nvr(addr, ux129_rit, j);
	rw8_nvr(addr, band, j);
	cptr = memname[band];
	for(i=0; i<MEM_NAME_LEN; i++){
		if(i == (MEM_NAME_LEN - 1)) j |= CS_CLOSE;
		rw8_nvr(addr, *cptr++, j);
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// read_nvmem() read the vfo state from the nv memory space
//-----------------------------------------------------------------------------
void read_nvmem(U8 band, U8 memnum){
	// mem structure follows this format:
	// VFO + OFFS + DPLX + CTCSS + SQ + VOL + XIT + RIT + BID + MEM_NAME_LEN
	U32	addr;		// temps
	U8	i;
	U8	j;
	char* cptr;

	addr = mem_band[band] + (memnum * MEM_LEN);
	j = CS_READ;
	vfo_p[band].vfo = rw32_nvr(addr, 0, j|CS_OPEN);
	vfo_p[band].offs = rw16_nvr(addr, 0, j);
	vfo_p[band].dplx = rw8_nvr(addr, 0, j);
	vfo_p[band].ctcss = rw8_nvr(addr, 0, j);
	vfo_p[band].sq = rw8_nvr(addr, 0, j);
	vfo_p[band].vol = rw8_nvr(addr, 0, j);
	ux129_xit = rw8_nvr(addr, 0, j);
	ux129_rit = rw8_nvr(addr, 0, j);
	band = rw8_nvr(addr, 0, j);
	cptr = memname[band];
	for(i=0; i<MEM_NAME_LEN; i++){
		if(i == (MEM_NAME_LEN - 1)) j |= CS_CLOSE;
		*cptr++ = rw8_nvr(addr, 0, j);
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// copy_vfo2temp() copy VFO to temp space
//-----------------------------------------------------------------------------
void copy_vfo2temp(U8 focus){
	U8	tempi;
	U8	band;

	if(focus == MAIN){
		band = bandid_m;
		tempi = band + ID1200;
	}else{
		band = bandid_s;
		tempi = band + ID1200;
	}
	ux129_xit_t = ux129_xit;
	ux129_rit_t = ux129_rit;
	vfo_p[tempi] = vfo_p[band];
	save_vfo(tempi);						// save to NVRAM

/*	vfo_p[tempi].vfo = vfo_p[band].vfo;
	vfo_p[tempi].offs = vfo_p[band].offs;
	vfo_p[tempi].dplx = vfo_p[band].dplx;
	vfo_p[tempi].ctcss = vfo_p[band].ctcss;
	vfo_p[tempi].sq = vfo_p[band].sq;
	vfo_p[tempi].vol = vfo_p[band].vol;*/
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// copy_temp2vfo() copy temp to VFO space
//-----------------------------------------------------------------------------
void copy_temp2vfo(U8 focus){
	U8	tempi;
	U8	band;

	if(focus == MAIN){
		band = bandid_m;
		tempi = band + ID1200;
	}else{
		band = bandid_s;
		tempi = band + ID1200;
	}
	ux129_xit = ux129_xit_t;
	ux129_rit = ux129_rit_t;
	vfo_p[band] = vfo_p[tempi];

/*	vfo_p[band].vfo = vfo_p[tempi].vfo;
	vfo_p[band].offs = vfo_p[tempi].offs;
	vfo_p[band].dplx = vfo_p[tempi].dplx;
	vfo_p[band].ctcss = vfo_p[tempi].ctcss;
	vfo_p[band].sq = vfo_p[tempi].sq;
	vfo_p[band].vol = vfo_p[tempi].vol;*/
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_bandid() writes the band id to the indicated register
//-----------------------------------------------------------------------------
void set_bandid(U8 focus, U8 b_id){

	if(focus == MAIN) bandid_m = b_id;
	else bandid_s = b_id;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// get_nameptr() returns pointer to name string
//-----------------------------------------------------------------------------
char* get_nameptr(U8 focus){

	if(focus == MAIN) return memname[bandid_m];
	else return memname[bandid_s];
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_bandnv() writes the band ids to NV ram
//-----------------------------------------------------------------------------
void set_bandnv(void){

	rw8_nvr(BIDM_0, bandid_m, CS_WRITE|CS_OPEN);
	rw8_nvr(BIDM_0, bandid_s, CS_WRITE|CS_CLOSE);
	return;
}

// end radio.c
