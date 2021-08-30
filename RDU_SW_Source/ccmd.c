/********************************************************************
 ************ COPYRIGHT (c) 2019 by ke0ff, Taylor, TX   *************
 *
 *  File name: ccmd.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  CCMD Command Interpreter source
 *
 *******************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
//#include <intrins.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "ccmd.h"

#include "cmd_fn.h"
#include "serial.h"
#include "version.h"
#include "eeprom.h"

//=============================================================================
// local registers


//=============================================================================
// local Fn declarations
U8 count_chrs(char* ptr);


//=============================================================================
// CCMD entry point
//	Processes CCMD commands from ACU.  Commands end with '\r'.
//	len		cmd descr.
//	3		"aXX" = set ACU loop time, "XX" = hex value
//				ACU loop time is the blackout time from the last message until the next update.
//				increasing the value reduces the serial message bandwidth, but increases the data
//				granularity of the encoder readings.  Valid range is 0 - 254 (ms).
//				"XX" = "ff" resets value to the compile-time default
//	2		"eN" = request encoder, "N" = enc# (0-3)
//	6		"fNXXXX" = set encoder, "N" = enc#, "XXXX" = hex value
//	4		"iNXX"  = set max LED level, N = LED#, "XX" = hex % value (0x00 - 0x63)
//				N = 0 is master level (does not apply to LED5)
//				N = 2 is dial illum
//				N = 3 is keypad illum
//				N = 4 is function LED
//				N = 5 is RESP LED
//				N = 6 is HM-151 F2 LED
//	3		"LNX"  = set LED state, N = LED#, "X" = on/off (1/0)
//	5		"reset" = warm restart (!! need to finish function in main.c to perform the reg inits !!)
//	2		"dN" = for N==1, enable HM151 data to IC7K, else, disable
//	2		"s7" = set hm151->ic7k, hsmic->acu (PTTb = ~/PTT2)
//	2		"sm" = set hsmic->acu, hfmod->ic7k (PTTb = ~/PTT2)
//	2		"sh" = set hm151->acu, hfmod->ic7k (PTTb = ~/PTTHM, PTT7K = ccmd driven)
//			-- 03/03/19: removed accesses to data control.  Must execute "d" separately from "s"
//	2		"t0" = temperature request (decimal, float, "TT.tt" format)
//	2		"t1" = light level request (decimal, int, raw adc)
//	2		"t2" = free-running timer value request (displays number of ms since last reset)
//	1		"v" = version request
//	1		"k" = activate ptt7k (if SW enabled)
//	1		"u" = de-activate ptt7k
//
//	Commands that have no response send an "=" response to acknowledge a proper
//		command syntax.  For error syntax, "~err" is sent.  Otherwise, the data
//		response is the acknowledgement.  Syntax looks at ccmd length (must be exactly
//		what is listed above) and range checks to params.
//	Max TX buffer is 100 chrs.  Typical version string is longest message at about 34 chrs.
//=============================================================================
void process_CCMD(U8 cmd){
#define CBUF_MAX_LEN 100		// max buffer length
#define	A_COUNT		3			// cmd chr counts
#define	E_COUNT		2
#define	F_COUNT		6
#define	I_COUNT		4
#define	L_COUNT		3
#define	l_COUNT		1
#define	R_COUNT		5
#define	D_COUNT		1
#define	S_COUNT		2
#define	T_COUNT		1
#define	V_COUNT		1
#define	K_COUNT		1
#define	U_COUNT		1

				char	ccbuf[CBUF_MAX_LEN];	// buf string
				char	i;						// param temp
				char*	ptr;					// ptr
				char*	rptr;					// response ptr
				char	err_flag = FALSE;
				U8		cclen;					// length temps
				U8		l;
//				U8		m;
//				U16		ii;						// U16 temp
//				U16		jj;						// U16 temp
				U32		temp32;					// param temp
//				U16		adc_buf[8];				// adc buffer

	if(cmd == CCMD_IPL){						// do IPL init
		ccmd_timer = 0;
		puts1("");
		ccmdSWvers(ccbuf);						// on init, broadcst SW version..
		puts1(ccbuf);
		return;
	}
	if(gotmsgn1()){								// if ccmd received, process here...
		// process ccmds
		ptr = ccbuf;							// set pointer to local buffer
		gets1(ptr);								// xfr message string to local buffer
		cclen = count_chrs(ptr);				// get cmd chr count
		switch(*ptr){							// select valid count based on cmd chr
		case 'a':
			l = A_COUNT;
			break;

		case 'd':
			l = D_COUNT;
			break;

		case 'e':
			l = E_COUNT;
			break;

		case 'f':
			l = F_COUNT;
			break;

		case 'i':
			l = I_COUNT;
			break;

		case 'l':
			l = l_COUNT;
			break;

		case 'r':
			l = R_COUNT;
			break;

		case 's':
			l = S_COUNT;
			break;

		case 't':
			l = T_COUNT;
			break;

		case 'k':
			l = K_COUNT;
			break;

		case 'u':
			l = U_COUNT;
			break;

		case 'v':
			l = V_COUNT;
			break;

		default:
			l = 0;
			break;
		}
		if(l != cclen){
			err_flag = TRUE;						// set err reply and abort
		}else{
			rptr = ccbuf;
			switch(*ptr){
			case 'r':							// reset
				if(*++ptr != 'e') err_flag = TRUE;
				if(*++ptr != 's') err_flag = TRUE;
				if(*++ptr != 'e') err_flag = TRUE;
				if(*++ptr != 't') err_flag = TRUE;
				if(!err_flag) warm_reset();
				break;

			case 'v':							// return SW version string
				ccmdSWvers(rptr);					// return SW version..
				break;

			case 'a':							// set ACU loop value
				ptr++;
				temp32 = 0xfe000000;				// set to invalid value
				sscanf(ptr,"%x",&temp32);			// get hex value
				if(temp32 < 0x100){
//					set_acu_loop((U8)temp32);		// set new value
					sprintf(rptr,"=");				// OK reply
				}else{
					err_flag = TRUE;				// set err reply
				}
				break;

			case 'e':							// read encoder value
				ptr++;
				i = *ptr++;							// convert param to binary
				if((i > '3') || (i < '0')){
					i = 0x7f;
				}else{
					i = i - '0';
				}
				switch(i){
				case 0:
//					ii = get_pos0();
					break;

				case 1:
//					ii = get_pos1();
					break;

				case 2:
//					ii = get_pos2();
					break;

				case 3:
//					ii = get_pos3();
					break;

				default:
					err_flag = TRUE;				// set err reply
					break;
				}
				if(!err_flag){
//					sprintf(rptr,"e%u%04x",i,ii);
				}
				break;

			case 'f':							// set encoder value
				ptr++;
				i = *ptr++;						// convert param to binary
				if((i > '3') || (i < '0')){
					i = 0x7f;
				}else{
					i = i - '0';
				}
				temp32 = 0xfe000000;				// set to invalid value
				sscanf(ptr,"%x",&temp32);			// get hex value
				if(temp32 < 0x10000){
					switch(i){
					case 0:
//						set_pos0((U16)temp32);
						break;

					case 1:
//						set_pos1((U16)temp32);
						break;

					case 2:
//						set_pos2((U16)temp32);
						break;

					case 3:
//						set_pos3((U16)temp32);
						break;

					default:
						err_flag = TRUE;			// set err reply
						break;
					}
					sprintf(rptr,"=");				// OK reply
				}else{
					err_flag = TRUE;				// set err reply
				}
				break;

			case 'i':							// set max LED PWM value
				ptr++;
				i = *ptr++;						// convert param to binary
				if(((i > '6') || (i < '0')) || (i == '1')){
					i = 0x7f;
				}else{
					i = i - '0';
				}
				temp32 = 0xfe000000;				// set to invalid value
				sscanf(ptr,"%x",&temp32);			// get hex value
				if((temp32 < 100) && (i != 0x7f)){
					if(i == 0){
						set_pwm(0,(U8)temp32);		// set new master value
						for(i=2; i<=6; i++){
							set_pwm(i,0xff);		// update all PWMs with new master value
						}
					}else{
						set_pwm(i,(U8)temp32);		// set new value
					}
					sprintf(rptr,"=");				// OK reply
				}else{
					err_flag = TRUE;				// set err reply
				}
				break;

/*			case 'L':							// set LED on/off
				ptr++;
				i = *ptr++;							// convert param to binary
				if(((i > '6') || (i < '0')) || (i == '1')){
					i = 0x7f;
				}else{
					i = i - '0';
				}
				l = *ptr++;							// convert param to binary
				if((l > '1') || (l < '0')){
					l = 0x7f;
				}else{
					l = l - '0';
				}
				if((i != 0x7f) && (l != 0x7f)){
					if(i == 0){
						for(i=2; i<=6; i++){
							set_led(i,l);			// turn on/off all
						}
					}else{
						set_led(i,l);				// turn led on/off
					}
					sprintf(rptr,"=");				// OK reply
				}else{
					err_flag = TRUE;				// set err reply
				}
				break;*/

			case 's':
				ptr++;
				i = *ptr++;								// get sub-cmd
				sprintf(rptr,"=");						// pre-set OK reply
				switch(i){
				case '7':							// mic data switch
//					GPIO_PORTD_DATA_R |= HMICEN;		// hm151->ic7k
//					GPIO_PORTC_DATA_R &= ~nMIC2EN;		// hsmic->acu
//					GPIO_PORTD_DATA_R |= n7KDEN;		// enable HM-151 data to IC7k
//					GPIO_PORTB_DATA_R &= ~PTT7K;		// disable SW ptt to IC7K
					set_pttmode(0);						// pttb follows ptt2
					break;

				case 'm':							// mic switch
//					GPIO_PORTD_DATA_R &= ~HMICEN;		// hfmod->ic7k
//					GPIO_PORTC_DATA_R &= ~nMIC2EN;		// hsmic->acu
//					GPIO_PORTD_DATA_R |= n7KDEN;		// disable HM-151 data to IC7k
//					GPIO_PORTB_DATA_R &= ~PTT7K;		// disable SW ptt to IC7K
					set_pttmode(0);						// pttb follows ptt2
					break;

				case 'h':							// HF mod switch
//					GPIO_PORTD_DATA_R &= ~HMICEN;		// hfmod(acu)->ic7k
//					GPIO_PORTC_DATA_R |= nMIC2EN;		// hm151mic->acu
//					GPIO_PORTD_DATA_R &= ~n7KDEN;		// enable HM-151 data to IC7k
//					GPIO_PORTB_DATA_R &= ~PTT7K;		// disable SW ptt to IC7K
					set_pttmode(1);						// pttb follows ptthm, ptt7k enabled
					break;

				default:
					err_flag = TRUE;					// set err reply
					break;
				}
				break;

			case 'k':							// activate PTT7K if enabled
				if(set_pttmode(0xff) == 1){
//					GPIO_PORTB_DATA_R |= PTT7K;		// key PTT7K
				}
				*rptr++ = '=';						// set OK response (fast)
				*rptr = '\0';
				break;

			case 'u':							// de-activate PTT7K (always)
//				GPIO_PORTB_DATA_R &= ~PTT7K;		// un-key PTT7K
				*rptr++ = '=';						// set OK response (fast)
				*rptr = '\0';
				break;

			default:
				err_flag = TRUE;					// set err reply
				break;
			}
		}
		if(err_flag){
			sprintf(ccbuf,"~err");				// error response
		}
		puts1(ccbuf);							// if ccmd received, there is always going to be a response
	}
}

//-----------------------------------------------------------------------------
// count_chrs() counts # chrs up to (not incl) CR.  If invalid chr detected,
//	returns 0
//-----------------------------------------------------------------------------
U8 count_chrs(char* ptr){
	U8	i = 0;			// count temp
	U8	f = FALSE;		// chr error flag, true if invalid chr detected
	char c;

	do{
		c = *ptr++;									// capture chr from buffer
		if((c <= ESC) || (c > 'z')) f = TRUE;		// set error if invalid chr
	}while((++i < CBUF_MAX_LEN) && (*ptr != '\r'));	// count until CR or end of buff
	if(f) i = 0;									// error return
	return i;
}

//-----------------------------------------------------------------------------
// get_temp() returns temperature reading, chan = 0 is TA, chan = 1 is TH
//	res is:  0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
//-----------------------------------------------------------------------------
float get_temp(U8 chan, U8 res){
	U8	i;					// temp resolution
	U16	k;					// raw data temp

	i = res;
	i <<= 5;													// align resolution to device bit field
	k = (75 << res) + 75;										// calc measurement time: tmeas(max) = 75 * 2^i (ms)
	switch(chan){
	case 0:
		wait(k);
		break;

	case 1:
		wait(k);
		break;
	}
	return temp_float(k);
}
