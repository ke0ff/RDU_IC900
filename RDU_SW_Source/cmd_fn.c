/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: cmd_fn.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  CLI Command Interpreter
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

#include "cmd_fn.h"						// fn protos, bit defines, rtn codes, etc.
#include "serial.h"
#include "version.h"
#include "eeprom.h"
#include "tiva_init.h"
#include "spi.h"
#include "lcd.h"
#include "adc.h"
#include "sio.h"
#include "radio.h"
#include "uxpll.h"

//=============================================================================
// local registers

U16	device_eprom_start;					// device parameter regs
U16	device_eprom_end;
U16	device_eeprom_start;
U16	device_eeprom_end;
U16	string_addr;						// string-parse next empty address
U16 sernum;								// serial number utility register
S8	device_eprom;
S8	device_eeprom;
U8	device_type;
S8	boot_len_fixed;
U16	device_max_bootlen;
U16	device_pprog;
char device_valid;
#define MAX_PRESP_BUF 80
char bcmd_resp_buf[MAX_PRESP_BUF + 10];
char* bcmd_resp_ptr;

// enum error message ID
enum err_enum{ no_response, no_device, target_timeout };

// enum list of command numerics
//	each enum corresponds to a command from the above list (lastcmd corresponds
//	with the last entry, 0xff)
const char cmd_list[] = {"B\0H\0K\0AT\0AS\0A\0D\0L\0P\0E\0F\0INFO\0NR\0NW\0NC\0U\0TI\0T\0?\0H\0VERS\0\xff"};
enum cmd_enum{ beeper,hm_data,kp_data,tst_att,tst_asc,adc_tst,dis_la,list_la,tst_pwm,tst_enc,tst_freq,info,nvrd,nvwr,nvcmd,tstuart1,timer_tst,
			   trig_la,help1,help2,vers,lastcmd,helpcmd };

#define	cmd_type	char	// define as char for list < 255, else define as int

// test init messages
U8	lcd_test_01a[] = { 0x41, 0xe0 };				// CS1
//							0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31    32    33    34    35
U8	lcd_test_01[] = { 0x5f, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf };				// CS1
U8	lcd_test_01b[] = { 0x42, 0xff, 0xdf };			// CS1

U8	lcd_test_02a[] = { 0x81, 0xe0 };				// CS2
U8	lcd_test_02[] = { 0x9f, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf };				// CS1
U8	lcd_test_02b[] = { 0x82, 0xff, 0xdf };			// CS2

U8	lcd_init_00[] = { 0x43, 0x49, 0x20, 0x00 };				// CS1
U8	lcd_init_00b[] = { 0x83, 0x49, 0x20, 0x00 };			// CS2
U8	lcd_init_01[] = { 0x44, 0xe5, 0xc0, 0xfc, 0x81 };		// ??
U8	lcd_init_02[] = { 0x44, 0xe5, 0xc0, 0xfc, 0x81 };		// ??

U8	lcd_init_tst0[] = { 0x42, 0xe0, 0xd4 };
U8	lcd_init_tst1[] = { 0x61, 1 };							// data


U8	lcd_init_03[] = { 0x43, 0xe9, 0xd4, 0x15 };
U8	lcd_init_04[] = { 0x66, 0, 2, 5, 6, 4, 1 };				// data
U8	lcd_init_05[] = { 0x41, 0xd4 };

U8	lcd_init_06[] = { 0x45, 0xe4, 0xd0, 0xd2, 0xe1, 0x15 };
U8	lcd_init_07[] = { 0x61, 0x02 };							// data
U8	lcd_init_08[] = { 0x87, 0xfe, 0xd0, 0xfe, 0xc0, 0xb2, 0xe1, 0xc0 }; //CS2
U8	lcd_init_09[] = { 0x44, 0xfd, 0x91, 0xff, 0xd0 };
U8	lcd_init_10[] = { 0x44, 0xe5, 0xc0, 0xfc, 0x81 };
U8	lcd_init_11[] = { 0x45, 0xe4, 0xd0, 0xd2, 0xe1, 0x15 };
U8	lcd_init_12[] = { 0x61, 0x02 };							// data
U8	lcd_init_13[] = { 0x82, 0xe0, 0xd6 }; 					//CS2
U8	lcd_init_14[] = { 0x43, 0xd0, 0xfe, 0xd0 };
U8	lcd_init_15[] = { 0x82, 0xe1, 0xd0 }; 					//CS2
U8	lcd_init_16[] = { 0x82, 0xe0, 0xd6 }; 					//CS2
U8	lcd_init_17[] = { 0x43, 0xd0, 0xfe, 0xd0 };
U8	lcd_init_18[] = { 0x82, 0xe1, 0xd0 }; 					//CS2
U8	lcd_init_19[] = { 0x82, 0xe8, 0x14 };					//CS2
U8	lcd_init_20[] = { 0xa5, 0x10, 0xd8, 0xd8, 0xeb, 0x10 };	// CS2 data

U8	lcd_init_21[] = { 0x42, 0x11, 0x1b };					//CS1 disp on
U8	lcd_init_21b[] = { 0x82, 0x11, 0x1b };					//CS2
U8	lcd_init_22[] = { 0x87, 0xfe, 0xd0, 0xfe, 0xc0, 0xb2, 0xe1, 0xc0 };					//CS2
U8	lcd_init_23[] = { 0x44, 0xfd, 0x91, 0xff, 0xd0 };

U8	lcd_init_L1[] = { 0x42, 0xe6, 0x14 };					// looping LCD data
U8	lcd_init_L2[] = { 0x61, 0x00 };
U8	lcd_init_L3[] = { 0x42, 0xe8, 0xb0 };

U8	lcd_init_L4[] = { 0x82, 0xfb, 0x14 };
U8	lcd_init_L5[] = { 0xa1, 0x00 };
U8	lcd_init_L6[] = { 0x82, 0xfd, 0xb0 };

//				   0123456789012345
const char un_ary[] = { "RDU-900,ke0ff\0\0\0" };					// init User SN string
const char teststr[] = { "THIS IS KE0FF " };							// test string

//=============================================================================
// local Fn declarations

void get_BCD32(char *sptr, U32 *bcdval);
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[ARG_MAX]);
cmd_type cmd_srch(char* string);
char do_cmd_help(U8 cmd_id);
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str);
void parse_ehex(char * sptr);
void disp_error(U8 errnum);
void disp_fail(char* buf, char* s, U16 c, U16 d);
void disp_wait_addr(char* buf);
U16 boot_se(U8* bptr, U16 start, U16 end, U16 ppaddr);
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a);
void disp_error_log(U8* lbuf, U16 len);
void do_help(void);
void disp_esc(char flag);

//=============================================================================
// CLI cmd processor entry point
//	Uses number of args (nargs) and arg pointer array (args[]) to lookup
//	command and execute.
//	offset is srecord offset value which is maintianed in main() and passed
//	forward to srecord functions (upload)
//=============================================================================

int x_cmdfn(U8 nargs, char* args[ARG_MAX], U16* offset){

#define	OBUF_LEN 110				// buffer length
#define	MAX_LOG_ERRS (OBUF_LEN / 4)	// max number of errors logged by verify cmd
	char	obuf[OBUF_LEN];			// gp output buffer
//	char	abuf[30];				// temp string buffer
//	char	bbuf[6];				// temp string buffer
	U16		params[ARG_MAX];		// holding array for numeric params
	char	c;						// char temp
//	char	d;						// char temp
	char	pc = FALSE;				// C flag (set if "-C" found in args)
	char	pw = FALSE;				// W flag (set if "-W" found in args)
	char	px = FALSE;				// X flag (set if "-X" found in args)
	char	ps = FALSE;				// s flag (set if <wsp>-S<wsp> found in args)
	char	pv = FALSE;				// V flag (set if <wsp>-V<wsp> found in args)
	char	pr = FALSE;				// R flag (set if <wsp>-R<wsp> found in args)
	int		cmd_found = TRUE;		// default to true, set to false if no valid cmd identified
//	char*	q;						// char temp pointer
	char*	s;						// char temp pointer
	char*	t;						// char temp pointer
	char	cmd_id;					// command enumerated id
//	U8		g;						// temp
//	U8		h;						// temp
	U8		i;						// temp
	U8		j;						// temp
	U8		l;						// temp
	U8		m;						// temp
//	U8*		ptr0;					// U8 mem pointer
	U8		dbuf[10];				// U8 disp buf
	S8		si;						// temp s
	S8		sj;
	U16		k;						// U16 temp
	U16		kk;						// U16 temp
//	U16		hh;						// U16 temp
	U16		adc_buf[8];				// adc buffer
	U16*	ip;						// U16 pointer
//	U16		tadc_buf[8];			// adc buffer
	uint32_t ii;
	uint32_t jj;
//	volatile uint32_t* pii;					// u32 pointer
//	S32		si;
	float	fa;
	float	fb;
	char	gp_buf[16];				// gen-purpose buffer

	bchar = '\0';																// clear global escape
    if (nargs > 0){
		for(i = 0; i <= nargs; i++){											//upcase all args
			s = args[i];
			str_toupper(s);
		}
		t = args[0];															// point to first arg (cmd)
		cmd_id = cmd_srch(t);													// search for command
		s = args[1];															// point to 2nd arg (1st param)
		if(*s == '?'){
			if((cmd_id == help1) || (cmd_id == help2)){
				do_help();														// set to list all cmd helps
				putsQ("");
				for(i = 0; i < lastcmd; i++){
					if(do_cmd_help(i)) putsQ("");
				}
			}else{
				do_cmd_help(cmd_id);											// do help for cmd only
			}
		}else{
			c = parm_srch(nargs, args, "-S");									// capture minus floater
			if(c){
				ps = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-V");									// capture v-flag floater
			if(c){
				pv = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-C");									// capture c-flag floater
			if(c){
				pc = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-W");									// capture w-flag floater
			if(c){
				pw = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-X");									// capture x-flag select floater
			if(c){
				px = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-R");									// capture x-flag select floater
			if(c){
				pr = TRUE;
				nargs--;
			}
			gas_gage(2);														// init gas gauge to disabled state
			switch(cmd_id){														// dispatch command
				case help1:
				case help2:														// MAIN HELP
					do_help();
					break;

				case info:														// info + version
					putsQ("SYSINFO:");
					sprintf(obuf,"LIM_END = %d",LIM_END);
					putsQ(obuf);
					sprintf(obuf,"MEM_LEN = %d",MEM_LEN);
					putsQ(obuf);
					sprintf(obuf,"NUM_MEMS = %d",NUM_MEMS);
					putsQ(obuf);
					sprintf(obuf,"Last NVRAM = %d (0x%04x)",MEM_END,MEM_END);
					putsQ(obuf);
					sprintf(obuf,"NVREV = 0x%04x",nvram_sn());
					putsQ(obuf);
				case vers:														// SW VERSION CMD
					dispSWvers();
					break;

				case beeper:													// beeper test
					putsQ("beep\n");
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					if(params[1] > 775){
						set_beep(params[1], (U16)(((U32)params[1] * 75L)/1000L));
					}
					do_beep(params[0]);
					break;

				case hm_data:													// debug, send SO] data (2 concat 16b params)
					if(pw){
						init_radio();
						break;
					}
					if(pv){
						if(*args[1]){									// if non-empty param...
							get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
							ii = ((uint32_t)params[0] << 16)|(uint32_t)params[1];
							sprintf(obuf,"SO data: %08x",ii);
							putsQ(obuf);
							send_so(ii);
							i = 30;
							do{
								ii = get_sin();
								if(ii){
									sprintf(obuf,"Si data: %08x",ii);
									putsQ(obuf);
								}
								wait(2);
							}while(i--);
						}
					}else{
						putsQ("SIN capture...");
						jj = 0;
						i = 75;
						do{
							ii = get_sin();
							if(ii){
								if(ii != jj){
									sprintf(obuf,"Si data: %08x",ii);
									putsQ(obuf);
									jj = ii;
								}else{
									if(!i--){
										i = 75;
										putdchQ('.');
									}
								}
							}
						}while(bchar != ESC);
					}
					break;

				case nvrd:
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					if(params[1] > params[0]){
						j = CS_OPEN;
						putssQ("NVRD:");
						l = 0;
						for(ii=(U32)params[0]; ii<(U32)params[1]+1; ii++){
							if(ii == params[1]) j = CS_CLOSE;
							i = rw8_nvr(ii, 0, j);
							j = 0;
							if(l == 0){
								sprintf(obuf,"\n%05x: %02x",ii, i);
								putssQ(obuf);
								l = 7;
							}else{
								sprintf(obuf," %02x", i);
								putssQ(obuf);
								l--;
							}
						}
						putsQ("\n");
					}else{
						i = rw8_nvr((U32)params[0], 0, CS_OPENCLOSE);
						sprintf(obuf,"NVRD: %05x: %02x",(U32)params[0], i);
						putsQ(obuf);
					}
					break;

				case nvwr:
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					if(px){
						putssQ("NV descending fill...");
						i = 0xff;
						j = CS_WRITE | CS_OPEN;
						for(ii=(U32)params[0]; ii<(U32)params[1]+1; ii++){
							if(ii == params[1]) j = CS_WRITE | CS_CLOSE;
							rw8_nvr(ii, i, j);
							j = CS_WRITE;
							if(l == 0){
								putssQ(".");
								l = 7;
							}else{
								l--;
							}
						}
						putsQ("\n");
					}else{
						rw8_nvr((U32)params[0], (U8)params[1], CS_WRITE|CS_OPENCLOSE);
						i = rw8_nvr((U32)params[0], 0, CS_OPENCLOSE);
						sprintf(obuf,"NVWR: %05x: %02x",(U32)params[0], i);
						putsQ(obuf);
					}
					break;

				case nvcmd:
					if(px){
						for(i=0; i<16; i++){
							gp_buf[i] = un_ary[i];
							rwusn_nvr((U8*)gp_buf, CS_WRITE);
						}
					}
					if(pc){
						wen_nvr();
						putsQ("enable NV writes");
					}
					if(ps){
						for(i=0; i<16; i++){
							gp_buf[i] = 0xff;
						}
						i = 0;
						if(pw) i = CS_WRITE;
						params[0] = 0;
						get_Dargs(1, nargs, args, params);						// parse param numerics into params[] array
						rws_nvr((U32)params[0], i);
						i = rws_nvr(0, 0);
						rwusn_nvr((U8*)gp_buf, 0);
						sprintf(obuf,"NV Status: %02x", i);
						putsQ("User SN:");
						for(i=0; i<16; i++){
							sprintf(obuf," %02x", gp_buf[i]);
							putssQ(obuf);
						}
						putsQ(" ");
						i = gp_buf[13];										// place a temp null, display, then restore the location of the null
						gp_buf[13] = '\0';
						putsQ(gp_buf);
						gp_buf[13] = i;
					}
					if(pr){
						storecall_nvr(0);
						putsQ("RECALL NV");
					}
					break;

				case kp_data:													// debug, disp T3 capture buf
					sprintf(obuf,"get key data: %08x",ii);
					putsQ(obuf);
					do{
						if(got_key()){
							putchar_b(get_key());
						}
					}while(bchar != ESC);
					putsQ("\n");
					break;

				case tst_att:
					params[0] = 0;
					get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
					sprintf(obuf,"atten: %06x",atten_calc((U8)params[0]));
					putsQ(obuf);
					break;

				case timer_tst:
					params[1] = 0;
					get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
					if(params[1] == 1){
						GPIO_PORTB_DATA_R |= CHECK;
					}else{
						GPIO_PORTB_DATA_R &= ~CHECK;
					}
					putsQ("Timer test (esc to exit)...");
					do{
//						GPIO_PORTC_DATA_R ^= SPARE2;
						wait(100);
					}while(bchar != ESC);
					sprintf(obuf,"PortB: %02x",GPIO_PORTB_DATA_R);
					putsQ(obuf);
					break;

				case tst_pwm:
					if(*args[1]){									// adjust pwm values
						get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
						sscanf(args[2],"%f",&fa);					// get float
						if( fa > 99.1) fa = 0;						// default to zero setpoint if invalid percentage
						k = (PWM_ZERO - PWM_MAX);
						sprintf(obuf,"MAX: %u, MIN: %u",PWM_MAX,PWM_ZERO);
						putsQ(obuf);
						sprintf(obuf,"span: %u",k);
						putsQ(obuf);
						kk = (U16)((float)k * fa / 100.0);
						kk += 1;
						k = PWM_ZERO - kk;
						sprintf(obuf,"Set PWM for LED%u: %u",params[0],k);
						putsQ(obuf);
						switch(params[0]){
							case 2:
								PWM1_1_CMPB_R = k;				// LED2
								break;

							case 3:
								PWM1_2_CMPA_R = k;				// LED3
								break;

							case 4:
								PWM1_2_CMPB_R = k;				// LED4
								break;

							case 5:
								PWM1_3_CMPA_R = k;				// LED5
								break;

							case 6:
								PWM1_3_CMPB_R = k;				// LED6
								break;
						}
					}
//					do{
//					}while(pw && (bchar != ESC));
					break;

/*				case tst_tempa:
					i = 0;														// default resolution = 0.5C
					if(*args[1]){												// 0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
						i = (*args[1]) & 0x03;
					}
					k = (75 << i) + 75;											// tmeas(max) = 75 * 2^i (ms)
					i <<= 5;
//					I2C_Send2(ADDR_TSENS0, PTR_CONFIG, i);
//					I2C_Send1(ADDR_TSENS0, PTR_TSENSE);
					putsQ("tsense ambient:");
					disp_esc(pw);												// display "press esc to exit" if pw is true
					wait(k);													// wait for 1st measurement to be ready
					do{
//						k = I2C_Recv2(ADDR_TSENS0);								// get temp sense data
						switch(i){
						default:
						case 0:
							sprintf(obuf,"%.1f C",temp_float(k));
							break;

						case 0x20:
							sprintf(obuf,"%.2f C",temp_float(k));
							break;

						case 0x40:
							sprintf(obuf,"%.3f C",temp_float(k));
							break;

						case 0x60:
							sprintf(obuf,"%.4f C",temp_float(k));
							break;
						}
						putsQ(obuf);
						wait(500);
					}while(pw && (bchar != ESC));
					break;*/

				case tst_asc:
					putsQ("Test asc7:");
					putsQ(args[1]);
					obuf[0] = *args[2];
					obuf[1] = '\0';
					putsQ(obuf);
					putsQ(args[3]);
					obuf[0] = *args[4];
					obuf[1] = '\0';
					putsQ(obuf);

					c = *args[2];
					mputs_lcd(args[1], 0);
					mmem(c);

					c = *args[4];
					sputs_lcd(args[3], 0);
					smem(c);
					break;

				case tst_freq:
					if(ps){
						puts_slide(SUB, (char*)teststr, 1);
						i = 0;
						do{
							if(!slide_time(0)){
//								i = puts_slide(SUB, get_nameptr(SUB), 0);
								i = puts_slide(SUB, (char*)teststr, 0);
								slide_time(1);
							}
							if(i){
								i = 0;
								putsQ("slid");
							}
						}while(bchar != ESC);
					}else{
						jj = 0;
						ii = 670L;
						sprintf(obuf,"%4d  <",ii);
						putsQ(obuf);
						ii = 1035L;
						sprintf(obuf,"%4d  <",ii);
						putsQ(obuf);
	//					get_BCD32(args[1], &jj);									// get main bcd
	//					sscanf(args[1],"%d",&jj);									// get long decimal freq
	//					mfreq(jj, 0);
	//					jj = 0;
	//					get_BCD32(args[2], &jj);									// get sub bcd
	//					sscanf(args[2],"%d",&jj);									// get long decimal freq
	//					sfreq(jj, 0);
						params[0] = 0;
	//					params[3] = 0;
	//					params[4] = 0;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						msmet(params[0], 0);
						ssmet(params[0], 0);
	//					mtonea(params[3]);
	//					mmema(params[4]);
	//					s = args[6];
	//					mdupa(*(++s));
					}
					break;

				case adc_tst:													// CIV test cmd
					if(pw){
						// "W" displays peripheral properties reg
						ii = ADC0_PP_R;
						sprintf(obuf,"ADC0_PP: 0x%08x",ii);
						putsQ(obuf);
					}
					if(!pc){
						// normal exec is one line (no "C")
						ip = adc_buf;
						i = adc_in(ip);
						sprintf(obuf,"#samps: %u,  S0: %u,  S1: %u",i,adc_buf[1],adc_buf[7]);
						putsQ(obuf);
						if(px){
							// if "X", display status bits
							sprintf(obuf,"stats:     s0: %04x, s1: %04x",adc_buf[0],adc_buf[6]);
							putsQ(obuf);
						}
					}else{
						// if "C", display milti-line (until ESC)
						do{														// display ascii version of msg
							ip = adc_buf;
							i = adc_in(ip);
							j = (2 * i) - 1;
							sprintf(obuf,"#samps: %u,  R0: %u,  R3: %u",i,adc_buf[1],adc_buf[j]);
							putsQ(obuf);
							if(pv){
								// if "V", display eng units
								//	TJ = 147.5 - (75 * (rawADC * 3.29 / 4096))
								fa = 147.5 - ((float)adc_buf[j] * 60.242E-3);
								fb = (float)adc_buf[1] * 803.223E-6;
								sprintf(obuf,"eng:  V0: %.2f V, Tj: %.2f C",fb,fa);
								putsQ(obuf);
								ii = ((1475 * 4096) - (75 * 33 * (U32)adc_buf[j])) / 40960;
								sprintf(obuf,"eng:  V0: %.2f V, Tj: %3d C",fb,ii);
								putsQ(obuf);
							}
							if(px){
								// if "X", display status bits
								sprintf(obuf,"stats:     s0: %04x, s3: %04x",adc_buf[0],adc_buf[j-1]);
								putsQ(obuf);
							}
							waitpio(100);
						}while(bchar != ESC);
					}
					break;

/*				case log_data:													// log data to com port
					// display log data:  F ?/t ms per sample (1000 default)
					// log continuous if "W" param included
					j = 0;
					k = 0;
					do{
						if(j == 0){
							putsQ("SAMP#, FAN Rps, TA, TH, TJ, IM, PF, PR");	// put up banner every 50 lines
							j = 50;
						}
						j -= 1;
						k += 1;
						ii = 0;
						fa = 30.0 * (float)SYSCLK / (float)ii;
						sprintf(obuf,"%u, %.0f,",k, fa);						// samp#, fan rps
						putsQ(obuf);
						fa = get_temp(0, i);
						sprintf(obuf," %.4f,",fa);								// TA
						putsQ(obuf);
						fa = get_temp(1, i);
						sprintf(obuf," %.4f,",fa);								// TH
						putsQ(obuf);
//						ip = adc_buf;
//						i = adc_in(ip);
						//	TJ = (-75 * ((rawADC * 3.3 / 4096) - 2.7)) - 55
						fa = -75.0 * ((adc_buf[7]) * 3.3 / 4096.0);
						fa += 147.5;
						sprintf(obuf," %.4f,",fa);								// TJ
						putsQ(obuf);
						sprintf(obuf," %u, %u, %u",adc_buf[1],adc_buf[3],adc_buf[5]);	// IM, PF, PF as raw unsigned
						putsQ(obuf);
						waitpio(1000);											// wait with process update
					}while(pw && (bchar != ESC));
					break;*/

				case tst_enc:													// pwr-off (sleep)
					putsQ("DIAL Debug (ESC to exit):");
					set_dial(0);
					sj = 1;
					do{
						si = get_dial(0);
						if(si != sj){
							sprintf(obuf,"DIAL: %d",si);
							putsQ(obuf);
							sj = si;
						}
					}while(bchar != ESC);

/*					//	read position of enc 2 (SW encoder)
					putsQ("ENC Debug:");
					k = QEI0_POS_R;
					kk = QEI1_POS_R;
					do{
						d = get_key();
						if(d){
							sprintf(obuf,"%c",d);
							putsQ(obuf);
						}
						//	read position:
						//	x = QEI0_POS_R;
						hh = QEI0_POS_R;
						if(hh != k){
							sprintf(obuf,"ENC0: %04x, DIR: %1x",QEI0_POS_R,QEI0_STAT_R & QEI_STAT_DIRECTION);
							putsQ(obuf);
							k = hh;
						}
						hh = QEI1_POS_R;
						if(hh != kk){
							sprintf(obuf,"ENC1: %04x, DIR: %1x",QEI1_POS_R,QEI1_STAT_R & QEI_STAT_DIRECTION);
							putsQ(obuf);
							kk = hh;
						}
						//	rot dir:
						//	dir = QEI0_STAT_R | QEI_STAT_DIRECTION; // 0 = fwd, 1 = reverse
						//	dir = QEI0_STAT_R | QEI_STAT_ERROR; // 0 = OK, 1 = error in gray code
						if(get_encstat2(1)){
							sprintf(obuf,"ENC2: %04x",get_pos2());
							putsQ(obuf);
						}
						if(get_encstat3(1)){
							sprintf(obuf,"ENC3: %04x",get_pos3());
							putsQ(obuf);
						}
					}while(bchar != ESC);*/
					break;

				case tstuart1:													// HOST MEM WR CMD
					// Test Uart0:  TSTU0 ?/<string>/W to loop
					ii = 1;
					putsQ("UART0 (ACU) loop test.");
					disp_esc(pw);												// display "press esc to exit" if pw is true
					parse_ehex(args[1]);										// parse control chrs
					do{
						if(ii != 0){											// if string is ready,
					    	putsQ("Sent: ");									// display sent string..
							putsNQ(args[1]);										// ..with hex expanded
							puts0(args[1]);										// send test string to CCMD bus
							ii = 0;												// clear string ready flag
							t = args[1];										// point to start of param string
						}
						if(gotmsgn()){
							s = gets(obuf);									// get recvd data
							*s = '\0';											// add EOS
							putsQ("Rcvd: ");									// display rcvd string
							putsNQ(obuf);										// display with hex expanded
						}
						if(gotchr()){
							c = getchr();
							*t++ = c;											// re-fill buffer
							if((c == '\n') || (c == '\r')){
								ii = 1;											// if EOL, set string valid
								*t = '\0';										// null term
							}
						}
					}while(pw && (bchar != ESC));								// if "W" for loop, repeat until ESC
					if(pw) putsQ("");
					break;
#define	deelay	5
				case trig_la:												// LCD debug trigger
					putsQ("LCD CMD line (ESC to exit)...\n");
					bchar = 0;
					l = 0x41;
					j = 0xe0;
					m = 1;
					do{
						switch(bchar){
						default:
						case 0:
//							if(bchar != ESC) bchar = 0;
							break;

						case 'R':
							reset_lcd();											// reset the LCD chipset
							wait2(deelay);
							put_spi(lcd_init_00,3);									// send captured init data
							wait2(deelay);
							put_spi(lcd_init_00b,3);
							wait2(deelay);
							l = 0x41;
							j = 0xe0;
							putsQ("Reset\n");
							if(bchar != ESC) bchar = 0;
							break;

						case '1':
							l = 0x41;
							putsQ("CS1\n");
							if(bchar != ESC) bchar = 0;
							break;

						case '2':
							l = 0x81;
							putsQ("CS2\n");
							if(bchar != ESC) bchar = 0;
							break;

						case '3':
							l = 0x81;
							putsQ("spi loop test\n");
							do{
								wait2(deelay);
								put_spi(lcd_init_00,3);								// send captured init data
								wait2(deelay);
								put_spi(lcd_init_00b,3);
								wait2(1000);
							}while(bchar != ESC);
							if(bchar != ESC) bchar = 0;
							break;

						case 'o':
							putsQ("disp on\n");
							put_spi(lcd_init_21,3);									// send captured init data
							wait2(deelay);
							put_spi(lcd_init_21b,3);
							wait2(deelay);
							if(bchar != ESC) bchar = 0;
							break;

						case 'w':
							putsQ("wr: ");
							bchar = 0;
							while((bchar != ESC) && (!bchar));						// wait for chr
							if(bchar != ESC){
								obuf[0] = bchar;
								obuf[1] = 0;
								putsQ(obuf);
								i = (asc_hex(bchar) & 0xf) | WR_DMEM;
								dbuf[0] = l+1;
								dbuf[1] = j;
								dbuf[2] = i;
								put_spi(dbuf,3);
								sprintf(obuf,"%02x\n",i & 0x0f);
								putsQ(obuf);
							}
							if(bchar != ESC) bchar = 0;
							break;

						case 'b':
							putsQ("bl: ");
							bchar = 0;
							while((bchar != ESC) && (!bchar));						// wait for chr
							if(bchar != ESC){
								obuf[0] = bchar;
								obuf[1] = 0;
								putsQ(obuf);
								i = (asc_hex(bchar) & 0xf) | WR_BMEM;
								dbuf[0] = l+1;
								dbuf[1] = j;
								dbuf[2] = i;
								put_spi(dbuf,3);
								sprintf(obuf,"%02x\n",i & 0x0f);
								putsQ(obuf);
							}
							if(bchar != ESC) bchar = 0;
							break;

						case 'p':
							putsQ("ptr: ");
							bchar = 0;
							while((bchar != ESC) && (!bchar));
							if(bchar != ESC){
								obuf[0] = bchar;
								obuf[1] = 0;
								putsQ(obuf);
								j = (asc_hex(bchar) << 4) & 0x10;
							}
							bchar = 0;
							while((bchar != ESC) && (!bchar));						// wait for chr
							if(bchar != ESC){
								obuf[0] = bchar;
								obuf[1] = 0;
								putsQ(obuf);
								j |= (asc_hex(bchar) & 0xf) | 0xe0;
								dbuf[0] = l;
								dbuf[1] = j;
								put_spi(dbuf,3);
								sprintf(obuf,"%02x\n",j & 0x1f);
								putsQ(obuf);
							}
							if(bchar != ESC) bchar = 0;
							break;

						case 'L':
							putsQ("Lamp test.\n");
							l = 0x41;
							j = 0xe0;
							for(i=0; i<0x20; i++){
								dbuf[0] = l+1;
								dbuf[1] = j++;
								dbuf[2] = 0xdf;
								put_spi(dbuf,3);
							}
							l = 0x81;
							j = 0xe0;
							for(i=0; i<0x20; i++){
								dbuf[0] = l+1;
								dbuf[1] = j++;
								dbuf[2] = 0xdf;
								put_spi(dbuf,3);
							}
							if(bchar != ESC) bchar = 0;
							break;

						case 'c':
							putsQ("clear mem.\n");
							dbuf[0] = 0x41;
							dbuf[1] = CLR_DMEM;
							put_spi(dbuf,CS_OPENCLOSE);
							dbuf[0] = 0x81;
							put_spi(dbuf,CS_OPENCLOSE);
							if(bchar != ESC) bchar = 0;
							break;

						case 'l':
							putsQ("Lamp test2.\n");
							put_spi(lcd_test_01a,1);
							put_spi(lcd_test_01,2);
							put_spi(lcd_test_01b,3);
							put_spi(lcd_test_02a,1);
							put_spi(lcd_test_02,2);
							put_spi(lcd_test_02b,3);
							if(bchar != ESC) bchar = 0;
							break;

						case 'f':
							putsQ("freq test.\n");
//							mfreq(0x045450L, 0);
							mfreq(29450L, 0);
							sfreq(1296450L, 0);
							if(bchar != ESC) bchar = 0;
							break;

						case 's':
							putsQ("msmet.\n");
							ssmet(m, 0);
							msmet(m++, 0);
							if(m > 7) m = 0;
							if(bchar != ESC) bchar = 0;
							break;
						}
					}while(bchar != ESC);


/*					get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
					reset_lcd();											// reset the LCD chipset
					wait2(deelay);
					put_spi(lcd_init_00,3);									// send captured init data
					wait2(deelay);
					put_spi(lcd_init_00b,3);
					wait2(deelay);

					put_spi(lcd_init_tst0,1);
					wait2(deelay);
					dbuf[0] = 0x61;
					dbuf[1] = (U8)params[0];
					put_spi(dbuf,2);
					wait2(deelay);

					put_spi(lcd_init_01,3);
					wait2(deelay);
					put_spi(lcd_init_02,3);
					wait2(deelay);
					put_spi(lcd_init_03,1);
					put_spi(lcd_init_04,0);
					put_spi(lcd_init_05,2);
					wait2(deelay);
					put_spi(lcd_init_06,1);
					put_spi(lcd_init_07,2);
					wait2(deelay);
					put_spi(lcd_init_06,1);
					put_spi(lcd_init_07,2);
					wait2(deelay);
					put_spi(lcd_init_08,3);
					wait2(deelay);
					put_spi(lcd_init_09,3);
					wait2(deelay);
					put_spi(lcd_init_10,3);
					wait2(deelay);
					put_spi(lcd_init_11,1);
					put_spi(lcd_init_12,2);
					wait2(deelay);
					put_spi(lcd_init_13,3);
					wait2(deelay);
					put_spi(lcd_init_14,3);
					wait2(deelay);
					put_spi(lcd_init_15,3);
					wait2(deelay);
					put_spi(lcd_init_16,3);
					wait2(deelay);
					put_spi(lcd_init_17,3);
					wait2(deelay);
					put_spi(lcd_init_18,3);
					wait2(deelay);
					put_spi(lcd_init_19,1);
					put_spi(lcd_init_20,2);
					wait2(deelay);
					put_spi(lcd_init_21,3);
					wait2(deelay);
					put_spi(lcd_init_21b,3);
					wait2(deelay);
//					put_spi(lcd_init_22,3);
//					wait2(deelay);
//					put_spi(lcd_init_23,3);
//					wait2(deelay);
					sprintf(obuf,"LCD ON. <esc> to exit...");
					putsQ(obuf);
					do{
						put_spi(lcd_init_L1,1);
						wait2(deelay);
						put_spi(lcd_init_L2,0);
						wait2(deelay);
						put_spi(lcd_init_L3,2);
						wait2(deelay);
						put_spi(lcd_init_L4,1);
						wait2(deelay);
						put_spi(lcd_init_L5,0);
						wait2(deelay);
						put_spi(lcd_init_L6,2);
						wait2(deelay);
					}while(bchar != ESC);		*/							// repeat until ESC
					sprintf(obuf,"Done.\n");
					putsQ(obuf);
					break;

				case dis_la:												// disarm (stop) analyzer
					reset_lcd();
					sprintf(obuf,"RST LCD.\n");
					putsQ(obuf);
					break;

				case list_la:												// list analyzer
					sprintf(obuf,"UX present = %02x", get_present());
					putsQ(obuf);
					sscanf(args[1],"%d",&jj);								// get param
					if(jj == 0){
						GPIO_PORTC_DATA_R &= ~(MRX_N | SRX_N);
						GPIO_PORTD_DATA_R &= ~MTX_N;
					}
					if(jj == 1){
						GPIO_PORTC_DATA_R |= MRX_N;
					}
					if(jj == 2){
						GPIO_PORTC_DATA_R |= SRX_N;
					}
					if(jj == 3){
						GPIO_PORTD_DATA_R |= MTX_N;
					}

/*					sprintf(obuf,"DispOpr.\n");
					putsQ(obuf);

//					U8	lcd_init_tst0[] = { 0x42, 0xe0, 0xd4 };
//					U8	lcd_init_tst1[] = { 0x61, 1 };				// data

					if(params[0] > 31){
						dbuf[0] = 0x41;
						dbuf[2] = 0xd4;
						put_spi(dbuf,1);
						wait2(deelay);
						dbuf[0] = 0x61;
						dbuf[1] = (U8)params[1];
						put_spi(dbuf,3);
						wait2(deelay);
					}else{
						dbuf[0] = 0x42;
						dbuf[1] = 0xe0 | ((U8)params[0] & 0x0f);
						dbuf[2] = 0xd4;
						put_spi(dbuf,1);
						wait2(deelay);
						dbuf[0] = 0x61;
						dbuf[1] = (U8)params[1];
						put_spi(dbuf,3);
						wait2(deelay);
					}*/
					break;

				default:
				case lastcmd:													// not valid cmd
					cmd_found = FALSE;
					break;
			}
		}
    }
	if(bchar == ESC) while(gotchrQ()) getchrQ();									// if ESC, clear CLI input chrs
	return cmd_found;
}

//=============================================================================
// do_help() displays main help screen
//=============================================================================
void do_help(void){

	putsQ("ke0ff KPU Debug CMD List:");
	putsQ("Syntax: <cmd> <arg1> <arg2> ... args are optional depending on cmd.");
	putsQ("\t<arg> order is critical except for floaters.");
	putsQ("\"?\" as first <arg> gives cmd help, \"? ?\" lists all cmd help lines. When");
	putsQ("selectively entering <args>, use \"-\" for <args> that keep default value.");
	putsQ("\"=\" must precede decimal values w/o spaces. Floating <args>: these non-number");
	putsQ("<args> can appear anywhere in <arg> list: \"W\" = wait for operator\n");
	putsQ("\tAdc test\t\tpgmr software VERSion");
	putsQ("\tH: disp HM-151 data\tK: disp keypad data");
	putsQ("\tUart1 test\t\tEncoder display");
	putsQ("\tPwm test\t\tLog data");
	putsQ("\tTImer test (toggles PC4 @200ms period)");
	putsQ("\tT: temp ambient (W to loop)");
	putsQ("Supports baud rates of 115.2, 57.6, 38.4, 19.2, and 9.6 kb.  Press <Enter>");
	putsQ("as first character after reset at the desired baud rate.");
}

//=============================================================================
// do_cmd_help() displays individual cmd help using cmd_id enum
//=============================================================================
char do_cmd_help(U8 cmd_id){

	char c = TRUE;
	
	switch(cmd_id){														// dispatch help line

		case hm_data:													// set ant stop
			putsQ("Hm data: ?");
			putsQ("\tdisplay HM-151 key data, ESC to exit");
			break;

		case kp_data:													// set ant stop
			putsQ("Kp data: ?");
			putsQ("\tdisplay key-pad data, ESC to exit");
			break;

		case tstuart1:													// UART1 tst CMD
			// Test UART1:  TSTU1 ?/<string>/W: loop output of <string>
			putsQ("TSTU1 (Test UART1): ?/<string>/W (loop until ESC)");
			break;

		case adc_tst:													// ADC tst CMD
			// Test ADC:  Adc test ?
			putsQ("Adc test: ?");
			break;

		case tst_enc:													// ADC tst CMD
			// Test Enc:  Enc ?
			putsQ("Enc test: ?");
			putsQ("\tDisp encoder changes, ESC to exit.");
			break;

		case tst_pwm:													// ADC tst CMD
			// Pwm debug:  P ?
			putsQ("Pwm test: <pwm#>/<value>/?");
			putsQ("\tDisplay min/max PWM and set PWM values.");
			putsQ("\t<pwm#> = 2 - 6");
			break;

/*		case tst_tempa:													// ADC tst CMD
			// Test I2C ambient temp:  Temp a ?
			putsQ("Temp ambient: <res>/W/?");
			putsQ("\t<res>: 0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625");
			putsQ("\tW to loop (ESC to exit)");
			break;*/

		case timer_tst:													// ADC tst CMD
			// Test timer:  TI ?
			putsQ("TImer test: ?");
			putsQ("\tToggles PC4 @200ms period");
			break;

		default:
			c = FALSE;
			break;
	}
	return c;
}
/*
//=============================================================================
// disp_error() displays standard error messages & device off status
//=============================================================================
void disp_error(U8 errnum){

	do_red_led(LED_BLINK);
	switch(errnum){
		case no_response:
			putsQ("!! ERR !! No response from DUT.");
			break;

		case no_device:
			putsQ("!! ERR !! No DUT selected.");
			break;

		case target_timeout:
			putsQ("!! ERR !! DUT timeout.");
			break;

		default:
			break;
	}
	if(!dut_pwr(TARGET_STAT)) putsQ("DEVICE OFF");		// signal device off
}*/
/*
//=============================================================================
// disp_fail() displays pgm/read error message with params
//	this is a debug function that displays the HC11 fn comms status
//	Also, this fn sets the status LED to blink to indicate a failure was encountered
//=============================================================================
void disp_fail(char* buf, char* s, U16 c, U16 d){

	do_red_led(LED_BLINK);
	sprintf(buf,"%s Failed!! errs: %02x %02x", s, (U16)c, (U16)d);
	putsQ(buf);
}*/

//=============================================================================
// disp_wait_addr() dead code
//=============================================================================
void disp_wait_addr(char* buf){

	sprintf(buf,"disp_wait_addr = dead code\n");
	putsQ(buf);
}

//=============================================================================
// exec_bcmd() dead code
//=============================================================================
void exec_bcmd(char* bcmdbuf_ptr, char* obuf, U16* offset){

}

//***********************//
// bcmd functions follow //
//***********************//

//******************************//
// hosekeeping functions follow //
//******************************//



//=============================================================================
// get BCD param from string
//=============================================================================
void get_BCD32(char *sptr, U32 *bcdval){

	char*	s = sptr;

	*bcdval = 0;
	while(*s){
		*bcdval <<= 4;
		*bcdval |= asc_hex(*s++);
	}
	return;
}

//=============================================================================
// get numeric params from command line args
//	argsrt specifies the first item to be searched and aligns argsrt with params[0]
//	for multi-arg cmds, fields must be entered in order.
//=============================================================================
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[8]){

	char*	s;
	U16*	ptr1;
	S32		temp32;
	U8		i;
	U8		count = 0;

	if(argsrt < nargs){											// test for start in limit (abort if not)
		for(i = argsrt; i < nargs; i++){						// convert strings to values
			s = args[i];										// set pointers to array items
			ptr1 = &params[i - argsrt];
			switch(*s){
				case '-':										// skip if user specified default
				case '\0':										// or if arg is empty
					break;

				default:
					count += sscanf(s,"%d",&temp32);			// get decimal value
					*ptr1 = (U16)(temp32);
					break;

				case '$':
					s++;
					count += sscanf(s,"%x",&temp32);			// get hex if leading "$"
					*ptr1 = (U16)(temp32);
					break;
			}
		}
	}
	return count;
}

//=============================================================================
// search for command keywords, return cmd ID if found
//	uses the cmd_list[] array which is constructed as an array of null terminated
//	strings compressed into a single string definition.  Commands are added by
//	placing the minimum required text from the command name with a '\0' terminating
//	null.  cmd_list[] is terminated by an $ff after all of the command names.
//	cmd_enum{} holds an enumerated, named list of all valid commands.  The enum
//	definition must be at the top of this file, so the commented-out version shown
//	below must be copied and pasted to the top of the file whan any changes are
//	made (commands added or deleted).
//
//	Some thought must be put into the order of command names.  Shorter matching
//	names (e.g., single chr entries) must follow longer names to allow the algortihm
//	to properly trap the shorter cmd name.
//	
//=============================================================================
cmd_type cmd_srch(char* string){
// dummy end of list, used to break out of search loop
const char end_list[] = {0xff};
// list of minimum length command words. cmd words are separated by '\0' and list terminated with 0xff
//const char cmd_list[] = {"H\0K\0A\0D\0L\0P\0E\0U\0S\0TI\0T\0?\0H\0VERS\0\xff"};
//enum cmd_enum{ hm_data,kp_data,adc_tst,log_data,tst_pwm,tst_enc,tstuart1,timer_tst,tst_tempa,help1,help2,vers,lastcmd,helpcmd };
//!!! make changes to cmd_enum here, move them to top of file, then un-comment !!!

	char*	ptr;							// temp ptr
	char	cmdid = 0;						// start at beginning of cmd_enum
	char	i;								// temp
	char	found = FALSE;					// cmd found flag (default to not found)

	ptr = (char*)cmd_list;												// start at beginning of serach list
	while((*ptr & 0x80) != 0x80){								// process until 0xff found in search list
		i = strncmp(string, ptr, strlen(ptr));					// inbound string match search list?
		if(i){
			cmdid++;											// no, advance to next cmdid 
			while(*ptr++);										// skip to next item in search list
		}else{
			ptr = (char*)end_list;										// found match,
			found = TRUE;										// set break-out criteria
		}
	}
	if(!found) cmdid = lastcmd;									// not found, set error cmd id
	return cmdid;
}

//=============================================================================
// parm_srch() looks for a match of parm_str in any non-empty args[] strings
//	if found, remove the args entry from param list and return 1st chr of parm_str,
//	else return '\0'
//=============================================================================
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str){

	U8		i;								// counter temp
	char	c = '\0';						// first chr of matched parm_str (first time thru loop, there is no match)
	static char null_str[] = "";			// null string that persists

//	if(nargs > 1){
	    for(i = 1; i <= nargs; i++){							// search starting with first args[] item
			if(c){												// if(c!=null)...
				args[i] = args[i+1];							// if there was a match, move the subsequent pointers down one
			}else{
				if(strlen(parm_str) == strlen(args[i])){		// in order to match, the lengths have to be equal...
					if(strncmp(args[i], parm_str, strlen(parm_str)) == 0){ // look for match
						c = *parm_str;							// if match, capture 1st chr in matched string
						i--;									// back-up one to edit this item out of the list
					}
				}
			}
	    }
//	}
 	if(c != '\0'){
		args[ARG_MAX - 1] = null_str;							// if there was a match, the last pointer goes to null
		
	}
	return c;													// return first chr in matched string, or null if no match
}

//=============================================================================
// disp_esc() if param true, display "Press esc to exit" msg
//=============================================================================
void disp_esc(char flag){

	if(flag){
		putsQ("  Press <ESC> to exit.");
	}
	putsQ("");
}

//=============================================================================
// convert all chrs in string to upper case
//=============================================================================
void str_toupper(char *string){

    while(*string != '\0'){
        *string++ = toupper(*string);
    }
}

//=============================================================================
// parse string for delimited arguments
//  on exit, the args[] array holds each delimited argument from the command string input:
//  args[0] holds first arg (command)
//  args[1] holds next arg
//  args[2] etc...
//  up to args[ARG_MAX]
//
//  nargs holds number of arguments collected.  i.e., nargs = 3 specifies that args[0] .. args[3]
//      all hold arguments (three total, including the command).
//=============================================================================
int parse_args(char* cmd_string, char* args[ARG_MAX]){
	int i;
	char quote_c = 0;
	static char null_string[2] = "";

    // clear args pointers
    for (i=0; i<ARG_MAX; i++){
        args[i] = null_string;
    }
    i = 0;
    do{
        if(quotespace(*cmd_string, 0)){         // process quotes until end quote or end of string
            quote_c = *cmd_string;              // close quote must = open quote
            args[i++] = ++cmd_string;               // start args with 1st char after quote
            while(!quotespace(*cmd_string,quote_c)){
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                cmd_string++;
            }
            *cmd_string++ = '\0';               // replace end quote with null
        }
        if(*cmd_string == '\0'){
            return i;                           // end of cmd string, exit
        }
        if(!whitespace(*cmd_string)){
            args[i++] = cmd_string++;			// when non-whitespace encountered, assign arg[] pointer
            if(i > ARG_MAX){
                return i;						// until all args used up
            }
            do{
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                if(whitespace(*cmd_string)){
                    *cmd_string = '\0';			// then look for next whitespace and delimit (terminate) the arg[] string
                    break;
                }
                cmd_string++;					// loop until end of cmd_string or next whitespace
            } while (1);
        }
        cmd_string++;							// loop...
    } while (1);
}

//=============================================================================
// parse_ehex() for embeded hex ($$) arguments
//  on exit, the string holds the original text with %xx replaced by a single
//	hex byte.
//=============================================================================
void parse_ehex(char * sptr){
	char* tptr;
	U8	i;

	while(*sptr){
		if((*sptr == '$') && (*(sptr+1) == '$')){
			i = asc_hex(*(sptr+2)) << 4;
			i |= asc_hex(*(sptr+3));
			*sptr++ = i;
			tptr = sptr;
			do{
				*tptr = *(tptr+3);
				tptr++;
			}while(*(tptr+2));
		}else{
			sptr++;
		}
	}
}
//=============================================================================
// test characer for whitespace
//=============================================================================
int whitespace(char c){

    switch (c){					// These are all valid whitespace:
        case '\n':          	// newline
        case '\r':          	// cr
        case '\t':          	// tab
        case 0x20:{         	// space
		case '/':				// slash is also wsp
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// test characer for quote
//=============================================================================
int quotespace(char c, char qu_c){

    if(qu_c == '\0'){
        switch (c){				// if qu_c is null, these are valid quotes:
            case '\'':          // newline
            case '\"':          // cr
            case '\t':          // tab
                return TRUE;
            }
    } else {
        if(c == qu_c){			// else, only qu_c results in a TRUE match
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// gas_gage() display up to 16 "*" chrs based on count rate.
//	Gauge appearance:
//	[****************]	all OK
//	[***.............]	errors detected
//
//	"len" cmds:
//	0: process gauge counter/display
//	1: set gauge error character = "."
//	2: disable gage counter/display (set clen = 0)
//	all others: set creset = count = len/16, display initial gauge characters
//	This calculation identifies how many bytes are in 1/16th of the total
//	byte count (len).  For count events (len == 0), this Fn decrements count, &
//	displays a gauge chr when count == 0.  count is then reloaded with creset.
//	process continues until 16 gauge chrs have been displayed.  After this,
//	any further count events result in no further change to the display.
//=============================================================================
U8 gas_gage(U16 len){

#define LENCMD_MAX 2		// max # of gas-gage() cmds

	static U16	creset;		// holding reg for data counter reset value
	static U16	count;		// data counter
	static U8	clen;		// gage chr counter
	static U8	gchr;		// gage chr storage
		   U8	c = 0;		// gage printed flag

	if(len <= LENCMD_MAX){
		if(!len && clen){
			if(--count == 0){ 
				putchar(gchr);					// disp gage chr
				count = creset;					// reset loop counters
				clen--;
				if(clen == 0) putchar(']');		// if end of gage, print end bracket
				c = 1;
			}
		}else{
			if(len == 1) gchr = '.';			// if error flag, change gauge chr to err mode
			if(len == 2) clen = 0;				// disable gauge
		}
	}else{
		creset = count = len >> 4;				// init count & count reset (creset) = len/16
		if(creset == 0) creset = 1;				// if overall length too short, set 1:1
		clen = 16;								// 16 gage chrs max
		gchr = '*';								// set * as gage chr
		putchar('[');							// print start bracket
		for(c = 0; c < 16; c++) putchar(' ');
		putchar(']');							// place end bracket for scale
		for(c = 0; c < 17; c++) putchar('\b');	// backspace to start of scale
		c = 1;
	}
	return c;
}

//=============================================================================
// log_error_byte() places error data into log buffer.  Log format is:
//	(device) (host) (addrH) (addrL).  Called by target verify fns to allow
//	a limited number of errors to be trapped (limit is the buffer used to
//	hold the error log).
//	returns updated pointer to next available log entry
//=============================================================================
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a){

	*lbuf++ = d;								// store device data
	*lbuf++ = h;								// store host data
	*lbuf++ = (U8)(a >> 8);						// store addr
	*lbuf++ = (U8)(a & 0xff);
	return lbuf;								// return updated pointer
}

//=============================================================================
// disp_error_log() displays errors logged into error string.  Log format is:
//	(device) (host) (addrH) (addrL)
//	Display format is:
//	nn: Dev ($xx) != $xx @$xxxx\n = 28 printed chrs
//	nn = err number (ordinal)
//	xx = data bytes
//	xxxx = error address
//=============================================================================
void disp_error_log(U8* lbuf, U16 len){

	char obuf[32];				// local buffer
	// use U16 type to simplify sprintf variable list
	U16  i;						// loop counter
	U16  d;						// device data
	U16  h;						// host data
	U16  a;						// addr

	len++;										// add 1 to end so that we can start loop at "1"
	for(i = 1; i < len; i++){					// loop from 1 to len+1 entries
		d = (U16)*lbuf++ & 0xff;				// format device data
		h = (U16)*lbuf++ & 0xff;				// format host data
		a = ((U16)*lbuf++ & 0xff) << 8;			// format addr
		a |= (U16)*lbuf++ & 0xff;
		sprintf(obuf,"%02u: Dev ($%02x) != $%02x @$%04x", i, d, h, a); // display err line
		putsQ(obuf);
	}
}

//=============================================================================
// bcmd_resp_init() inits bcmd_resp_ptr
//=============================================================================
void bcmd_resp_init(void){

	bcmd_resp_ptr = bcmd_resp_buf;
	*bcmd_resp_ptr = '\0';
}

//=============================================================================
// asc_hex() converts ascii chr to 4-bit hex.  Returns 0xff if error
//=============================================================================
U8 asc_hex(S8 c){

	U8 i;

	if((c >= 'a') && (c <= 'f')) c = c - ('a' - 'A');	// upcase
	if((c >= '0') && (c <= '9')){			// if decimal digit,
		i = (U8)(c - '0');					// subtract ASCII '0' to get hex nybble
	}else{
		if((c >= 'A') && (c <= 'F')){		// if hex digit,
			i = (U8)(c - 'A' + 0x0A);		// subtract ASCII 'A', then add 0x0A to get hex nybble
		}else{
			i = 0xff;						// if not valid hex digit, set error return
		}
	}
	return i;	
}

//=============================================================================
// temp_float() converts MCP9800 binary temp to a float (degrees C)
//=============================================================================
float temp_float(U16 k){
	U8		i;			// temp
	U8		j = 0;		// temp sign
	float	fa;			// temp float

	if(k & 0x8000){												// if negative,
		j = 1;													// preserve sign and
		k = ~k + 1;												// convert value to positive
	}
	i = k >> 8;													// get integer portion
	fa = (float)i;												// convert to float
	if(k & 0x0080) fa += 0.5;									// add fractional portion
	if(k & 0x0040) fa += 0.25;
	if(k & 0x0020) fa += 0.125;
	if(k & 0x0010) fa += 0.0625;
	if(j){														// if negative, convert
		fa *= -1;
	}
	return fa;
}
