/********************************************************************
 ************ COPYRIGHT (c) 2022 by ke0ff, Taylor, TX   *************
 *
 *  File name: cmd_fn.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  CLI Command Interpreter
 *  
 *******************************************************************/

// generally required includes
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "typedef.h"

// application-specific includes
#include "inc/tm4c123gh6pm.h"
#include "init.h"						// App-specific SFR Definitions
#include "cmd_fn.h"						// fn protos, bit defines, rtn codes, etc.
#include "dpl_fn.h"						// dpl extension cmd/support.
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

//===================================================================================================
// Command token syntax defines and instantiations.
//
// This is the define list for the command line tokens used in this CLI.  Each "CMD" define is a quoted string that
//	corresponds to the minimum typed command line token.  The "ENUM" define sets the token used in the parsing
//	switch.  The "CMD" defines are entered into an array of arrays with a terminating array of 0xff, eg:
//		char* cmd_list[] = { CMD_1, CMD_2, ... , "\xff" };
//	The "ENUM" defines are likewise entered into an "enum" statement, eg:
//		enum cmd_enum{ ENUM_1, ENUM_2, ... , ENUM_LAST };
//	The primary requirement is for the subscript numbers for the "CMD" and "ENUM" must be in matching order.
//		The list order of the CMD/ENUM defines establishes the search order.  Lower-indexed (left-most) items are
//		searched first.  This means that the least-ambiguous items should be included earlier in the list to prevent
//		situations whereby a command of "BT" is entered, but matches on "B" in the list because it was the first
//		encountered.  So, for this example, "BT" should be first in the
//
// cmd_type is adjusted to the number of tokens in the list:

#define	cmd_type	char	// define as char for list < 255, else define as int

#define	CMD_1		"~"				// hm-133 wired remote command ID
#define	ENUM_1		hm_cmd
#define	CMD_2		"bt"			// Bluetooth debug command
#define	ENUM_2		bttest
#define	CMD_3		"b"				// beeper debug
#define	ENUM_3		beeper
#define	CMD_31		"help"			// Alternate help syntax
#define	ENUM_31		help2
#define	CMD_32		"hi"			// hi/lo power debug
#define	ENUM_32		hilo
#define	CMD_4		"h"				// HM-133 data debug
#define	ENUM_4		hm_data
#define	CMD_41		"key"			// HM-133 key signal
#define	ENUM_41		set_hmkey
#define	CMD_5		"k"				// KPU debug data
#define	ENUM_5		kp_data
#define	CMD_6		"at"
#define	ENUM_6		tst_att
#define	CMD_7		"as"
#define	ENUM_7		tst_asc
#define	CMD_8		"a"
#define	ENUM_8		adc_tst
#define	CMD_81		"dup"
#define	ENUM_81		dup
#define	CMD_9		"d"
#define	ENUM_9		dis_la
#define	CMD_10		"l"
#define	ENUM_10		list_la
#define	CMD_101		"o"
#define	ENUM_101	set_offset
#define	CMD_102		"ptts"
#define	ENUM_102	pttsub
#define	CMD_11		"p"
#define	ENUM_11		tst_pwm
#define	CMD_12		"e"
#define	ENUM_12		tst_enc
#define	CMD_13		"frq"
#define	ENUM_13		tst_freq
#define	CMD_131		"f"
#define	ENUM_131	set_freq
#define	CMD_14		"info"
#define	ENUM_14		info
#define	CMD_15		"mstr"
#define	ENUM_15		mstr
#define	CMD_160		"nvall"
#define	ENUM_160	setnvall
#define	CMD_16		"nr"
#define	ENUM_16		nvrd
#define	CMD_17		"nw"
#define	ENUM_17		nvwr
#define	CMD_18		"nc"
#define	ENUM_18		nvcmd
#define	CMD_19		"u"
#define	ENUM_19		tstuart1
#define	CMD_20		"scan"
#define	ENUM_20		scan_cmd
#define	CMD_21		"sto"			// store memory data
#define	ENUM_21		sto_mem
#define	CMD_210		"s"				// squelch
#define	ENUM_210	squc
#define	CMD_211		"to"
#define	ENUM_211	tone
#define	CMD_212		"tsa"
#define	ENUM_212	tsac
#define	CMD_213		"tsb"
#define	ENUM_213	tsbc
#define	CMD_22		"ti"
#define	ENUM_22		timer_tst
#define	CMD_23		"t"
#define	ENUM_23		trig_la
#define	CMD_24		"?"				// help list
#define	ENUM_24		help1
#define	CMD_26		"vers"			// version info
#define	ENUM_26		vers
#define	CMD_27		"v"			// version info
#define	ENUM_27		volc
#define	ENUM_LAST	lastcmd


char* cmd_list[] = { CMD_1, CMD_2, CMD_3, CMD_31, CMD_32, CMD_4, CMD_41, CMD_5, CMD_6, CMD_7, CMD_8, CMD_81, CMD_9, CMD_10, CMD_101, CMD_102, CMD_11, \
				     CMD_12, CMD_13, CMD_131, CMD_14, CMD_15, CMD_160, CMD_16, CMD_17, CMD_18, CMD_19, CMD_20, CMD_21, CMD_210, CMD_211, CMD_212, CMD_213, \
				     CMD_22, CMD_23, CMD_24, CMD_26, CMD_27, "\xff" };

enum       cmd_enum{ ENUM_1, ENUM_2, ENUM_3, ENUM_31, ENUM_32, ENUM_4, ENUM_41, ENUM_5, ENUM_6, ENUM_7, ENUM_8, ENUM_81, ENUM_9, ENUM_10, ENUM_101, ENUM_102, ENUM_11, \
	   	   	   	   	 ENUM_12, ENUM_13, ENUM_131, ENUM_14, ENUM_15, ENUM_160, ENUM_16, ENUM_17, ENUM_18, ENUM_19, ENUM_20, ENUM_21, ENUM_210, ENUM_211, ENUM_212, ENUM_213, \
					 ENUM_22, ENUM_23, ENUM_24, ENUM_26, ENUM_27, ENUM_LAST };

// enum error message ID
enum err_enum{ no_response, no_device, target_timeout };

//===================================================================================================

//=============================================================================
// local registers

#define MAX_PRESP_BUF 80
char bcmd_resp_buf[MAX_PRESP_BUF + 10];
char* bcmd_resp_ptr;
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
// HM-133 MFmic support
U8	hm_buf[HM_BUFF_END];				// signal buffer
U8	hm_hptr;
U8	hm_tptr;
U8	shftm;								// fn-shift mem register (MFmic)


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

//				          0123456789012345
const char un_ary[] =  { "RDU-900,ke0ff\0\0\0" };			// init User SN string
const char teststr[] = { "THIS IS KE0FF " };				// test string

char	band_str[6][5] = {
		{"10m \0"},
		{"6m  \0"},
		{"2m  \0"},
		{"220 \0"},
		{"440 \0"},
		{"1296\0"}
};

//=============================================================================
// local Fn declarations

void get_BCD32(char *sptr, U32 *bcdval);
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U32 params[ARG_MAX]);
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
U8 sto_nvmem(U8 band, U8 memnum, char* sptr);
char* char_srch(char* sptr, char searchr);
U8 str_chks(char* sptr);
U32 dpl_calc(U16 dplcode);
U8 cadd(U16 dplcode, U8 index);
void hm_cmd_exec(char* sptr, U8 cmd);
void hm_map(U8 cm, U8 hm);
void hm_sto(U8 j);

//=============================================================================
// CLI cmd processor entry point
//	Somewhere in the application, there must be a constant polling for input characters or end of
//		line signal.  The goal is to produce a command line array that was terminated by an EOL
//		(ASCII 0x0d).  This array is passed through parse_args() before calling x_cmdfn().  The
//		base command line array and the args arrays are maintained elsewhere in the application
//		(usually in main.c).
//	Uses number of args (nargs) and arg pointer array (args[]) to lookup command and dispatch to
//		a command-specific code code block for execution.
//
//	offset is srecord offset value which is maintained in main() and passed
//	forward for the srecord functions (upload), if used.
//
//	Note that the original command line array is "chopped up" by parse_args() by replacing
//		whitespace characters with null-terminators (unless brackets by quotes, as described below).
//
//	"Switch" operands, such as "-C", are processed here.  These switches set flags and are then
//		removed from the parameter arrays, so that the subsequent command code doesn't have to
//		try and filter them out.  If a switch is included in a parameter field for a command that
//		does not interpret it, the switch flag is simply ignored.
//
//	get_Dargs() (get decimal args) converts ASCII numbers into integer values and is called from
//		each individual parser so that "no-parameter-entered" defaults can be established.  To
//		enforce these defaults, first initialize the params[] array with the default values, then
//		call get_Dargs().
//
//	whitespace() defines what ASCII characters are white-space parameter separators used to separate
//		parameters within the command line array.  Multiple, consecutive whitespaces are allowed and
//		are filtered out by parse_args().
//	quotespace() defines what ASCII characters are to be used as quotes inside the parameter field.
//		Quotes allow character strings which contain whitespace to be contained within a single
//		arg[] array.  Quotes effectively suspend whitespace delimiting within quote pairs.
//		The system doesn't differentiate opening or closing quotes.  As long as it is defined in
//		quotespace(), any character can be used to open and close a quoted parameter string.
//			e.g.: if a double (") and single (') quote character are defined in quotespace(),
//			any of the following are valid quoted strings:
//				"Now is the time on schprockets when we dance'
//				'No monkeys allowed!"
//				"Go for launch"
//
//	Parameter ordinal management is specified by the command-specific parser code blocks and Fns.
//		Generally, a fixed order is easiest to manage.  Non-entered values in a list of 2 or more
//		parameters are indicated by a "-" character.  Negation of decimal values must also be
//		addressed in the cmd-specific code.  Other management schemes are possible but would be
//		up to the cmd-specific code to execute.
//
//		Parametric switches ("-x" in the command line) do not have any ordinal requirements nor can
//		any be enforced under the current architecture.
//
//=============================================================================

int x_cmdfn(U8 nargs, char* args[ARG_MAX], U16* offset){

#define	OBUF_LEN 110				// buffer length
#define	MAX_LOG_ERRS (OBUF_LEN / 4)	// max number of errors logged by verify cmd
	char	obuf[OBUF_LEN];			// gp output buffer
//	char	abuf[30];				// temp string buffer
//	char	bbuf[6];				// temp string buffer
	U32		params[ARG_MAX];		// holding array for numeric params
	char	c;						// char temp
	char	d;						// char temp
//	char	pm = FALSE;				// M flag, MAIN (set if "-M" found in args)
	char	ps = FALSE;				// S flag, SUB (set if "-S" found in args)
	char	pc = FALSE;				// C flag (set if "-C" found in args)
	char	pw = FALSE;				// W flag (set if "-W" found in args)
	char	px = FALSE;				// X flag (set if "-X" found in args)
//	char	ps = FALSE;				// s flag (set if <wsp>-S<wsp> found in args)
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
	S32		mem_buf8[7];			// U8 mem buffer
//	U8*		ptr0;					// U8 mem pointer
	U16		k;						// U16 temp
	U16		kk;						// U16 temp
	U32		hh;						// U16 temp
	U16		adc_buf[8];				// adc buffer
	U16*	ip;						// U16 pointer
//	U16		tadc_buf[8];			// adc buffer
	uint32_t ii;
	uint32_t jj;
//	volatile uint32_t* pii;					// u32 pointer
//	S32		si;
	float	fa;
	char	gp_buf[20];				// gen-purpose buffer

#ifdef DEBUG
	U8		m;						// temp
	U8		dbuf[10];				// U8 disp buf
	S8		si;						// temp s
	S8		sj;
	float	fb;
#endif

	bchar = '\0';																// clear global escape
    if (nargs > 0){
    	if((args[0][0] != '~')){
//    		for(i = 0; i <= nargs; i++){										//upcase all args !!! might just upcase command token ???
//    			s = args[i];
//    			str_toupper(s);
//    		}
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
			c = parm_srch(nargs, args, "-s");									// capture sub floater
			if(c){
				ps = TRUE;
				nargs--;
			}
/*			c = parm_srch(nargs, args, "-m");									// capture main floater
			if(c){
				pm = TRUE;
				nargs--;
			}*/
			c = parm_srch(nargs, args, "-v");									// capture v-flag floater (verbose)
			if(c){
				pv = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-c");									// capture c-flag floater
			if(c){
				pc = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-w");									// capture w-flag floater
			if(c){
				pw = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-x");									// capture x-flag select floater
			if(c){
				px = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "-r");									// capture x-flag select floater
			if(c){
				pr = TRUE;
				nargs--;
			}
			gas_gage(2);														// init gas gauge to disabled state

// ====== PARSING SWITCH ==========================================================================================================================================

			switch(cmd_id){														// dispatch command
				case help1:
				case help2:														// MAIN HELP
					do_help();
					break;

				case info:														// info + version
					putsQ("SYSINFO:");
					sprintf(obuf,"LIM_END = %d", LIM_END);
					putsQ(obuf);
					sprintf(obuf,"MEM_LEN = %d", MEM_LEN);
					putsQ(obuf);
					sprintf(obuf,"NUM_MEMS = %d", NUM_MEMS);
					putsQ(obuf);
					sprintf(obuf,"Last NVRAM = %d (0x%04x)", MEM_END,MEM_END);
					putsQ(obuf);
					sprintf(obuf,"NVREV = 0x%04x", nvram_sn());
					putsQ(obuf);
					sprintf(obuf,"VFO_0 = %d", VFO_0);
					putsQ(obuf);
					sprintf(obuf,"VFO_LEN = %d", VFO_LEN);
					putsQ(obuf);
					sprintf(obuf,"SQ_0 = %d", SQ_0);
					putsQ(obuf);
					sprintf(obuf,"buferrs = %d", get_error());
					putsQ(obuf);
				case vers:														// SW VERSION CMD
					dispSWvers();
					break;

				case set_hmkey:
					// send HM-133 MFmic key signal
					//	KEY <key-chr>
					c = args[1][0];
					hm_map(c, 'p');					// send press
					hm_map(c, 'r');					// send release
					putsQ("#OK$");
					break;

				case set_freq:
					// set module frequency in hz
					//	F freq
					//	Finds module associated with freq or err if no match
					params[0] = 0;
					get_Dargs(1, nargs, args, params);
					i = get_modulid(params[0] / 1000000L);
					if(i != BANDOFF){
						set_vfo(params[0]/1000L, i-1);
						vfo_change(MAIN);
						vfo_change(SUB);
						upd_lcd(i);
						putsQ("#OK$");
					}else{
						putsQ("#ERR$");
					}
					break;

				case set_offset:
					// set module offset in hz
					//	O freq mid
					// !!! need to validate frequency before storing to VFO
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);
					i = (U8)params[1] - 1;
					if(i <= ID1200_IDX){
						set_offs2(params[0]/1000L, i);
						vfo_change(MAIN);
						vfo_change(SUB);
						putsQ("#OK$");
					}else{
						putsQ("#ERR$");
					}
					break;
																				// str = "VFO + OFFS + DPLX/pwr/skip + CTCSS + SQ + VOL + XIT + RIT + BID-1 + !MEM_NAME! + STR_CHKS"
																				// if STR_CHKS omitted or is incorrect, an error is reported, but the cmd line processes anyway
																				// STO 3 A XX "1296000 20000 nn 103.5 34 34 -7 +7 5 !1234567890123456!"
																				// 1234567890123456789012345678901234567890123456789012345678901234567890
																				//          1         2         3         4         5         6         7
				case sto_mem:													// store mem: p[0] = band, p[1] = mem#, p[2] = chks, a[4] = mem_param_string
					params[0] = 0;								// band#		// STO <bnum> <mchr> <chks> "str"
					params[2] = 0;								// checks		//
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					s = args[4];								// upcase string to be sure
					str_toupper(s);
					params[1] = mem2ordinal(args[2][0]);						// convert mem chr to mem#
					if(--params[0] > 5) params[1] = NUM_MEMS;
					if(params[1] < NUM_MEMS){
//						ii = get_memaddr((U8)params[0], (U8)params[1]) + MEM_STR_ADDR; // band/mem#
//						sscanf(args[3],"%d %d %c %f", &ii, &hh, &c, &fa);
//																		frq  ofs  dplx/pwr/skp  pl   sq            vol           xit           rit           bid           namestr
						sscanf(args[4],"%d %d %d %f %d %d %d %d %d %s", &ii, &hh, &mem_buf8[0], &fa, &mem_buf8[2], &mem_buf8[3], &mem_buf8[4], &mem_buf8[5], &mem_buf8[6], gp_buf);
						l = (U8)mem_buf8[0];
						k = (U16)(fa * 10.0);

						kk = (U16)str_chks(args[4]);
						t = char_srch(args[4], '!') + 1;
						*char_srch(t, '!') = '\0';
						if(pv){								// if verbose...
							sprintf(obuf,"STOMEM: %s; mem#: %c; name: '%s'", band_str[params[0]], ordinal2mem(params[1]), t);
							putsQ(obuf);
							sprintf(obuf,"  FREQ: %d", ii);
							putsQ(obuf);
							sprintf(obuf,"  OFFS: %d", hh);
							putsQ(obuf);
							switch(l & DPLX_MASK){
							case DPLX_S:
								d = 'S';
								break;

							case DPLX_P:
								d = 'P';
								break;

							case DPLX_M:
								d = 'M';
								break;
							}
							sprintf(obuf,"  DPLX: %c", d);
							putsQ(obuf);
							if(l & LOHI_F) putsQ("   PWR: LOW");
							else putsQ("   PWR: HI");
							sprintf(obuf," CTCSS: %d  %02x", k, lookup_pl(k)+1);
							putsQ(obuf);
							sprintf(obuf,"    SQ: %d", mem_buf8[2]);
							putsQ(obuf);
							sprintf(obuf,"   VOL: %d", mem_buf8[3]);
							putsQ(obuf);
							sprintf(obuf,"   XIT: %d", mem_buf8[4]);
							putsQ(obuf);
							sprintf(obuf,"   RIT: %d", mem_buf8[5]);
							putsQ(obuf);
							sprintf(obuf,"  B_ID: %d", mem_buf8[6]);
							putsQ(obuf);
							if(!(l & SCANEN_F)) putsQ("  SKIP: ON");
							else putsQ("  SKIP: off");
						}
						if(params[2] != kk){									// checks test
							sprintf(obuf,"#ERRCHKS = %d$", kk);
							putsQ(obuf);
						}else{
							kk = get_memaddr((U8)params[0], (U8)params[1]);
							rw32_nvr(kk, ii, CS_WRITE|CS_OPEN);						// write freq
							rw16_nvr(kk, (U16)hh, CS_WRITE);						// write offs
							rw8_nvr(kk, (U8)mem_buf8[0], CS_WRITE);					// duplex/pwr/skip
							rw8_nvr(kk, lookup_pl(k)+1, CS_WRITE);					// PL tone code
							rw8_nvr(kk, (U8)mem_buf8[2], CS_WRITE);					// sq
							rw8_nvr(kk, 0, CS_WRITE);								// vol deprecated (vfo_p[band].vol now a spare) //
							rw8_nvr(kk, (U8)mem_buf8[4], CS_WRITE);					// xit
							rw8_nvr(kk, (U8)mem_buf8[5], CS_WRITE);					// rit
							rw8_nvr(kk, (U8)mem_buf8[6], CS_WRITE|CS_CLOSE);					// bid
							kk = get_memaddr((U8)params[0], (U8)params[1]) + MEM_STR_ADDR;			// band/mem#
							j = CS_WRITE | CS_OPEN;											// namestr
							for(i=0; i<MEM_NAME_LEN; i++){
								if(i == (MEM_NAME_LEN - 1)) j = CS_WRITE | CS_CLOSE;
								rw8_nvr(kk, *t++, j);
								j = CS_WRITE;
							}
							putsQ("#OK$");
						}
					}else{
						putsQ("#ERR$");
					}
					break;

				case mstr:														// mem string: p[0] = band+1, p[1] = mem#, p[2] = string
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					s = args[3];												// upcase string to be sure
					str_toupper(s);
					params[1] = mem2ordinal(args[2][0]);						// convert mem chr to mem#
					if(--params[0] > 5) params[1] = NUM_MEMS;
					if(params[1] < NUM_MEMS){
						ii = get_memaddr((U8)params[0], (U8)params[1]) + MEM_STR_ADDR;			// band/mem#
						if(args[3][0] != '\0'){									// copy new string to NVRAM
							j = CS_WRITE | CS_OPEN;
							for(i=0; i<MEM_NAME_LEN; i++){
								if(i == (MEM_NAME_LEN - 1)) j = CS_WRITE | CS_CLOSE;
								rw8_nvr(ii, args[3][i], j);
								j = CS_WRITE;
							}
						}
						j = CS_READ | CS_OPEN;									// read NVRAM
						for(i=0; i<MEM_NAME_LEN; i++){
							if(i == (MEM_NAME_LEN - 1)) j = CS_READ | CS_CLOSE;
							gp_buf[i] = rw8_nvr(ii, 0, j);
							j = CS_READ;
						}
//						if(params[1] < NUM_MEMS){
							sprintf(obuf,"MEM String, %s: #: %c, '%s'\n", band_str[params[0]], ordinal2mem(params[1]), gp_buf);
							putsQ(obuf);
//						}else{
//							putsQ("!Error!");
//						}
					}else{
						putsQ("!Error!");
					}
					break;

				case tone:														// tone: p[0] = mid, p[1] = tone freq, p[2] = on/off
					params[0] = 0;
					params[3] = 1;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					sscanf(args[2],"%f",&fa);									// get float of tone freq
					if( fa > 300.0) fa = 0;										// default to zero if invalid tone
					k = (U16)(fa * 10.0);
					i = (U8)params[0] - 1;
					// search for tone code
					j = lookup_pl(k);
					if((i <= ID1200_IDX) && (j != 0xff)){
						// store code
						set_ctcss(j, i);
						vfo_change(MAIN);
						vfo_change(SUB);
						upd_lcd(i);
						putsQ("#OK$");
					}else{
						putsQ("#ERR$");
					}
					break;

				case dup:														// duplex: p[0] = mid, p[1] = "+", "-", or "S"
					params[0] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					i = (U8)params[0] - 1;
			    	switch(args[2][0]){
			    	case '+':
			    		j = DPLX_P;
			    		break;

			    	case '-':
			    		j = DPLX_M;
			    		break;

			    	case 'S':
			    	case 's':
			    		j = 0;
			    		break;

			    	default:
			    		j = 0xff;
			    		break;
			    	}
					if((i <= ID1200_IDX) && (j != 0xff)){
						set_dplx(j, i);
						vfo_change(MAIN);
						vfo_change(SUB);
						upd_lcd(i);
						putsQ("#OK$");
					}else{
						putsQ("#ERR$");
					}
					break;

				case hilo:														// hilo: p[0] = mid, p[1] = "h", "l", "1", or "0"
					params[0] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					i = (U8)params[0] - 1;
			    	switch(args[2][0]){
			    	case 'h':
			    	case 'H':
			    	case '1':
			    		j = 0;
			    		break;

			    	case 'l':
			    	case 'L':
			    	case '0':
			    		j = LOHI_F;
			    		break;

			    	default:
			    		j = 0xff;
			    		break;
			    	}
					if((i <= ID1200_IDX) && (j != 0xff)){
						set_lohi(j, i);
						vfo_change(MAIN);
						vfo_change(SUB);
						upd_lcd(i);
						putsQ("#OK$");
					}else{
						putsQ("#ERR$");
					}
					break;

					case volc:														// vol: p[0] = mid, p[1] = vol
						params[0] = 99;
						params[1] = 99;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						i = (U8)params[0] - 1;
						j = params[1];
						if((i <= ID1200_IDX) && (j <= LEVEL_MAX)){
							// store code
							set_vol(j, i);
							force_push();
							putsQ("#OK$");
						}else{
							putsQ("#ERR$");
						}
						break;

					case squc:														// squ: p[0] = mid, p[1] = squ
						params[0] = 99;
						params[1] = 99;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						i = (U8)params[0] - 1;
						j = params[1];
						if((i <= ID1200_IDX) && (j <= LEVEL_MAX)){
							// store code
							set_squ(j, i);
							force_push();
							putsQ("#OK$");
						}else{
							putsQ("#ERR$");
						}
						break;

					case tsac:														// tsa: p[0] = mid, p[1] = ts value
						params[0] = 99;
						params[1] = 99;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						i = (U8)params[0] - 1;
						j = params[1] / TS_PER;
						if((i <= ID1200_IDX) && (j <= TS_MAX)){
							// store code
							set_tsa(j, i);
//							force_push();
							putsQ("#OK$");
						}else{
							putsQ("#ERR$");
						}
						break;

					case tsbc:														// tsb: p[0] = mid, p[1] = ts value
						params[0] = 99;
						params[1] = 99;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						i = (U8)params[0] - 1;
						j = params[1] / TS_PER;
						if((i <= ID1200_IDX) && (j <= TS_MAX)){
							// store code
							set_tsb(j, i);
//							force_push();
							putsQ("#OK$");
						}else{
							putsQ("#ERR$");
						}
						break;

// pttsub (bflags)
//	PTTSUB_SMUT	0x40						// ptt-sub edge action mode bits and field mask
//	PTTSUB_CALL	0x80
//	PTTSUB_M	(PTTSUB_SMUT | PTTSUB_CALL)
//	PTTSUB_MOD0	0
//	PTTSUB_MOD1	PTTSUB_SMUT
//	PTTSUB_MOD2	PTTSUB_CALL
//	PTTSUB_MOD3	(PTTSUB_SMUT | PTTSUB_CALL)
//					U8  get_bflag(U8 focus, U8 cmd, U8 bfset){

					case pttsub:													// pttsub: p[0] = mid, p[1] = 1-4, none, smute, scall, mcall
						params[0] = 99;
						params[1] = 99;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						i = (U8)params[0] - 1;
				    	switch(params[1]){
				    	case 1:
				    		j = PTTSUB_MOD0;
				    		break;

				    	case 2:
				    		j = PTTSUB_MOD1;
				    		break;

				    	case 3:
				    		j = PTTSUB_MOD2;
				    		break;

				    	case 4:
				    		j = PTTSUB_MOD3;
				    		break;

				    	default:
				    		j = 0xff;
				    		break;
				    	}
						if((i <= ID1200_IDX) && (j != 0xff)){
							set_pttsub(j, i);
							vfo_change(MAIN);
							vfo_change(SUB);
							putsQ("#OK$");
						}else{
							if((params[1] == 99) && (i <= ID1200_IDX)){							// if set params == default, querry PTTSUB
								//	send pttsub querry response
			  	  				// bflags hold the pttsub action status.  read the current bflags
			  	  				i = set_pttsub(0xff, i);
			  	  				j = ((i & PTTSUB_M) >> PTTSUB_bs) + 1;
			  	  				sprintf(gp_buf,"#OK %d$",j);
			  	  				putsQ(gp_buf);
							}else{
								putsQ("#ERR$");
							}
						}
						break;

					case setnvall:													// update NVRAM
						params[0] = ID10M_IDX + 1;
						params[1] = ID1200_IDX + 1;
						get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
						params[0] -= 1;												// convert mids to bids
						params[1] -= 1;
						if((params[0] <= ID1200_IDX) && (params[1] <= ID1200_IDX)){
							nvwr_vfo((U8)params[0], (U8)params[1]+1);
							set_bandnv();
							set_vnv(MAIN);
							set_vnv(SUB);
							putsQ("#OK$");
						}else{
							putsQ("#ERR$");
						}
						break;

// ==  DEBUG Fns FOLLOW +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=

				case beeper:													// beeper test
					putsQ("beep\n");
					params[0] = 0;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					if(params[1] > 775){
						set_beep(params[1], (U16)(((U32)params[1] * 75L)/1000L));
					}
					do_beep(params[0]);
//					send_stat(MAIN, gp_buf);
//					putsQ(gp_buf);
//					put_status(U8 focus, char* sptr, U8 sid)
					put_vfo(0x86, obuf, 0);
					putsQ(obuf);
					put_vfo(0x82, obuf, 0);
					putsQ(obuf);
					put_vfo(1, obuf, 1);
					putsQ(obuf);
					put_vfo(0, obuf, 1);
					putsQ(obuf);
					put_vfo(1, obuf, 2);
					putsQ(obuf);
					put_vfo(0, obuf, 2);
					putsQ(obuf);
					put_vfo(1, obuf, 3);
					putsQ(obuf);
					put_vfo(0, obuf, 3);
					putsQ(obuf);
					break;

				case scan_cmd:
					params[0] = MAIN;
					params[1] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					doscan((U8)params[0], (U8)params[1]);
					putsQ("doscan");
					break;

				case hm_cmd:													// process HM-133 MFmic data stream intercepts
/*					putsQ("\n{hmc:");
					putsQ(args[0]);		// debug !!!
					putsQ("}\n");		// debug*/
					hm_cmd_exec(args[0], 0);
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
#ifdef DEBUG
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
#endif
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
#ifdef DEBUG
					sprintf(obuf,"get key data: %08x",ii);
					putsQ(obuf);
					do{
						if(got_key()){
							putchar_b(get_key());
						}
					}while(bchar != ESC);
					putsQ("\n");
#endif
					break;

				case tst_att:
					params[0] = 0;
					get_Dargs(1, nargs, args, params);			// parse param numerics into params[] array
					sprintf(obuf,"atten: %06x",atten_calc((U8)params[0]));
					putsQ(obuf);
					break;

				case timer_tst:
#ifdef DEBUG
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
#endif
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
					break;

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
#ifdef DEBUG
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
#endif
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
#ifdef DEBUG
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
#endif
					}
					break;


				case tst_enc:													// pwr-off (sleep)
#ifdef DEBUG
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
#endif
					break;

				case tstuart1:													// HOST MEM WR CMD
#ifdef DEBUG
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
//						if(gotmsgn()){
							s = gets(obuf);									// get recvd data
							*s = '\0';											// add EOS
							putsQ("Rcvd: ");									// display rcvd string
							putsNQ(obuf);										// display with hex expanded
//						}
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
#endif
					break;
#define	deelay	5
				case trig_la:													// LCD debug trigger
#ifdef DEBUG
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
					sprintf(obuf,"Done.\n");
					putsQ(obuf);
#endif
					break;

				case dis_la:												// disarm (stop) analyzer
/*					reset_lcd();
					sprintf(obuf,"RST LCD.\n");
					putsQ(obuf);*/

					params[0] = 0;
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					dpl_cmd(obuf, &params[0], ps);
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
					break;

				default:
				case lastcmd:													// not valid cmd
					cmd_found = FALSE;
					break;


				case bttest:
					putsQ("BTtst");
/*					if(gotmsgn()){
						getss(obuf);
						putssQ("{");
						putsQ(obuf);
						putssQ("}");
					}*/
					putss(args[1]);
//					putssQ(args[1]);
//					wait(1100);
//					putss("\r");
/*					do{
						if(gotmsgn(0)){
							putsQ(get_btptr());
							clr_btptr();
							gotmsgn(1);
						}
					}while(bchar != ESC);*/
//					putss("---\r");
					break;
			} //=========== END PARSING SWITCH ======================================================================================
		}
    }
	if(bchar == ESC) while(gotchrQ()) getchrQ();									// if ESC, clear CLI input chrs
	return cmd_found;
}

//=============================================================================
// do_help() displays main help screen
//=============================================================================
void do_help(void){

	putsQ("IC-900 RDU-Clone CMD List:");
	putsQ("Syntax: <cmd> <arg1> <arg2> ... args are optional depending on cmd.");
	putsQ("\t<arg> order is critical except for floaters.");
	putsQ("\"?\" as first <arg> gives cmd help,  \"? ?\" lists all cmd help lines. When");
	putsQ("selectively entering <args>, use \"-\" for <args> that keep default value.");
	putsQ("\"=\" must precede decimal values w/o spaces. Floating <args>: these non-number");
	putsQ("<args> can appear anywhere in <arg> list: \"W\" = wait for operator\n");
	putsQ("\tFreq set\tofp VERSion");
	putsQ("\tset Offset\trmt KEY input");
	putsQ("\tSTOre memory\tTONE set tone code");
	putsQ("\tDUP Duplex\tHIlo RF power");
	putsQ("\tVol set\t\tSqu set");
	putsQ("\tTSA set\t\tTSB set");
	putsQ("\tPTTSub action\tNVALL saves vfo struct");
	putsQ("Supports baud rates of 115.2, 57.6, 38.4, 19.2, and 9.6 kb.  Press <Enter>");
	putsQ("as first character after reset at the desired baud rate.");
}

//=============================================================================
// do_cmd_help() displays individual cmd help using cmd_id enum
//=============================================================================
char do_cmd_help(U8 cmd_id){
	char 	c = TRUE;	// temps
	U16		k;
	U8		i;
	char	hbuf[10];

	switch(cmd_id){														// dispatch help line
		case set_hmkey:
			putsQ("KEY <chr> ?");
			putsQ("\tRemote keypress.  <ch> is single chr ID:");
			putsQ("\t'L' MHz   'T' Call  'X' M/S swap");
			putsQ("\t'/' UP    'V' V/M   'M' H/L pwr");
			putsQ("\t'\\' DN    'F' SUB   'G' SHFT");
			putsQ("\tTX: '0-9', 'A-D', '*', '#' = DTMF, !! NOP if issued during TX !!");
			putsQ("\tRX: '0-9' = DFE, '*' = DFE cancel, '#' DFE enter");
			putsQ("\t    'A'=CHK  'B'=TONE  'C'=BAND  'D'=SMUTE");
			putsQ("");
			putsQ("\tFUNC/Shifted Keys:");
			putsQ("\t'p' SHFT  'o' SET   'n' SUB");
			putsQ("\t'k'       'm' MW    'l'");
			putsQ("\t'j'");
			putsQ("\t'a' BRT   'b'       'c' Qup   'q' Vup");
			putsQ("\t'd' DIM   'e'       'f' Qdn   'r' Vdn");
			putsQ("\t'g'       'h' DUP   'i'       's' PTT sub querry");
			putsQ("\t'+' TS    '`' T     '$'       't' PTT sub mode++");
			break;

/*			code char fnkey_code[] = { 'p',   'o',   'n',   'k',   'm',   'l',   'j',  '|',
								  '!',   'a',   'b',   'c',   'q',   'd',   'e',   'f',
								  'r',   'g',   'h',   'i',   's',   '+',   '`',   '$',
								  't',   '\0' };
			// Normal mode keycodes for HM-151 and HM-133 (see above for HM-133 notes)
			code char   key_code[] = { 'L',   'T',   'X',   '/',   'V',   'M',   '\\',  'F',
								  'G',   '1',   '2',   '3',   'A',   '4',   '5',   '6',
								  'B',   '7',   '8',   '9',   'C',   '*',   '0',   '#',
								  'D',   '\0' };*/

		case set_freq:
			putsQ("F <frq Hz> ?");
			putsQ("\tSet freq (Hz).  Sets VFO in corresponding band module.");
			putsQ("\tERR if <frq> not in range of any module.");
			break;

		case set_offset:
			putsQ("O <frq Hz> <mid>?");
			putsQ("\tSet Offs freq (Hz) <bid> = 1 for UX-19 up to 6 for UX-129");
			break;

		case sto_mem:													// store mem: p[0] = band, p[1] = mem#, p[2] = chks, a[4] = mem_param_string
			putsQ("STO <mid> <mchr> <chks> 'str' ?");
			putsQ("\tStore mem, <mid> is module ID (Ux19 = 1, UX-129 = 6),");
			putsQ("\t<mchr> = the mem# chr ID, <chks> = checksum of 'str' ");
			putsQ("\t       Valid = AlphaNum & ( ) [ ] !! EXCEPT C, I, O, Q !!");
			putsQ("\t'str' = 'VFO + OFFS + DPLX/pwr/skip + CTCSS + SQ + VOL + XIT + RIT +");
			putsQ("\tBID + !MEM_NAME!");
			putsQ("\t\tVFO/OFFS = KHz");
			putsQ("\t\tDPLX/pwr/skip = bitmap");
			putsQ("\t\tCTCSS = bitmap");
			putsQ("\t\tSQ/VOL = 0-34");
			putsQ("\t\tXIT/RIT = bitmap");
			putsQ("\t\tBID = <mid>-1");
			putsQ("\t\t!MEM_NAME! = mem name str (15 bytes max)");
			break;
			// str = "VFO + OFFS + DPLX/pwr/skip + CTCSS + SQ + VOL + XIT + RIT + BID-1 + !MEM_NAME! + STR_CHKS"
			// if STR_CHKS omitted or is incorrect, an error is reported, but the cmd line processes anyway
			// STO 3 A XX "1296000 20000 nn 103.5 34 34 -7 +7 5 !1234567890123456!"
			// 1234567890123456789012345678901234567890123456789012345678901234567890
			//          1         2         3         4         5         6         7
			// store mem: p[0] = band, p[1] = mem#, p[2] = chks, a[4] = mem_param_string
			// band#		// STO <bnum> <mchr> <chks> "str"

		case mstr:														// mem string: p[0] = band+1, p[1] = mem#, p[2] = string
			putsQ("MSTR <mid> <mchr> 'str' ?");
			putsQ("\tStore MEM Name String, <mid> is module ID (Ux19 = 1, UX-129 = 6),");
			putsQ("\t<mchr> = the mem# chr ID");
			putsQ("\t       Valid = AlphaNum & ( ) [ ] !! EXCEPT C, I, O, Q !!");
			putsQ("\t'str' = mem name str (15 bytes max)");
			break;

		case tone:														// tone: p[0] = mid, p[1] = tone freq, p[2] = on/off
			putsQ("TONE <mid> <tfreq> <on/off> ?");
			putsQ("\tStore tone & set on/off, <mid> is module ID (Ux19 = 1, UX-129 = 6),");
			putsQ("\tIf <tfreq> omitted, just adjust on/off");
			putsQ("\tIf <on/off> omitted, just adjust tfreq");
			i = 0;
			do{
				k = pl_lookup(i);
				if(k != 0xffff){
					sprintf(hbuf,"\t%3.1f",(float)(k)/10.0);
					putssQ(hbuf);
					if((++i%6) == 0) putsQ("");
				}
			}while(k != 0xffff);
			putsQ("");
			break;

		case dup:														// duplex: p[0] = mid, p[1] = +/-/S
			putsQ("DUP <mid> <+/-/S> ?");
			putsQ("\tSet duplex, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case hilo:														// hilo: p[0] = mid, p[1] = H/L/1/0
			putsQ("HIlo <mid> <H/L/1/0> ?");
			putsQ("\tSet RF power, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case volc:														// vol: p[0] = mid, p[1] = vol
			putsQ("Vol <mid> <0-34> ?");
			putsQ("\tSet VOL, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case squc:														// squ: p[0] = mid, p[1] = squ
			putsQ("Squ <mid> <0-34> ?");
			putsQ("\tSet SQU, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case tsac:														// tsa: p[0] = mid, p[1] = ts value
			putsQ("TSA <mid> <0-1000 KHz> ?");
			putsQ("\tSet TSA, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case tsbc:														// tsb: p[0] = mid, p[1] = ts value
			putsQ("TSB <mid> <0-1000 KHz> ?");
			putsQ("\tSet TSB, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

		case pttsub:													// pttsub: p[0] = mid, p[1] = 1-4, none, smute, scall, mcall
			putsQ("PTT-SUB <mid> <mode> ?");
			putsQ("\tSet PTTSub, <mid> is module ID (Ux19 = 1, UX-129 = 6),");
			putsQ("\t<mode> = '1', no-action at PTT xsitions");
			putsQ("\t<mode> = '2', toggle SMUTE at PTT xsitions");
			putsQ("\t<mode> = '3', toggle SUB-CALL at PTT xsitions");
			putsQ("\t<mode> = '4', toggle MAIN-CALL at PTT xsitions");
			break;

		case setnvall:													// nvall: p[0] = start_bid, p[1] = stop_bid
			putsQ("NVALL <start_mid> <stop_mid> ?");
			putsQ("\tSave VFO(s) to NVRAM, <mid> is module ID (Ux19 = 1, UX-129 = 6)");
			break;

/////////////////////////////////////
		case hm_data:													// debug
			putsQ("Hm data: ?");
			putsQ("\tdisplay HM-151 key data, ESC to exit");
			break;

/*		case kp_data:													// debug
			putsQ("Kp data: ?");
			putsQ("\tdisplay key-pad data, ESC to exit");
			break;

		case tstuart1:													// debug UART1 tst CMD
			// Test UART1:  TSTU1 ?/<string>/W: loop output of <string>
			putsQ("TSTU1 (Test UART1): ?/<string>/W (loop until ESC)");
			break;

		case adc_tst:													// debug ADC tst CMD
			// Test ADC:  Adc test ?
			putsQ("Adc test: ?");
			break;

		case tst_enc:													// debug ADC tst CMD
			// Test Enc:  Enc ?
			putsQ("Enc test: ?");
			putsQ("\tDisp encoder changes, ESC to exit.");
			break;

		case tst_pwm:													// debug ADC tst CMD
			// Pwm debug:  P ?
			putsQ("Pwm test: <pwm#>/<value>/?");
			putsQ("\tDisplay min/max PWM and set PWM values.");
			putsQ("\t<pwm#> = 2 - 6");
			break;

		case timer_tst:													// debug ADC tst CMD
			// Test timer:  TI ?
			putsQ("TImer test: ?");
			putsQ("\tToggles PC4 @200ms period");
			break;*/

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
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U32 params[8]){

	char*	s;
	U32*	ptr1;
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
					*ptr1 = temp32;
					break;

				case '$':
					s++;
					count += sscanf(s,"%x",&temp32);			// get hex if leading "$"
					*ptr1 = temp32;
					break;

				case 'O':
				case 'o':
					s++;
					count += sscanf(s,"%o",&temp32);			// get octal if leading "O"
					*ptr1 = temp32;
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

	ptr = cmd_list[cmdid];										// start at beginning of search list
	while((*ptr & 0x80) != 0x80){								// process until 0xff found in search list
		i = strncmp(string, ptr, strlen(ptr));					// inbound string match search list?
		if(i){
			cmdid++;											// no, advance to next cmdid
			ptr = cmd_list[cmdid];
//			while(*ptr++);										// skip to next item in search list
		}else{
			ptr = (char*)end_list;								// found match,
			found = TRUE;										// set break-out criteria
		}
	}
	if(!found) cmdid = lastcmd;									// not found, set error cmd id
	return cmdid;
}

/*	char*	ptr;							// temp ptr
	char	cmdid = 0;						// start at beginning of cmd_enum
	char	i;								// temp
	char	found = FALSE;					// cmd found flag (default to not found)

	ptr = (char*)cmd_list;										// start at beginning of search list
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
}*/

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
// parse command line array for delimited arguments.  Called prior to entry into x_cmdfn().
//  on exit, the args[] array (an array of char pointers) holds each delimited argument
//	from the command string input:
//  args[0] holds first arg (the command token)
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
    // no return here, we'll never get here anyway...
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
            case '\'':          // single
            case '\"':          // double
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

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// sto_nvmem() writes the unparsed string to the nv memory space
//	returns true if error, false if OK
//-----------------------------------------------------------------------------
U8 sto_nvmem(U8 band, U8 memnum, char* sptr){
	// mem structure follows this format:
	// VFO + OFFS + DPLX + CTCSS + SQ + VOL + XIT + RIT + BID + MEM_NAME_LEN
/*	U32	addr;		// temps
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
	rw8_nvr(addr, 0, j);								// vol deprecated (vfo_p[band].vol now a spare) //
	rw8_nvr(addr, ux129_xit, j);
	rw8_nvr(addr, ux129_rit, j);
	rw8_nvr(addr, band, j);
	cptr = memname[band];
	for(i=0; i<MEM_NAME_LEN; i++){
		if(i == (MEM_NAME_LEN - 1)) j |= CS_CLOSE;
		rw8_nvr(addr, *cptr++, j);
	}*/
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// char_srch() looks for a character in a target string.
//	returns pointer to char (if pointed location == '\0', char not found)
//-----------------------------------------------------------------------------
char* char_srch(char* sptr, char searchr){
	char* sp = sptr;

	while((*sp != searchr) && (*sp)) sp++;
	return sp;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// str_chks() calculates U8 checksum until a quote is reached.  returns U8 checksum
//-----------------------------------------------------------------------------
U8 str_chks(char* sptr){
	char* sp = sptr;
	U8	i = 0;

	while((*sp != '"') && (*sp)){
		i += *sp++;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// hm_cmd_exec() executes the hm command intercept and fills the hm_buf[]
//	with keypress actions from the MFmic interface.  This stream gets merged
//	with the main button stream inside main().
//	This gets called once for each MFmic key packet received.  MFmic packets
//	all start with '~'.  Valid packets have the following:
//		pcsk\r
//		p = '~'
//		c = command chr
//		s = status chr
//		k = check = (c ^ s) | 0x80
//		\r = carriage return (this is stripped by the cmd-ln pre-processing
//			 and thus becomes a '\0' character)
//		'c' and 's' are printable ASCII. 'k' is made to be a non-ASCII character
//			by virtue of the hi-bit being set.  Setting the hi-bit also guarantees
//			that the check won't ever be a null, allowing standard null-terminated
//			array handling to be valid.
//-----------------------------------------------------------------------------
void hm_cmd_exec(char* sptr, U8 pcmd){
		U8	cm;				// extracted command
		U8	hm;				// extracted press/hold/release status
		U8	j;				// temp
//		U8	k;				// temp
//		U8	m;				// temp
//		U8	n;				// temp
static	U8	hm_state;		// state machine process variable
static	U8	cm_last;		// last cmd
//static	U8	hm_last;		// last status

	// IPL init of statics
	if(pcmd == 0xff){
		hm_map(0xff, 0xff);
		hm_state = HMST_IDLE;
//		hm_last = '\0';
		cm_last = '\0';
		return;
	}
	// calculate check-digit -- check = (cmd ^ param) | 0x80
/*	m = *(sptr+1);
	k = *(sptr+2);
	n = m ^ k;*/

	j = (*(sptr+1) ^ *(sptr+2)) | 0x80;
	// if command packet valid, process state machine
	if((j == *(sptr+3)) && (*(sptr+4) == '\0')){
//	if(*(sptr+4) == '\0'){
		// capture command and press/hold/release status from the packet payload
		cm = *(sptr+1);
		hm = *(sptr+2);
		// state machine to process key press-hold-release sequence
		switch(hm_state){
		default:
		case HMST_IDLE:
			// look for a press or hold (if we see a hold here, we assume that the press packet was missed)
			if((hm == 'p') || (hm == 'h')){
				hm_map(cm, 'p');					// map initial keypress into signal buffer
				hm_state = HMST_PRESS;				// point to next state
				hmk_time(1);						// start hold timer (the same hold delay time as for the panel buttons)
				cm_last = cm;						// save the command.  Us this saved value for subsequent actions until release
			}
			break;

		case HMST_PRESS:
			if(cm_last != cm){
				putsQ(("#")); //!!!
			}
			// At this point in the process, hold or release is expected
			// a press is assumed to be a repeat press packet and is ignored
			switch(hm){
			case 'R':
			case 'r':
				hm_map(cm_last, 'r');				// map release into signal buffer
				hm_state = HMST_IDLE;				// return to start
				break;

			case 'h':
				if(!hmk_time(0)){					// if "hold" is indicated AND there is a timeout:
					hm_map(cm_last, hm);			// map hold keypress into signal buffer
					hm_state = HMST_HOLD;			// point to next state
					hmk_time(1);					// reset hold timer
				}
				break;

			default:
				putsQ(("@")); //!!!
				break;
			}
			break;

		case HMST_HOLD:
			// if there is a timeout of the hmk_timer, we assume a critical loss and force the keypress to be released
			if(!hmk_time(0)){
				hm = 'r';							// force release
			}
			// once the hold mode is encountered, we look for release or a timeout (loss of comms from the MFmic).
			switch(hm){
			default:
				break;

			case 'h':								// a valid hold packet resets the timeout
				hmk_time(1);
				break;

			case 'R':								// valid release
			case 'r':
				hm_map(cm_last, 'r');				// map release keypress into signal buffer
				hm_state = HMST_IDLE;				// point to next state
				cm_last = '\0';						// clear cmd mem
				break;

			// a press here is an error.  Assume that the release of the previous key was lost & start a new press sequence
			case 'p':
				// process the release of the previous key
				hm_map(cm_last, 'r');				// map release keypress into signal buffer
				// start a new press
				hm_map(cm, 'p');					// map keypress into signal buffer
				hm_state = HMST_PRESS;				// point to next state
				hmk_time(1);						// start hold timer
				cm_last = cm;						// set cmd mem
				break;
			}
			break;
		}
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// hm_map() maps the MFmic hm command intercept to a local key ID and fills the hm_buf[]
//	cm = key code (ASCII, maps to keypad on HM-133 interface)
//	hm = (p)ress/(h)old/(r)elease
//-----------------------------------------------------------------------------
//char keychr_lut[] = {  TSchr, };
void hm_map(U8 cm, U8 hm){
	U8	j;		// temps
	U8	i;

	// trap IPL init
/*	if(cm == 0xff){
		shftm = 0;
		// init process indicator
		asow(0);
		return;
	}*/
	// trap null inputs
	if((cm == '\0') || (hm == '\0')){
		return;
	}
	if(!shftm){
		switch(cm){
		default:
			break;

//		case SH_LOKEY:
		case SH_CALLKEY:
		case SH_XFCKEY:
		case SH_UPKEY:
		case SH_VMKEY:
		case SH_MWKEY:
		case SH_DNKEY:
		case SH_F1KEY:
		case SH_F2KEY:
		case SH_1:
		case SH_2:
		case SH_3:
		case SH_A:
		case SH_4:
		case SH_5:
		case SH_6:
		case SH_B:
		case SH_7:
		case SH_8:
		case SH_9:
		case SH_C:
		case SH_STR:
		case SH_0:
		case SH_PND:
		case SH_D:
			set_shift(1);
/*			shftm = 1;
			asow(1);
			do_1beep();
			shft_time(1);								// start the shift timer*/
			break;
		}
	}
	// map keys with shift mode active
	if(shftm == 1){
		shft_time(1);									// kick the shift timer
		switch(cm){
			default:
				set_shift(0);
/*				shftm = 0;								// undefined keys cancel shift mode
				j = '\0';
				// clear process indicator
				asow(0);*/
				do_1beep();
				break;

			case F2KEY:
			case LOKEY:
			case SH_LOKEY:								// (FUNC-LOCK) == shift-toggle key
				j = '\0';
				if(hm == 'p'){
					set_shift(0);
/*					shftm = 0;
					// update process indicator
					asow(0);
					do_2beep();*/
				}
				break;

			case '\\':
			case '/':
				j = '\0';
				// these keys do no-op
				break;

			case SH_CALLKEY:
			case CALLKEY:
				j = SETchr;
				break;

			case SH_XFCKEY:
			case XFCKEY:
				j = SUBchr;
				break;

			case SH_VMKEY:
			case VMKEY:
				j = MWchr;
				break;

			case SH_3:
			case '3':
				j = Qupchr;
				break;

			case SH_6:
			case '6':
				j = Qdnchr;
				break;

			case SH_A:
			case 'A':
				j = Vupchr;
				break;

			case SH_B:
			case 'B':
				j = Vdnchr;
				break;


			case SH_C:
  			case 'C':
  				// only act on press cmd
  				if(hm == 'p'){
  	  				// bflags hold the pttsub action status.  read the current bflags
  	  				i = get_bflag(MAIN, 0, 0);
  	  				j = i & PTTSUB_M;
  	  				// response beeps indicate status
  	  				switch(j){
  	  				case PTTSUB_MOD0:
  	  					do_1beep();
  						break;

  	  				case PTTSUB_MOD1:
  	  					do_2beep();
  	  					break;

  	 				case PTTSUB_MOD2:
  	  					do_3beep();
  						break;

  	 				default:
  	  				case PTTSUB_MOD3:
  	  					do_4beep();
  	 					break;
  	 				}
  				}
				j = '\0';
				break;

			case SH_D:
  			case 'D':
  				// only act on press cmd
  				if(hm == 'p'){
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
  	  					do_1beep();
  						break;

  	  				case PTTSUB_MOD1:
  	  					do_2beep();
  	  					break;

  	 				case PTTSUB_MOD2:
  	  					do_3beep();
  						break;

  	 				default:
  	  				case PTTSUB_MOD3:
  	  					do_4beep();
  	 					break;
  	 				}
  				}
				j = '\0';
				break;

			case SH_2:
  			case '2':
//				j = ?;			// dial up (same as up/dn buttons at this point)
				break;

			case SH_5:
			case '5':
//				j = ?;			// dial up (same as up/dn buttons at this point)
				break;

			case SH_1:
			case '1':
				if(hm == 'p'){
					// bright
					j = backl_adj(0xff) + 1;
					if(j > BRT_MAX) j = BRT_MAX;
					backl_adj(j);
					do_1beep();
				}
				j = '\0';
				break;

			case SH_4:
			case '4':
				if(hm == 'p'){
					// dim
					j = backl_adj(0xff) - 1;
					if(j > BRT_MAX) j = 0;
					backl_adj(j);
					do_1beep();
				}
				j = '\0';
				break;

			case SH_8:
			case '8':
				j = DUPchr;
				break;

			case SH_STR:
			case '*':
				j = TSchr;
				break;

			case SH_0:
			case '0':
				j = Tchr;
				break;
		}
	// map keys in normal mode
	}else{
		switch(cm){
			default:
			case '\\':
			case '/':
				j = '\0';
				break;

			case VMKEY:
				j = MRchr;
				break;

			case XFCKEY:
				j = MSchr;
				break;

			case CALLKEY:
				j = CALLchr;
				break;

			case MWKEY:
				j = HILOchr;
				break;

			case F1KEY:
				j = SUBchr;
				break;

			case 'D':
				j = SMUTEchr;
				break;

			case 'C':
				j = VFOchr;
				break;

			case 'B':
				j = TONEchr;
				break;

			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				j = cm;
				break;

			case '#':
				j = ENTchr;
				break;

			case '*':
				j = DOTchr;
				break;

			case 'A':
				j = CHKchr;
				break;

			case F2KEY:
			case SH_LOKEY:
				j = '\0';
				if(hm == 'p'){
					set_shift(1);
/*					shftm = 1;
					// update process indicator
					asow(1);
					do_1beep();
					shft_time(1);									// start the shift timer*/
				}
				break;

			case LOKEY:
				j = MHZchr;
				break;
		}
	}
	// j == mapped key
	if(j){
		switch(hm){
//		// invalid edge cmd
//		default:
//			j = '\0';
//			break;

		// process initial press
		case 'p':
			break;

		// process hold
		case 'h':
			j |= KHOLD_FLAG;
			break;

		default:
		case 'r':
		case 'R':
			j |= KREL_FLAG;
//			hmk_time(0xff);
			break;
		}
		// store to buffer
		hm_sto(j);
/*		hm_buf[hm_hptr++] = j;
		// Process rollover
		if(hm_hptr == HM_BUFF_END){
			hm_hptr = 0;
		}
		// Process overflow -- discards oldest entry
		if(hm_hptr == hm_tptr){
			hm_tptr += 1;
			if(hm_tptr == HM_BUFF_END){
				hm_tptr = 0;
			}
		}*/
	}
	return;
} // end key processing

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// set_shift() sets/clears func shift mode
//	returns shftm
//-----------------------------------------------------------------------------
U8 set_shift(U8 tf){

	if(tf == 1){
		shftm = 1;
		asow(1);
		do_1beep();
		shft_time(1);								// start the shift timer
	}
	if(tf == 0){
		shftm = 0;
		asow(0);
		do_2beep();
		shft_time(0xff);							// clear the shift timer
	}
	return shftm;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// process_CMD() handles periodic polling tasks for the cmd_fn.c file scope
//-----------------------------------------------------------------------------
char process_CMD(U8 flag){

	if(flag == PROC_INIT){
		// init file-local statics
		hm_hptr = 0;									// init MFmic head/tail buffer indexes
		hm_tptr = 0;
		shft_time(CLEAR_TIMER);							// clear timeout
		pttsub_togg(0);									// init pttsub statics
		return 0;
	}
	// process shift timeout -- turn off shift and execute beeps
	if((shftm) && !shft_time(0)){
		set_shift(0);									// un-shift and update DU status
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// got_hm_asc() returns true if there is a MFmic cmd waiting
//-----------------------------------------------------------------------------
U8 got_hm_asc(void){
	U8	rtn = '\0';

	if(hm_hptr != hm_tptr){
		rtn = 0xff;										// there is data
	}
	return rtn;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// hm_asc() fetches code characters from the MFmic input stream
//-----------------------------------------------------------------------------
U8 hm_asc(void){
	U8	rtn = '\0';

	if(hm_hptr != hm_tptr){
		rtn = hm_buf[hm_tptr++];						// get keypad code
		if(hm_tptr == HM_BUFF_END){						// update buf ptr w/ roll-over
			hm_tptr = 0;
		}
	}
	return rtn;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// hm_sto() stores kkey code characters to the MFmic input stream
//-----------------------------------------------------------------------------
void hm_sto(U8 j){

	// store to buffer
	hm_buf[hm_hptr++] = j;
	// Process rollover
	if(hm_hptr == HM_BUFF_END){
		hm_hptr = 0;
	}
	// Process overflow -- discards oldest entry
	if(hm_hptr == hm_tptr){
		hm_tptr += 1;
		if(hm_tptr == HM_BUFF_END){
			hm_tptr = 0;
		}
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// pttsub_togg() issues call or smute toggle as an ersatz keypress based on
//	the bflags
//	!! For mode 4, we need to track MEM status since changes were made to how MEM/CALL
//		transitions are handled.  This also necessitates knowing the state of PTT
//j |= KREL_FLAG;
//-----------------------------------------------------------------------------
void pttsub_togg(U8 bflags){
			U8	i;			// temp
	static	U8	memcall;	// mem/call memory

	i = bflags & PTTSUB_M;
	switch(i){
	// mode 1
	default:							// nop
		memcall = 0;					// init statics
		break;

	// mode 2
	case PTTSUB_SMUT:					// issue mute toggle
		hm_sto(SMUTEchr);
		break;

	// mode 3
	case PTTSUB_CALL:					// issue sub-call toggle
		hm_sto(SSUBchr);
		hm_sto(CALLchr);
		hm_sto(RSUBchr);
		break;

	// mode 4
	case PTTSUB_CALL|PTTSUB_SMUT:		// issue mem toggle
	if((xmode_stat(MAIN) & MEM_XFLAG) || (memcall & MEM_XFLAG)){
		// mem active...
		if(query_tx()){
			// save mem/call status
			memcall = xmode_stat(MAIN);
			hm_sto(MMAINchr);
			hm_sto(CALLchr);
			hm_sto(RMAINchr);
		}else{
			// restore mem status
			hm_sto(MMAINchr);
			hm_sto(MRchr);
			hm_sto(RMAINchr);
			memcall = 0;
		}
	}else{
		// VDO mode
		hm_sto(MMAINchr);
		hm_sto(CALLchr);
		hm_sto(RMAINchr);
	}

		break;
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// send_stat() issues a serial status message
//	All status messages feature printable ASCII characters except for the NEWLINE terminator.
//	This gives 6 bits of payload per character with bit 0x40 set.  Checksum is 12 bits divided
//	between two payload bytes.
//	<type> identifies which status type is send, "r" = SRF, "s" = status, "f" = freq, & "o" = offs
// SRF status:
//	#R<srf_main><srf_sub><cos/ptt><checkH><checkL>$<\n>
// MAIN status:
//	#S<flag1><flag2><vol><squ><tonecode><rit><xit><checkH><checkL>$<\n>
//	#F<freq_hz><checkH><checkL>$<\n>
//	#O<offs_hz><checkH><checkL>$<\n>
// SUB status:
//	#s<flag1><flag2><vol><squ><tonecode><rit><xit><checkH><checkL>$<\n>
//	#f<freq_hz><checkH><checkL>$<\n>
//	#o<offs_hz><checkH><checkL>$<\n>
//-----------------------------------------------------------------------------
void send_stat(U8 focus, U8 type, char* sptr){
	char*	lptr = sptr;	// temps
	U16		ii;

	// send SRF params
	*sptr++ = '#';										// open response
	*sptr++ = 'R';
	*sptr++ = get_srf(MAIN) | 0x40;
	*sptr++ = get_srf(SUB) | 0x40;
	*sptr++ = get_cos() | get_changestat() | 0x40;		// combine COS/PTT/and change status => [01ms 0pMS] ms = main/sub change status
														// p = PTT status, MS = main/sub COS status
	ii = scheck(lptr+1, 4);								// split checksum into 2, 6bit values
	*sptr++ = ((ii >> 6) & 0x3f) | 0x40;
	*sptr++ = (ii & 0x3f) | 0x40;
	*sptr++ = '$';										// close response
//	*sptr++ = '\r';
	*sptr = '\0';

/*	adjust_squ(focus, 0);
	adjust_vol(focus, 0);
	read_tone(focus);
	read_dplx(focus);

	sprintf(obuf,"#$", LIM_END);
	putsQ(obuf);*/

	return;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// scheck() calculates the U16 algebraic sum of "len" bytes of the "sptr" array
//-----------------------------------------------------------------------------
U16 scheck(char* sptr, U8 len){
	U8		i;		// counter
	U16		ii;		// temp sum

	for(i=0, ii=0; i<len; i++){
		ii += *sptr++;
	}
	return ii;
}

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// upd_lcd() updates LCD based on changes input via CAT commands
//	bid is the band id that was changed
//-----------------------------------------------------------------------------
void upd_lcd(U8 bid){
	U8	l;
	U8	j;
	U8	h;

	l = query_mode();						// get current display mode (M/S)
	if(l == MAIN) j = SUB;					// set j = alternate band
	else j = MAIN;
	h = get_band_index(l);					// get bid of current band
	if(bid == h) update_lcd(l, l);			// if that band was changed, update LCD
	h = get_band_index(j);					// get bid of other band
	if(bid == h) update_lcd(l, j);			// if THAT band was changed, update LCD
	return;
}
// eof
