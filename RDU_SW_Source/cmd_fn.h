/********************************************************************
 ************ COPYRIGHT (c) 2014 by ke0ff, Taylor, TX   *************
 *
 *  File name: cmd_fn.h
 *
 *  Module:    Control
 *
 *  Summary:
 *  CLI Command Interpreter header
 *  
 *******************************************************************/

//=============================================================================
// Global defines

#define NO_DEVICE 0xff				// invalid device selector
#define ARG_MAX (6 + 1)				// 1 + max # of supported args (not including cmd)
#define	BOOT_RESP_VOID	0
#define BOOT_RESP_TO	1
#define	BOOT_RESP_ER	2
#define	BOOT_RESP_OK	4
#define	BOOT_RESP_EPERR	5
#define	BOOT_RESP_RAERR	6

#define	HM_BUFF_END		20			// HM-1xx MFmic buffer length
// MFmic HM-key state machine defines
#define	HMST_IDLE		1			// initial state
#define	HMST_PRESS		2			// pressed state: "h" or "r/R" to exit
#define	HMST_HOLD		3			// hold state: "r/R" to exit
#define	HMST_REL		4			// release debounce

//=============================================================================
// public command fn declarations

int x_cmdfn(U8 nargs, char* args[], U16* offset);
void str_toupper(char* string);
int parse_args(char* cmd_string, char* args[]);
int whitespace(char c);
int quotespace(char c, char qu_c);
void bcmd_resp_init(void);
char dev_sel(U8 devid);
void exec_bcmd(char* bcmdbuf_ptr, char* obuf, U16* offset);
U8 gas_gage(U16 len);
void string_addr_init(void);
U8 asc_hex(S8 c);
float temp_float(U16 k);
U8 got_hm_asc(void);
U8 hm_asc(void);
//void cmd_fn_init(void);
char process_CMD(U8 flag);
U8 set_shift(U8 tf);
void pttsub_togg(U8 bflags);
