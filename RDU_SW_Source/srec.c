/********************************************************************
 ************ COPYRIGHT (c) 2014 by ke0ff, Taylor, TX   *************
 *
 *  File name: srec.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the S-record I/O module for the FF-PGMR11
 *             protocol converter.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

#define	SREC_C
#include <string.h>
#include <ctype.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "srec.h"
#include "init.h"
#include "stdio.h"
#include "serial.h"
#include "cmd_fn.h"

//-----------------------------------------------------------------------------
// Local Variable Declarations & defines
//-----------------------------------------------------------------------------

char bcmdbuf[BCMD_BUFLEN];	// bcmd data buffer
U8   bcmdptr;					// bcmd buf pointer

#define SR_BUFLEN 110			// max length of srec line packet
char srbuf[SR_BUFLEN];	// srec data buffer
U8   srptr;						// srec buf pointer
#define SR_TYPE 1				// Srec type pointer
#define	SR_LEN	2				// Srec length pointer
#define SR_ADDR 4				// Srec target ADDR pointer
#define SR_DATA 8				// start of Srec data pointer

U8	sr_err;						// srec error reg

U8	dut_xdata[RAM_LEN];	// byte array to access RAM
								// this array estabishes a pointer to the XDATA SRAM.

#define	BCMD_STORE	0x00		// store d to buffer
#define	BCMD_CLEAR	0x10		// clear buffer ptr and status
#define	BCMD_QERY	0x11		// return status

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------

char process_sline(U8 dmode, char* obuf, U16* offset);
char get_sline(void);
char process_bcmd(U8 d, U8 cmd);
char endofrec(char c);
char abort_char(char c);
char valid_hex(char c);
U8 get_byte(U8 p);
//U8 asc_hex(char c);
//void put_byte(U8 d);
char hex_asc(U8 c);

//-----------------------------------------------------------------------------
// process_srx() processes rx srecord cmds & state machine
//	dmode specifies verify (PD_VRFY) or write (PD_STOR) data mode
//-----------------------------------------------------------------------------
char process_srx(U8 dmode, char* obuf, U16* offset){

	char	srx_status = SRS_NULL;		// status response
	char	c;							// temps
//	char	d;
//	char	x;							// xmode temp

	is_xmode();
	sr_err = 0;
	process_data(0, 0, PD_INIT);		// init data status
	do{
		c = get_sline();				// get a data record line from the input source
		if(!abort_char(c)){
			c = process_sline(dmode, obuf, offset); // process the data
			if(c == 0x80) puts0("sline chkserr");
			if(xoffsent){
				putchar(XON);			// re-enable remote source (XON)
				xoffsent = FALSE;
			}
		}
	}while(!endofrec(c));
	if(endofrec(c)) srx_status = c;
	return srx_status;
}

//-----------------------------------------------------------------------------
// process_sline() processes sline in srbuf[] dmode sets data mode
//	will also process intel hex records.
// Intel HEX record Fields:
//	":nnaaaattdd..ddcc"
//	'S' = start of record.
//	tt = record type ('00' = dats rec, '1' = terminate)
//	nn = number of dd..dd chr pairs
//	aaaa = 16 bit addr pointer
//	dd..dd = data byte pairs
//	ii = 2's complement of checksum of "nn..dd"
//
// S-record control flags & regs
// S-record fields:
//	"Stnnaaaadd...ddcc"
//	'S' = start of record.
//	t = record type ('0' = ref rec, '1' = data, '9' = terminate)
//	nn = number of chr pairs between & including "aa..cc"
//	aaaa = 16 bit addr pointer
//	dd..dd = data byte pairs
//	cc = 1's complement of checksum of "nn..dd"
//
//	All chrs (except start) are printable ASCII representations of
//	byte nybbles
//-----------------------------------------------------------------------------
char process_sline(U8 dmode, char* obuf, U16* offset){
#define SR_TYPE 1				// Srec type pointer
#define	SR_LEN	2				// Srec length pointer
#define SR_ADDR 4				// Srec target ADDR pointer
#define SR_DATA 8				// start of Srec data pointer

#define IR_TYPE 7				// Irec type pointer
#define	IR_LEN	1				// Irec length pointer
#define IR_ADDR 3				// Irec target ADDR pointer
#define IR_DATA 9				// start of Irec data pointer

	char	status = 0;
	char	c;
	char*	optr = obuf;		// holding reg for start of obuf
	U8		t;					// temp
	U8		len;				// s rec length
	U8		chksp;				// ptr to chksum
	U16		addr;				// addr ptr
	U8		i;					// looping temp/gp pointer

	for(i=1; i<srptr; i++){
		status |= valid_hex(srbuf[i]);		// see if buffer is all valid hex
	}
	if(status == 0){
		if(srbuf[0] == SRSTART){
			c = srbuf[SR_TYPE];				// extract S type
		}else{
			c = get_byte(IR_TYPE);			// extract I type
		}
		switch(c){
			case '0':						// embeded pgmr command record (SREC)
			case '1':						// S data record
				t = 0;
				len = get_byte(SR_LEN);								// get record length
				chksp = (len * 2) + SR_LEN;							// calc pointer to checksum
				for(i = SR_LEN; i < chksp; i += 2){					// calc srec chksum
					t += get_byte(i);
				}
				t = ~t;
				if(t != get_byte(chksp)) status = 0x80;				// if checksum invalid, set chkserr
				addr = (((U16)get_byte(SR_ADDR)) << 8) & 0xff00;
				addr |= ((U16)get_byte(SR_ADDR+2)) & 0xff;
				addr -= RAM_START;
				addr += *offset;
				if(c == '1'){
					for(i = SR_DATA; i < chksp; i += 2){
						process_data(addr++, get_byte(i), dmode);	// move/vfy data to mem
					}
				}else{
					if(dmode == PD_QEMBED){
						i = SR_DATA;								// init data pointer
						do{
							c = get_byte(i);						// get byte from S0 line
							i += 2;
							if(c == '\0') c = ' ';					// convert nulls to spaces
							if(c != '\n') *obuf++ = c;				// put in buffer (except newline)
						}while((c != '\n') && (i < chksp));
						*obuf = '\0';
						if(!is_xmode()) puts0(optr);					// send bcmd line to display
					}else{
						process_bcmd(0, BCMD_CLEAR);
						for(i = SR_DATA; i < chksp; i += 2){
							process_bcmd(get_byte(i), BCMD_STORE); // copy data into command buff
						}
						if(process_bcmd(0, BCMD_QERY)) exec_bcmd(bcmdbuf, obuf, offset);
						else process_bcmd(0, BCMD_CLEAR);
					}
				}
				break;

			case 0x00:						// I data record
				t = 0;
				len = get_byte(IR_LEN);								// get record length
				chksp = (len * 2) + 9;								// calc pointer to checksum
				for(i = IR_DATA; i < chksp; i += 2){				// calc irec chksum
					t += get_byte(i);
				}
				t = (~t) + 1;
				if(t != get_byte(chksp)) status = 0x80;				// if checksum invalid, set chkserr
				addr = (((U16)get_byte(IR_ADDR)) << 8) & 0xff00;
				addr |= ((U16)get_byte(IR_ADDR+2)) & 0xff;
				addr -= RAM_START;
				for(i = IR_DATA; i < chksp; i += 2){
					process_data(addr++, get_byte(i), dmode);		// move/vfy data to mem
				}
				break;

			case 0x01:						// Irecord end
			case '9':						// Srecord end
				status = myEOF;
				break;

			default:						// un-recognized records
				break;
		}
	}
	return status;
}

//-----------------------------------------------------------------------------
// get_sline() gets an srecord or irecord line into the srecord buffer.
//	If an abort char (CAN, EOT, ESC, or EOF) is encountered, processing stops
//	and the Fn returns the abort character.  Else, returns 0
//-----------------------------------------------------------------------------
char get_sline(void){

	char	status = 0;
	char	c = 0;
	char	x = is_xmode(); 					// xmode temp

	do{											// look for srec start
		while(!c){
			c = gotchr();						// wait for any chr
		}
		if(x){
			switch(c){
				case XMOD_ABORT:				// if xmodem, look for abort conditions
				case XMOD_ERR:
				case XMOD_DONE:
					return EOT;
			}
		}
		c = toupper(getchr());					// convert input to upcase
	}while((c != SRSTART) && (c != IRSTART) && !abort_char(c)); //repeat until a valid start is found, or abort
	srptr = 0;									// init s-rec pointer to start
	while(!endofrec(c)){						// until end of record,
		if((c >= ' ') && (c <= 127)){			// first pass uses previous getchr() call
			srbuf[srptr++] = toupper(c);		// if printable ascii, put in buffer
		}
		c = getchr();
	}
	if(!x && handshake){
		putchar(XOFF);							// turn off remote source if not xmodem
		xoffsent = TRUE;
	}
	srbuf[srptr] = '\0';						// null terminate
	if(abort_char(c)) status = c;
	return status;
}

//-----------------------------------------------------------------------------
// process_data() takes data and performs store or compare based on data mode
//	(dmode) parameter.
//-----------------------------------------------------------------------------
U8 process_data(U16 addr, U8 d, U8 dmode){

	static U8	status;
	static U16	faddr;

	switch(dmode){
		default:
			status |= 0xf0;						// system fault
			break;

		case PD_INIT:
			status = PD_OK;
			faddr = 0;
			break;
		
		case PD_STOR:
			if(addr <= RAM_LEN){
				dut_xdata[addr] = d;			// move data to mem & verify
				if(dut_xdata[addr] != d){
					status |= PD_WERR;
					faddr = addr;
				}
			}else{
				status |= PD_WERR;
			}
			break;

		case PD_VRFY:							// verify data against mem
			if(dut_xdata[addr] != d){
				status |= PD_VERR;
				faddr = addr;
			}
			break;

		case PD_QEMBED:
		case PD_QERY:							// just send status
			break;

		case PD_QERYH:							// just send faddrh
			return (U8)(faddr >> 8);

		case PD_QERYL:							// just send faddrl
			return (U8)(faddr & 0xff);
	}
	return status;
}

//-----------------------------------------------------------------------------
// process_bcmd() stores data to embedded command buffer.  Allows pgmr cmds to
//	be embedded in the object data as "S0" records.
//	returns TRUE if no errors.  If FALSE, cmd buffer was overflowed (do not use)
//	must add '\0' to end of bcmdbuf after record EOF
//-----------------------------------------------------------------------------
char process_bcmd(U8 d, U8 cmd){

	static	char	status;

	switch(cmd){
		case BCMD_STORE:						// store data to buffer, update status
			if(bcmdptr < BCMD_BUFLEN) bcmdbuf[bcmdptr++] = d;
			else status = FALSE;
			break;

		case BCMD_CLEAR:						// init pointer and status (no store)
			status = TRUE;
			bcmdptr = 0;
			bcmdbuf[bcmdptr] = '\0';
			break;

		default:
		case BCMD_QERY:							// NOP, just return status
			break;
	}
	return status;
}

//-----------------------------------------------------------------------------
// endofrec() tests for an abort char: CAN, EOT, ESC, or EOF or EOL
//	returns TRUE if abort chr detected
//-----------------------------------------------------------------------------
char endofrec(char c){

	char	tf;

	switch(c){
		case EOT:
		case ESC:
		case myEOF:
		case CAN:
		case '\r':
		case '\n':
			tf = TRUE;
			break;
		
		default:
			tf = FALSE;
			break;
	}
	return tf;
}

//-----------------------------------------------------------------------------
// abort_char() tests for an abort char: CAN, EOT, ESC, or EOF
//	returns TRUE if abort chr detected
//-----------------------------------------------------------------------------
char abort_char(char c){

	char	tf;

	switch(c){
		case EOT:
		case ESC:
		case myEOF:
		case CAN:
			tf = TRUE;
			break;
		
		default:
			tf = FALSE;
			break;
	}
	return tf;
}

//-----------------------------------------------------------------------------
// get_byte(p) gets converts 2-byte ascii hex at srbuf[p] and returns
//-----------------------------------------------------------------------------
U8 get_byte(U8 p){

	U8	b;			// byte value
	
	b = asc_hex(srbuf[p++]) << 4;
	b |= asc_hex(srbuf[p]);
	return b;
}

//-----------------------------------------------------------------------------
// asc_hex() converts ascii hex to binary nybble and returns
//-----------------------------------------------------------------------------

// U8 asc_hex(char c){

// 	U8	b;			// byte value
// 	
// 	b = (U8)c - '0';
// 	if(b > 9) b -= 'A' - '9' - 1;
// 	return b;
// }

//-----------------------------------------------------------------------------
// valid_hex() tests for a valid hex chr (returns 0x00 if valid, 0x80 if not valid)
//	a valid hex is any ascii from '0' to '9' or 'A' to 'F' (assumes upcase input)
//-----------------------------------------------------------------------------
char valid_hex(char c){

	if(c < '0') return 0x80;
	if(c > 'F') return 0x80;
	if((c > '9') && (c < 'A')) return 0x80;
	return 0x00;
}

//-----------------------------------------------------------------------------
// process_stx() sends srecord lines to serial port.  Data from addrstart to
//	addrstop.  addrstop is the last byte to be sent
//-----------------------------------------------------------------------------
void process_stx(U16 addrstart, U16 addrstop){
#define	DEF_SLEN 32

	U16	addr = addrstart;	// running addr reg
	U16	temp;				// temp
	U8	len;				// srec data length
	U8	lastline = FALSE;	// last line flag

	while((!lastline) && (bchar != ESC)){
		temp = addrstop - addr + 1;					// calc # bytes left to send
		if(temp <= DEF_SLEN){						// if at or less than deflen, do last line
			len = (U8)(temp & 0xff);				// set last line length
			lastline = TRUE;						// set exit flag
		}else{
			len = DEF_SLEN;							// not last, use default length
		}
		addr = put_sline(dut_xdata, addr, len, 1);	// send line to serial port
	}
	if(bchar != ESC) put_sline(dut_xdata, 0, 0, 9); // EOF record
}

//-----------------------------------------------------------------------------
// put_sline() writes an srecord to the serial port
//	returns next SRAM addr to be read
//-----------------------------------------------------------------------------
U16 put_sline(U8* sdata, U16 addr, U8 len, U8 type){

	U8	chks = len + 3;		// chks temp
	U8	i;					// loop temp
	U8	d;					// data temp

	if(addr >= RAM_START){
		sdata += addr - RAM_START;
	}
	bchar = '\0';									// clear global escape flag
	putchar('S');									// start of line
	if(bchar == ESC) return 0;
	putchar(hex_asc(type));							// type
	put_byte(len+3);								// length
	d = (U8)(addr >> 8);							// addr
	chks += d;
	put_byte(d);
	d = (U8)(addr & 0x00ff);
	chks += d;
	put_byte(d);
	for(i = 0; i < len; i++){						// data
		d = *sdata++;
		chks += d;
		put_byte(d);
		addr++;
	}
	put_byte(~chks);								// s-rec checksum
	putchar('\n');									// EOL
//	putchar('\r');
	return addr;
}

//-----------------------------------------------------------------------------
// ram_pattern_pass() fills sram ($8000-ffff) with a test pattern, then verifies
//	return TRUE if pass, else FALSE
//	This is a system test function to validate the external SRAM
//-----------------------------------------------------------------------------
char ram_pattern_pass(void){

	U16	addr = 0x0000;	// addr pointer
	U16	pattern = 0xfff0;
	U16	d;
	char v = TRUE;
	
	do{
		dut_xdata[addr++] = (U8)(pattern >> 8) & 0xff;
		dut_xdata[addr++] = (U8)(pattern--);
	}while(addr != RAM_LEN);
	addr = 0x0000;	// addr pointer
	pattern = 0xfff0;
	do{
		d = (U16)dut_xdata[addr++] << 8;
		d |= (U16)dut_xdata[addr++] & 0xff;
		if(d != pattern--){
			v = FALSE;
		}
	}while(addr != RAM_LEN);
	return v;
}

//-----------------------------------------------------------------------------
// put_byte(p) gets converts U8 to 2-byte ascii hex and sends to serial port
//-----------------------------------------------------------------------------
void put_byte(U8 d){
	
	putchar(hex_asc(d >> 4));
	putchar(hex_asc(d));
}

//-----------------------------------------------------------------------------
// hex_asc(p) converts binary nybble to ascii hex and returns
//-----------------------------------------------------------------------------
char hex_asc(U8 c){

	char b;

	b = (char)(c & 0x0f) + '0';			// add '0' to convert decimal nyb to ascii
	if(b > '9') b += 'A' - '9' - 1;		// if hex, add ("A' - '9' - 1) to get ascii
	return b;
}
