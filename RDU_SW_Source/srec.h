/********************************************************************
 ************ COPYRIGHT (c) 2014 by ke0ff, Taylor, TX   *************
 *
 *  File name: srec.h
 *
 *  Module:    Control
 *
 *  Summary:   This is the header file for S-record processing.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

//------------------------------------------------------------------------------
// extern defines
//------------------------------------------------------------------------------

// S-rec defines
#define SRSTART 'S'
#define IRSTART ':'
#define SRS_NULL 0
#define BCMD_BUFLEN 48						// max length of bcmd line

#define  RAM_START     0x8000				// base address of external RAM
#define  RAM_LEN       0x0100				// length of RAM window in bytes

#ifndef	SREC_C
extern U8  dut_xdata[RAM_LEN];		// byte array to access RAM
#endif

// process_data() cmds: (also used in process_srx())
#define	PD_QERY 0x00			// querry status
#define	PD_QERYH 0x01			// querry fail addr H
#define	PD_QERYL 0x02			// querry fail addr L
#define	PD_QEMBED 0x03			// do nothing with data
#define	PD_INIT 0x10			// init vars (IPL), defaults to VRFY mode
#define	PD_STOR	0x11			// set store mode
#define	PD_VRFY	0x12			// set verify mode

// process_data() modes:
#define	PD_WR	0x01			// store data mode
#define	PD_CMP	0x00			// compare data mode

// process_data() status:
#define	PD_OK	0x00			// status OK (no compare errors)
#define	PD_VERR	0x01			// verify compare error
#define	PD_WERR	0x02			// write compare error

//------------------------------------------------------------------------------
// public Function Prototypes
//------------------------------------------------------------------------------

char process_srx(U8 dmode, char* obuf, U16* offset);		// get and process S or I record file
char* pass_bcmd(void);										// return pointer to bcmd buffer
void process_stx(U16 addrstart, U16 addrstop);				// send srecords to serial port
U16 put_sline(U8* sdata, U16 addr, U8 len, U8 type);
char ram_pattern_pass(void);
void put_byte(U8 d);
U8 process_data(U16 addr, U8 d, U8 dmode);
