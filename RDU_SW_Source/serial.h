/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.h
 *
 *  Module:    Control
 *
 *  Summary:   This is the header file for serial I/O.
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
#define TIMER_MIS_AMASK	(TIMER_MIS_TAMMIS | TIMER_MIS_CAEMIS | TIMER_MIS_CAMMIS | TIMER_MIS_TATOMIS)

#define	CHR_DLY0	87					// max char length, UART0, (us) (= 10/baud)
#define	CHR_DLY1	87					// max char length, UART1, (us) (= 10/baud)
#define	CHR_DLY2	521					// max char length, UART2, (us) (= 10/baud)
#define	CHR_WAT0	6					// max delay to wait for UART0 TX to clear (ms)
#define	CHR_WAT1	2					// max delay to wait for UART1 TX to clear (ms)
#define	CHR_WAT2	3					// max delay to wait for UART2 TX to clear (ms)

#define	TSIP_IDLE	0x00				// TSIP protocol states, IDLE is wait for DLE
#define	TSIP_START	0x01				// START is when DLE is rcvd
#define	TSIP_RUN	0x02				// RUN is waiting for DLE/ETX
#define	FRAME_CLR	0x00				// no framing locator at given buffer location
#define	FRAME_SRT	0xff				// signals start of new message
#define	FRAME_ERR	0xfe				// signals start of new message (previous message has error)

#define RXD_ERR			0x01			// rx data error
#define RXD_CHAR		0x08			// rx char ready
#define RXD_TSIPRDY		0x10			// tsip msg ready
#define	RXD_TSIPERR		0x20			// tsip msg error
#define TXD_CHRS		0x01			// tx chr sent
#define TXD_ERR			0x10			// tx data error (buffer overflow)
#define TXD_CERR		0x20			// tx data error (collision)
#define TXD_JAMMER		0x40			// jammer code detect
#define TXD_SND			0x80			// tx data sending
#define SOH		0x01
#define ETX		0x03
#define	EOT		0x04
#define AKN		0x06
#define	EOM1	13						// <CR> is EOM1
#define DLE		0x10
#define XON		0x11
#define XOFF	0x13
#define NAK		0x15
#define CAN		0x18
#define ESC		27
#define SW_ESC	0xa5					// sw restart escape code
#define	myEOF	26

#define TI1		0x02					// SCON1 bitmaps
#define	RI1		0x01

// xmodem defines
#define	XPOLY	0x1021					// xmodem polynomial (the CRC default)

// RX state commands
#define	RX_STATE_PROC	0x00			// state command to process state
#define	RX_STATE_INIT	0x20			// state command to set initialization state & init vars
#define	RX_STATE_PACK	0x21			// state command to terminate reception
#define	RX_STATE_CLEAR	0x22			// state command to clear xmode
#define	RX_STATE_QERY	0x23			// state command to querry data ready
// RX status responses
#define	XMOD_NULL		0x00			// nothing to report
#define	XMOD_DR			0x01			// xmodem data buffer ready
#define	XMOD_ABORT		0x7f			// xmodem error (aborted)
#define	XMOD_PKAK		0x02			// packet ack processed, seek next packet
#define	XMOD_ERR		0x03			// transmitter done sending (CAN)
#define	XMOD_DONE		0x04			// transmitter done sending
// TX state commands
#define	TX_STATE_PROC	0x00			// cmd to process TX data
#define	TX_STATE_INIT	0x20			// cmd to start a TX session
#define	TX_STATE_CLEAR	0x22			// cmd to do IPL
#define	TX_STATE_LAST	0x21			// cmd to signal end of tx

// Timer and retry limits
#define	XM_TO			(3*ONESEC)		// 3 seconds for xmodem timeout
#define ACKTRIES		10				// # tries to do start of packet before abort
										// this gives 30 sec to start an xmodem transfer in CRC before
										// SW reverts to checksum
#define	CIV_PREA		0xfe			// CIV message preamble
#define	CIV_HADDR		0xe0			// cntlr addr
#define	CIV_RADDR		0x70			// radio address
#define	CIV_EOM			0xfd			// CIV message end-of-msg
#define	CIV_OK			0xfb			// CIV message OK
#define	CIV_NG			0xfa			// CIV message no-good
#define	CIV_JAM			0xfc			// CIV jammer code
#define CIV_TIMEOUT 	100				// (ms) CIV send chr timeout limit
#define	CIV_RETRY_LIM	4				// retry limit
#define	CIV_JAMOUT		30				// (ms) CIV jammer code timeout
#define	CIV_MSGWAIT		50				// (ms) CIV message wait delay


//------------------------------------------------------------------------------
// Command line UART function defines.  This centralizes the CLUI to allow
//	UART swaps in one central location.  Use the "Q" macros for all CLI
//	calls.
// To make a swap, rename all of the target Fns below to match the new UART
//	assignment.
//------------------------------------------------------------------------------

#define putcharQ(x)		putchar1(x)
#define	getchrQ()		getchr1()
#define	gotchrQ()		gotchr1()
#define	gotmsgQ()		gotmsgn1()
#define	gotmsgnQ()		gotmsgn1()
#define	getsQ(x)		gets1(x)
#define	putsQ(x)		puts1(x)
#define	putssQ(x)		putss1(x)
#define	getch0Q()		getch01()
#define	putchar_bQ(x)	putchar_b(x)
#define	putdchQ(x)		putdch(x)
#define	puthexQ(x)		puthex(x)
#define	putsNQ(x)		putsN(x)

//------------------------------------------------------------------------------
// public Function Prototypes
//------------------------------------------------------------------------------

//char process_xrx(U8 state_cmd);
//char process_xtx(char dc, U8 state_cmd);
void initserial(void);
char putchar0(char c);
char putdch(char c);
U8 puthex(U8 dhex);
char getchr(void);
char gotchr(void);
U8 gotmsgn(void);
int putss(const char *string);
int puts0(const char *string);
int putsN(const char *string);
char getch00(void);
//char xmode_off(void);
//U16 calcrc(char c, U16 oldcrc, U16 poly);
char is_xmode(void);
char putchar_b(char c);
U32 init_uart0(U32 baud);
char set_baud(U32 baud);

U32 init_uart1(U32 baud);
U32 config_uart1(U32 baud);
char txstatus1(void);
char putchar1(char c);
char getchr1(void);
char gotchr1(void);
//U8 goteom1(void);
U8 gotmsgn1(void);
//int puts1(const char *string);
int puts1(const char *string);
char* gets1(char *string);
//char* gets1_dle(char *string);
void init_tsip1(void);
int puts1(const char *string);
int putss1(const char *string);
char getch01(void);

void rxd_intr(void);
void rxd1_intr(void);
void Timer1A_ISR(void);
