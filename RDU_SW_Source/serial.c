/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the serial I/O module for the HFBridge
 *             protocol converter.  Handles UART0, UART1, and UART2 I/O
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"
#include "stdio.h"
#include "serial.h"
#include "cmd_fn.h"
#define UART1_ENABLED

//------------------------------------------------------------------------------
// Define Statements
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Variable Declarations
//-----------------------------------------------------------------------------

// xmodem control flags & regs
U8  xmode;						// xmodem transfer enable flag, if == 1, getchr
								//		is primary interface to xmodem protocol
U8  xcrc;						// crc checksum enable
U8  xtxdr;						// tx data ready flag
U8  xrxdr;						// rx data ready flag
#define XM_PACKETLEN (1+2+128+1) // length of checksum packet (SOH,N,~N,128 data bytes,CHKS)
#define	XBUF_MAX 135			// max# xmodem buffer bytes
char xmbuf[XBUF_MAX];			// xmodem data buffer
U8   xmptr;						// xmodem buf pointer
#define XM_DATASTART 2          // start of data payload in xmbuf
#define XM_DATAEND (128+2)      // end of data payload in xmbuf (one past end)
#define	XM_TX_DATASRT 3			// TX xm pointers
#define	XM_TX_DATAEND (XM_TX_DATASRT + 128)

// UART0 control flags & regs (DEBUG CLI I/O)
U8  TI0B;						// UART0 TI0 reflection (set by interrupt)
#define RXD0_BUFF_END 40		// CLI rx buff len
#define RXD1_BUFF_END 40		// CCMD rx buff len
#define RXD2_BUFF_END 40		// CIV rx buff len
#define TXD_BUFF_END 30			// tx buffs
#define TXD1_BUFF_END 100		// tx1 buff

S8   volatile rxd_buff[RXD0_BUFF_END];	// rx data buffer
U8   volatile rxd_hptr;					// rx buf head ptr
U8   volatile rxd_tptr;					// rx buf tail ptr
U8   volatile rxd_stat;					// rx buff status

// UART1 (CCMD I/O, RS485)
U8   volatile rxd1_buff[RXD1_BUFF_END];	// rx1 data buffer
U8   volatile rxd1_hptr;					// rx1 buf head ptr
U8   volatile rxd1_tptr;					// rx1 buf tail ptr
U8   volatile rxd1_stat;					// rx1 buff status
U8   volatile frm1_buff[RXD1_BUFF_END];	// rx1 frame buffer
U8   volatile rxd1_state;				// rx1 tsip state
U8   volatile rxd1_dle_count;			// rx1 tsip dle counter
U8   volatile rxd1_lastc;				// rx1 tsip last chr reg
U8   volatile rxd1_msgcnt;				// tsip message counter
U8   volatile txd1_buff[TXD1_BUFF_END];	// tx1 data buffer
U8   volatile txd1_hptr;					// tx1 buf head ptr
U8   volatile txd1_tptr;					// tx1 buf tail ptr
U8   volatile txd1_stat;					// tx1 buff status
U8   volatile txd1_lastc;				// rx1 last-chr-sent reg
U8   volatile txd1_rdy;					// txd pacing timer ready

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------

//char process_xrx(U8 state_cmd);
char getchr_b(void);
char gotchr_b(void);
char do_txeot(void);
U32 config_uart0(U32 baud);
void jam_wait(void);
void txstart1(void);

//********************//
// UART Init routines //
//********************//

//-----------------------------------------------------------------------------
// initserial() initializes serial buffers and fn state machines
//-----------------------------------------------------------------------------
void initserial(void){

    TI0B = 1;
    rxd_hptr = 0;
    rxd_tptr = 0;
    rxd_stat = 0;
    rxd1_hptr = 0;
    rxd1_tptr = 0;
    rxd1_stat = 0;
    frm1_buff[0] = FRAME_SRT;
//    txd1_stat = 0;

    init_uart0(115200L);
//	process_xrx(RX_STATE_INIT);
	xmode = FALSE;
	handshake = FALSE;				// xon/xoff enable
	xoffsent = FALSE;				// xoff sent
#ifdef UART1_ENABLED
    init_uart1(115200L);
#endif
}

/****************
 * init_uart inits UART0 to (baud), N81
 */
U32 init_uart0(U32 baud)
{
	U32	i;

	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;			// enable UART0 peripheral
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;			// enable PA clock
	GPIO_PORTA_AFSEL_R |= 0x0003L;						// connect UART0 RX & TX to PA0 & PA1
	GPIO_PORTA_PCTL_R &= 0xFFFFFF00L;					// enable UART0 function
	GPIO_PORTA_PCTL_R |= 0x0011L;
	GPIO_PORTA_DIR_R |= 0x0002;							// txd = out
	GPIO_PORTA_DEN_R |= 0x0003;							// RXD = TXD = digital
	for(i=0;i<100000;i++);
	return config_uart0(baud);							// config UART0
}

/****************
 * config_uart0 inits UART0 to (baud), N81
 */
U32 config_uart0(U32 baud)
{
	U32	IR;
	U32	i;

	UART0_CTL_R &= 0xfffffffeL;							// disable UART
	UART0_CTL_R &= 0xffff33ffL;							// disable HS, leave TE and RE alone
	IR = (uint32_t)PIOCLK * 4L;							// IR = (sysclk * 64) / (16 * baud)
	IR /= baud;											//    = sysclk * 4 /baud
	UART0_IBRD_R &= 0xffff0000L;
	UART0_IBRD_R = (IR>>6) & 0xffff;
	UART0_FBRD_R &= 0xffffffc0L;
	UART0_FBRD_R |= IR & 0x3fL;
	UART0_CC_R = UART_CC_CS_PIOSC;						// set PIOSC as UART source
	UART0_LCRH_R |= 0x0060L;							// 8 bits
	UART0_IM_R &= 0xfffce000L;							// set UART0 to int on RX
	UART0_IM_R |= 0x0010L;
	UART0_CTL_R |= 0x0300L;								// set TE and RE
	UART0_CTL_R |= 0x0001L;								// enable UART
	for(i=0;i<100000;i++);								// delay a bit...
	return IR;
}
/*
//-----------------------------------------------------------------------------
// process_xrx() processes RX xmodem cmds & xmodem state machine
//	useage is for system to call process_xrx(RX_STATE_INIT) to begin xmodem rcv.
//	next, call process_xrx(RX_STATE_PROC) until XMOD_DR is returned.  If XMOD_ABRT
//		or XMOD_ERR is returned, xfer has been aborted and flow should return to
//      the CLI.  XMOD_DONE indicates that the transmitter has finished sending
//      data and there is no more data to process (e.g., EOT received).
//	Once XMOD_DR is signalled, data is available from the xmbuf[] array using xmptr
//      and xmptr == XM_DATAEND to signal end of data buffer.
//	When the XMbuffer is empty, the system should call process_xrx(RX_STATE_PKAK)
//	Generally, the receiver will reach the end of useable data before the end
//		of the xmodem buffer (due to the padding that xmodem uses to keep packets
//		the same size).  The end of data is usually the terminating record
//		of an Srecord or Intel Hex file, but could also be when a CNTL-Z (EOF)
//		is encountered unexpectedly in the xmodem data stream (only true since
//		this appl doesn't support binary transfers).  When the system reaches
//		its end-of-file point (either a terminating record, or CNTL-Z), it
//      should call xrx_querry(RX_STATE_PACK) repeatedly until XMOD_DONE is
//      returned.
//	Finally, once XMOD_DONE is received, call xrx_querry(RX_STATE_CLEAR) to
//		terminate the xmodem transfer.  xrx_querry(RX_STATE_CLEAR) must also be
//		called after a return of XMOD_ABORT or XMOD_ERR.
//-----------------------------------------------------------------------------
char process_xrx(U8 state_cmd){
// state machine defines
#define	XRX_1			0x01			// state: wait for start of packet
#define	XRX_DATA		0x02			// state: get packet (N to checksum)
#define	XRX_GOTPACK		0x03			// state: got packet, data ready
#define XRX_ABORT		0x04			// state: abort xfr

	static	U8		rx_state;			// state maching state reg
	static	U8		xtries;				// retry counter
	static	char	pacN;				// ded-reconing packet count
	static	U8		byte_count;			// packet data byte counter
	static	U8		firstpack;			// first packet flag

	    	U16		checksum;			// crc checksum (16 bit) or additive checksum (8 bit)
	    	U16		chktemp;			// packet checksum (16 bit) temp reg
			U8		i;					// por loop var
			char	c;					// character temp reg
			char	c2;					// temp
			char	xrx_status = XMOD_NULL;	// got data return flag

	if(state_cmd == RX_STATE_INIT){
		xmode = TRUE;									// turn on xmodem reception
		xtries = ACKTRIES;								// init statics for start of packet reception
		pacN = 1;										// packet# starts @ 1
		xcrc = TRUE;									// start with crc
		xrxdr = FALSE;									// no data ready signal to getchr
		rx_state = XRX_1;								// set entry state
		xm_timer = 0;									// clear activity timeout timer
		firstpack = TRUE;								// enable first packet logic
		return xrx_status;
	}
	if(state_cmd == RX_STATE_CLEAR){
		xmode = FALSE;									// clear xmodem reception
        xrx_status = XMOD_DONE;	
		return xrx_status;
	}
	if(state_cmd == RX_STATE_QERY){
        if((rx_state == XRX_GOTPACK) && (xmptr < XM_DATAEND)){
            xrx_status = XMOD_DR;
        }	
		return xrx_status;
	}
	switch(rx_state){
		default:
			rx_state = XRX_ABORT;						// default state, abort
			xrx_status = XMOD_ERR;
			break;
		
		case XRX_1:
			if(xm_timer == 0){							// if chr timeout:
				xm_timer = XM_TO;						// reset timer
				if(firstpack){							// for first packet,
					if(--xtries == 0){
						xcrc = FALSE;					// cancel crc chksum after retries exhausted
						xtries = ACKTRIES;				// reset retry limit
					}
					if(xcrc){
						putchar_b('C');					// send crc start request
					}else{
						if(xtries == 0){
							rx_state = XRX_ABORT;		// if 2nd retry cycle exhausted, abort
							xrx_status = XMOD_ERR;
						}else{
							putchar_b(NAK);				// send sum chksum start request
						}
					}
				}else{									// for all other packets,
					if(--xtries == 0){
						rx_state = XRX_ABORT;			// if retries exhausted, abort
						xrx_status = XMOD_ERR;
					}else{
						putchar_b(NAK);					// if timeout, send NAK
					}
				}
			}else{
				c = getchr_b();							// look for chr
				switch(c){
					case CAN:							// if cancel, abort
						rx_state = XRX_ABORT;
						xm_timer = ONESEC;
						break;
					
					case EOT:							// if TX done, set done on this end
						putchar_b(AKN);
						xrx_status = XMOD_DONE;
						xmode = FALSE;
						break;

					case SOH:							// if start of packet,
						rx_state = XRX_DATA;			// go get entire packet
 						byte_count = XM_PACKETLEN - 1;  // yes, init packet byte count (minus SOH)
						if(xcrc) byte_count++;			// if crc, add another byte
 						xmptr = 0;						// init data pointer
						xm_timer = XM_TO;
						break;
				}										// all other chrs ignored
			}
			break;
		
		case XRX_DATA:
			if(xm_timer == 0){
				rx_state = XRX_1;						// if timeout waiting for data, restart packet
			}else{
				if(gotchr_b()){							// wait until there is received data
					c = getchr_b();
					xmbuf[xmptr++] = c;					// store data in buffer
					if(--byte_count == 0){				// update byte count, if end of data, go look for chksum,
						// validate packet
						c2 = ~((U8)xmbuf[1]);
						if(xmbuf[0] == c2){
 							if(xmbuf[0] == (pacN - 1)){	// seq# = last, discard and ACK
 								putchar_b(AKN);
 								rx_state = XRX_1;
								xm_timer = XM_TO;						
 							}else{
								checksum = 0;
								for(i=XM_DATASTART;i<XM_DATAEND;i++){
									if(xcrc){
										checksum = calcrc(xmbuf[i], checksum, XPOLY); // if crc, update crcsum
									}else{
										checksum += (U16)xmbuf[i]; // else do checksum
										checksum &= 0xff;
									}
								}
								if(xcrc){
									chktemp = ((U16)xmbuf[130] << 8) | ((U16)xmbuf[131] & 0xff);
								}else{
									chktemp = ((U16)xmbuf[130]) & 0xff;
								}
								if(chktemp == checksum){
									rx_state = XRX_GOTPACK; // packet received, go to data ready idle state,
									xrx_status = XMOD_DR;
									xrxdr = TRUE;
									xmptr = XM_DATASTART; // point to start of data
								}else{
									rx_state = XRX_1;	// seq# and ~seq# test failed, NAK/retry
									xm_timer = 0;
								}
							}
						}else{
							rx_state = XRX_1;			// seq# and ~seq# test failed, NAK/retry
							xm_timer = 0;
						}
					}
				}
			}
			break;
		
		case XRX_GOTPACK:								// stay here until system empties data buffer and ends packet
			xrx_status = XMOD_DR;							// return data ready
			if(state_cmd == RX_STATE_PACK){				// if system sends packet ackn signal, send ACK, update seq#,
				putchar_b(AKN);							// and recycle for next packet
				pacN++;
				firstpack = FALSE;
				rx_state = XRX_1;
				xm_timer = XM_TO;
				xtries = ACKTRIES;						// reset retry limit		
				xrx_status = XMOD_PKAK;					// return packet ack'd
			}
			break;
			
		case XRX_ABORT:									// xm_timer is chr received timer for abort,
			if(xm_timer == 0){							// no chr received for 1 sec is cancellation success
				if(gotchr_b()){							// since chrs received, we aren't cancelled yet,
					while(gotchr_b()) getchr_b();		// clear out rx buffer,
					putchar_b(CAN);						// send cancel,
					xm_timer = ONESEC;					// set chr clearing timeout,
					xrx_status = XMOD_ERR;				// return error until abort is qualified
				}else{
					xrx_status = XMOD_ABORT;			// signal abort success
				}
			}	
			break;
	}
	return xrx_status;									// return status signal
}

//-----------------------------------------------------------------------------
// process_xtx() processes TX xmodem cmds & xmodem state machine
//	useage is for system to call process_xtx(0, TX_STATE_INIT) to begin xmodem
//		TX. '0' is not transmited.
//	next, call process_xtx(chr, TX_STATE_PROC) with data until last data byte
//		has been sent. Finally, call process_xtx(CNTL-Z, TX_STATE_LAST) until
//		XMOD_DONE, XMOD_ERR, or XMOD_ABORT is returned.
//
//	If XMOD_ABRT is returned, xfer has been aborted and flow should return to
//      the CLI w/ err msg.  XMOD_DONE indicates that the TX was successful.
//-----------------------------------------------------------------------------
char process_xtx(char dc, U8 state_cmd){
// state machine defines
#define	XTX_SRTWAIT		0x01			// state: wait for start of packet
#define	XTX_BUILD		0x02			// state: build packet
#define XTX_ABORT		0x03			// state: abort xfr
#define XTX_DONE		0x04			// state: xfr complete

	static	U8		tx_state;			// state maching state reg
	static	char	pacN;				// ded-reconing packet count

			U8		xtries;				// retry counter
	    	U16		checksum;			// crc checksum (16 bit) or additive checksum (8 bit)
			U8		i;					// loop var
			U8		xmbufend;			// temp end buf pointer
			char	c;					// character temp reg
			char	xtx_status = XMOD_NULL;	// got data return flag

	if(state_cmd == TX_STATE_INIT){
		xmode = TRUE;									// turn on xmodem
		xtries = ACKTRIES;								// init statics for start of packet reception
		pacN = 1;										// packet# starts @ 1
		xm_timer = ONEMIN;								// set activity timeout timer
		tx_state = XTX_SRTWAIT;							// set entry state
		xmptr = 3;										// start of xmodem data
		return xtx_status;
	}
	if(state_cmd == TX_STATE_CLEAR){
		xmode = FALSE;									// clear xmodem
        xtx_status = XMOD_DONE;
		tx_state = XTX_DONE;
		return xtx_status;
	}
	switch(tx_state){
		case XTX_SRTWAIT:
			xmbuf[xmptr++] = dc;
			do{
				c = getchr_b();
				switch(c){								// process start of session response from receiver
					case NAK:
						xcrc = FALSE;					// normal response (checksum)
						c = AKN;
						break;

					case 'C':
						xcrc = TRUE;					// CRC response
						c = AKN;
						break;

					default:
						if((c == CAN) || (c == ESC)) c = CAN; // abort response
						break;
				}
			}while((c != CAN) && (c != AKN));
			if(c == CAN){
				tx_state = XTX_ABORT;					// process abort
				xmode = FALSE;
			}else{
				tx_state = XTX_BUILD;					// go to build packet state...
			}
			break;

		case XTX_BUILD:
			if(state_cmd == TX_STATE_LAST){
				if(xmptr == XM_TX_DATASRT){				// buffer empty, don't need to send data
					do_txeot();							// send EOT
					xmode = FALSE;						// disable xmodem
		    	    xtx_status = XMOD_DONE;				// set done status
					tx_state = XTX_DONE;
					return xtx_status;					// break out of fn here
				}else{
					do{
						xmbuf[xmptr++] = dc;			// fill buffer with chr
					}while(xmptr < XM_TX_DATAEND);
				}
			}else{
				xmbuf[xmptr++] = dc;					// put chr into buffer
			}
			if(xmptr == XM_TX_DATAEND){
				xmbuf[0] = SOH;
				xmbuf[1] = pacN;
				xmbuf[2] = ~pacN;
				if(xcrc){
					checksum = 0;						// calculate CRC
					for(i = XM_TX_DATASRT; i < XM_TX_DATAEND; i++){
						checksum = calcrc(xmbuf[i], checksum, XPOLY);
					}
					xmbuf[XM_TX_DATAEND] = (U8)(checksum >> 8);
					xmbuf[XM_TX_DATAEND+1] = (U8)(checksum & 0xff);
					xmbufend = XM_TX_DATAEND+2;
				}else{
					checksum = 0;						// calculate checksum
					for(i = XM_TX_DATASRT; i < XM_TX_DATAEND; i++){
						checksum += xmbuf[i];
					}
					xmbuf[XM_TX_DATAEND] = (U8)(checksum & 0xff);
					xmbufend = XM_TX_DATAEND+1;
				}
				do{
					for(i = 0; i < xmbufend; i++){		// send packet
						putchar_b(xmbuf[i]);
					}
					xm_timer = XM_TO;
					i = TRUE;
					xtries = ACKTRIES;
					do{
						c = getchr_b();					// wait for response
						switch(c){
							case CAN:
								xtries = 1;				// force exit
							case NAK:
							case AKN:
								i = FALSE;
								break;

							default:
								break;
						}
						if(xm_timer == 0){				// timed out
							i = FALSE;
							c = '\0';
						}
					}while(i);
					xtries--;
				}while((c == NAK) && (xtries != 0));	// tried out or nak'd
				if(xtries != 0){
					pacN++;								// packet sent, ready for next
					xmptr = XM_TX_DATASRT;
				}else{
					tx_state = XTX_ABORT;				// error, tried out
					xmode = FALSE;
				}
				if((state_cmd == TX_STATE_LAST) && xmode){ // send EOT to finish
					do_txeot();
					xmode = FALSE;
					tx_state = XTX_DONE;
				}
			}
			break;

		case XTX_ABORT:
			xtx_status = XMOD_ABORT;					// respond with abort
			break;

		case XTX_DONE:
			xtx_status = XMOD_DONE;						// respond with done
			break;

		default:
			tx_state = XTX_DONE;						// default to done
			xmode = FALSE;
			break;
	}
	return xtx_status;
}

//-----------------------------------------------------------------------------
// is_xmode() returns t/f of xmode.  This is an external call to request
//	local status info.
//-----------------------------------------------------------------------------
char is_xmode(void){

	char c = FALSE;

	if(xmode) c = TRUE;
	return c;
}

//-----------------------------------------------------------------------------
// do_txeot() sends TX EOT
//-----------------------------------------------------------------------------
char do_txeot(void){

	U8	xtries;
	char c;

	xtries = ACKTRIES + 1;						// set try counter (add 1 because we use it right off the bat)
	xm_timer = 0;								// start with timer = 0
	do{
		if(xm_timer == 0){						// if timer == 0,
			xm_timer = XM_TO;					// reset timer
			xtries--;							// update tries
			putchar_b(EOT);						// send EOT
		}
		c = getchr_b();							// get response
		if(c == NAK) xm_timer = 0;				// if NAK, use timer to signal
	}while((c != AKN) && (xtries != 0));		// do untio AKN or no more tries left
	xmode = FALSE;								// disable xmodem
	c = FALSE;									// default to error return
	if(xtries != 0) c = TRUE;					// no error return
	return c;
}
*/
//-----------------------------------------------------------------------------
// calcrc() calculates incremental crcsum using supplied poly
//	(xmodem poly = 0x1021)
//-----------------------------------------------------------------------------
/*U16 calcrc(char c, U16 oldcrc, U16 poly){

	U16 crc;
	U8	i;
	
	crc = oldcrc ^ ((U16)c << 8);
	for (i = 0; i < 8; ++i){
		if (crc & 0x8000) crc = (crc << 1) ^ poly;
		else crc = crc << 1;
	 }
	 return crc;
}
*/
//-----------------------------------------------------------------------------
// set_baud() sets baud rate to passed value (if supported).  If rate is valid,
//	return TRUE, else FALSE.  0 = 115,200 baud
//-----------------------------------------------------------------------------
char set_baud(U32 baud){

	char c = FALSE;
	
	switch(baud){
		case 9600L:
		case 19200L:
		case 38400L:
		case 57600L:
		case 115200L:
			config_uart0(baud);
			c = TRUE;
			break;

		case 0L:
			config_uart0(115200L);
			c = TRUE;
			break;
	}
	return c;
}

//-----------------------------------------------------------------------------
// putchar (UART0) serial output with xmodem layering
//	does cr/lf translation if not in xmode
//	xmodem transmit state machine fn (process_xtx()) is inserted here
//-----------------------------------------------------------------------------
char putchar0(char c){

	// output cr if c = \n		// no xmodem
	if(c == '\n'){
	    wait_reg0(&UART0_FR_R, UART_FR_TXFF, CHR_WAT0);	// wait up for fifo to clear
//		while((UART0_FR_R & 0x0020) == 0x0020);
		UART0_DR_R = '\r';
	}
	// output character
    wait_reg0(&UART0_FR_R, UART_FR_TXFF, CHR_WAT0);	// wait up for fifo to clear
//	while((UART0_FR_R & 0x0020) == 0x0020);
	UART0_DR_R = c;
	return (c);
}

//-----------------------------------------------------------------------------
// getchr looks for chr @ port0 rx.  If none, return '\0'
//	maps into xmodem layer if xmode == TRUE 
//	uses rollover buffer rxd_buff[] which is filled in the UART0 interrupt
//-----------------------------------------------------------------------------
char getchr(void){

	char c = '\0';
//	char x;

/*	if(xmode){
        x = process_xrx(RX_STATE_PROC);
		if(x == XMOD_DR){
			if(xmptr < XM_DATAEND){
				c = xmbuf[xmptr++];
			}
			if(xmptr == XM_DATAEND){
                process_xrx(RX_STATE_PACK);
			}
		}
	}else{*/
		if(rxd_tptr != rxd_hptr){
			c = rxd_buff[rxd_tptr++];
			if(rxd_tptr >= RXD0_BUFF_END){
				rxd_tptr = 0;
			}
		}
//	}
	return c;
}

//-----------------------------------------------------------------------------
// gotchr looks for chr in UART0 rx buf.  If none, return FALSE, else return TRUE
//	maps into xmodem layer if xmode == TRUE
//-----------------------------------------------------------------------------
char gotchr(void){

	char c = FALSE;			// return val
//	char j;					// temp

/*	if(xmode){
		do{
			j = process_xrx(RX_STATE_PROC);
			switch(j){
				case XMOD_DR:
				case XMOD_ABORT:
				case XMOD_ERR:
				case XMOD_DONE:
					c = TRUE;
					break;
			}
		}while(!c);
    }else{*/
	    if(rxd_tptr != rxd_hptr){
            c = TRUE;
    	}
//    }
	return c;
}

//-----------------------------------------------------------------------------
// gotmsgn returns the rcvd msg count.
//-----------------------------------------------------------------------------
U8 gotmsgn(void){

//	return rxd0_msgcnt;								// pass the msg count status
	return 0;
}

//-----------------------------------------------------------------------------
// getchr_b looks for chr @ UART0 rx.  If none, return '\0'
//	waits for a chr and returns (no xmodem traps)
//-----------------------------------------------------------------------------
char getchr_b(void){

	char c = '\0';

	if(rxd_tptr != rxd_hptr){
		c = rxd_buff[rxd_tptr++];
		if(rxd_tptr >= RXD0_BUFF_END){
			rxd_tptr = 0;
		}
	}
	return c;
}

//-----------------------------------------------------------------------------
// gotchr_b looks for chr in UART0 rx buf.  If none, return FALSE,
//	else return TRUE (no xmodem traps
//-----------------------------------------------------------------------------
char gotchr_b(void){

	char c = FALSE;

	if(rxd_tptr != rxd_hptr){
		c = TRUE;
	}
	return c;
}

//-----------------------------------------------------------------------------
// puts0() does puts to UART0
//-----------------------------------------------------------------------------
int puts0(const char *string){

	while(*string){
		putchar0(*string++);
	}
	putchar0('\n');
	return 0;
}

//-----------------------------------------------------------------------------
// putss() does puts w/o newline
//-----------------------------------------------------------------------------
int putss(const char *string){

	while(*string){
		putchar0(*string++);
	}
	return 0;
}

//-----------------------------------------------------------------------------
// getch00 checks for input @ RX0.  If no chr, wait forever.
//	waits for a chr and returns.  Note: Processor spends most of its idle
//	time waiting in this fn.
//	looks for input in rxd_buff[] which is filled in the UART0 interrupt
//-----------------------------------------------------------------------------
char getch00(void){

	char c;

	while(rxd_tptr == rxd_hptr){
		if(process_IO(0)){					// process system I/O
			rxd_buff[rxd_hptr++] = '\r';	// phake a CR to bump the system out of the getch00 loop and back up to main()...
			if(rxd_hptr == RXD0_BUFF_END){
				rxd_hptr = 0;
			}
		}
	}
	c = rxd_buff[rxd_tptr++];
	if(rxd_tptr == RXD0_BUFF_END){
		rxd_tptr = 0;
	}
	return c;
}

//==================== UART1 Fns =================================
// CCMD I/O
// Uses TSIP protocol (DLE/ETX)
/****************
 * init_uart inits UART1 to (baud), N81
 */
U32 init_uart1(U32 baud)
{
	U32	i;

	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;		// enable UART peripheral
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;		// enable PB clock
	GPIO_PORTB_AFSEL_R |= RXD1|TXD1;				// connect UART RX & TX
	GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M|GPIO_PCTL_PB0_M); // enable UART function
	GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB1_U1TX|GPIO_PCTL_PB0_U1RX);
//	GPIO_PORTB_DIR_R |= 0x0002;						// txd = out
//	GPIO_PORTB_DEN_R |= 0x0003;						// RXD = TXD = digital
	init_tsip1();									// init tsip machine
	txd1_rdy = 0;
	for(i=0;i<100000;i++);
	return config_uart1(baud);						// config UART
}

/****************
 * config_uart1 inits UART1 to (baud), N81
 */
U32 config_uart1(U32 baud){
	U32	IR;
	U32	i;

	// disable UART, disable HS (leave TE & RE alone)
	UART1_CTL_R &= ~(UART_CTL_UARTEN|UART_CTL_CTSEN|UART_CTL_RTSEN|UART_CTL_RTS);
	IR = (uint32_t)PIOCLK * 4L;						// IR = (sysclk * 64) / (16 * baud)
	IR /= baud;										//    = sysclk * 4 /baud
	UART1_IBRD_R &= ~UART_IBRD_DIVINT_M;
	UART1_IBRD_R = (IR>>6) & UART_IBRD_DIVINT_M;
	UART1_FBRD_R &= ~UART_FBRD_DIVFRAC_M;
	UART1_FBRD_R |= IR & UART_FBRD_DIVFRAC_M;
	UART1_CC_R = UART_CC_CS_PIOSC;					// set PIOSC as UART source
	UART1_LCRH_R |= UART_LCRH_WLEN_8;				// 8 bits
	UART1_IM_R = UART_IM_RXIM;						// enable RX interrupt
	UART1_CTL_R |= UART_CTL_RXE | UART_CTL_TXE;		// set TE and RE
	UART1_CTL_R |= UART_CTL_UARTEN;					// enable UART
	for(i=0;i<100000;i++);							// delay a bit...
	return IR;
}

/****************
 * init_tsip1 inits tsip rcv logic.  Clears buffers and counters
 */
void init_tsip1(void){

	NVIC_DIS0_R = NVIC_EN0_UART1;					// disable UART1 intr
	rxd1_state = TSIP_IDLE;							// rx1 tsip state: idle starts the rcv process
	rxd1_dle_count = 0;								// rx1 tsip dlecnt: clear
	rxd1_lastc = 0;									// rx1 tsip last chr reg: clear
	rxd1_msgcnt = 0;								// rx1 message counter: clear
	frm1_buff[0] = FRAME_SRT;						// set start of new frame
	rxd1_hptr = 0;									// init pointers
	rxd1_tptr = 0;
	txd1_hptr = 0;
	txd1_tptr = 0;
//	NVIC_EN0_R = NVIC_EN0_UART1;					// enable UART1 intr
}

//-----------------------------------------------------------------------------
// putchar1 (UART1) serial output
//	no cr/lf translation
//-----------------------------------------------------------------------------
char putchar1(char c){
#ifdef UART1_ENABLED
	// output cr if c = \n		// no xmodem
	if(c == '\n'){
	    wait_reg0(&UART1_FR_R, UART_FR_TXFF, CHR_WAT0);	// wait up for fifo to clear
//		while((UART0_FR_R & 0x0020) == 0x0020);
		UART1_DR_R = '\r';
	}
	// output character
    wait_reg0(&UART1_FR_R, UART_FR_TXFF, CHR_WAT0);	// wait up for fifo to clear
//	while((UART0_FR_R & 0x0020) == 0x0020);
	UART1_DR_R = c;
#endif
	return (c);
}

//-----------------------------------------------------------------------------
// getchr1 looks for chr @ port1 rx.  If none, return '\0'
//	uses circular buffer rxd1_buff[] which is filled in the UART1 interrupt
//-----------------------------------------------------------------------------
char getchr1(void){

	char c = '\0';

	if(rxd1_tptr != rxd1_hptr){						// if head != tail,
		c = rxd1_buff[rxd1_tptr++];					// get chr from circ-buff and update tail ptr
		if(rxd1_tptr >= RXD1_BUFF_END){				// process tail wrap-around
			rxd1_tptr = 0;
		}
	}
	return c;
}

//-----------------------------------------------------------------------------
// gotchr1 checks if there are any chrs in the buffer.
//-----------------------------------------------------------------------------
char gotchr1(void){
	char c = FALSE;			// return val, default to "no chr"

    if((rxd1_tptr != rxd1_hptr) && rxd1_msgcnt){	// if (head != tail) && there is at least 1 msg..
    	c = TRUE;									// .. set chr ready to get flag
	}
	return c;
}

//-----------------------------------------------------------------------------
// gotmsgn1 returns the rcvd msg count.
//-----------------------------------------------------------------------------
U8 gotmsgn1(void){

	return rxd1_msgcnt;								// pass the msg count status
}

//-----------------------------------------------------------------------------
// gets1() does gets from UART1 buffers
//	On entry, string points to start of the move-to destination.  On exit,
//	string points to end of dest string + 1.
//-----------------------------------------------------------------------------
char* gets1(char *string){
	char c = '\0';

	if(rxd1_msgcnt){								// if there is at least 1 message in the buffer..
		do{
			if(gotchr1()){
				c = getchr1();						// ..transfer rx1 buffer to string
				*string++ = c;
			}
		}while((c != EOM1) && (c != '\0'));
		*string++ = '\0';							// null terminate
		NVIC_DIS0_R = NVIC_EN0_UART1;				// disable UART1 intr
		rxd1_msgcnt--;								// decrement msg count
		NVIC_EN0_R = NVIC_EN0_UART1;				// enable UART1 intr
	}
	return string;									// return pointer to (end of dest) + 1
}


//-----------------------------------------------------------------------------
// puts1() does puts to UART1
//-----------------------------------------------------------------------------
int puts1(const char *string){

	while(*string){
		putchar1(*string++);
	}
	putchar1('\n');
	return 0;
}

//-----------------------------------------------------------------------------
// putss1() does puts w/o newline
//-----------------------------------------------------------------------------
int putss1(const char *string){

	while(*string){
		putchar1(*string++);
	}
	return 0;
}

//-----------------------------------------------------------------------------
// getch01 checks for input @ RX0.  If no chr, wait forever.
//	waits for a chr and returns.  Note: Processor spends most of its idle
//	time waiting in this fn.
//	looks for input in rxd_buff[] which is filled in the UART0 interrupt
//-----------------------------------------------------------------------------
char getch01(void){

	char c;

	while(rxd1_tptr == rxd1_hptr){
		if(process_IO(0)){					// process system I/O
			rxd1_buff[rxd1_hptr++] = '\r';	// phake a CR to bump the system out of the getch00 loop and back up to main()...
			if(rxd1_hptr == RXD1_BUFF_END){
				rxd1_hptr = 0;
			}
		}
	}
	c = rxd1_buff[rxd1_tptr++];
	if(rxd1_tptr == RXD1_BUFF_END){
		rxd1_tptr = 0;
	}
	return c;
}

//-----------------------------------------------------------------------------
// putchar_b (UART1) base serial output (no xmodem layering)
//	no character translation or additions
//	TI0B is tx empty flag mirror
//-----------------------------------------------------------------------------
char putchar_b(char c){

#ifdef	UART1_ENABLED
	// output character
    wait_reg0(&UART1_FR_R, UART_FR_TXFF, CHR_WAT0);	// wait up for fifo to clear
//	while((UART0_FR_R & 0x0020) == 0x0020);
	UART1_DR_R = c;
#endif
	return c;
}

//-----------------------------------------------------------------------------
// putdch(), UART1
//	strips control chars from output
//-----------------------------------------------------------------------------
char putdch(char c){

	if((c >= ' ') && (c <= '~')){
		putchar1(c);
	}
	return (c);
}

//-----------------------------------------------------------------------------
// puthex(), UART1
//	displays param as ascii hex
//-----------------------------------------------------------------------------
U8 puthex(U8 dhex){

	char c;

	c = (dhex >> 4) + '0';
	if(c > '9') c += 'A' - '9' - 1;
	putchar1(c);
	c = (dhex & 0x0f) + '0';
	if(c > '9') c += 'A' - '9' - 1;
	putchar1(c);
	return (dhex);
}

//-----------------------------------------------------------------------------
// putsN() does puts to UART1 converting all cntl codes to "$xx"
//-----------------------------------------------------------------------------
int putsN(const char *string){

	while(*string){
		if((*string < ' ') || (*string > 0x7e)){
			putchar1('$');
			puthex(*string++);
		}else{
			putchar1(*string++);
		}
	}
	putchar1('\n');
	return 0;
}

//==================== UART INTRPT Fns =================================

//-----------------------------------------------------------------------------
// rxd_intr() UART rx intr (CLI I/O).  Captures RX data and places into
//	circular buffer
//-----------------------------------------------------------------------------
void rxd_intr(void){
	char c;			// temp

	if(!(UART0_RIS_R & UART_RIS_RXRIS)){			// if intrpt is NOT rxd,
													// clear ALL intr flags and abort ISR
		UART0_ICR_R = UART_ICR_9BITIC|UART_ICR_OEIC|UART_ICR_BEIC|UART_ICR_PEIC|UART_ICR_FEIC|UART_ICR_RTIC|UART_ICR_TXIC|UART_ICR_CTSMIC;
		return;
		}
	while(!(UART0_FR_R & UART_FR_RXFE)){			// process RXD chrs
		c = UART0_DR_R;
		if((c >= ' ') || (c == ESC)) bchar = c;		// if not nul, set global char escape reg
		rxd_buff[rxd_hptr++] = c;					// feed the buffer
		if(rxd_hptr >= RXD0_BUFF_END){
			rxd_hptr = 0;							// wrap buffer ptr
		}
		if(rxd_hptr == rxd_tptr){
			rxd_stat |= RXD_ERR;					// buffer overrun, set error
		}
	}
	UART0_ICR_R = UART_ICR_RXIC;					// clear RXD intr flag
	return;
}

//-----------------------------------------------------------------------------
// rxd1_intr() UART1 rx intr (CCMD I/O).  Captures RX data and places
//	into circular buffer.  Also handles pre-processed TX data.
//	Control characters (except '\r') are stripped from the incoming data
//	stream.
//	NOTE:
//	rxd1buff[], frm1_buff[], rxd1_state, rxd1_tptr, rxd1_hptr, rxd1_stat,
//	rxd1_msgcnt, rxd1_dlecnt, and rxd1_lastc are all defined as file-local
//	variables to allow local function access.
//-----------------------------------------------------------------------------
//	TX data is processed from a head/tail tx buffer.  putchar1() fills the buffer, and
//	txstart1() starts the TX process by sending the first chr from the buffer.
//	When the buffer is empty, the TXINT is cleared and the buffer fill/start process must
//	be repeated to send another message.  This system only allows one message to be sent
//	at a time, so the message layer must call txstatus1() to see if the message is done
//	sending before attempting to send another message.

void rxd1_intr(void){
	char c;			// temp

	if(!(UART1_RIS_R & UART_RIS_RXRIS)){			// if intrpt is NOT rxd,
													// clear ALL intr flags and abort ISR
		UART1_ICR_R = UART_ICR_9BITIC|UART_ICR_OEIC|UART_ICR_BEIC|UART_ICR_PEIC|UART_ICR_FEIC|UART_ICR_RTIC|UART_ICR_TXIC|UART_ICR_CTSMIC;
		return;
	}
	while(!(UART1_FR_R & UART_FR_RXFE)){			// process RXD chrs
		c = UART1_DR_R;
		if((c >= ' ') || (c == ESC)) bchar = c;		// if not nul, set global char escape reg
		rxd1_buff[rxd1_hptr++] = c;					// feed the buffer
		if(rxd1_hptr >= RXD1_BUFF_END){
			rxd1_hptr = 0;							// wrap buffer ptr
		}
		if(rxd1_hptr == rxd1_tptr){
			rxd1_stat |= RXD_ERR;					// buffer overrun, set error
		}
	}
	UART1_ICR_R = UART_ICR_RXIC;					// clear RXD intr flag
    return;
}

//-----------------------------------------------------------------------------
// Timer1A_ISR() drives UART TX for UART 2, 5, & 7
//-----------------------------------------------------------------------------
//
// Called when timer 1A overflows (NORM mode):
//	Timer1_A runs as 16 bit reload timer set to 104us interrupt rate
//	This is 12 bits at 115200 baud
//
//-----------------------------------------------------------------------------

void Timer1A_ISR(void)
{

//    GPIO_PORTD_DATA_R ^= SPARE4;

	txd1_rdy = 1;
	TIMER1_ICR_R = TIMER1_MIS_R & TIMER_MIS_AMASK;
    return;
}
