/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: main.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the main code file for the IC-900 RDU Clone application.
 *
 *******************************************************************/

/********************************************************************
 *  Project scope rev notes:
 *    				 To-do Checklist time!
 *    				 * Modify send_vfo() and all other HIB interfaces to use SPI-NVRAM
 *    				 * test band switching (VFO button)
 *    				 * Memory/Call mode:
 *    				 	- establish memory map for stored channels
 *    				 	- work out how to swap vfo pointer between VFO mode and memory mode
 *    				 	- MWchr/CALLchr: memory/call mode transitions - need to remember previous mode.  Canceling a mode
 *    				 		transitions back to the previous mode (need a bubble chart here)
 *    				 	- MWchr_H/CALLchr_H: store memory
 *    				 * SET loop:
 *    				 	- DIM brightness setting, LCD and button
 *    				 	- BRT brightness setting, LCD and button
 *    				 * XIT/RIT adjust (perhaps use VFOchr_H ???)
 *    				 * come up with a way to suppress key-hold beeps if there is no feature implemented for a specific key
 *
 *
 *    Project scope rev History:
 *    09-09-21 jmh:	 Incorporated NVRAM mod (revA of the schem, dated 9/09/21 or later).  Connects a 1Mb auto-store SPI NVRAM
 *    					to the SPI port.  Use this instead of HIB and EEPROM to store NV data.  Requires bit-bang SPI because
 *    					there are no SSIRX I/O pins available without significantly upsetting the GPIO map.
 *    				 Added Timer1B to pace the bit-bang SPI.
 *    				 Added shift_spi() to localize the generic parts of the BB SPI algorithm.
 *    				 Modified send_spi3() to simply wait for busy, then send inverted data to shift_spi().
 *    				 Added beginnings of support Fns and #defines for the NVRAM command interface.
 *    				 Replaced IC2 on LCD board to correct wonky segment behavior.  Initial results good (always, until they are not).
 *    09-06-21 jmh:	 Placed hooks for managing the MUX of LOCK and MISO (renamed #defines and added inhibit to lock/dim read)
 *    09-03-21 jmh:	 mic u/d buttons debugged and working with step-repeat after 1 sec (added step-repeat and debounce)
 *    				 Added 2 and 3 beep mode to main.c to allow lcd.c to signal mode timeouts.  These are handled in the
 *    				 	main timer interrupt, so there is no delay to the lcd.c process loops.
 *    				 M/S during s-mute: revamped the mute mechanism to handle band swaps (removed the save/restore volume
 *    				 	array nonsense).  radio.c now has a mirror flag which is used to mute audio at the SOUT "source".
 *    				 	mute_radio & get_mute_radio are the gateway fns.  Hi-bit of local mirror flag indicates when the mute
 *    				 	message has been delivered to the radio.  This can be used to control band-swap mutes.  However, some
 *    				 	kind of trap needs to be implemented to cover band swaps while smute functions are in play.
 *    				 	!!! need some code debug here to handle the band-swaps
 *    				 TSchr_H mode to adjust a/b freq in steps (A/B) of 5/10, 5/25, or 10/25 Khz.
 *    				 Added code to cancel vol/squ if VFO, MR, SUB, MS, or CALL keys are pressed.
 *    08-23-21 jmh:	 VOL/SQU display working by modifying srf display routines to differentiate the levels from SRF values
 *    					using the hi-bit of the calling parameter ( see msmet() and ssmet() ).
 *    08-22-21 jmh:	 hib crc working.  Re-worked hib structures to save some memory.
 *    				 thumbwheel freq mode working.  Press-hold MHZ to enter thumbwheel mode.  MHZ cycles through digits,
 *    				 	timeout after 10 sec, any key except SQU/VOL will cancel.
 *    				 Duplex works and freq updates during TX.
 *    				 Next up: VOL, SQU, and PL tone.  Use press-hold to modify tone (TONE), step (TS), and tx offset (DUP).
 *    08-21-21 jmh:	 Modified wrwhib_ram to use a U8 address index
 *    08-19-21 jmh:	 Got key-hold feature incorporated.  Produces same ASCII char as for initial press, but with hi-bit set.
 *    				 Broke out process_main, process_sub, process_mem, and process_set from process_ui. These are called
 *    				 as the current DU mode dictates.
 *    08-18-21 jmh:	 Basic T/R process working.
 *    					HIB ram working and a structure of module data is configured to store there
 *    					A crc validates the NVRAM and copies it over to working registers at IPL.
 *    					If invalid, the working registers are filled with defaults.
 *    					The high byte of 10m and 6m VFO are used to hold XIT/RIT.  The high byte of 2m, 220, and 440
 *    					can also be used to hold data.  Other than these locations, only one more byte of NVRAM
 *    					is currently available.
 *    					!!! This storage paradigm is endian-sensitive !!! porting this code to a different processor
 *    					will likey break the "high byte" scavange scheme.
 *    08-14-21 jmh:	 Basic RX process in-place.  A fixed freq lashup for 2m/440.  Updates SMET and RX leds.
 *    08-10-21 jmh:	 Added process_SIN() to input status data from base unit.
 *    08-08-21 jmh:	 Freq main and sub debugged.  Also msmet and ssmet and channel display.  Finished
 *    					ASCII display Fns for M/S freq and M/S mem channel.
 *    				 All basic LCD routines are coded and mostly tested.  The code is sufficient to allow
 *    				 	a basic VFO radio to be constructed (including the TX/RX LEDs).
 *    08-07-21 jmh:	 CHECK & SMUTE were shorted (fixed).  DIAL_DN was also shorted to GND at the encoder
 *    					terminals (fixed).
 *    				 uPD7225 max-clock speed seems much lower than the datasheet specs.  Need to run Zo
 *    				 	GND tape/wire and see if the max rate can be increased.
 *    08-06-21 jmh:	 Modified keyscan to incorporate discrete buttons (CHECK, HILO, & SMUTE). {CHECK & SMUTE
 *    					appear to be shorted in the hardware, need to confirm/fix}. Added prescale loop to
 *    					keyscan ISR to allow COL to settle longer before gathering inputs.
 *    08-05-21 jmh:	 Edits for RDU application...  Moved QEI code to "encoder.c"
 *    03-23-19 jmh:  Added timer1a as serial pacing timer for UART1.  Runs at 10KHz, giving an extra
 *    					2 stop bits to serial outputs from UART1.
 *    03-12-19 jmh:  Debugged HM-133 support.  Had to increase min bit time from 200 to 220 to get
 *    					both the HM151 and HM133 to function.
 *    03-07-19 jmh:  Added HM-133 support.  DTMF, func, and PTT bits in character stream are
 *    					trapped and the keycode outputs modified (DTMF has hi-bit set, Fn mode
 *    					uses all lower-case characters).  Added comments to arrays and Fns.
 *						Also added "PTT" code, ";" support.
 *    03-03-19 jmh:  Changed "s" commands to removed accesses to IC7K data control. Must now use
 *						"d" command separately to set data status.
 *					 Cleaned up comments for HMic keypad.
 *    12-29-16 jmh:  fixed bug in keypad that would trap a double keypress and try to send a NULL to
 *    				 	the ACU.
 *    12-28-16 jmh:  Added LED control logic.
 *    				 re-worked RESP LED to use a sequence table.  RESP LED responds to i5xx commands
 *    				 	to set level, but not to master level.  All other LEDs are controlled by
 *    				 	the LED and master levels.  L5x will turn resp LED on or off.
 *    				 re-worked ccmds to increase syntax error reporting.
 *    				 re-worked getch00 to allow it to trap swcmd == SW_ESC signal and return to main()
 *    				 	to allow warm-reset to be processed (restarts main()).
 *    				 Added free-running timer read to ccmd list.
 *    				 debugged ACU loop time ccmd and POR init.
 *    				 fixed bug in keypad that would trap a double keypress and try to send a NULL to
 *    				 	the ACU.
 *    12-26-16 jmh:  Updated keypad matrix LUT to include CW paddle buttons (left, right, and PB)
 *    					"{", "}", and "^" are the characters sent for these buttons.
 *    				 Added PTT repeat logic for PTTb and PTT7K.
 *    				 Added ccmd support for mic switches and HM151 data switch.
 *    				 Fixed HM151 data capture to ignore data if IC7K is the mic switch load.
 *    				 Fixed HM151 key capture: was not sending ";" terminator until next keypress.
 *    09-20-16 jmh:  creation date
 *    				 <VERSION 0.0>
 *
 *******************************************************************/

//-----------------------------------------------------------------------------
// main.c
//  Interfaces keypad/encoders to ACU
//  UART0 is used for bluetooth I/O.
//
//  Debug CLI is a simple maintenance/debug port (via UART1 @PB[1:0]) with the following core commands:
//		VERS - interrogate SW version.
//		See "cmd_fn.c" for CLI details
//
//  Bluetooth module communicates via UART0 @PA[1:0].  A future option to allow remote access to the
//		radio control core.
//
//	Uses GPIO to drive scan decoder and scan inputs for the RDU key interface (16 keys on a 2x8 matrix,
//		and three keys on a dedicated GPIO).  There are also two slide switches that are routed to
//		dedicated GPIOs.
//
//		Timer2 drives COL[1:0] and reads the 8 key inputs.  These signals and activity timers feed into
//		a state machine that qualifies and buffers matrix key-presses.
//
//		Discrete keys/switches are captured with an edge detect flag processed inside the timer2 loop.
//
//	SPI3 (PD0 = SCLK, PD3 = MOSI) sends data to the LCD controllers.  GPIOs are used for the handshake
//		signals.
//
//  Interrupt Resource Map:
//	*	Timer0A			PF0:		SW gated 1KHz pulse output to drive piezo spkr (uses PWM and ISR to generate and gate off the beep)
//	*	Timer1A			--			serial pacing timer
//	*	SSI1			PF1:		ASO async output (4800 baud, 1 start, 30 bit + 1 stop (plus an implied stop bit)
//		Timer2A			PF4:		isr ASI async input (4800 baud, 1 start, ... )
//		GPIO FE			PF4:		ISR (detects ASI start bit, starts Timer2A)
//	*	UART0 			PA[1:0]:	Bluetooth serial port
//		UART1			PB[1:0]:	PGM command port (TTL to RS232 to PC)
//	*	M1PWM (6&7)		PF2, PF3:	LED PWMs (PF3 = backlight, PF2 = all other LEDs)
//		GPIO edge		PC[6:5]:	Main dial (up/dn type), edge intrpts to count encoder pulses
//		SSI3			PD0, PD3:	LCD command/data comms.  Uses PD1, PD6, PE0, PE1, & PB7
//	*	Timer3A			--			Application timers & keyscan
//		Timer3B			--			debounce timer (dial and keys)
//		ADC0			PD2:		Ambient light sensor
//
//		QEI1			(future) @PC[6:5]: Main dial
//
//
//  I/O Resource Map:
//      See "init.h"
//
//	SW: Basic functions:
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
// compile defines

#define MAIN_C
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdio.h>
#include <string.h>
#include "init.h"
#include "typedef.h"
#include "version.h"
#include "serial.h"
#include "cmd_fn.h"
#include "tiva_init.h"
#include "eeprom.h"
#include "lcd.h"
#include "radio.h"
//#include "encoder.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

//  see init.h for main #defines
#define	MAX_REBUF	4		// max # of rebufs
#define	MAX_BARS	7		// max # of led bars
#define	GETS_INIT	0xff	// gets_tab() static initializer signal
#define DAC_LDAC 20			// dac spi defines
#define DAC_PWRUP 21
#define DAC_CLR 22


// quick beep macro
#define	q_beep   beep_counter = beep_count; \
				 TIMER0_CTL_R |= (TIMER_CTL_TAEN);

// dial beep macro
#define	d_beep   beep_counter = DIAL_BEEP_COUNT; \
				 TIMER0_CTL_R |= (TIMER_CTL_TAEN);

//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------

// Processor I/O assignments
//bitbands
#define DUT_ON  (*(volatile uint32_t *)(0x40058000 + (0x04 * 4)))
#define DUT_OFF (*(volatile uint32_t *)(0x40058000 + (0x08 * 4)))
#define DUT_SCK (*(volatile uint32_t *)(0x40058000 + (0x10 * 4)))
#define DUT_D   (*(volatile uint32_t *)(0x40058000 + (0x20 * 4)))
#define DUT_Q   (*(volatile uint32_t *)(0x40058000 + (0x40 * 4)))
#define DUT_CS  (*(volatile uint32_t *)(0x40058000 + (0x80 * 4)))

//-----------------------------------------------------------------------------
// Global variables (extern conditionals are in init.h)
//-----------------------------------------------------------------------------
char	bchar;							// break character trap register - traps ESC ascii chars entered at terminal
char	swcmd;							// software command flag
S8		handshake;						// xon/xoff enable
S8		xoffsent;						// xoff sent

//-----------------------------------------------------------------------------
// Local variables in this file
//-----------------------------------------------------------------------------
U32		abaud;							// 0 = 115.2kb (the default)
U8		iplt2;							// timer2 ipl flag
U16		waittimer;						// gp wait timer
U8		dialtimer;						// dial debounce wait timer
U8		sintimer;						// sin activity timer
U8		souttimer;						// sout pacing timer
U16		mhztimer;						// mhz digit access timer
U16		qtimer;							// squ access timer
U16		vtimer;							// vol access timer
U16		offstimer;						// offs access timer
U16		settimer;						// set access timer
U16		subtimer;						// sub focus timer
U8		beepgaptimer;					// beep gap timer
U16		mictimer;						// mic button repeat timer
U8		micdbtimer;						// mic button dbounce timer
U8		mutetimer;						// vol mute timer
U16		tstimer;						// TS adj mode timer
U32		free_32;						// free-running ms timer
S8		err_led_stat;					// err led status
U8		idx;
U16		ipl;							// initial power on state

#define KBD_ERR 0x01
#define KBD_BUFF_END 5
U16		S4_stat;						// holds de-mux'd status of spare_S4 switch
U16		kbd_buff[KBD_BUFF_END];			// keypad data buffer
U8		kbd_hptr;						// keypad buf head ptr
U8		kbd_tptr;						// keypad buf tail ptr
U8		kbd_stat;						// keypad buff status
U8		kbdn_flag;						// key down or hold
U8		kbup_flag;						// key released
U32		sys_error_flags;				// system error flags
U8		debug_i;
U8		ptt_mode;						// ptt update mode (process_io)
//U8		led_2_on;						// led on regs (1/0)
//U8		led_3_on;
//U8		led_4_on;
U8		led_5_on;
U8		led_6_on;
//U8		led_2_level;					// led level regs (0-100%)
//U8		led_3_level;
//U8		led_4_level;
U8		led_5_level;
U8		led_6_level;
//U16		pwm2_reg;						// led pwm regs
//U16		pwm3_reg;
//U16		pwm4_reg;
U16		pwm5_reg;
U16		pwm6_reg;
U8		pwm_master;						// led master level
S8		main_dial;
U16		beep_count;						// beep duration register
U16		beep_counter;					// beep duration counter
U8		num_beeps;						// number of beeps counter
U8		sw_state;
U8		sw_change;
//char	dbbuf[30];	// debug buffer
//U8		dbug8;
U8		hib_access;						// HIB intrpt lock-out flag

//-----------------------------------------------------------------------------
// Local Prototypes
//-----------------------------------------------------------------------------

void Timer_Init(void);
void Timer_SUBR(void);
char *gets_tab(char *buf, char *save_buf[3], int n);
char kp_asc(U16 keycode);

//*****************************************************************************
// main()
//  The main function runs a forever loop in which the main application operates.
//	Prior to the loop, Main performs system initialization, boot status
//	announcement, and main polling loop.  The loop also calls CLI capture/parse fns.
//	The CLI maintains and processes re-do buffers (4 total) that are accessed by the
//	TAB key.  Allows the last 4 valid command lines to be recalled and executed
//	(after TAB'ing to desired recall command, press ENTER to execute, or ESC to
//	clear command line).
//	Autobaud rate reset allows user to select alternate baudarates after reset:
//		115,200 baud is default.  A CR entered after reset at 57600, 38400,
//		19200, or 9600 baud will reset the system baud rate and prompt for user
//		acceptance.  Once accepted, the baud rate is frozen.  If rejected, baud rate
//		returns to 115200.  The first valid command at the default rate will also
//		freeze the baud rate.  Once frozen, the baud rate can not be changed until
//		system is reset.
//*****************************************************************************
int
main(void)
{
	volatile uint32_t ui32Loop;
//	uint32_t i;
//    uint8_t	tempi;			// tempi
    char	buf[40];			// command line buffer
    char	rebuf0[40];			// re-do buffer#1
    char	rebuf1[40];			// re-do buffer#2
    char	rebuf2[40];			// re-do buffer#3
    char	rebuf3[40];			// re-do buffer#4
    char	got_cmd;			// first valid cmd flag (freezes baud rate)
    U8		argn;				// number of args
    char*	cmd_string;			// CLI processing ptr
    char*	args[ARG_MAX];		// ptr array into CLI args
    char*	rebufN[4];			// pointer array to re-do buffers
    U16		offset;				// srecord offset register
    U16		cur_baud;			// current baud rate
    U8		fault_found;		// fault detected flag

    fault_found = FALSE;								// only trap one fault-restart
	got_cmd = FALSE;
	offset = 0;
	cur_baud = 0;
//	iplt3 = 1;											// init timer3
    iplt2 = 1;											// init timer1
    ipl = proc_init();									// initialize the processor I/O
    main_dial = 0;
    init_spi3();
    do{													// outer-loop (do forever, allows soft-restart)
        rebufN[0] = rebuf0;								// init CLI re-buf pointers
    	rebufN[1] = rebuf1;
    	rebufN[2] = rebuf2;
    	rebufN[3] = rebuf3;
    	while(iplt2);									// wait for timer to finish intialization
    	wait(200);										// else, pad a bit of delay for POR to settle..
    	dispSWvers(); 									// display reset banner
    	wait(10);										// a bit of delay..
    	rebuf0[0] = '\0';								// clear cmd re-do buffers
    	rebuf1[0] = '\0';
    	rebuf2[0] = '\0';
    	rebuf3[0] = '\0';
    	bcmd_resp_init();								// init bcmd response buffer
    	wait(10);										// a bit more delay..
//    	GPIO_PORTB_DATA_R &= ~PTT7K;					// set PTT7K in-active
//    	GPIO_PORTD_DATA_R &= ~PTTb;						// set PTTb in-active
    	while(gotchrQ()) getchrQ();						// clear serial input in case there was some POR garbage
    	gets_tab(buf, rebufN, GETS_INIT);				// initialize gets_tab()
        set_pwm(0, 100);								// master = 100%
        set_led(0xff, 0x00);							// init LED levels
        set_led(5, 1);									// enable LEDs at 30% until the OS takes over
        set_pwm(5, 30);
        set_led(6, 1);
        set_pwm(6, 30);
    	process_IO(0xff);								// init process_io
    	// GPIO init
    	//...
    	swcmd = 0;										// init SW command
    	// main loop
        do{
    		putchar_b(XON);
    		buf[0] = '\0';
    		putssQ("rdu>");										// prompt
    		cmd_string = gets_tab(buf, rebufN, 80); 			// get cmd line & save to re-do buf
    		if(!got_cmd){										// if no valid commands since reset, look for baud rate change
    			if(cur_baud != abaud){							// abaud is signal from gets_tab() that indicates a baud rate change
    				if(set_baud(abaud)){						// try to set new baud rate
    					putsQ("");								// move to new line
    					dispSWvers();							// display reset banner & prompt to AKN new baud rate
    					while(gotchrQ()) getchrQ();				// clear out don't care characters
    					putssQ("press <Enter> to accept baud rate change: ");
    					while(!gotchrQ());						// wait for user input
    					putsQ("");								// move to new line
    					if(getchrQ() == '\r'){					// if input = CR
    						cur_baud = abaud;					// update current baud = new baud
    						got_cmd = TRUE;						// freeze baud rate
    					}else{
    						set_baud(0);						// input was not a CR, return to default baud rate
    						cur_baud = abaud = 0;
    					}
    				}else{
    					abaud = cur_baud;						// new baud rate not valid, ignore & keep old rate
    				}
    			}else{
    				got_cmd = TRUE;								// freeze baud rate (@115.2kb)
    			}
    		}
    		argn = parse_args(cmd_string,args);					// parse cmd line
    		if(x_cmdfn(argn, args, &offset)) got_cmd = TRUE;	// process cmd line, set got_cmd if cmd valid
    		if((NVIC_FAULT_STAT_R) && !fault_found){			// test for initial fault
    	        putsQ("nvic FLT\n");							// fault message
    			swcmd = SW_ESC;									// abort to restart
    			fault_found = TRUE;
    		}
        }while(swcmd != SW_ESC);
        swcmd = 0;
/*    	NVIC_DIS0_R = 0xFFFFFFFF;								// disable ISRs
    	NVIC_DIS1_R = 0xFFFFFFFF;
    	NVIC_DIS2_R = 0xFFFFFFFF;
    	NVIC_DIS3_R = 0xFFFFFFFF;
    	NVIC_DIS4_R = 0xFFFFFFFF;*/
    }while(1);
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// gets_tab puts serial input into buffer, UART0.
//-----------------------------------------------------------------------------
// Main loop for command line input.
// waits for a chr and puts into buf.  If 1st chr = \t, copy re-do buf into
//  cmdbuf and cycle to next re-do buf.  if more than n chrs input, nul term buf,
//	disp "line too long", and return.  if \n or \r, copy buf to save_buf & return
//  returns buf (pointer).
//	if n == 0xff, initialize statics and exit.
//
//	11/08/13: Modified to support 4 (MAX_REBUF) rolling cmd save buffers
//	11/15/13: Modified to support auto-baud detect on CR input
//		For the following, each bit chr shown is one bit time at 115200 baud (8.68056us).
//			s = start bit (0), p = stop bit (1), x = incorrect stop, i = idle (1), bits are ordered  lsb -> msb:
//	 the ascii code for CR = 10110000
//			At 115.2kb, CR = s10110000p = 0x0D
//
//			At 57.6 kb, CR = 00110011110000000011 (1/2 115.2kb)
//			@115.2, this is: s01100111ps00000001i = 0xE6, 0x80
//
//			At 38.4 kb, CR = 000111000111111000000000000111 (1/3 115.2kb)
//			@115.2, this is: s00111000p11111s00000000xxxiii = 0x1c, 0x00
//
//			At 19.2 kb, CR = 000000111111000000111111111111000000000000000000000000111111 (1/6 115.2kb)
//			@115.2, this is: s00000111piis00000111piiiiiiiis00000000xxxxxxxxxxxxxxxiiiiii = 0xE0, 0xE0, 0x00
//
//			At 9600 b,  CR = 000000000000111111111111000000000000111111111111111111111111000000000000000000000000000000000000000000000000111111111111 (1/12 115.2kb)
//			@115.2, this is: s00000000xxxiiiiiiiiiiiis00000000xxxiiiiiiiiiiiiiiiiiiiiiiiis00000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxiiiiiiiiiiii = 0x00, 0x00, 0x00
//
//		Thus, @ 57.6 kb, a CR = 0xE6 followed by 0x80
//			  @ 38.4 kb, a CR = 0x1C followed by 0x00
//			  @ 19.2 kb, a CR = 0xE0 followed by 0xE0 (plus a 0x00)
//			  @ 9600 b, a  CR = 0x00 followed by 0x00 (plus a 0x00)
//
//		NOTE: gets_tab is only used for command line input and thus should not
//		see non-ascii data under normal circumstances.

char *gets_tab(char *buf, char *save_buf[], int n)
{
	char	*cp;
	char	*sp;
	char	c;
	int		i = 0;
	U8		j;
	static	U8   rebuf_num;
	static	U8	 last_chr;

//	if((rebuf_num >= MAX_REBUF) || (n == GETS_INIT)){ // n == 0xff is static initializer signal
	if(n == GETS_INIT){ // n == 0xff is static initializer signal
		rebuf_num = 0;									// init recall buffer pointer
		last_chr = 0xff;								// init to 0xff (not a valid baud select identifier chr)
		return buf;										// skip rest of Fn
	}
    cp = buf;
    sp = save_buf[rebuf_num];
    do{
        c = getch0Q();
        switch(c){
			case 0xE0:									// look for 19.2kb autoselect
				if(last_chr == 0xE0){
					abaud = 19200L;
					c = '\r';
				}
				break;

			case 0x00:									// look for 38.4kb or 9600b autoselect
				if(last_chr == 0x1C){
					abaud = 38400L;
					c = '\r';
				}else{
					if(last_chr == 0x00){
						abaud = 9600L;
						c = '\r';
					}
				}
				break;

			case 0x80:									// look for 57.6kb autoselect
				if(last_chr == 0xE6){
					abaud = 57600L;
					c = '\r';
				}
				break;

            case '\t':
				if(i != 0){								// if tab, cycle through saved cmd buffers
					do{
						i--;							// update count/point
						cp--;
						if((*cp >= ' ') && (*cp <= '~')){
							putcharQ('\b');				// erase last chr if it was printable
							putcharQ(' ');
							putcharQ('\b');
						}
					}while(i != 0);
					cp = buf;							// just in case we got out of synch
				}
				//copy saved string up to first nul, \n, or \r
				j = rebuf_num;
				do{
					if(--rebuf_num == 0xff){
						rebuf_num = MAX_REBUF - 1;
					}
					if(*save_buf[rebuf_num]) j = rebuf_num;
				}while(j != rebuf_num);
				sp = save_buf[rebuf_num];
				while((*sp != '\0') && (*sp != '\r') && (*sp != '\n')){
					putdchQ(*sp);
					*cp++ = *sp++;
					i++;
				}
                break;

            case '\b':
            case 0x7f:
                if(i != 0){								// if bs & not start of line,
                    i--;								// update count/point
                    cp--;
                    if((*cp >= ' ') && (*cp <= '~')){
                        putcharQ('\b');					// erase last chr if it was printable
                        putcharQ(' ');
                        putcharQ('\b');
                    }
                }
                break;

            case '\r':									// if cr, nul term buf & exit
            case '\n':									// if nl, nul term buf & exit
                i++;
                *cp++ = c;
                break;

            case ESC:									// if esc, nul buf & exit
                cp = buf;
                c = '\r';								// set escape condition
				i = 0;
                break;

            default:
                i++;
                *cp++ = c;								// put chr in buf
                putdchQ(c);								// no cntl chrs here
                break;
        }
		last_chr = c;									// set last chr
    } while((c != '\r') && (c != '\n') && (i < n));		// loop until c/r or l/f or buffer full
	if(i >= n){
		putsQ("!! buffer overflow !!");
		*buf = '\0';									// abort line
	}else{
		putsQ("");										// echo end of line to screen
		*cp = '\0';										// terminate command line
		if((*buf >= ' ') && (*buf <= '~')){				// if new buf not empty (ie, 1st chr = printable),
			strncpy(save_buf[rebuf_num], buf, n);		// copy new buf to save
			if(++rebuf_num >= MAX_REBUF) rebuf_num = 0;
		}
	}
    return buf;
}

//-----------------------------------------------------------------------------
// process_IO() processes system I/O
//-----------------------------------------------------------------------------
char process_IO(U8 flag){

	// process IPL init
	if(flag == 0xff){									// perform init/debug fns
		// init LCD
		init_lcd();
		ptt_mode = 0xff;								// force init
		process_SIN(flag);
		process_SOUT(flag);
		process_UI(flag);
//		process_CCMD(flag);
	}
	process_SIN(0);
	process_SOUT(0);
	process_UI(0);
//	process_CCMD(0);									// process CCMD inputs
	return swcmd;
}

//-----------------------------------------------------------------------------
// get_status() returns true if:
//	* any encoder change
//	* got_key == true OR kbdn_flag != 0
//	* got_hm == true
//-----------------------------------------------------------------------------
U8 get_status(U8* keyh_mem){
	U8	rtn = 0;

/*
//	rtn = get_enc_status();
	if(got_key()) rtn |= KEY_CHNG;
	//if keyup AND keydn, then it is a key released event
	if(kbdn_flag && kbup_flag) rtn |= KEY_CHNG;
	if(kbdn_flag & KEY_HOLD_FL){
		kbdn_flag &= ~KEY_HOLD_FL;
		rtn |= KEYH_CHNG;
	}
	if(got_hmd()) rtn |= KEYHM_CHNG;
	if((debounceHM_timer == 0) && (*keyh_mem != KEY_NULL)) rtn |= KEYHM_CHNG;*/
	return rtn;
}

//-----------------------------------------------------------------------------
// set_pttmode() sets the value of the pttmode register
//	0 = PTTb follows ~/PTT2, PTT7K disabled
//	1 = PTTB follows ~/PTTHM, ccmds to drive PTT7K enabled
//	any other value returns the current setting with no changes
//-----------------------------------------------------------------------------
U8 set_pttmode(U8 value){

	if(value == 0x00) ptt_mode = 0;
	if(value == 0x01) ptt_mode = 1;
	return ptt_mode;
}

//-----------------------------------------------------------------------------
// set pwm values
//-----------------------------------------------------------------------------
//
// set_pwm() sets the specified PWM to the percent value
//	pwmnum specifies the LED number.  LED 0 is the master setting.  This only sets the master,
//		it does not update any of the LED pwm registers.
//	percent is the percent level, 0 - 100.  If percent > 100, the value is unchanged
//		and the PWM settings are calculated based on stored led and master values
//

void set_pwm(U8 pwmnum, U8 percent){
	U32	kk;				// temp U32

	if(percent <= 100){
		switch(pwmnum){						// store level % value
		case 0:								// set master value
			pwm_master = percent;
			break;

/*		case 2:
			led_2_level = percent;
			break;

		case 3:
			led_3_level = percent;
			break;

		case 4:
			led_4_level = percent;
			break;*/

		case 5:
			led_5_level = percent;
			break;

		case 6:
			led_6_level = percent;
			break;
		}
	}
	switch(pwmnum){
/*	case 2:								// LED2
		// calc PWM value in U32 to avoid overflow
		kk = PWM_ZERO - ((PWM_ZERO - PWM_MAX) * (U32)led_2_level * (U32)pwm_master / 10000L) - 1L;
		// store PWM value
		pwm2_reg = (U16)kk;
		// update pwm if led is on
		if(led_2_on) PWM1_1_CMPB_R = pwm2_reg;
		break;

	case 3:								// LED3
		kk = PWM_ZERO - ((PWM_ZERO - PWM_MAX) * (U32)led_3_level * (U32)pwm_master / 10000L) - 1L;
		pwm3_reg = (U16)kk;
		if(led_3_on) PWM1_2_CMPA_R = kk;
		break;

	case 4:								// LED4
		kk = PWM_ZERO - ((PWM_ZERO - PWM_MAX) * (U32)led_4_level * (U32)pwm_master / 10000L) - 1L;
		pwm4_reg = (U16)kk;
		if(led_4_on) PWM1_2_CMPB_R = kk;
		break;*/

	case 5:								// LED5
		kk = PWM_ZERO - (((U32)(PWM_ZERO - PWM_MAX) * (U32)led_5_level * (U32)pwm_master) / 10000L) - 1L;
		pwm5_reg = (U16)kk;
		if(led_5_on) PWM1_3_CMPA_R = kk;
		break;

	case 6:								// LED6
		kk = PWM_ZERO - ((PWM_ZERO - PWM_MAX) * (U32)led_6_level * (U32)pwm_master / 10000L) - 1L;
		pwm6_reg = (U16)kk;
		if(led_6_on) PWM1_3_CMPB_R = kk;
		break;
	}
}

//-----------------------------------------------------------------------------
// set_led() turns on/off the LED outputs
// if lednum == 0xff, do a POR init of the registers
// if lednum == 0x00, value = master %, reset all PWM values
//	else, value = 1/0 for on/off.  save to reg and update pwm reg to set on or off
//-----------------------------------------------------------------------------
void set_led(U8 lednum, U8 value){

	if(lednum == 0xff){
/*		led_2_on = 0;						// init registers all off
		led_3_on = 0;
		led_4_on = 0;*/
		led_5_on = 0;
		led_6_on = 0;
/*		led_2_level = 24;					// led level regs (0-100%)
		led_3_level = 16;
		led_4_level = 50;*/
		led_5_level = 22;
		led_6_level = 99;
		pwm_master = 99;					// led master level
		// led pwm regs
/*		pwm2_reg = (U16)(PWM_ZERO - ((PWM_ZERO - PWM_MAX) * 24L / 100L) - 1L);
		pwm3_reg = (U16)(PWM_ZERO - ((PWM_ZERO - PWM_MAX) * 16L / 100L) - 1L);
		pwm4_reg = (U16)(PWM_ZERO - ((PWM_ZERO - PWM_MAX) * 50L / 100L) - 1L);*/
		pwm5_reg = (U16)(PWM_ZERO - ((PWM_ZERO - PWM_MAX) * 22L / 100L) - 1L);
		pwm6_reg = (U16)(PWM_ZERO - ((PWM_ZERO - PWM_MAX) * 99L / 100L) - 1L);
	}else{
		// process led settings
		if(value < 2){
			switch(lednum){
/*			case 2:
				led_2_on = value;
				if(led_2_on) PWM1_1_CMPB_R = pwm2_reg;
				else PWM1_1_CMPB_R = PWM_ZERO - 1;
				break;

			case 3:
				led_3_on = value;
				if(led_3_on) PWM1_2_CMPA_R = pwm3_reg;
				else PWM1_2_CMPA_R = PWM_ZERO - 1;
				break;

			case 4:
				led_4_on = value;
				if(led_4_on) PWM1_2_CMPB_R = pwm4_reg;
				else PWM1_2_CMPB_R = PWM_ZERO - 1;
				break;*/

			case 5:
				led_5_on = value;
				if(!led_5_on) PWM1_3_CMPA_R = PWM_ZERO - 1;
				break;

			case 6:
				led_6_on = value;
				if(led_6_on) PWM1_3_CMPB_R = pwm6_reg;
				else PWM1_3_CMPB_R = PWM_ZERO - 1;
				break;
			}
		}
	}
}

//-----------------------------------------------------------------------------
// wrwhib_ram() writes a word to the HIBRAM array
//-----------------------------------------------------------------------------
void wrwhib_ram(U32 data, volatile U8 addr){
	volatile uint32_t* pii; // Vu32 pointer

	pii = &HIB_DATA_R;
	pii += addr;
	hib_access = HIB_APPL;						// lock out intr access
	while(!(HIB_CTL_R & HIB_CTL_WRC));			// pause until HIBRAM is ready
	*pii = data;								// write new data
	hib_access = HIB_INTR;						// enable intr access
}

//-----------------------------------------------------------------------------
// wrhib_ram() writes a databyte to the HIBRAM array using a byte address
//-----------------------------------------------------------------------------
void wrhib_ram(uint8_t data, U8 addr){
				U8			i;						// temp
	volatile	uint32_t*	pii;					// Vu32 pointer
				uint32_t	dd = (uint32_t)data;	// temp data
				uint32_t	ee;						// temp data
				uint32_t	maskdd = 0x000000ff;	// mask

	while(!(HIB_CTL_R & HIB_CTL_WRC));			// pause until HIBRAM is ready
	pii = &HIB_DATA_R;							// get base pointer (assumed to be word aligned)
	pii += (addr >> 2);							// point to desired word addr
	ee = *pii;									// get existing data
	i = addr & 0x03;							// get byte addr
	while(i){									// align byte and mask
		dd <<= 8;
		maskdd <<= 8;
		i--;
	}
	ee &= ~maskdd;								// mask existing data
	ee |= dd;									// combine new daata
	hib_access = HIB_APPL;						// lock out intr access
//	dd = HIB_CTL_R;								// do a dummy read
	*pii = ee;									// write new data
	hib_access = HIB_INTR;						// enable intr access
}

//-----------------------------------------------------------------------------
// rdwhib_ram() reads a word from the HIBRAM array using a word address
//-----------------------------------------------------------------------------
uint32_t rdwhib_ram(U8 addr){
	volatile 	uint32_t* pii;		// Vu32 pointer
				uint32_t	ee;		// temp data

	while(!(HIB_CTL_R & HIB_CTL_WRC));			// pause until HIBRAM is ready
	pii = &HIB_DATA_R;							// get base pointer
	pii += (addr);								// point to desired word addr
	ee = *pii;									// get existing data
	return ee;
}

//-----------------------------------------------------------------------------
// rdhib_ram() reads a databyte from the HIBRAM array using a byte address
//-----------------------------------------------------------------------------
uint8_t rdhib_ram(U8 addr){
				U8	i;				// temp
	volatile 	uint32_t* pii;		// Vu32 pointer
				uint32_t	ee;		// temp data

	while(!(HIB_CTL_R & HIB_CTL_WRC));			// pause until HIBRAM is ready
	pii = &HIB_DATA_R;							// get base pointer
	pii += (addr >> 2);							// point to desired word addr
	ee = *pii;									// get existing data
	i = addr & 0x03;							// get byte addr
	while(i){									// align byte to uint8_t
		ee >>= 8;
		i--;
	}
	ee &= 0xff;
	return (uint8_t)ee;
}

//-----------------------------------------------------------------------------
// waitpio() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//	loops through process_IO during wait period.
//-----------------------------------------------------------------------------
void waitpio(U16 waitms){
//	U32	i;

//	i = 545 * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0) process_IO(0);
    return;
}

//-----------------------------------------------------------------------------
// wait() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//-----------------------------------------------------------------------------
void wait(U16 waitms)
{
//	U32	i;

//	i = 545L * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0);
    return;
}

//-----------------------------------------------------------------------------
// wait2() does quick delay pace =  (5.6us * waitms)
//		Emperical measurements with SYSCLK = 50 MHz give the following
//			timing value: waitms = 2 gives a time delay of about 11.2 us
//			(about 560ns per for() cycle) {+/- interrupt variations}
//-----------------------------------------------------------------------------
void wait2(U16 waitms)
{
	U32	i;

	i = 20 * (U32)waitms;
    waittimer = waitms;
    for(;i!=0;i--);		// patch
//    while(waittimer != 0);
    return;
}

//-----------------------------------------------------------------------------
// wait_busy0() waits for (delay timer == 0) or (LCD BUSY == 0)
//	if delay expires, return TRUE, else return FALSE
//-----------------------------------------------------------------------------
U8 wait_busy0(U16 delay){
	U8 loopfl = TRUE;

    waittimer = delay;
    while(loopfl){
    	if(!waittimer) loopfl = FALSE;
    	if(!(GPIO_PORTE_DATA_R & BUSY_N)) loopfl = FALSE;
    	if(GPIO_PORTE_RIS_R & (BUSY_N)) loopfl = FALSE;
    }
	GPIO_PORTE_ICR_R = (BUSY_N);						// clear edge flag
    return !waittimer;
}

//-----------------------------------------------------------------------------
// wait_busy1() waits for (delay timer == 0) or (LCD BUSY == 0)
//	then wait for BUSY_N == 1
//	if delay expires, return TRUE, else return FALSE
//-----------------------------------------------------------------------------
U8 wait_busy1(U16 delay){
	U8 loopfl = TRUE;

	wait_busy0(delay);
	waittimer = delay;
    while(loopfl){
    	if(!waittimer) loopfl = FALSE;
    	if(GPIO_PORTE_DATA_R & BUSY_N) loopfl = FALSE;
    }
    return !waittimer;
}

//-----------------------------------------------------------------------------
// wait_reg0() waits for (delay timer == 0) or (regptr* & clrmask == 0)
//	if delay expires, return TRUE, else return FALSE
//-----------------------------------------------------------------------------
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & clrmask) != 0));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// wait_reg1() waits for (delay timer == 0) or (regptr* & setmask == setmask)
//	if delay expires, return TRUE, else return FALSE
//-----------------------------------------------------------------------------
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & setmask) != setmask));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// getipl() returns current ipl flags value
//-----------------------------------------------------------------------------
U16 getipl(void){

	return ipl;
}

//-----------------------------------------------------------------------------
// get_syserr() returns current system error flags value
//	if opr == true, clear flags
//-----------------------------------------------------------------------------
U32 get_syserr(U8 opr){

	if(opr) sys_error_flags = 0;
	return sys_error_flags;
}

//-----------------------------------------------------------------------------
// not_key() returns true if key is released.  Optional flag clears released status
//-----------------------------------------------------------------------------
/*U8 not_key(U8 flag){
	char	rtn = FALSE;	// return value

	if(kbup_flag) rtn = TRUE;							// key release is detected
	if(flag) kbup_flag = FALSE;							// clear key_up if flag is true
	return rtn;
}*/

//-----------------------------------------------------------------------------
// got_key() returns true if key is pressed.
//-----------------------------------------------------------------------------
U8 got_key(void){
	char	rtn = FALSE;	// return value

	if(kbd_hptr != kbd_tptr) rtn = TRUE;				// key in buffer means a key was pressed
	return rtn;
}

//-----------------------------------------------------------------------------
// get_key() returns keypad ASCII key or 0x00 if none
//-----------------------------------------------------------------------------
char get_key(void){
	char	rtn = '\0';	// return value
	U16		j;

	if(kbd_hptr != kbd_tptr){
		j = kbd_buff[kbd_tptr++];						// get keypad code
		if(kbd_tptr == KBD_BUFF_END){					// update buf ptr w/ roll-over
			kbd_tptr = 0;
		}
		rtn = kp_asc(j);								// get ASCII
	}
	return rtn;
}

//-----------------------------------------------------------------------------
// convert keycodes to ASCII
//	keycode = [xccc|caaa] c = column nybble (GND 1of4), a = row addr, x = don't
//	care (mask to 0).
//
// keypad LUT.  Keycode is constructed by converting 1of4 col code to 2 bit binary,
//	then left shifting 2 bits and combining the codes to create a continuous index
//	into the ASCII lookup table. NOTE: '#' is an invalid keypad code
//
// Return chr == '-' indicates hold time reached.
// Return chr == '^' indicates release.

#define	KBD_MAXCODE			22			// max # keys

char keychr_lut[] = { SUBchr, TONEchr, CALLchr, MWchr, TSchr, Tchr, Vupchr, Vdnchr, HILOchr, CHKchr, errorchr,
					  MSchr, DUPchr, VFOchr, MRchr, MHZchr, SETchr, Qupchr, Qdnchr, HILOchr, CHKchr, SMUTEchr };

//-----------------------------------------------------------------------------

char kp_asc(U16 keycode){
	U16		ii;			// temps
	U16		jj;
	U16		kk;
	U8		j;
	char	h = '\0';	// hold flag register
	char 	c = '\0';	// ascii temp, default to invalid char (null)

	if(keycode & KEY_RELEASE_KEY){
		h = KREL_FLAG;
		keycode &= ~(KEY_RELEASE_KEY | KEY_HOLD_KEY);
	}else{
		if(keycode & KEY_HOLD_KEY){
			h = KHOLD_FLAG;
			keycode &= ~KEY_HOLD_KEY;
		}
	}
	jj = keycode & (COL0_BIT);
	ii = (keycode & (U16)(KB_LOKEY)) | ((keycode & KB_HIKEY) >> 2);
/*	sprintf(dbbuf,"keyraw: %04x  %d,  %04x,  %02x,  %02x",ii,jj, S4_stat, GPIO_PORTB_DATA_R, dbug8);
	putsQ(dbbuf);*/

	kk = KB_SCNKEY;
	j = KB_SCNCNT;
	while(((ii & kk) != 0) && kk){
		kk >>= 1;
		j -= 1;
	}
	if(jj){								// if COL0 = 1, it is a COL1 key, so set 2nd row of keys
		j += KB_SCNCNT;
	}
	if(j <= KBD_MAXCODE){				// if valid, pull ascii from LUT
		c = keychr_lut[j-1] | h;		// combine with hold flag
	}
	return c;
}

//-----------------------------------------------------------------------------
// warm reset
//-----------------------------------------------------------------------------
//
// warm_reset() causes main() to re-start.
//

void warm_reset(void){
	swcmd = SW_ESC;				// trigger restart
}

//-----------------------------------------------------------------------------
// free_run
//-----------------------------------------------------------------------------
//
// free_run() returns value of free-running timer
//

U32 free_run(void){
	return free_32;				// return timer value
}

//-----------------------------------------------------------------------------
// sin_time
//-----------------------------------------------------------------------------
//
// sin_time() sets/reads the sin activity timer (0 reads)
//

U8 sin_time(U8 cmd){

	if(cmd){
		sintimer = cmd;
	}
	return sintimer;				// return timer value
}

//-----------------------------------------------------------------------------
// sout_time
//-----------------------------------------------------------------------------
//
// sout_time() sets/reads the sout pacing timer (0xff reads)
//

U8 sout_time(U8 cmd){

	if(cmd != 0xff){
		souttimer = cmd;
	}
	return souttimer;				// return timer value
}

//-----------------------------------------------------------------------------
// mhz_time
//-----------------------------------------------------------------------------
//
// mhz_time() sets/reads the mhz digit timer (1 sets, 0xff clears)
//

U8 mhz_time(U8 tf){

	if(tf == 0xff){
		mhztimer = 0;
	}else{
		if(tf){
			mhztimer = MHZ_TIME;
		}
	}
	if(mhztimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// set_time
//-----------------------------------------------------------------------------
//
// set_time() sets/reads the set-mode timer (1 sets, 0xff clears)
//

U8 set_time(U8 tf){

	if(tf == 0xff){
		settimer = 0;
	}else{
		if(tf){
			settimer = SET_TIME;
		}
	}
	if(settimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// v_time
//-----------------------------------------------------------------------------
//
// v_time() sets/reads the vol/squ timer (1 sets, 0xff clears)
//

U8 v_time(U8 tf){

	if(tf == 0xff){
		vtimer = 0;
	}else{
		if(tf){
			vtimer = VQ_TIME;
		}
	}
	if(vtimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// q_time
//-----------------------------------------------------------------------------
//
// q_time() sets/reads the vol/squ timer (1 sets, 0xff clears)
//

U8 q_time(U8 tf){

	if(tf == 0xff){
		qtimer = 0;
	}else{
		if(tf){
			qtimer = VQ_TIME;
		}
	}
	if(qtimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// ts_time
//-----------------------------------------------------------------------------
//
// ts_time() sets/reads the ts adj timer (1 sets, 0xff clears)
//

U8 ts_time(U8 tf){

	if(tf == 0xff){
		tstimer = 0;
	}else{
		if(tf){
			tstimer = TSW_TIME;
		}
	}
	if(tstimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// offs_time
//-----------------------------------------------------------------------------
//
// offs_time() sets/reads the offset digit timer (1 sets, 0xff clears)
//

U8 offs_time(U8 tf){

	if(tf == 0xff){
		offstimer = 0;
	}else{
		if(tf){
			offstimer = MHZ_TIME;
		}
	}
	if(offstimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// sub_time
//-----------------------------------------------------------------------------
//
// sub_time() sets/reads the sub-focus timer (1 sets, 0xff clears)
//

U8 sub_time(U8 tf){

	if(tf == 0xff){
		subtimer = 0;
	}else{
		if(tf){
			subtimer = SUB_TIME;
		}
	}
	if(subtimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// mic_time
//-----------------------------------------------------------------------------
//
// mic_time() sets/reads the mic button repeat timer (1 sets, 0xff clears)
//

U8 mic_time(U8 set){

	if(set == 0xff){
		mictimer = 0;
	}else{
		if(set == 1){
			mictimer = MIC_RPT_TIME;
		}
		if(set == 2){
			mictimer = MIC_RPT_WAIT;
		}
	}
	if(mictimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// micdb_time
//-----------------------------------------------------------------------------
//
// micdb_time() sets/reads the mic button debounce timer (1 sets, 0xff clears)
//

U8 micdb_time(U8 tf){

	if(tf == 0xff){
		micdbtimer = 0;
	}else{
		if(tf == 1){
			micdbtimer = MIC_DB_TIME;
		}
	}
	if(micdbtimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// mute_time
//-----------------------------------------------------------------------------
//
// mute_time() sets/reads the vol mute timer (1 sets, 0xff clears)
//

U8 mute_time(U8 tf){

	if(tf == 0xff){
		mutetimer = 0;
	}else{
		if(tf == 1){
			mutetimer = MUTE_TIME;
		}
	}
	if(mutetimer) return TRUE;
	return FALSE;
}

//-----------------------------------------------------------------------------
// set_dial, get_dial
//-----------------------------------------------------------------------------
//
// set_dial() sets value of main dial reg
//

void set_dial(S8 value){

	main_dial = value;
	return;
}

// get_dial() returns value of main dial reg
//

S8 get_dial(U8 tf){
	S8 i = main_dial;

	if(tf) main_dial = 0;
	return i;
}

//-----------------------------------------------------------------------------
// do_dial_beep, triggers a dial beep (or 2 or 3 q-beeps)
//-----------------------------------------------------------------------------
void do_dial_beep(void){

	d_beep;										// dial beep (short)
	return;
}

void do_1beep(void){

	q_beep;										// long beep
	num_beeps = 0;
	return;
}

void do_2beep(void){

	q_beep;										// 2x long beep
	num_beeps = 1;
	return;
}

void do_3beep(void){

	q_beep;										// 3x long beep
	num_beeps = 2;
	return;
}


//-----------------------------------------------------------------------------
// gpiod_isr
//-----------------------------------------------------------------------------
//
// GPIO_PORTC isr, processes the rotary encoder for MAIN
//		Simple up-dn counter pulses (not a dual-phase encoder).
//		!! need to flip-flop through a timer delay  ISR to debounce !!
//

void gpioc_isr(void){
	U8	maindial_in;

	maindial_in = GPIO_PORTC_MIS_R & PORTC_DIAL;			// grab dial interrupt flags
	GPIO_PORTC_ICR_R = PORTC_DIAL;							// clear int flags
	if(maindial_in){
		if(maindial_in & DIAL_UP){
			main_dial += 1;									// do up
			d_beep;											// dial beep
		}else{
			if(maindial_in & DIAL_DN){
				main_dial -= 1;								// do dn
				d_beep;										// dial beep
			}
		}
		// disable gpioc
		GPIO_PORTC_IM_R &= ~PORTC_DIAL;						// disable edge intr
		// set debounce timer
		dialtimer = DIAL_DEBOUNCE;							// set debounce
		// NOTE: debounce timer disables itself, clears gpioc flags, and enables gpioc
	}
	return;
}

//-----------------------------------------------------------------------------
// set_beep() sets the beep frequency
//-----------------------------------------------------------------------------
//
// beep_frq = frequency of pulsed beeper
//

void set_beep(U16 beep_frq, U16 b_count){

	TIMER0_TAILR_R = (uint16_t)(SYSCLK/((U32)beep_frq));				// set period
	TIMER0_TAMATCHR_R = (uint16_t)(SYSCLK/(2L * (U32)beep_frq));		// 50% DUTY CYCLE
	beep_count = b_count;												// beep_count is duration
	return;
}

//-----------------------------------------------------------------------------
// do_beep() sets the beep counter and starts Timer1A
//-----------------------------------------------------------------------------
//
// beep_cycles = # of 1KHz cycles to beep.  100 cycles = 0.1 sec
//

void do_beep(U16 beep_cycles){

	if(beep_cycles) beep_counter = beep_cycles;
	else beep_counter = beep_count;
	TIMER0_CTL_R |= (TIMER_CTL_TAEN);									// enable timer;
	return;
}

//-----------------------------------------------------------------------------
// Timer0A_ISR
//-----------------------------------------------------------------------------
//
// Called when timer0 A overflows (NORM mode):
//	used to count cycles of the beep output to establish the beep duration.
//	when beep_count == 0, ISR turns itself off.
//
//-----------------------------------------------------------------------------

void Timer0A_ISR(void){

	TIMER0_ICR_R = TIMER0_MIS_R;
	if(--beep_counter == 0){
		TIMER0_CTL_R &= ~(TIMER_CTL_TAEN);								// disable timer;
	}
	return;
}

//-----------------------------------------------------------------------------
// Timer3A_ISR
//-----------------------------------------------------------------------------
//
// Called when timer3 A overflows (NORM mode):
//	intended to update app timers @ 1ms per lsb
//	also drives RGB "I'm alive" LED.  The LED transitions through the color
//	wheel in a way that can be modified by the #defines below. RATE, PERIOD,
//	and START values may be adjusted to taylor the color transitions,
//	transition rate, and cycle period.
//
//-----------------------------------------------------------------------------

void Timer3A_ISR(void){
//static	U16	prescale;				// prescales the ms rate to the LED update rate
//		U16	t2_temp;				// temp
//		U32	t2_temp32;
//static	U8	keydb_tmr;
		U16	key_temp;				// keypad temp
		U16	jj;
static	U8	kpscl;					// keypad prescale
static	U8	kp_state;				// keypad state machine reg
static	U16	kp_keypat;				// key-down pattern
static	U16	kphold_timer;			// keypad hold timer
static	U8	keybd_timer;			// keypad debounce timer
static	U16	beepdly_timer;			// double beep delay timer
static	U16	key_last;				// last key

#define	PWM_RATE_RED	4			// delta duty cycle values (this is added/subtracted to/fr the DCreg every 10ms)
#define	LED_PERIOD		10000		// sets length of LED cycle (ms)

//	GPIO_PORTB_DATA_R |= LOCK;		// toggle debug pin -- 2.25 us ISR exec time at 1.0003ms rate
	TIMER3_ICR_R = TIMER3_MIS_R;
	if(iplt2){										// if flag is set, perform ipl initialization
		iplt2 = 0;
		free_32 = 0;
		waittimer = 0;				// gp wait timer
		dialtimer = 0;				// dial debounce wait timer
		sintimer = 0;				// sin activity timer
		souttimer = 0;				// sout pacing timer
		mhztimer = 0;				// mhz digit access timer
		qtimer = 0;					// squ access timer
		vtimer = 0;					// vol access timer
		offstimer = 0;				// offs access timer
		settimer = 0;				// set access timer
		subtimer = 0;				// sub focus timer
		kp_state = KEYP_IDLE;
		kphold_timer = 0;
		kbdn_flag = 0;
		kbup_flag = 0;
		kpscl = KEY_PRESCALE;
		beepdly_timer = 0;
		beepgaptimer = BEEP_GAP;
		num_beeps = 0;
		mictimer = 0;
//		prescale = 0;				// init resp led regs
	}
	// state machine to process local keypad
	// kbdn_flag == true if key pressed (app needs to clear), also has "hold" flag that indicates
	//	that the key has been pressed for at least the KEY_HOLD time (app needs to clear)
	// kbup_flag == true if key is released (app needs to clear)
	// Row address to switch matrix is only advanced when there is no keypress (so, this alg. doesn't do
	// key rollover).

	if(--kpscl == 0){
		kpscl = KEY_PRESCALE;
		// read keypad row bits
//		dbug8 = GPIO_PORTB_DATA_R;
		key_temp = ((U16)(GPIO_PORTB_DATA_R & KB_NOKEYB) << 6) | ((U16)(GPIO_PORTA_DATA_R & KB_NOKEYA) << 2) | ((U16)(GPIO_PORTE_DATA_R & KB_NOKEYE) >> 2);
		if(!(key_temp & COL0_BIT)){						// de-mux spare_S4 switch
			S4_stat = key_temp & S4_BIT;				// strip out the status of the S4 switch
			key_temp |= S4_BIT;							// set to "no key" state
		}
		jj = key_temp & ~(COL1_BIT|COL0_BIT);			// mask off col bits
		switch(kp_state){
		default:
			kp_state = KEYP_IDLE;						// re-cage state
		case KEYP_IDLE:
			if(jj == KB_NOKEY){
				// advance key addr to next row (flip-flops COL0 and COL1)
				if(GPIO_PORTE_DATA_R & COL0){
					GPIO_PORTE_DATA_R |= (COL0|COL1);
					GPIO_PORTE_DATA_R = (GPIO_PORTE_DATA_R & ~COL0) | COL1;
				}else{
					GPIO_PORTE_DATA_R |= (COL0|COL1);
					GPIO_PORTE_DATA_R = (GPIO_PORTE_DATA_R & ~COL1) | COL0;
				}
			}else{
				keybd_timer = KP_DEBOUNCE_DN;			// set debounce timer (dn)
				kp_state = KEYP_DBDN;					// advance state
				kp_keypat = jj;							// save pattern
			}
			break;

		case KEYP_DBDN:
			if(jj != kp_keypat){
				kp_state = KEYP_IDLE;					// false key, return to idle
			}else{
				if(--keybd_timer == 0){
					kbdn_flag = KEY_PR_FL;				// set key pressed flag
					kbd_buff[kbd_hptr++] = key_temp;	// store key code in buff
					key_last = key_temp;
					if(kbd_hptr == KBD_BUFF_END){		// update buf ptr w/ roll-over
						kbd_hptr = 0;
					}
					if(kbd_hptr == kbd_tptr){			// flag buffer error
						kbd_stat |= KBD_ERR;
					}
					kphold_timer = KEY_HOLD_TIME;		// set hold timer (~~2 sec)
					kp_state = KEYP_PRESSED;			// advance state
					q_beep;
				}
			}
			break;

		case KEYP_PRESSED:
			if(jj == KB_NOKEY){
				keybd_timer = KP_DEBOUNCE_UP;			// set debounce timer (up)
				kp_state = KEYP_DBUP;					// up debounce state
			}else{
				if(kphold_timer == 0){
					kbdn_flag |= KEY_HOLD_FL;			// set key-hold flag
					kp_state = KEYP_HOLD;				// hold state
					kbd_buff[kbd_hptr++] = key_last | KEY_HOLD_KEY;	// store key code with HOLD flag in buff
					if(kbd_hptr == KBD_BUFF_END){		// update buf ptr w/ roll-over
						kbd_hptr = 0;
					}
					beepdly_timer = BEEP2_COUNT;
					q_beep;
				}else{
					kphold_timer -= 1;					// update hold timer
				}
			}
			break;

		case KEYP_HOLD:
			if(jj == KB_NOKEY){
				keybd_timer = KP_DEBOUNCE_UP;			// set debounce timer (up)
				kp_state = KEYP_DBUP;					// up debounce state
			}
			break;

		case KEYP_DBUP:
			if(jj != KB_NOKEY){
				kp_state = KEYP_HOLD;					// false key up, return to HOLD
			}else{
				if(--keybd_timer == 0){
					kbup_flag = 1;						// set key up flag
					kp_state = KEYP_IDLE;				// advance state
					kbd_buff[kbd_hptr++] = key_last | KEY_RELEASE_KEY;	// store key release code in buff
					if(kbd_hptr == KBD_BUFF_END){		// update buf ptr w/ roll-over
						kbd_hptr = 0;
					}
				}
			}
			break;
		}
	}
	// process app timers
	free_32++;											// update large free-running timer
	if (beepdly_timer != 0){							// update wait timer
		if(--beepdly_timer == 0){
			q_beep;										// do beep# 2
		}
	}
	if (waittimer != 0){								// update wait timer
		waittimer--;
	}
	if (sintimer != 0){									// update sin activity timer
		sintimer--;
	}
	if (souttimer != 0){								// update sin activity timer
		souttimer--;
	}
	if (mhztimer != 0){									// update mhz digit timer
		mhztimer--;
	}
	if (vtimer != 0){									// update vol/squ adjust timer
		vtimer--;
	}
	if (qtimer != 0){									// update vol/squ adjust timer
		qtimer--;
	}
	if (settimer != 0){									// update set mode timer
		settimer--;
	}
	if (offstimer != 0){								// update offs adjust timer
		offstimer--;
	}
	if (subtimer != 0){									// update sub adjust timer
		subtimer--;
	}
	if (mictimer != 0){									// update mic button repeat timer
		mictimer--;
	}
	if (micdbtimer != 0){								// update mic button debounce timer
		micdbtimer--;
	}
	if (mutetimer != 0){								// volume mute timer
		mutetimer--;
	}
	if (tstimer != 0){									// update set mode timer
		tstimer--;
	}
	if(num_beeps){
		if (beepgaptimer != 0){							// update beep gap timer
			--beepgaptimer;
		}else{
			q_beep;										// dial beep
			num_beeps--;
			beepgaptimer = BEEP_GAP;
		}
	}
	if (dialtimer != 0){								// update wait timer
		if(!(--dialtimer)){
			GPIO_PORTC_ICR_R = PORTC_DIAL;				// clear int flags
			GPIO_PORTC_IM_R |= PORTC_DIAL;				// enable edge intr
		}
	}
//	GPIO_PORTB_DATA_R &= ~LOCK;			// toggle debug pin
	return;
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

