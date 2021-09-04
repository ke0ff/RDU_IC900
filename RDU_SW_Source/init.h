/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: init.h
 *
 *  Module:    Control, IC-900 RDU Controller Clone
 *
 *  Summary:   defines and global declarations for main.c
 *  			Also defines GPIO pins, timer constants, and other
 *  			GPIO system constants
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>

#ifndef INIT_H
#define INIT_H
#endif

//#define	USE_QSPI 1

//////////////////////////////////////////////////////////////////////////////////////
//#define	USE_QSPI 1																	//
//#define	LA_ENABLE					// define if logic analyzer is enabled (debug)	//
//////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define SYSCLKL 10000L
#define PIOCLK	(16000000L)			// internal osc freq (in Hz)
#define EXTXTAL           			// un-comment if external xtal is used
									// define EXTXTAL #def to select ext crystal
									// do not define for int osc
#ifdef EXTXTAL
#define XTAL 1
//#define XTAL_FREQ	SYSCTL_RCC_XTAL_16MHZ	// set value of external XTAL
#define XTAL_FREQ	SYSCTL_RCC_XTAL_20MHZ	// set value of external XTAL
#define SYSCLK	(50000000L)			// sysclk freq (bus clk)
#else
#define XTAL 0
#define SYSCLK	PIOCLK				// sysclk freq (bus clk) same as PIOCLK
#endif

#define OSC_LF 4            		// osc clock selects
#define OSC_HF 0
#define OSC_EXT 1

#define SEC10MS    10           	// timer constants (ms)
#define SEC33MS    33
#define SEC50MS    50
#define SEC75MS    74
#define SEC100MS  100
#define SEC250MS  250
#define SEC500MS  500
#define SEC750MS  750
#define SEC1     1000
#define ONESEC   SEC1
#define MS1500   1500
#define SEC2     2000
#define SEC3     3000
#define SEC5     5000
#define SEC10   10000
#define SEC15   15000
#define SEC30   30000
#define SEC60   60000
#define SEC300 300000L
#define	ONEMIN	(SEC60)
#define	REG_WAIT_DLY 200			// 200ms wait limit for register action
#define RESP_SAMP SEC100MS			// sets resp rate
#define	SOUT_PACE_TIME	8			// 33/4800 s for one SOUT msg

// ssi bit rate defns
// BR = SYSCLK/(CPSR * (1 + SCR))
// SCR = (SYSCLK/(BR * CPSR)) - 1
// see SSICLK_calc.xls for minimum error values for CPSDVSR and SCR
// 4800 baud for SSI1
#define	SIO_BAUD		4800L				// IC900 serial I/O baud rate
#define	SSI1_BR			SIO_BAUD			// ssi1 clock rate (Hz)
#define SSI1_CPSDVSR	(248)
#define	SSI1_SCR		(41)				//((SYSCLK/(SSI1_BR * SSI1_CPSDVSR)) - 1)
// 100K baud for SSI3
#define	LCD_BAUD		25000L				// LCD serial I/O baud rate
#define	SSI3_BR			LCD_BAUD			// ssi3 clock rate (Hz)
#define SSI3_CPSDVSR	(200)
#define	SSI3_SCR		(2)					//((SYSCLK/(SSI3_BR * SSI3_CPSDVSR)) - 1)

// SIN bit time defines... assumes PS = 0 (SYSCLK/1 is used for timer)
#define TIMER2A_PS 0
#define	SIN_BIT_TIME		(SYSCLK/(SIO_BAUD * (TIMER2A_PS + 1)))		// 10416
#define	SIN_HALF_BIT_TIME	(SIN_BIT_TIME/2)							// 5208

// timer definitions
#define TIMER1_PS 31				// prescale value for timer1
#define	TIMER1_FREQ	9600			// timer1 intr freq
//#define TIMER3_ILR 0xffff			// timer 3 interval (24 bit)
//#define TIMER3_PS 0xff
//#define TIMER1_PS 32
#define TIMER3_ILR 0xffff			// timer 3 interval (24 bit)
#define TIMER3_PS 32

#define	TPULSE	(100L)				// in usec
#define TMIN	(((SYSCLK / 100) * TPULSE)/10000L)	// minimum pulse width

// Logic analyzer defines:
// port B
#define LA_RST_N		0x80		// in		ssi1clk, LCD coms
// port D
#define LA_SCK_N		0x01		// in		ssi1clk, LCD coms
#define LA_MOSI			0x02		// in		data
#define LA_CS1_N		0x04		// in		LCD IC1 cs
#define LA_CS2_N		0x08		// in		LCD IC2 cs
// port E
#define LA_CM_DA		0x01		// in		LCD cmd/data
#define LA_BUSY_N		0x02		// in		LCD busy

// Port A defines
#define TXD0_BT			0x01		// out		uart0
#define RXD0_BT			0x02		// in		uart0
#define CALVFO			0x04		// in		pbsw inputs..
#define MWMR			0x08		// in
#define TSET			0x10		// in
#define TSMHZ			0x20		// in
#define UPVS			0x40		// in
#define DNVS			0x80		// in
#define PORTA_DIRV		(TXD0_BT)
#define	PORTA_DENV		(TXD0_BT|RXD0_BT|CALVFO|MWMR|TSET|TSMHZ|UPVS|DNVS)
#define	PORTA_PURV		(CALVFO|MWMR|TSET|TSMHZ|UPVS|DNVS)
#define	PORTA_INIT		0

// Port B defines
#define RXD1			0x01		// in		uart1
#define TXD1			0x02		// out		uart1
#define LOCK			0x04		// in		(dedicated input) lock slide sw
#define DIM				0x08		// in		(dedicated input) dim slide sw
#define HILO			0x10		// in		(dedicated input) hi/lo pbsw
#define CHECK			0x20		// in		(dedicated input) check pbsw
#define SMUTE			0x40		// in		(dedicated input) smute pbsw
#define LCDRST			0x80		// out		lcd controller reset
#define PORTB_DIRV		(TXD1|LCDRST)
#define	PORTB_DENV		(RXD1|TXD1|LOCK|DIM|HILO|CHECK|SMUTE|LCDRST)
#define	PORTB_PURV		(DIM|LOCK|HILO|CHECK|SMUTE)
#define	PORTB_INIT		(LCDRST)
#define	BBUTTON_MASK	(HILO|CHECK|SMUTE)

// Port C defines
#define MRX_N			0x10		// out		main rx squ open
#define DIAL_UP			0x20		// in		main dial up pulses (future qei1A)
#define DIAL_DN			0x40		// in		main dial dn pulses (future qei1B)
#define SRX_N			0x80		// out		sub rx squ open
#define PORTC_DIRV		(MRX_N|SRX_N)
#define	PORTC_DENV		(MRX_N|DIAL_UP|DIAL_DN|SRX_N)
#define	PORTC_PURV		(DIAL_UP|DIAL_DN)
#define	PORTC_DIAL		(DIAL_UP|DIAL_DN)
#define	PORTC_INIT		(0)
#define	DIAL_DEBOUNCE	50			// 25 ms dial debounce

// Port D defines
#define SCK				0x01		// out		ssi3clk, LCD coms
#define CS1				0x02		// out		LCD IC1 cs
#define sparePD2		0x04		// IN/AIN	spare (AIN5)
#define MOSI_N			0x08		// out		ss13Tx, data out
#define sparePD4		0x10		// out
#define sparePD5		0x20		// out
#define CS2				0x40		// out		LCD IC2 cs		qei0A
#define MTX_N			0x80		// out						qei0B
#define PORTD_DIRV		(SCK|CS1|CS2|MOSI_N|sparePD5|MTX_N)
#define	PORTD_DENV		(SCK|CS1|CS2|MOSI_N|sparePD4|sparePD5|MTX_N)
#define	PORTD_PURV		(sparePD4)
#define	PORTD_INIT		(MOSI_N|sparePD5)

#define	LCD_IC1			0x01		// LCD CS identifiers
#define	LCD_IC2			0x02

// Port E defines
#define DA_CM			0x01		// out
#define BUSY_N			0x02		// in
#define SUBMS			0x04		// in
#define	DUPTONE			0x08		// in
#define COL0			0x10		// out		key pb sw col0 out
#define COL1			0x20		// out		key pb sw col1 out
#define PORTE_DIRV		(DA_CM|COL0|COL1)
#define	PORTE_DENV		(DA_CM|BUSY_N|SUBMS|DUPTONE|COL0|COL1)
#define	PORTE_PURV		(BUSY_N|SUBMS|DUPTONE)
#define	PORTE_INIT		0

// keypad defines
#define	KB_COL_M		(COL1|COL0)	// mask for kb col 1of2
#define	KB_NOKEYA		(0xFC)							// key-mask semaphore (port A)
#define	KB_NOKEYB		(HILO|CHECK|SMUTE)				// key-mask semaphore (port B)
#define	KB_NOKEYE		(COL1|COL0|SUBMS|DUPTONE)		// key-mask semaphore (port E)
#define	KB_NOKEY		(0x1FF3)						// no keypressed semaphore (composite)
#define	KB_LOKEY		(0x003)							// lower key bits (composite)
#define	KB_HIKEY		(0x1FF0)						// upper key bits (composite)
#define	KB_SCNKEY		(0x400)							// scan start value
#define	KB_SCNCNT		(11)							// scan count
#define	COL0_BIT		(0x04)							// composite key-bit locations
#define	COL1_BIT		(0x08)
#define	S4_BIT			(0x1000)
#define	S4_FLAG			(0x02)

#define	KP_DEBOUNCE_DN	3			// 10 ms of keypad debounce
#define	KP_DEBOUNCE_UP	5			// 25 ms of keypad debounce

// Port F defines
// IC-900 SM defines the beep frequency as 1/525us = 1905 Hz.  However, tests indicate that 3375 Hz
//	has a greater resonance and thus sounds much louder.
#define	BEEP_DURATION	75L								// #ms for beep
#define	BEEP_SHORT		20L								// #ms for beep
#define	BEEP_FREQ		3375L							// 3.3KHz beep frequency
#define	BEEP_COUNT		(BEEP_FREQ * BEEP_DURATION / 1000L)		// 0.075s beep pulse
#define	BEEP2_COUNT		(110)							// ~~2x beep pulse delay
#define	DIAL_BEEP_COUNT	(BEEP_FREQ * BEEP_SHORT / 1000L)
#define	BEEP_GAP		(BEEP_DURATION * 2)
#define BEEP			0x01		// out		T0CCP0	{beeper}
#define SOUT_TTL		0x02		// out		SSI1TX,	remote serial out (4800 baud, 32b)		(M1PWM5)
#define LED_PWM			0x04		// out		M1PWM6	PBSW LED PWM
#define BL_PWM			0x08		// out		M1PWM7	DU backlight PWM
#define SIN_TTL			0x10		// in		T2CCP0	remote serial in
#define PORTF_DIRV		(BEEP|BL_PWM|LED_PWM|SOUT_TTL)
#define	PORTF_DENV		(BEEP|BL_PWM|LED_PWM|SOUT_TTL|SIN_TTL)
#define	PORTF_PURV		(SOUT_TTL|SIN_TTL)
#define	PORTF_INIT		0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

#ifndef MAIN_C
extern U16	app_timer1ms;		// app timer
extern U16	xm_timer;			// xmodem timer
extern U16	ccmd_timer;			// ccmd timer
extern char	bchar;				// global bchar storage
extern char	swcmd;				// global swcmd storage
extern S8	handshake;			// xon/xoff enable
extern S8	xoffsent;			// xoff sent
#endif

// system error flag defines

#define	NOP				FALSE			// no-operation flag.  Passed to functions that have the option to perform an action or return a status
#define	CLR				TRUE			// perform read-and-clear operation flag.  Passed to functions that have the option to perform read status and clear
#define	KEY_NULL		'~'				// null chr for HMD keycode LUT
#define KEYP_IDLE		0x00			// keypad ISR state machine state IDs
#define KEYP_DBDN		0x01
#define KEYP_PRESSED	0x02
#define KEYP_HOLD		0x03
#define KEYP_DBUP		0x04
#define	KEY_PR_FL		0x01			// key-pressed bit field
#define	KEY_HOLD_FL		0x02			// key-hold bit field
#define	KEY_PRESCALE	10				// sets COL hold time (in ms +1)
#define KEY_HOLD_TIME	(SEC1/KEY_PRESCALE) // keypad hold timer value (~~ 1 sec)
#define	KEY_HOLD_KEY	0x8000			// set hi bit of key buffer entry to signal hold
#define	KEY_RELEASE_KEY	0x4000			// key release keycode
#define	KHOLD_FLAG		0x80			// flag bit for key hold character
#define	KREL_FLAG		0x40			// flag bit for key release character
#define	KEY_RELEASE_CHR	0x3e			// key release character
// key press character defines
#define	SUBchr			33
#define	TONEchr			34
#define	CALLchr			35
#define	MWchr			36
#define	TSchr			37
#define	Tchr			38
#define	Vupchr			39
#define	Vdnchr			40
#define	HILOchr			41
#define	CHKchr			42
#define	errorchr		43

#define	MSchr			44
#define	DUPchr			45
#define	VFOchr			46
#define	MRchr			47
#define	MHZchr			48
#define	SETchr			49
#define	Qupchr			50
#define	Qdnchr			51
#define	SMUTEchr		52
// key hold chr codes
#define	SUBchr_H		(33 | KHOLD_FLAG)
#define	TONEchr_H		(34 | KHOLD_FLAG)
#define	CALLchr_H		(35 | KHOLD_FLAG)
#define	MWchr_H			(36 | KHOLD_FLAG)
#define	TSchr_H			(37 | KHOLD_FLAG)
#define	Tchr_H			(38 | KHOLD_FLAG)
#define	Vupchr_H		(39 | KHOLD_FLAG)
#define	Vdnchr_H		(40 | KHOLD_FLAG)
#define	HILOchr_H		(41 | KHOLD_FLAG)
#define	CHKchr_H		(42 | KHOLD_FLAG)
#define	errorchr_H		(43 | KHOLD_FLAG)

#define	MSchr_H			(44 | KHOLD_FLAG)
#define	DUPchr_H		(45 | KHOLD_FLAG)
#define	VFOchr_H		(46 | KHOLD_FLAG)
#define	MRchr_H			(47 | KHOLD_FLAG)
#define	MHZchr_H		(48 | KHOLD_FLAG)
#define	SETchr_H		(49 | KHOLD_FLAG)
#define	Qupchr_H		(50 | KHOLD_FLAG)
#define	Qdnchr_H		(51 | KHOLD_FLAG)
#define	SMUTEchr_H		(52 | KHOLD_FLAG)
// key release chr codes
#define	SUBchr_R		(33 | KREL_FLAG)
#define	TONEchr_R		(34 | KREL_FLAG)
#define	CALLchr_R		(35 | KREL_FLAG)
#define	MWchr_R			(36 | KREL_FLAG)
#define	TSchr_R			(37 | KREL_FLAG)
#define	Tchr_R			(38 | KREL_FLAG)
#define	Vupchr_R		(39 | KREL_FLAG)
#define	Vdnchr_R		(40 | KREL_FLAG)
#define	HILOchr_R		(41 | KREL_FLAG)
#define	CHKchr_R		(42 | KREL_FLAG)
#define	errorchr_R		(43 | KREL_FLAG)

#define	MSchr_R			(44 | KREL_FLAG)
#define	DUPchr_R		(45 | KREL_FLAG)
#define	VFOchr_R		(46 | KREL_FLAG)
#define	MRchr_R			(47 | KREL_FLAG)
#define	MHZchr_R		(48 | KREL_FLAG)
#define	SETchr_R		(49 | KREL_FLAG)
#define	Qupchr_R		(50 | KREL_FLAG)
#define	Qdnchr_R		(51 | KREL_FLAG)
#define	SMUTEchr_R		(52 | KREL_FLAG)

#define	HIB_APPL	0					// HIB is being accessed by application
#define	HIB_INTR	1					// HIB may be accessed by intrpt

//-----------------------------------------------------------------------------
// main.c Fn prototypes
//-----------------------------------------------------------------------------

void Init_Device(void);
char process_IO(U8 flag);
void  process_chg(U8 chg, char* buf, U8* key_mem, U8* keyh_mem);
U8 get_status(U8* keyh_mem);
U8 set_pttmode(U8 value);
void set_pwm(U8 pwmnum, U8 percent);
void set_led(U8 lednum, U8 value);

void wrwhib_ram(U32 data, volatile U8 addr);
void wrhib_ram(uint8_t data, U8 addr);
uint32_t rdwhib_ram(U8 addr);
uint8_t rdhib_ram(U8 addr);

void waitpio(U16 waitms);
void wait(U16 waitms);
void wait2(U16 waitms);
U16 getipl(void);
U32 get_syserr(U8 opr);
U8 wait_busy0(U16 delay);
U8 wait_busy1(U16 delay);
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay);
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay);

U8 got_key(void);
//U8 not_key(U8 flag);
char get_key(void);

void warm_reset(void);
U32 free_run(void);
void set_dial(S8 value);
S8 get_dial(U8 tf);
void do_dial_beep(void);
void do_1beep(void);
void do_2beep(void);
void do_3beep(void);
U8 sin_time(U8 cmd);
U8 sout_time(U8 cmd);
U8 mhz_time(U8 tf);
U8 v_time(U8 tf);
U8 q_time(U8 tf);
U8 set_time(U8 tf);
U8 offs_time(U8 tf);
U8 sub_time(U8 tf);
U8 mic_time(U8 tf);
U8 micdb_time(U8 tf);
U8 mute_time(U8 tf);
U8 ts_time(U8 tf);

void set_beep(U16 beep_frq, U16 b_count);
void do_beep(U16 beep_cycles);
void Timer0A_ISR(void);

void gpiob_isr(void);
void gpioc_isr(void);
void Timer3A_ISR(void);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
