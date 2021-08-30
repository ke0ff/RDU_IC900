/********************************************************************
 ************ COPYRIGHT (c) 2021 by KE0FF, Taylor, TX   *************
 *
 *  File name: radio.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for radio.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>
#include "sio.h"


//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
// Processor I/O assignments
// ...see init.h

#define	MAIN	1
#define	SUB		0

#define	LEN_VFO_ARY		(ID1200)
#define	LEN_ARY			(ID1200)
#define	LEN_HIB			(56/4)	//(6*(LEN_VFO_ARY + 5))
#define	XIT_HIB_ADR		3
#define	RIT_HIB_ADR		7

// CTCSS flags
#define	CTCSS_MASK		0x3F				// tone code mask
#define	TONE_MAX		38					// max # PL tones
#define	CTCSS_OFF		0x80				// CTCSS off flag

// duplex/RFpwr flag bits
#define	DPLX_S			0x00				// simplex
#define	DPLX_P			0x01				// plus
#define	DPLX_M			0x02				// minus
#define	DPLX_MASK		(DPLX_P | DPLX_M)	// field mask
#define	LOHI_F			0x04				// low power if == 1
#define	TSA_F			0x08				// use TSA if == 1
#define	MEMODE_F		0x10				// memory mode active if == 1

#define	XIT_REG			0x0f				// xit register (0x08 is dir, 0x07 is count)

#define	SIN_ACTIVITY	200		// SIN activity timeout

// SIN activity flag bits
#define SIN_SQS_F		((SIN_SQSA|SIN_SQSB) >> 16)	// COS
#define SIN_MSRF_F		(SIN_SRFA >> 16)			// SRF main
#define SIN_SSRF_F		(SIN_SRFB >> 16)			// SRF sub
#define SIN_SEND_F		SIN_SEND					// PTT
#define SIN_DSQ_F		(SIN_DSQA|SIN_DSQB)			// tone detected
#define SIN_MCK_F		(SIN_MCK|SIN_MUP)			// MIC u/d button change
#define SIN_SEL_F		(SIN_SEL11|SIN_SEL12|SIN_SEL21|SIN_SEL22)	// OPT
#define	SIN_VFOM_F		0x00010000L					// vfo change flag
#define	SIN_VFOS_F		0x00020000L					// vfo change flag
#define SIN_SINACTO_F	0x00040000L					// SIN timeout has occurred

// SOUT signal flag bits
#define SOUT_TONE_F		0x01	// tone update
#define SOUT_TONE_N		0x00	// tone ordinal
#define SOUT_MSQU_F		0x02	// main squ update SIN_VFOM_F
#define SOUT_MSQU_N		0x01	// msqu ordinal
#define SOUT_SSQU_F		0x04	// sub squ update
#define SOUT_SSQU_N		0x02	// ssqu ordinal
#define SOUT_MVOL_F		0x08	// main vol update
#define SOUT_MVOL_N		0x03	// mvol ordinal
#define SOUT_SVOL_F		0x10	// sub vol update
#define SOUT_SVOL_N		0x04	// svol ordinal
#define SOUT_VUPD_F		0x20	// force vfo push update
#define SOUT_VUPD_N		0x05	// force vfo push ordinal
#define SOUT_VFOS_F		0x40	// sub vfo update
#define SOUT_VFOS_N		0x06	// svfo ordinal
#define SOUT_VFOM_F		0x80	// mvfo vfo update
#define SOUT_VFOM_N		0x07	// tone ordinal

#define	XPOLY	0x1021			// crc polynomial
//#define	HIB_SEL	0
//#define	VFO_SEL	1
#define	CRC_HIB_ADDR	62		// CRC16 goes at top 2 bytes of HIB RAM

#define	MHZ_TIME	SEC10		// MHZ digit mode timeout
#define	VQ_TIME		SEC3		// vol/squ adj timeout
#define	SET_TIME	(3*SEC10)	// set mode timeout
#define	SUB_TIME	(3*SEC10)	// sub-focus timeout

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

U32 init_radio(void);
void process_SIN(U8 cmd);
void process_SOUT(U8 cmd);
void  save_vfo(void);
void  recall_vfo(void);
//U16 crc_vfo(void);
U16 crc_hib(void);
U16 calcrc(U8 c, U16 oldcrc);
void push_vfo(void);
U32 fetch_sin(U8 addr);
U32 read_sin_flags(U32 flag);
void vfo_change(U8 band);
U32* setpll(U8 bid, U32 *plldata, U8 is_tx, U8 is_main);
U8  get_present(void);
//U32 set_squ(U8 mainsub);
//U32 set_vol(U8 mainsub);
U32 atten_calc(U8 aval);
U8 adjust_squ(U8 mainsub, S8 value);
U8 adjust_vol(U8 mainsub, S8 value);
U8 adjust_tone(U8 mainsub, S8 value);
U8 adjust_toneon(U8 mainsub, U8 value);
U8 inc_dplx(U8 main);
U8 read_dplx(U8 main);
S8 add_vfo(U8 main, S8 adder, U8 daddr);
U8 get_band_index(U8 main);
U8 set_next_band(U8 focus);
U8 set_swap_band(void);
U8 read_tsab(U8 main, U8 absel);
void set_tsab(U8 main, U8 absel, U8 value);
void set_ab(U8 main, U8 tf);
S32 set_mhz_step(S32 sval);
S8 is_mic_updn(void);
U32 get_freq(U8 main);
void copy_vfot(U8 main);
void temp_vfo(U8 main);
U32 get_vfot(void);
void  force_push_radio(void);
U8 get_lohi(U8 bid, U8 setread);
U8 get_memnum(U8 main);
U8 get_bit(U8 value);
U8 set_bit(U8 value);
void  update_radio_all(void);
void set_offs(U8 focus, U16 value);
U16 get_offs(U8 focus);
U8 inv_duplex(U8 focus);
U8 inv_vfo(U8 focus);
