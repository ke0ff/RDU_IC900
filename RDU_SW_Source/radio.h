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

// VFO struct
struct vfo_struct {
	U32	vfo;						// main vfo frequency (RX) in KHz
	U16	offs;						// TX offset in KHz
	U8	dplx;						// duplex/RFpwr/XIT/MEM - packed flags for each resource
	U8	ctcss;						// PL setting and "OFF" control bit
	U8	sq;							// SQ setting
	U8	vol;						// VOL setting
	U8	bflags;						// expansion flags
	U8	scanflags;					// scan flags (expansion)
	U8	tsa;						// frq step "A" setting
	U8	tsb;						// frq step "B" setting
};

// NVRAM memory map
#define	NVRAM_BASE	(0L)
// Each VFO is stored to NVRAM as a "union" with individual elements of disparate arrays
// grouped together to reduce SPI transfer overhead.
#define	VFO_0		(NVRAM_BASE)				// main vfo freq
#define	OFFS_0		(VFO_0 + (sizeof(U32)))		// TX offset freq
#define	DPLX_0		(OFFS_0 + (sizeof(U16)))	// duplex/RFpwr/XIT/MEM
#define	CTCSS_0		(DPLX_0 + sizeof(U8))		// PL setting
#define	SQ_0		(CTCSS_0 + sizeof(U8))		// SQ
#define	VOL_0		(SQ_0 + sizeof(U8))			// VOL
#define	MEM_0		(VOL_0 + sizeof(U8))		// mem#
#define	CALL_0		(MEM_0 + sizeof(U8))		// call-mem#
#define	BFLAGS_0	(CALL_0 + sizeof(U8))		// expansion flags
#define	SCANFLAGS_0	(BFLAGS_0 + sizeof(U8))		// scan (expansion) flags
#define	TSA_0		(SCANFLAGS_0 + sizeof(U8))	// frq step "A"
#define	TSB_0		(TSA_0 + sizeof(U8))		// frq step "B"

#define	VFO_LEN		((TSB_0 + sizeof(U8)) - VFO_0)
#define	XIT_0		((VFO_LEN * NUM_VFOS) + VFO_0) // xit reg
#define	RIT_0		(XIT_0 + sizeof(U8))		// rit reg
#define	BIDM_0		(RIT_0 + sizeof(U8))		// bandidm reg
#define	BIDS_0		(BIDM_0 + sizeof(U8))		// bandids reg
#define	VFO_END		(BIDS_0 + sizeof(U8))		// start of next segment

#define	XMODET_0	VFO_END						// xmode flags

#define	TXULIM_0	(XMODET_0 + ((sizeof(U8) * ID1200)))	// TX upper limits (per band)
#define	TXLLIM_0	(TXULIM_0 + ((sizeof(U32) * ID1200)))	// TX lower limits (per band)
#define	LIM_END		(TXLLIM_0 + ((sizeof(U32) * ID1200)))	// start of next segment

#define	MEM_NAME_LEN	16
#define	MEM0_BASE	(LIM_END)
					// mem structure follows this format:
					// VFO + OFFS + DPLX + CTCSS + SQ + VOL + XIT + RIT + BID + MEM_NAME_LEN
#define	MEM_LEN		(sizeof(U32) + sizeof(U16) + (sizeof(U8) * 7) + MEM_NAME_LEN)
#define	MEM_STR_ADDR	(sizeof(U32) + sizeof(U16) + (sizeof(U8) * 7))
#define	NUM_MEMS	34							// 30 mems, + 4 call mems
#define	MAX_MEM		30
#define	CALL_MEM	30

#define	ID10M_MEM	MEM0_BASE
#define	ID6M_MEM	(ID10M_MEM + (NUM_MEMS * MEM_LEN))
#define	ID2M_MEM	(ID6M_MEM + (NUM_MEMS * MEM_LEN))
#define	ID220_MEM	(ID2M_MEM + (NUM_MEMS * MEM_LEN))
#define	ID440_MEM	(ID220_MEM + (NUM_MEMS * MEM_LEN))
#define	ID1200_MEM	(ID440_MEM + (NUM_MEMS * MEM_LEN))
#define	MEM_END		(ID1200_MEM + (NUM_MEMS * MEM_LEN))

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
#define	SCANEN_F		0x10				// mem scan enable flag ("S"kip = on if zero)

#define	XIT_REG			0x0f				// xit register (0x08 is dir, 0x07 is count)

#define	SIN_ACTIVITY	200		// SIN activity timeout

// update_radio_all ordinal defines
#define	UPDATE_ALL		1
#define	MAIN_ALL		2
#define	SUB_ALL			3
#define	MAIN_FREQ		4
#define	SUB_FREQ		5
#define	MAIN_VQ			6
#define	SUB_VQ			7

// PTT mem defines
#define	NO_PTT			0
#define	PTT_KEYED		1
#define	PTT_EDGE		0x80

// SIN activity flag bits
#define SIN_SQSM_F		((SIN_SQSA) >> 16)			// COS main
#define SIN_SQSS_F		((SIN_SQSB) >> 16)			// COS sub
#define SIN_MSRF_F		(SIN_SRFA >> 16)			// SRF main
#define SIN_SSRF_F		(SIN_SRFB >> 16)			// SRF sub
#define SIN_SEND_F		SIN_SEND					// PTT
#define SIN_DSQ_F		(SIN_DSQA|SIN_DSQB)			// tone detected
#define SIN_DSQA_F		(SIN_DSQA)					// tone detected
#define SIN_DSQB_F		(SIN_DSQB)					// tone detected
#define SIN_MCK_F		(SIN_MCK)					// MIC u/d button change
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

#define	MHZ_TIME		SEC10		// MHZ digit mode timeout
#define	VQ_TIME			SEC3		// vol/squ adj timeout
#define	SET_TIME		(3*SEC10)	// set mode timeout
#define	SUB_TIME		(3*SEC10)	// sub-focus timeout
#define	MIC_RPT_TIME	80			// mic u/d button 80ms repeat period
#define	MIC_RPT_WAIT	1000		// mic u/d button repeat wait period
#define	MIC_DB_TIME		20			// mic u/d button debounce wait period
#define	MUTE_TIME		250			// volume mute delay for band swaps
#define	TSW_TIME		SEC10		// TS adj timeout
#define	SLIDE_TIME		SEC300MS	// text slider display shift rate
#define	SCAN_TIME		SEC400MS	// scan channel rate
#define	SCAN_TIME2		SEC1		// scan channel LOS hold time

// set/read_tsab():
#define	TSA_SEL			1			// selects TSA
#define	TSB_SEL			2			// selects TSB
#define	TS_5			1			// 5 KHz step
#define	TS_10			2			// 10 KHz step
#define	TS_25			5			// 25 KHz step

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

void init_radio(void);
void process_SIN(U8 cmd);
U8 process_SOUT(U8 cmd);
void  save_vfo(U8 b_id);
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
S8 is_mic_updn(U8 ipl, U8 focus, U8 xmq);
U32 get_freq(U8 focust);
void copy_vfot(U8 main);
void temp_vfo(U8 main);
U32 get_vfot(void);
void  force_push_radio(void);
U8 get_lohi(U8 bid, U8 setread);
U8 get_memnum(U8 main, U8 adder);
U8 get_callnum(U8 main, U8 adder);
U8 get_bit(U8 value);
U8 set_bit(U8 value);
U8 bit_set(U8 value);
void  update_radio_all(U8 vector);
void set_offs(U8 focus, U16 value);
U16 get_offs(U8 focus);
U8 inv_duplex(U8 focus);
U8 inv_vfo(U8 focus);
void mute_radio(U8 mutefl);
U8 get_mute_radio(void);
void set_bandid(U8 focus, U8 b_id);
void write_mem(U8 focus, U8 memnum);
void read_mem(U8 focus, U8 memnum);
void write_nvmem(U8 band, U8 memnum);
void read_nvmem(U8 band, U8 memnum);
void copy_vfo2temp(U8 focus);
void copy_temp2vfo(U8 focus);
void save_mc(U8 focus);
char* get_nameptr(U8 focus);
void set_bandnv(void);
void set_qnv(U8 focus);
void set_vnv(U8 focus);
void set_tonenv(U8 focus);
U8 get_scanmem(U8 focus);
U8 togg_scanmem(U8 focus);
U32 get_memaddr(U8 band, U8 memnum);
