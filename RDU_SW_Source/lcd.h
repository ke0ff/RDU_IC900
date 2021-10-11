/********************************************************************
 ************ COPYRIGHT (c) 2021 by KE0FF, Taylor, TX   *************
 *
 *  File name: lcd.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for lcd.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>


//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
// Processor I/O assignments
// ...see init.h

// mem scan COS memory defines
#define	MAX_COS_MEM		4		// "last cos" memory

// mode defines
#define	MODE_MASK		0x0f	// main modes mask
#define	FOCUS_MASK		0x01	// focus mask
#define	SUB_MODE		0x00	// "normal" mode, sub focus
#define	MAIN_MODE		0x01	// "normal" mode, main focus
#define	SUB_D			0x02	// disp signal sub flag
#define	SET_MODE		0x04	// set/config loop mode
//#define	???		0x08	// spare
#define	VMODE_MASK		0xf0	// "normal" vfo display mode mask
#define	VMODE_TSDISP	0x10	// display TS adj
#define	VMODE_TDISP		0x20	// display PL tone
#define	VMODE_ISTX		0x40	// tx mode
#define	VMODE_ODISP		0x80	// display offset

// process_VFODISP defines
#define	TONE_DISP		0x01
#define	TS_DISP			0x02
#define	NORM_DISP		0x03
#define	TXTSLID_DISP	0x04

//#define VFOTR_IS_TX		0x40	// flag to signal fetch of vfotr to display TX frequency from uxpll update
//#define VFO_DISP_F		0x80	// flag to signal vfo display update

// Xq-flags
#define	VOL_XFLAG		0x01		// VOL display/edit active
#define	SQU_XFLAG		0x02		// SQU display/edit active
#define	TONE_XFLAG		0x04		// tone display/edit active
#define	OFFS_XFLAG		0x08		// offset display/edit active
#define	TEXTM_SLIDE		0x10		// text slider display active flags (M/S)
#define	TEXTS_SLIDE		0x20
#define	MSCANM_XFLAG	0x40		// mem scan active flags (M/S)
#define	MSCANS_XFLAG	0x80

// Xz-flags
#define	MSCANM1_XFLAG	0x40		// mem scan first active flags (M/S)
#define	MSCANS1_XFLAG	0x80

// XMODE flags
#define	MEM_XFLAG		0x10
#define	CALL_XFLAG		0x20
#define	MC_XFLAG		(MEM_XFLAG|CALL_XFLAG)

// chkmode flags
#define	REV_FLAG		0x01		// reverse mode enabled
#define	REV_SQU_FLAG	0x02		// check mode enabled

// smute flags
#define	MS_MUTE			0x01		// both bands band mute
#define	SUB_MUTE		0x02		// sub band mute

// sys_err (freq display Fn error flags)
#define	NO_MUX_PRSNT	0x01		// no main band present
#define	NO_SUX_PRSNT	0x02		// no sub band present
#define	NO_B_PRSNT		0x04		// no base present

// mfreq/sfreq blink flags
#define	LEAD0		0xff
#define	MAIN_CS		0x80			// sets main lcd chip in digblink()

#define	MHZ_ONE		0x80
#define	MHZ_OFFS	0x40
#define	MHZ_OFF		0x20
#define	MHZ_MASK	0x1f

// LCD chip command masks
#define	CS2_MASK	0x80
#define	CS1_MASK	0x40
#define	DA_CM_MASK	0x20
#define	LEN_MASK	0x1f

// LCD chip commands
#define	MODE_SET	0x49			// /3 time-div, 1/3 bias, 2E-8 fdiv
#define	BLINK_SLOW	0x1A			// low-bit is flash-rate
#define	BLINK_FAST	0x1B			//  "   " ...
#define	BLINK_OFF	0x18
#define	DISP_ON		0x11
#define	DISP_OFF	0x10
#define	WITH_DECODE	0x15
#define	WITHOUT_DECODE	0x14
#define	LOAD_PTR	0xE0			// OR with (0x1f masked address)
#define	WR_DMEM		0xD0			// OR with (0x0f masked data)
#define	OR_DMEM		0xB0			// OR with (0x0f masked data)
#define	AND_DMEM	0x90			// OR with (0x0f masked data)
#define	CLR_DMEM	0x20
#define	WR_BMEM		0xC0			// OR with (0x0f masked data)
#define	OR_BMEM		0xA0			// OR with (0x0f masked data)
#define	AND_BMEM	0x80			// OR with (0x0f masked data)
#define	CLR_BMEM	0x00

#define	MAX_SRF		7				// #srf bargraph segments
#define	MSMET_ADDR	0x06
#define	SSMET_ADDR	0x1b

// LCD segment bit defines (addr and segment map)
//	CS1
#define	MMEM_ADDR		0x01
#define	MDUP		0x1
#define	MDUP_ADDR		0x04
#define	MMIN		0x2
#define	MMIN_ADDR		0x04
#define	MSKP		0x4
#define	MSKP_ADDR		0x04
#define	MTNE		0x1
#define	MTNE_ADDR		0x05
#define	MM			0x2
#define	MM_ADDR			0x05

#define	MSRF6		0x4
#define	MSRF6_ADDR		0x06
#define	MSRF3		0x1
#define	MSRF4		0x2
#define	MSRF5		0x4
#define	MSRF345_ADDR	0x07
#define	MSRF0		0x1
#define	MSRF1		0x2
#define	MSRF2		0x4
#define	MSRF012_ADDR	0x08
#define	M00			0x2
#define	M00_ADDR	0x09
#define	M0_ADDR		0x0a

#define	AOW			0x1
#define	AOW_ADDR		0x1d
#define	ALOW		0x2
#define	ALOW_ADDR		0x1d

#define	ARIT		0x1
#define	ARIT_ADDR		0x1f
#define	AVXO		0x2
#define	AVXO_ADDR		0x1f
#define	ATS			0x4
#define	ATS_ADDR		0x1f

//	CS2
#define	SMEM_ADDR	0x02
#define	SDUP		0x1
#define	SDUP_ADDR	0x05
#define	SMIN		0x2
#define	SMIN_ADDR	0x05
#define	SSKP		0x4
#define	SSKP_ADDR	0x05
#define	STNE		0x1
#define	STNE_ADDR	0x06
#define	SM			0x2
#define	SM_ADDR		0x06
#define	S00			0x2
#define	S00_ADDR	0x07

#define	S0_ADDR		0x08
#define	S6			0x1
#define	S6_ADDR		0x1a

#define	SSRF6		0x4
#define	SSRF6_ADDR		0x1b
#define	SSRF3		0x1
#define	SSRF4		0x2
#define	SSRF5		0x4
#define	SSRF345_ADDR	0x1c
#define	SSRF0		0x1
#define	SSRF1		0x2
#define	SSRF2		0x4
#define	SSRF012_ADDR	0x1d

#define	ASUB		0x1
#define	ASUB_ADDR	0x1e
#define	ALCK		0x2
#define	ALCK_ADDR	0x1e

#define	APRG		0x1
#define	APRG_ADDR	0x1f
#define	AMHZ		0x2
#define	AMHZ_ADDR	0x1f
#define	ABND		0x4
#define	ABND_ADDR	0x1f

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

void init_lcd(void);
void reset_lcd(void);
void process_UI(U8 cmd);
void digblink(U8 digaddr, U8 tf);
void mfreq(U32 dfreq, U8 blink);
void sfreq(U32 dfreq, U8 blink);
void msmet(U8 srf, U8 blink);
void ssmet(U8 srf, U8 blink);
U8 mputs_lcd(char *s, U8 dp_tf);
U8 sputs_lcd(char *s, U8 dp_tf);
void mmem(U8 mn);
void smem(U8 mn);
void mtonea(U8 tf);
void mema(U8 focus, U8 tf);
void stonea(U8 tf);
void mskpa(U8 focus, U8 tf);
void mdupa(char dplx);
void sdupa(char dplx);
void mdupa_blink(U8 tf);
void sdupa_blink(U8 tf);
void alow(U8 tf);
void amhz(U8 tf);
void abnd(U8 tf);
void asub(U8 tf);

void amtx(U8 tf);
void amrx(U8 tf);
void asrx(U8 tf);

void bin32_bcds(U32 bin32, U8* sptr);
U32 bcds_bin32(U8* sptr);
void add_bcds(U8* sptr, S8 adder, U8 addr, U8 max, U8 min);
void set_vfo_display(U8	sig);
void rev_vfo(U8	focus);
void force_push(void);
U8 get_mutefl(void);
U8 get_xmode(U8 b_id);
void write_xmode(U8 main);
void read_xmode(void);
void clear_xmode(void);
void set_sys_err(U8 err);
void clr_sys_err(U8 err);
U8 puts_slide(U8 focus, char* sptr, U8 cmd);
U8 doscan(U8 focus, U8 tf);
U8 get_xmodeq(void);
void set_ptt(U8 pttstat);
void set_slide(U8 focus, U8 tf);
U8 mem2ordinal(char cm);
char ordinal2mem(U8 memnum);
U8 lookup_pl(U16 ctcss);
