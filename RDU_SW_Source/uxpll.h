/********************************************************************
 ************ COPYRIGHT (c) 2021 by KE0FF, Taylor, TX   *************
 *
 *  File name: uxpll.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for radio.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>


// PLL calculation defines
// UX band module defines
#define BANDOFF 0x00
#define ID10M	0x01					// ordinal ID numbers
#define ID6M	0x02
#define ID2M	0x03
#define ID220	0x04
#define ID440	0x05
#define ID1200	0x06
#define IDBUNIT	0x07
#define BAND_ERROR	0xff
#define	NUM_VFOS	(ID1200 * 2)		// 2 for each UX band
#define ID10M_IDX	0x00				// ordinal index values
#define ID6M_IDX	0x01
#define ID2M_IDX	0x02
#define ID220_IDX	0x03
#define ID440_IDX	0x04
#define ID1200_IDX	0x05

#define ID10M_B		0x01				// bitmapped ID numbers
#define ID6M_B		0x02
#define ID2M_B		0x04
#define ID220_B		0x08
#define ID440_B		0x10
#define ID1200_B	0x20

#define IDALL_B		(ID10M_B | ID6M_B | ID2M_B | ID220_B | ID440_B | ID1200_B)

// RX base freqs (KHz)
#define BASE_RX_10M		28000L
#define BASE_RX_6M		40000L
#define BASE_RX_2M		136000L			// bit7 skip
#define BASE_RX_220		220000L			// bit7 skip
#define BASE_RX_440		400000L
#define BASE_RX_1200	1200000L		// bit7 skip
// TX offset freqs (5KHz channels)
#define BASE_TX_10M		(10695L/5L)		// (-)
#define BASE_TX_6M		(13990L/5L)		// (-)
#define BASE_TX_2M		(17200L/5L)		// (+)
#define BASE_TX_220		(17200L/5L)		// (+) 
#define BASE_TX_440		(23150L/5L)		// (+)
#define BASE_TX_1200	(136600L/10L)	// (+) 1.2G is 10KHz channel
// PLL base bitmaps
#define INIT_PLL_6M		0x01325L			// PLL init pattern for 6M & 10M
#define INIT_PLL_10M	0x01325L
#define PLL_10M			0x01e3bL			// must <<1 after adding freq
#define PLL_6M			0x02a2eL			// must <<1 after adding freq
#define PLL_2M			0x05CD0L
#define PLL_220			0x09e70L
#define PLL_440			0x1266AL
//#define	PLL_BITLEN		0x14000000L		// PLL xfer bit length
//#define	PLL_1200BITLEN	0x04000000L		// 1200 init frames PLL xfer bit length
// the TC9181 PLL IC takes LSB 1st, which requires all of the PLL streams to be bit reversed and MSB aligned
// prior to transmission with the msb-first drivers.
#define INIT_PLL_1200	0x000000A1L		// PLL init pattern for 1200 (16 bits, bit reversed sets ref osc)
#define INIT_PLL_1201	0x00000000L		// "HL" config, 4 bits only
#define INIT_PLL_1202	0x00000003L		// "GPIO" reset config, 4 bits only
#define INIT_PLL_1203	0x0000000fL		// "GPIO" config, 4 bits only
#define PLL_1200_N		0x00040000L		// "code" bits for N frames
#define PLL_1200_MASK	0x0003ffffL		// mask for N frames
#define PLL_1200		0x19f64L		// must bit reverse
#define UX_XIT_MASK		0xf0000000L		// XIT code mask
#define	UX_XIT			0x80000000L		// XIT code
#define	UX_XIT_UP		0x00000010L		// XIT up code
#define	UX_XIT_COUNT	0x0000000fL		// XIT count mask
#define	UX_XIT_CKUP		0x00000007L		// XIT clk up code (1st half-clk)
#define	UX_XIT_CKDN		0x0000000bL		// XIT clk dn code (1st half-clk)
#define	UX_XIT_CK0		0x0000000fL		// XIT, 2nd half-clk
#define	REG_XIT_UP		0x10			// XIT/RIT up bit
#define	REG_XIT_CNT		0x0f			// count value

#define TX_IDX			5
#define	DATA_ARRAY_LEN	11
#define	BND_IDX			10

//#define	BND_IDX8		40
#define BNDID_MASK8		0xf0
#define BND_MASK		0xffffff00
#define BNDID_MASK		0xfffffff0
#define BNDID_FMASK		0x0000000f
#define	BNDID_HIPWR		0x00000020
#define	BNDID_WBIF		0x00000010
#define	BNDID_SOVRD		0x00000040
#define	BNDID_Q			0x00000080
#define BND_SHIFT		0

// SOUT defines
#define	ADDR_SOUT		0x38000000					// mask for addr bits
#define	OPT12_SOUT		0x04000000					// Selects OPT1 = main, 2 = sub if "1", else OPT1 = sub, 2 = main
#define	VOL_SOUT		0x02000000					// if "1", vol level
#define	SQU_SOUT		0x01000000					// if "1", SQU level
#define	TONE_SOUT		0x00800000					// if "1", tone code
#define	DSQL_SOUT		0x00400000					// if "1", DSQ ?
#define	XFR12_SOUT		0x00200000					// "1" = OPT1 is serial xfr dest, else OPT2
#define	OPT_DEFLT		(OPT12_SOUT|XFR12_SOUT)		// default OPT state

#define	ATTEN_FINE_LIM		5
#define	ATTEN_FINE			0x000400
#define	ATTEN_COARSE_LIM	7
#define	ATTEN_COARSE		0x020000
#define	LEVEL_MAX 		((ATTEN_FINE_LIM * ATTEN_COARSE_LIM) - 1)
#define	ATTEN_MAIN		0x00000010					// main select
#define	ATTEN_SUB		0x00000008					// sub select

#define	VOL_ADDR		(ADDR_SOUT|VOL_SOUT)		// sout addr for vol
#define	SQU_ADDR		(ADDR_SOUT|SQU_SOUT)		// sout addr for squ
#define	TONE_ADDR		(ADDR_SOUT|TONE_SOUT)		// sout addr for tone enc

///////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// external function prototypes
//-----------------------------------------------------------------------------

