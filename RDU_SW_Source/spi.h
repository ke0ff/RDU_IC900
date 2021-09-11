/********************************************************************
 ************ COPYRIGHT (c) 2015 by KE0FF, Taylor, TX   *************
 *
 *  File name: spi.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for spi.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>
#ifndef SPI_H_
#define SPI_H_


//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
// Processor I/O assignments
// ...see init.h

// TIMER1B isr mask
#define TIMER_MIS_BMASK	(TIMER_MIS_TBMMIS | TIMER_MIS_CBEMIS | TIMER_MIS_CBMMIS | TIMER_MIS_TBTOMIS)

// putspi defines
#define	CS_OPEN			1
#define	CS_CLOSE		2
#define	CS_OPENCLOSE	(CS_OPEN | CS_CLOSE)
#define	CS_IDLE			0
#define	CS_WRITE		0x80

// BUSY WAIT TIMEOUT
#define	BUSY_WAT	5					// max delay to wait for UART2 TX to clear (ms)

// ANV32AA1A NVRAM defines
#define	WREN		0x06				// write enable command (1 byte)
#define	WRDI		0x04				// write disable command (1 byte)
// STATUS bitmasks
#define	RDY_N		0x01				// /ready bit
#define	WEN			0x02				// write enable
#define	BP0			0x04				// block protect 0		BP[00] = none protected		BP[01] = 0x18000 - 0x1ffff
#define	BP1			0x08				// block protect 1		BP[10] = 0x10000 - 0x1ffff	BP[11] = 0x00000 - 0x1ffff
#define	SWM_N		0x10				// secure write result
#define	PDIS		0x40				// power store disable

#define	RDSR		0x05				// read status command (1 byte command, followed by 1 byte read)
#define	WRSR		0x01				// write status  (1 byte command, followed by 1 byte write)
#define	READ		0x03				// read memory (1 byte cmd, 3 byte addr write, 1 byte data read .. continuous reads return next memory address)
#define	SREAD		0x13				// secure read memory (see datasheet)
#define	WRITE		0x02				// write memory (1 byte cmd, 3 byte addr write, 1 byte data write .. continuous writes go to next memory address)
#define	SWRITE		0x12				// secure write memory (see datasheet)
#define	STORE		0x08				// store memory to NV space command (1 byte)
#define	RECALL		0x09				// recall memory from NV space command (1 byte)
#define	RDSNR		0xC3				// read user sernum command (1 byte, 2 bytes read)
#define	WRSNR		0xC2				// write user sernum command (1 byte, 2 bytes write)
#define	NVHIBR		0xB9				// NVRAM hibernate command (1 byte)

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

void init_spi3(void);
uint8_t send_spi3(uint8_t data);
int put_spi(const U8 *string, U8 mode);
void open_spi(uint8_t addr);
void close_spi(void);
void spi1_clean(void);
void lcd_cmd(U8 cmd);
void send_so(uint32_t data);
// NVRAM Fns
void close_nvr(void);
void wen_nvr(void);
void storecall_nvr(U8 tf_fl);
U8 rws_nvr(U8 dataw, U8 mode);
U8 rw8_nvr(U32 addr, U8 dataw, U8 mode);
U16 rw16_nvr(U32 addr, U16 dataw, U8 mode);
U32 rw32_nvr(U32 addr, U32 dataw, U8 mode);
U16 rwusn_nvr(U16 dataw, U8 mode);

#endif /* SPI_H_ */
