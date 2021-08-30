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

// putspi defines
#define	CS_OPEN			1
#define	CS_CLOSE		2
#define	CS_OPENCLOSE	(CS_OPEN | CS_CLOSE)
#define	CS_IDLE			0

// BUSY WAIT TIMEOUT
#define	BUSY_WAT	5					// max delay to wait for UART2 TX to clear (ms)

//-----------------------------------------------------------------------------
// Global Fns
//-----------------------------------------------------------------------------

uint8_t send_spi3(uint8_t data);
int put_spi(const U8 *string, U8 mode);
void open_spi(uint8_t addr);
void close_spi(void);
void spi1_clean(void);
void lcd_cmd(U8 cmd);
void send_so(uint32_t data);

#endif /* SPI_H_ */
