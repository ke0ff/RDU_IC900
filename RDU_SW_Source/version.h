/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: version.h
 *
 *  Module:    Control
 *
 *  Summary:   This header contains the software version number
 *             as a character string.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  0.0 creation date
 *    12-27-16 jmh:  0.1 First field release
 *    03-03-19 jmh:  0.2 Modified to support SIL changes for uACU
 *
 *******************************************************************/

#ifdef VERSOURCE
const S8    version_number[] = {"0.0"};
const S8    date_code[]      = {"10-Oct-2021"};
#endif

//-----------------------------------------------------------------------------
// Public Fn Declarations
//-----------------------------------------------------------------------------
void dispSWvers(void);
void ccmdSWvers(char* sbuf);
U16 nvram_sn(void);

#define VERSION_INCLUDED
