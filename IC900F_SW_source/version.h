/********************************************************************
 ************ COPYRIGHT (c) 2022 by ke0ff, Taylor, TX   *************
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
 *    03-23-24 jmh:  0.14 IC900F initial field beta
 *    07-06-22 jmh:  0.12 MFmic support release
 *    04-09-16 jmh:  0.1 First field release
 *    07-30-14 jmh:  0.0 creation date
 *
 *******************************************************************/

#ifdef VERSOURCE
const S8    version_number[] = {"0.15"};
const S8    date_code[]      = {"19-May-2024"};
#endif

//-----------------------------------------------------------------------------
// Public Fn Declarations
//-----------------------------------------------------------------------------
void dispSWvers(void);
void ccmdSWvers(char* sbuf);
U16 nvram_sn(void);

#define VERSION_INCLUDED
