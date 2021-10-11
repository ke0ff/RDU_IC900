/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: dpl_fn.h
 *
 *  Module:    Control
 *
 *  Summary:
 *  dpl support functions header
 *  
 *******************************************************************/

//=============================================================================
// Global defines

//=============================================================================
// public command fn declarations

void dpl_cmd(char* obuf, U16* params, char ps);
U32 dpl_calc(U16 dplcode);
U8 cadd(U16 dplcode, U8 index);
