//*****************************************************************************
//
// Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include <stdint.h>
#include "typedef.h"
#include "init.h"
#include "serial.h"
#include "sio.h"
#include "spi.h"
#include "radio.h"
#include "inc/tm4c123gh6pm.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void UART0Handler(void);
static void UART1Handler(void);
static void GPIO_C_Handler(void);
static void GPIO_F_Handler(void);
static void TIMER0AHandler(void);
static void TIMER1AHandler(void);
static void TIMER1BHandler(void);
static void TIMER2AHandler(void);
static void TIMER2BHandler(void);
static void TIMER3AHandler(void);
//static void TIMER3BHandler(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
// To be added by user

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_TOP),
                                            //00 The initial stack pointer
    // Proc execptions
    ResetISR,                               //01 The reset handler
    NmiSR,                                  //02 The NMI handler
    FaultISR,                               //03 The hard fault handler
    IntDefaultHandler,                      //04 The MPU fault handler
    IntDefaultHandler,                      //05 The bus fault handler
    IntDefaultHandler,                      //06 The usage fault handler
    0,                                      //07 Reserved
    0,                                      //08 Reserved
    0,                                      //09 Reserved
    0,                                      //10 Reserved
    IntDefaultHandler,                      //11 SVCall handler
    IntDefaultHandler,                      //12 Debug monitor handler
    0,                                      //13 Reserved
    IntDefaultHandler,                      //14 The PendSV handler
    IntDefaultHandler,                      //15 The SysTick handler
    //int vectors
    IntDefaultHandler,                      //16 GPIO Port A						// 0
	IntDefaultHandler,                      //17 GPIO Port B						// 1
	GPIO_C_Handler,     	               	//18 GPIO Port C						// 2
	IntDefaultHandler,                     	//19 GPIO Port D						// 3
    IntDefaultHandler,                      //20 GPIO Port E						// 4
    UART0Handler,                      		//21 UART0 Rx and Tx					// 5
    UART1Handler,                           //22 UART1 Rx and Tx					// 6
    IntDefaultHandler,                      //23 SSI0 Rx and Tx						// 7
    IntDefaultHandler,                      //24 I2C0 Master and Slave				// 8
    IntDefaultHandler,                      //25 PWM0 Fault							// 9
    IntDefaultHandler,                      //26 PWM0 Generator 0					// 10
    IntDefaultHandler,                      //27 PWM0 Generator 1					// 11
    IntDefaultHandler,                      //28 PWM0 Generator 2					// 12
    IntDefaultHandler,                      //29 Quadrature Encoder 0				// 13
    IntDefaultHandler,                      //30 ADC Sequence 0						// 14
    IntDefaultHandler,                      //31 ADC Sequence 1						// 15
    IntDefaultHandler,                      //32 ADC Sequence 2						// 16
    IntDefaultHandler,                      //33 ADC Sequence 3						// 17
    IntDefaultHandler,                      //34 Watchdog timer						// 18
	TIMER0AHandler,                 	    //35 Timer 0 subtimer A					// 19
    IntDefaultHandler,                      //36 Timer 0 subtimer B					// 20
    TIMER1AHandler,                         //37 Timer 1 subtimer A					// 21
	TIMER1BHandler,                      	//38 Timer 1 subtimer B					// 22
    TIMER2AHandler,                         //39 Timer 2 subtimer A					// 23
	TIMER2BHandler,	                        //40 Timer 2 subtimer B					// 24
    IntDefaultHandler,                      //41 Analog Comparator 0				// 25
    IntDefaultHandler,                      //42 Analog Comparator 1				// 26
    IntDefaultHandler,                      //43 Analog Comparator 2				// 27
    IntDefaultHandler,                      //44 System Control (PLL, OSC, BO)		// 28
    IntDefaultHandler,                      //45 FLASH Control						// 29
	GPIO_F_Handler,                      	//46 GPIO Port F						// 30
    IntDefaultHandler,                      //47 N/U GPIO Port G					// 31
    IntDefaultHandler,                      //48 N/U GPIO Port H					// 32
	IntDefaultHandler,                      //49 UART2 Rx and Tx					// 33
    IntDefaultHandler,                      //50 SSI1 Rx and Tx						// 34
	TIMER3AHandler,                         //51 Timer 3 subtimer A					// 35
    IntDefaultHandler,                      //52 Timer 3 subtimer B					// 36
    IntDefaultHandler,                      //53 I2C1 Master and Slave				// 37
    IntDefaultHandler,                      //54 Quadrature Encoder 1				// 38
    IntDefaultHandler,                      //55 CAN0								// 39
    IntDefaultHandler,                      //56 CAN1								// 40
    0,                                      //57 Reserved							// 41
    0,                                      //58 Reserved							// 42
    IntDefaultHandler,                      //59 Hibernate							// 43
    IntDefaultHandler,                      //60 USB0								// 44
    IntDefaultHandler,                      //61 PWM Generator 3					// 45
    IntDefaultHandler,                      //62 uDMA Software Transfer				// 46
    IntDefaultHandler,                      //63 uDMA Error							// 47
    IntDefaultHandler,                      //64 ADC1 Sequence 0					// 48
    IntDefaultHandler,                      //65 ADC1 Sequence 1					// 49
    IntDefaultHandler,                      //66 ADC1 Sequence 2					// 50
    IntDefaultHandler,                      //67 ADC1 Sequence 3					// 51
    0,                                      //68 Reserved							// 52
    0,                                      //69 Reserved							// 53
    IntDefaultHandler,                      //70 GPIO Port J						// 54
    IntDefaultHandler,                      //71 GPIO Port K						// 55
    IntDefaultHandler,                      //72 GPIO Port L						// 56
    IntDefaultHandler,                      //73 SSI2 Rx and Tx						// 57
    IntDefaultHandler,                      //74 SSI3 Rx and Tx						// 58
    IntDefaultHandler,                      //75 UART3 Rx and Tx					// 59
    IntDefaultHandler,                      //76 UART4 Rx and Tx					// 60
    IntDefaultHandler,                      //77 UART5 Rx and Tx					// 61
    IntDefaultHandler,                      //78 UART6 Rx and Tx					// 62
    IntDefaultHandler,                      //79 UART7 Rx and Tx					// 63
    0,                                      //80 Reserved							// 64
    0,                                      //81 Reserved							// 65
    0,                                      //82 Reserved							// 66
    0,                                      //83 Reserved							// 67
    IntDefaultHandler,                      //84 I2C2 Master and Slave				// 68
    IntDefaultHandler,                      //85 I2C3 Master and Slave				// 69
    IntDefaultHandler,                      //86 Timer 4 subtimer A					// 70
    IntDefaultHandler,                      //87 Timer 4 subtimer B					// 71
    0,                                      //88 Reserved							// 72
    0,                                      //89 Reserved							// 73
    0,                                      //90 Reserved							// 74
    0,                                      //91 Reserved							// 75
    0,                                      //92 Reserved							// 76
    0,                                      //93 Reserved							// 77
    0,                                      //94 Reserved							// 78
    0,                                      //95 Reserved							// 79
    0,                                      //96 Reserved							// 80
    0,                                      //97 Reserved							// 81
    0,                                      //98 Reserved							// 82
    0,                                      //99 Reserved							// 83
    0,                                      //100 Reserved							// 84
    0,                                      //101 Reserved							// 85
    0,                                      //102 Reserved							// 86
    0,                                      //103 Reserved							// 87
    0,                                      //104 Reserved							// 88
    0,                                      //105 Reserved							// 89
    0,                                      //106 Reserved							// 90
    0,                                      //107 Reserved							// 91
    IntDefaultHandler,                      //108 Timer 5 subtimer A				// 92
    IntDefaultHandler,                      //109 Timer 5 subtimer B				// 93
    IntDefaultHandler,                      //110 Wide Timer 0 subtimer A			// 94
    IntDefaultHandler,                      //111 Wide Timer 0 subtimer B			// 95
    IntDefaultHandler,                      //112 Wide Timer 1 subtimer A			// 96
    IntDefaultHandler,                      //113 Wide Timer 1 subtimer B			// 97
    IntDefaultHandler,                      //114 Wide Timer 2 subtimer A			// 98
    IntDefaultHandler,                      //115 Wide Timer 2 subtimer B			// 99
    IntDefaultHandler,                      //116 Wide Timer 3 subtimer A			// 100
    IntDefaultHandler,                      //117 Wide Timer 3 subtimer B			// 101
    IntDefaultHandler,                      //118 Wide Timer 4 subtimer A			// 102
    IntDefaultHandler,                      //119 Wide Timer 4 subtimer B			// 103
    IntDefaultHandler,                      //120 Wide Timer 5 subtimer A			// 104
    IntDefaultHandler,                      //121 Wide Timer 5 subtimer B			// 105
    IntDefaultHandler,                      //122 FPU								// 106
    0,                                      //123 Reserved							// 107
    0,                                      //124 Reserved							// 108
    IntDefaultHandler,                      //125 I2C4 Master and Slave				// 109
    IntDefaultHandler,                      //126 I2C5 Master and Slave				// 110
    IntDefaultHandler,                      //127 GPIO Port M						// 111
    IntDefaultHandler,                      //128 GPIO Port N						// 112
    IntDefaultHandler,                      //129 Quadrature Encoder 2				// 113
    0,                                      //130 Reserved							// 114
    0,                                      //131 Reserved							// 115
    IntDefaultHandler,                      //132 GPIO Port P (Summary or P0)		// 116
    IntDefaultHandler,                      //133 GPIO Port P1						// 117
    IntDefaultHandler,                      //134 GPIO Port P2						// 118
    IntDefaultHandler,                      //135 GPIO Port P3						// 119
    IntDefaultHandler,                      //136 GPIO Port P4						// 120
    IntDefaultHandler,                      //137 GPIO Port P5						// 121
    IntDefaultHandler,                      //138 GPIO Port P6						// 122
    IntDefaultHandler,                      //139 GPIO Port P7						// 123
    IntDefaultHandler,                      //140 GPIO Port Q (Summary or Q0)		// 124
    IntDefaultHandler,                      //141 GPIO Port Q1						// 125
    IntDefaultHandler,                      //142 GPIO Port Q2						// 126
    IntDefaultHandler,                      //143 GPIO Port Q3						// 127
    IntDefaultHandler,                      //144 GPIO Port Q4						// 128
    IntDefaultHandler,                      //145 GPIO Port Q5						// 129
    IntDefaultHandler,                      //146 GPIO Port Q6						// 130
    IntDefaultHandler,                      //147 GPIO Port Q7						// 131
    IntDefaultHandler,                      //148 GPIO Port R						// 132
    IntDefaultHandler,                      //149 GPIO Port S						// 133
    IntDefaultHandler,                      //150 PWM 1 Generator 0					// 134
    IntDefaultHandler,                      //151 PWM 1 Generator 1					// 135
    IntDefaultHandler,                      //152 PWM 1 Generator 2					// 136
    IntDefaultHandler,                      //153 PWM 1 Generator 3					// 137
    IntDefaultHandler                       //154 PWM 1 Fault						// 138
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
__asm("    .global _c_int00\n"
      "    b.w     _c_int00");
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Fault trap.  Assumes POR init (specifically, UART0) is complete.
    //
   	// send "FLT\n" msg
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'N';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'M';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'I';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = '\n';
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
	U8	trap = 1;
    //
    // Fault trap.  Assumes POR init (specifically, UART0) is complete.
    //
   	// send "FLT\n" msg
   	while((UART1_FR_R & 0x0020) == 0x0020);
   	UART1_DR_R = 'F';
   	while((UART1_FR_R & 0x0020) == 0x0020);
   	UART1_DR_R = 'L';
   	while((UART1_FR_R & 0x0020) == 0x0020);
   	UART1_DR_R = 'T';
   	while((UART1_FR_R & 0x0020) == 0x0020);
   	UART1_DR_R = '\n';
   	while((UART1_FR_R & 0x0020) == 0x0020);
   	UART1_DR_R = '\r';
   	while(trap);
}
//*****************************************************************************
//
// This is the code that gets called when the processor receives a UART intr.
//
//*****************************************************************************
static void
UART0Handler(void)
{
	rxd_intr();						// process UART rcv interrupt
}

static void
UART1Handler(void)
{
	rxd1_intr();						// process UART rcv interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a GPIO intr.
//
//*****************************************************************************

static void
GPIO_C_Handler(void)
{
	gpioc_isr();					// process gpioc interrupt
}

static void
GPIO_F_Handler(void)
{
	gpiof_isr();					// process gpiof interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a TIMER0 intr.
//
//*****************************************************************************
static void
TIMER0AHandler(void)
{
	Timer0A_ISR();					// process timer1 interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a TIMER1 intr.
//
//*****************************************************************************
static void
TIMER1AHandler(void)
{
	Timer1A_ISR();					// process timer1A interrupt
}

static void
TIMER1BHandler(void)
{
	Timer1B_ISR();					// process timer1B interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a TIMER2 intr.
//
//*****************************************************************************
static void
TIMER2AHandler(void)
{
	Timer2A_ISR();					// process timer2A interrupt
}

//*****************************************************************************
//
// TIMER2B ISR (process_SOUT)
//
//*****************************************************************************
static void
TIMER2BHandler(void)
{
	process_SOUT(0);				// process timer2B interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a TIMER3 intr.
//
//*****************************************************************************
static void
TIMER3AHandler(void)
{
	Timer3A_ISR();					// process timer3A interrupt
}
/*
static void
TIMER3BHandler(void)
{
//	Timer3B_ISR();					// process timer3B interrupt
}*/

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Fault trap.  Assumes POR init (specifically, UART0) is complete.
    //
   	// send "ERQ\n" msg
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'E';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'R';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'Q';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = '\n';
}
