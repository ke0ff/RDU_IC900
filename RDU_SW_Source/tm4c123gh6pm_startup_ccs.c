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
static void TIMER1Handler(void);
static void TIMER2AHandler(void);
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
    IntDefaultHandler,                      //16 GPIO Port A
	IntDefaultHandler,                      //17 GPIO Port B
	GPIO_C_Handler,     	               	//18 GPIO Port C
	IntDefaultHandler,                     	//19 GPIO Port D
    IntDefaultHandler,                      //20 GPIO Port E
    UART0Handler,                      		//21 UART0 Rx and Tx
    UART1Handler,                           //22 UART1 Rx and Tx
    IntDefaultHandler,                      //23 SSI0 Rx and Tx
    IntDefaultHandler,                      //24 I2C0 Master and Slave
    IntDefaultHandler,                      //25 PWM0 Fault
    IntDefaultHandler,                      //26 PWM0 Generator 0
    IntDefaultHandler,                      //27 PWM0 Generator 1
    IntDefaultHandler,                      //28 PWM0 Generator 2
    IntDefaultHandler,                      //29 Quadrature Encoder 0
    IntDefaultHandler,                      //30 ADC Sequence 0
    IntDefaultHandler,                      //31 ADC Sequence 1
    IntDefaultHandler,                      //32 ADC Sequence 2
    IntDefaultHandler,                      //33 ADC Sequence 3
    IntDefaultHandler,                      //34 Watchdog timer
	TIMER0AHandler,                 	    //35 Timer 0 subtimer A
    IntDefaultHandler,                      //36 Timer 0 subtimer B
    TIMER1Handler,                          //37 Timer 1 subtimer A
    IntDefaultHandler,                      //38 Timer 1 subtimer B
    TIMER2AHandler,                         //39 Timer 2 subtimer A
    IntDefaultHandler,                      //40 Timer 2 subtimer B
    IntDefaultHandler,                      //41 Analog Comparator 0
    IntDefaultHandler,                      //42 Analog Comparator 1
    IntDefaultHandler,                      //43 Analog Comparator 2
    IntDefaultHandler,                      //44 System Control (PLL, OSC, BO)
    IntDefaultHandler,                      //45 FLASH Control
	GPIO_F_Handler,                      	//46 GPIO Port F
    IntDefaultHandler,                      //47 N/U GPIO Port G
    IntDefaultHandler,                      //48 N/U GPIO Port H
	IntDefaultHandler,                      //49 UART2 Rx and Tx
    IntDefaultHandler,                      //50 SSI1 Rx and Tx
	TIMER3AHandler,                         //51 Timer 3 subtimer A
    IntDefaultHandler,                      //52 Timer 3 subtimer B
    IntDefaultHandler,                      //53 I2C1 Master and Slave
    IntDefaultHandler,                      //54 Quadrature Encoder 1
    IntDefaultHandler,                      //55 CAN0
    IntDefaultHandler,                      //56 CAN1
    0,                                      //57 Reserved
    0,                                      //58 Reserved
    IntDefaultHandler,                      //59 Hibernate
    IntDefaultHandler,                      //60 USB0
    IntDefaultHandler,                      //61 PWM Generator 3
    IntDefaultHandler,                      //62 uDMA Software Transfer
    IntDefaultHandler,                      //63 uDMA Error
    IntDefaultHandler,                      //64 ADC1 Sequence 0
    IntDefaultHandler,                      //65 ADC1 Sequence 1
    IntDefaultHandler,                      //66 ADC1 Sequence 2
    IntDefaultHandler,                      //67 ADC1 Sequence 3
    0,                                      //68 Reserved
    0,                                      //69 Reserved
    IntDefaultHandler,                      //70 GPIO Port J
    IntDefaultHandler,                      //71 GPIO Port K
    IntDefaultHandler,                      //72 GPIO Port L
    IntDefaultHandler,                      //73 SSI2 Rx and Tx
    IntDefaultHandler,                      //74 SSI3 Rx and Tx
    IntDefaultHandler,                      //75 UART3 Rx and Tx
    IntDefaultHandler,                      //76 UART4 Rx and Tx
    IntDefaultHandler,                      //77 UART5 Rx and Tx
    IntDefaultHandler,                      //78 UART6 Rx and Tx
    IntDefaultHandler,                      //79 UART7 Rx and Tx
    0,                                      //80 Reserved
    0,                                      //81 Reserved
    0,                                      //82 Reserved
    0,                                      //83 Reserved
    IntDefaultHandler,                      //84 I2C2 Master and Slave
    IntDefaultHandler,                      //85 I2C3 Master and Slave
    IntDefaultHandler,                      //86 Timer 4 subtimer A
    IntDefaultHandler,                      //87 Timer 4 subtimer B
    0,                                      //88 Reserved
    0,                                      //89 Reserved
    0,                                      //90 Reserved
    0,                                      //91 Reserved
    0,                                      //92 Reserved
    0,                                      //93 Reserved
    0,                                      //94 Reserved
    0,                                      //95 Reserved
    0,                                      //96 Reserved
    0,                                      //97 Reserved
    0,                                      //98 Reserved
    0,                                      //99 Reserved
    0,                                      //100 Reserved
    0,                                      //101 Reserved
    0,                                      //102 Reserved
    0,                                      //103 Reserved
    0,                                      //104 Reserved
    0,                                      //105 Reserved
    0,                                      //106 Reserved
    0,                                      //107 Reserved
    IntDefaultHandler,                      //108 Timer 5 subtimer A
    IntDefaultHandler,                      //109 Timer 5 subtimer B
    IntDefaultHandler,                      //110 Wide Timer 0 subtimer A
    IntDefaultHandler,                      //111 Wide Timer 0 subtimer B
    IntDefaultHandler,                      //112 Wide Timer 1 subtimer A
    IntDefaultHandler,                      //113 Wide Timer 1 subtimer B
    IntDefaultHandler,                      //114 Wide Timer 2 subtimer A
    IntDefaultHandler,                      //115 Wide Timer 2 subtimer B
    IntDefaultHandler,                      //116 Wide Timer 3 subtimer A
    IntDefaultHandler,                      //117 Wide Timer 3 subtimer B
    IntDefaultHandler,                      //118 Wide Timer 4 subtimer A
    IntDefaultHandler,                      //119 Wide Timer 4 subtimer B
    IntDefaultHandler,                      //120 Wide Timer 5 subtimer A
    IntDefaultHandler,                      //121 Wide Timer 5 subtimer B
    IntDefaultHandler,                      //122 FPU
    0,                                      //123 Reserved
    0,                                      //124 Reserved
    IntDefaultHandler,                      //125 I2C4 Master and Slave
    IntDefaultHandler,                      //126 I2C5 Master and Slave
    IntDefaultHandler,                      //127 GPIO Port M
    IntDefaultHandler,                      //128 GPIO Port N
    IntDefaultHandler,                      //129 Quadrature Encoder 2
    0,                                      //130 Reserved
    0,                                      //131 Reserved
    IntDefaultHandler,                      //132 GPIO Port P (Summary or P0)
    IntDefaultHandler,                      //133 GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
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
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'F';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'L';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = 'T';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = '\n';
   	while((UART0_FR_R & 0x0020) == 0x0020);
   	UART0_DR_R = '\r';
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
TIMER1Handler(void)
{
	Timer1A_ISR();					// process timer1 interrupt
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a TIMER2 intr.
//
//*****************************************************************************
static void
TIMER2AHandler(void)
{
	Timer2A_ISR();					// process timer2 interrupt
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
