/********************************************************************
 ************ COPYRIGHT (c) 2016 by ke0ff, Taylor, TX   *************
 *
 *  File name: tiva_init.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  Tiva processor init functions
 *
 *******************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "tiva_init.h"
#include "serial.h"
#include "PLL.h"
#include "eeprom.h"
#include "spi.h"
#include "lcd.h"
#include "sio.h"
#include "adc.h"

#include "version.h"

//=============================================================================
// local registers


//=============================================================================
// local Fn declarations


//*****************************************************************************
// proc_init()
//  initializes the processor I/O peripherals
//	returns bitmapped initialize result status as U16
//
//*****************************************************************************
U16 proc_init(void){
	U8			j;					// temp U8
	volatile uint32_t ui32Loop;
	U16	ipl;						// initialize response value
	uint32_t i;

	// Enable the GPIO port clocks.
	SYSCTL_RCGCGPIO_R = PORTF|PORTE|PORTD|PORTC|PORTB|PORTA;

	// init Ports A & B
	GPIO_PORTA_DATA_R = PORTA_INIT;
	GPIO_PORTB_DATA_R = PORTB_INIT;
	GPIO_PORTA_DIR_R = PORTA_DIRV;
	GPIO_PORTA_DEN_R = PORTA_DENV;
	GPIO_PORTA_PUR_R = PORTA_PURV;
	GPIO_PORTB_DIR_R = PORTB_DIRV;
	GPIO_PORTB_DEN_R = PORTB_DENV;
	GPIO_PORTB_PUR_R = PORTB_PURV;

	// init LCD BUSY edge detect
	j = GPIO_PORTE_IM_R;												// disable SIN edge intr
	GPIO_PORTE_IM_R = 0x00;
	GPIO_PORTE_IEV_R &= ~BUSY_N;										// falling edge
	GPIO_PORTE_IBE_R &= ~BUSY_N;										// one edge
	GPIO_PORTE_IS_R &= ~BUSY_N;											// edge ints
	GPIO_PORTE_ICR_R = 0xff;											// clear int flags
	GPIO_PORTE_IM_R = j;												// re-enable SIN edge intr

	// init PLL
	for(i = 0; i<10000; i++);
	PLL_Init(SYSCLK);
	ipl = IPL_PLLINIT;

	// init UARTs
	initserial();												// init UART0-1
	NVIC_EN0_R = NVIC_EN0_UART0|NVIC_EN0_UART1;					// enable UART0 & UART1 intr
	ipl |= IPL_UART0INIT;
	ipl |= IPL_UART1INIT;

	// init HIB module...disabled in this app
	ipl |= hib_init(0);

	// Do a dummy read to insert a few cycles after enabling the peripheral.
	ui32Loop = SYSCTL_RCGCGPIO_R;

	// init Ports C - F
	GPIO_PORTF_LOCK_R = 0x4C4F434B;										// unlock PORTF
	GPIO_PORTF_CR_R = 0xff;
	GPIO_PORTF_DIR_R = PORTF_DIRV;
	GPIO_PORTF_DEN_R = PORTF_DENV;
	GPIO_PORTF_AFSEL_R = 0;
	GPIO_PORTE_DIR_R = PORTE_DIRV;
	GPIO_PORTE_DEN_R = PORTE_DENV;
	GPIO_PORTE_PUR_R = PORTE_PURV;
	GPIO_PORTD_LOCK_R = 0x4C4F434B;										// unlock PORTD
	GPIO_PORTD_CR_R = 0xff;
	GPIO_PORTD_DIR_R = PORTD_DIRV;
	GPIO_PORTD_DEN_R = PORTD_DENV;
	GPIO_PORTD_PUR_R = PORTD_PURV;
	GPIO_PORTC_DIR_R &= 0x0f;											// preserve JTAG pin assignments
	GPIO_PORTC_DEN_R &= 0x0f;
	GPIO_PORTC_DIR_R |= (PORTC_DIRV & 0xf0);
	GPIO_PORTC_DEN_R |= (PORTC_DENV & 0xf0);
	GPIO_PORTC_PUR_R |= (PORTC_PURV & 0xf0);

	GPIO_PORTF_DATA_R = PORTF_INIT;
	GPIO_PORTE_DATA_R = PORTE_INIT;
	GPIO_PORTD_DATA_R = PORTD_INIT;
	GPIO_PORTC_DATA_R = PORTC_INIT;

//	encoder_init();
	// DIAL up/dn config
	j = GPIO_PORTC_IM_R;												// disable edge intr
	GPIO_PORTC_IM_R = 0;												// disable edge intr
	GPIO_PORTC_IEV_R &= ~PORTC_DIAL;									// falling edge
	GPIO_PORTC_IBE_R &= ~PORTC_DIAL;									// one edge
	GPIO_PORTC_IS_R = ~PORTC_DIAL;										// edge ints
	GPIO_PORTC_ICR_R = 0xff;											// clear int flags
	j |= PORTC_DIAL;													// enable dial edge intr
	GPIO_PORTC_IM_R = j;
	NVIC_EN0_R = NVIC_EN0_GPIOC;										// enable GPIOC intr in the NVIC_EN regs

	// init IC-900 ASYNC serial I/O
	ipl |= init_sio();

	// init timer0A (piezo beep)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
	ui32Loop = SYSCTL_RCGCGPIO_R;
	// unlock PORTF
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R = 0xff;
	///////////////
	GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF0_M);
	GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF0_T0CCP0);
	GPIO_PORTF_AFSEL_R |= BEEP;
	TIMER0_CTL_R &= ~(TIMER_CTL_TAEN);										// disable timer
	TIMER0_CTL_R |= (TIMER_CTL_TAEVENT_NEG);								// enable FE PWM intr
	TIMER0_CFG_R = TIMER_CFG_16_BIT; //0x4; //0;
	TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TAAMS | TIMER_TAMR_TAPWMIE | TIMER_TAMR_TAMIE;
	TIMER0_TAPR_R = 0;
	set_beep(BEEP_FREQ, BEEP_COUNT);										// init timer regs to default beep freq/duration
	TIMER0_IMR_R = TIMER_IMR_CAEIM;											// enable timer intr
	TIMER0_ICR_R = TIMER0_MIS_R;											// clear any flagged ints
	NVIC_EN0_R = NVIC_EN0_TIMER0A;											// enable timer0A intr in the NVIC_EN regs

	// init timer1A (serial pacing timer, count down, no GPIO)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
	ui32Loop = SYSCTL_RCGCGPIO_R;
	TIMER1_CTL_R &= ~(TIMER_CTL_TAEN);				// disable timer
	TIMER1_CFG_R = TIMER_CFG_16_BIT; //0x4; //0;
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
	TIMER1_TAPR_R = TIMER1_PS;
	TIMER1_TAILR_R = (uint16_t)(SYSCLK/(TIMER1_FREQ * (TIMER1_PS + 1)));
	TIMER1_IMR_R = TIMER_IMR_TATOIM;				// enable timer intr
//	TIMER1_CTL_R |= (TIMER_CTL_TAEN);				// enable timer
	TIMER1_ICR_R = TIMER1_MIS_R;					// clear any flagged ints
	NVIC_EN0_R = NVIC_EN0_TIMER1A;					// enable timer1A intr in the NVIC_EN regs

	// init timer1B (bit-bang SPI, no GPIO)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
	ui32Loop = SYSCTL_RCGCGPIO_R;
	TIMER1_CTL_R &= ~(TIMER_CTL_TBEN);				// disable timer
//	TIMER1_CFG_R = TIMER_CFG_16_BIT; //0x4; //0;
	TIMER1_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
	TIMER1_TBPR_R = TIMER1B_PS;
	TIMER1_TBILR_R = (uint16_t)(SYSCLK/(BBSPICLK_FREQ * (TIMER1B_PS + 1)));
	TIMER1_IMR_R |= TIMER_IMR_TBTOIM;				// enable timer intr
//	TIMER1_CTL_R |= (TIMER_CTL_TBEN);				// enable timer
	TIMER1_ICR_R = TIMER1_MIS_R;					// clear any flagged ints
	NVIC_EN0_R = NVIC_EN0_TIMER1B;					// enable timer1A intr in the NVIC_EN regs

	// init Timer3A (appl timer, count down, no GPIO)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
	ui32Loop = SYSCTL_RCGCTIMER_R;
	TIMER3_CTL_R &= ~(TIMER_CTL_TAEN);									// disable timer
	TIMER3_CFG_R = TIMER_CFG_16_BIT;
	TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
	TIMER3_TAPR_R = (uint16_t)(TIMER3_PS - 1);							// prescale reg = divide ratio - 1
	TIMER3_TAILR_R = (uint16_t)(SYSCLK/(1000L * TIMER3_PS));
	TIMER3_IMR_R = TIMER_IMR_TATOIM;									// enable timer intr
	TIMER3_CTL_R |= (TIMER_CTL_TAEN);									// enable timer
	TIMER3_ICR_R = TIMER3_MIS_R;
	NVIC_EN1_R = NVIC_EN1_TIMER3A;										// enable timer intr in the NVIC
	ipl |= IPL_TIMER_ALL_INIT;

	// init LED PWMs on PF2 & PF3	(commented out unused PWMs... PF1-3, PE4-5 are all the PWMs supported here)
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
	ui32Loop = SYSCTL_RCGCPWM_R;										// delay a few cycles
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
	ui32Loop = SYSCTL_RCGCGPIO_R;										// delay a few cycles
	GPIO_PORTF_AFSEL_R |= BL_PWM|LED_PWM;								// enable alt fn, PF1-2
	GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF2_M|GPIO_PCTL_PF3_M);
	GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF2_M1PWM6|GPIO_PCTL_PF3_M1PWM7);
	SYSCTL_RCC_R = (SYSCTL_RCC_R & ~SYSCTL_RCC_PWMDIV_M) | (PWM_DIV << 17) | SYSCTL_RCC_USEPWMDIV;
//	PWM1_1_CTL_R = 0;
//	PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO|PWM_1_GENA_ACTLOAD_ONE;	// M1PWM2
//	PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO|PWM_1_GENB_ACTLOAD_ONE;	// M1PWM3
//	PWM1_1_LOAD_R = PWM_ZERO;
//	PWM1_1_CMPA_R = PWM_ZERO - 1;
//	PWM1_1_CMPB_R = PWM_ZERO - 1;
//	PWM1_2_CTL_R = 0;
//	PWM1_2_GENA_R = PWM_2_GENA_ACTCMPAD_ZERO|PWM_2_GENA_ACTLOAD_ONE;	// M1PWM4
//	PWM1_2_GENB_R = PWM_2_GENB_ACTCMPBD_ZERO|PWM_2_GENB_ACTLOAD_ONE;	// M1PWM5
//	PWM1_2_LOAD_R = PWM_ZERO;
//	PWM1_2_CMPA_R = PWM_ZERO - 1;
//	PWM1_2_CMPB_R = PWM_ZERO - 1;
	PWM1_3_CTL_R = 0;
	PWM1_3_GENA_R = PWM_3_GENA_ACTCMPAD_ZERO|PWM_3_GENA_ACTLOAD_ONE;	// M1PWM6
	PWM1_3_GENB_R = PWM_3_GENB_ACTCMPBD_ZERO|PWM_3_GENB_ACTLOAD_ONE;	// M1PWM7
	PWM1_3_LOAD_R = PWM_ZERO;
	PWM1_3_CMPA_R = PWM_ZERO - 1;
	PWM1_3_CMPB_R = PWM_ZERO - 1;
//	PWM1_1_CTL_R = PWM_1_CTL_ENABLE;
//	PWM1_2_CTL_R = PWM_2_CTL_ENABLE;
	PWM1_3_CTL_R = PWM_3_CTL_ENABLE;
	PWM1_ENABLE_R = PWM_ENABLE_PWM6EN|PWM_ENABLE_PWM7EN;
	ipl |= IPL_PWM1INIT;

	// init ADC
	adc_init();
	ipl |= IPL_ADC0INIT;

	// init QEI
/*	SYSCTL_RCGCQEI_R = SYSCTL_RCGCQEI_R1|SYSCTL_RCGCQEI_R0;			// enable clock to qei1/0
	GPIO_PORTC_AFSEL_R |= ENC1A|ENC1B;					// enable alt fn
	GPIO_PORTD_AFSEL_R |= ENC0A|ENC0B;					// enable alt fn
	GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC6_M|GPIO_PCTL_PC5_M);							// PB4 = port pin for IC
	GPIO_PORTC_PCTL_R |= (GPIO_PCTL_PC6_PHB1|GPIO_PCTL_PC5_PHA1);
	GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD7_M|GPIO_PCTL_PD6_M);							// PB4 = port pin for IC
	GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD7_PHB0|GPIO_PCTL_PD6_PHA0);
	QEI0_CTL_R = QEI_CTL_RESMODE|QEI_CTL_CAPMODE;				// cap mode = both edges, 0 = pha only
	QEI0_POS_R = 0x8000;							// 0x8000 = 0, 0x8001 = +1, 0x7fff = -1, ...
	QEI0_MAXPOS_R = 0x0000ffff;
	QEI1_CTL_R = QEI_CTL_RESMODE|QEI_CTL_CAPMODE;
	QEI1_POS_R = 0x8000;
	QEI1_MAXPOS_R = 0x0000ffff;
	QEI0_CTL_R = QEI_CTL_ENABLE;
	QEI1_CTL_R = QEI_CTL_ENABLE;
//	read position:
//	x = QEI0_POS_R;
//	rot dir:
//	dir = QEI0_STAT_R | QEI_STAT_DIRECTION; // 0 = fwd, 1 = reverse
//	dir = QEI0_STAT_R | QEI_STAT_ERROR; // 0 = OK, 1 = error in gray code
*/
	// init EEPROM
	ipl |= eeprom_init();
	do_beep(1);

	return ipl;
}

//*****************************************************************************
// hib_init()
//  initializes the HIB peripheral
//	init_switch == 0, turns on HIB osc only
//	else, remainder of HIB init (only allowed if timer interrupts enabled)
//
//*****************************************************************************
U16 hib_init(U8 init_switch){
	volatile uint32_t ui32Loop;
	U16	ipl = 0;

//	if(!init_switch){
	    SYSCTL_RCGCHIB_R |= 0x01;
	    ui32Loop = HIB_CTL_R;							// dummy read
		if(!(ui32Loop & HIB_CTL_CLK32EN)){				// if CLK32EN == 0, this is the first POC, turn on HIB osc
		    HIB_IM_R = HIB_IM_WC;						// enable WC mask
		    if((ui32Loop & HIB_CTL_WRC) == 0){
		    	ipl = IPL_HIBERR;						// if WC set, then there is an HIB error
		    }else{
				HIB_CTL_R = HIB_CTL_CLK32EN;			// enable HIB clock
				ipl = IPL_HIBINIT;							// init was completed on previous power cycle
		    }
		}else{
			ipl = IPL_HIBINIT;							// init was completed on previous power cycle
		}
/*	}else{
		wait(1500);										// wait 1500ms for HIB osc to start
		while(!(HIB_CTL_R & HIB_CTL_WRC));				// make sure WC is clear
		HIB_CTL_R = HIB_CTL_PINWEN | HIB_CTL_CLK32EN | HIB_CTL_VBATSEL_2_1V;	// enable /WAKE control
		while(!(HIB_CTL_R & HIB_CTL_WRC));				// wait for WC to clear
		ipl = IPL_HIBINIT;								// set init complete
	}*/
	return ipl;
}
