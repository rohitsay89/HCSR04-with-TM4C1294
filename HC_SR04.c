#include <stdint.h>
#include <stdbool.h>
#include "UART.h"
#include <stdio.h>
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"

/*
 * This code is for interfacing the TM4C1294 Tiva C series microcontroller 
 * with the HC-SR04 ultrasonic trasducer pair sensor. The interrupt modules 
 * of the code are derived from http://users.ece.utexas.edu/~valvano/arm/ 
 * 
 * The interfacing of the tiva module with the sensor is as follows:
 * 		HC-SR04					  TM4C1294
 * 		  Vcc	 ===============> 	+5v
 * 		  Gnd 	 ===============> 	Gnd
 * 		 Trigger ===============> 	PN2 (Port N pin 2)
 * 		  Echo	 ===============> 	PN3 (Port N pin 3) 
 * 
 * contact me at rohitsay89@hotmail.com for further questions
*/

volatile unsigned int count = 0;							/* no of counts after +edge of echo */

void PortFunctionInit(void){
	volatile uint32_t ui32Loop;   
	/* Enable the clock of the GPIO port N that is used for the on-board LED. */
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
	/* Do a dummy read to insert a few cycles after enabling the peripheral. */
	ui32Loop = SYSCTL_RCGCGPIO_R;
	GPIO_PORTN_DIR_R |= 0x04;						      /* make PN 2 as output to sensor trigger pin */
	GPIO_PORTN_DEN_R |= 0x04;						      /* enable data I/O on PN 2 */
	GPIO_PORTN_DIR_R &= ~0x08;						    /* make PN 3 as input from sensor echo pin */
	GPIO_PORTN_DEN_R |= 0x08;					      	/* Enable data I/O on PN3 */
	GPIO_PORTN_DIR_R |= 0x01;						      /* PN0 LED direction output */
	GPIO_PORTN_DEN_R |= 0x01;						      /* PN0 LED */
	GPIO_PORTN_AMSEL_R &= ~0x0D;   					  /* disable analog function on PN7-0 */
	GPIO_PORTN_PCTL_R = 0x00000000;   				/* configure PN7-0 as GPIO */  
	GPIO_PORTN_AFSEL_R = 0x00;    					  /* regular port function */
	GPIO_PORTN_DEN_R = 0xFF;       					  /* enable digital port */
}

void IntGlobalEnable(void){
    __asm("    cpsie   i\n");					    	/* Globally enable interrupts */
}

void Interrupt_Init(void){
	NVIC_EN2_R |= 0x00000200;  		            /* enable interrupt (IRQ number) 73 in NVIC (GPIO PORTN) */
	NVIC_PRI18_R = 0x00000000; 		            /* configure GPIO PORTN interrupt priority as 0 [4n+1] format for 73 = [4*18 + 1] */
	GPIO_PORTN_IM_R |= 0x08;   		            /* arm interrupt on PN3 */
	GPIO_PORTN_IS_R &= ~0x08;		              /* PN3 is edge-sensitive */
	GPIO_PORTN_IBE_R &= ~0x08;   	            /* PN3 both edges trigger */
	GPIO_PORTN_IEV_R |= 0x08;		              /* PN3 rising edge event */
	IntGlobalEnable();        		            /* globally enable interrupt */
}

void Timer0A_Init(unsigned long period){		/* Timer 0A initialization */
	volatile uint32_t ui32Loop; 
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;		/* activate timer0 */
	ui32Loop = SYSCTL_RCGCTIMER_R;	 				  /* Do a dummy read to insert a few cycles after enabling the peripheral. */
	TIMER0_CTL_R &= ~0x00000001;     				  /* disable timer0A during setup */
	TIMER0_CFG_R = 0x00000000;       				  /* configure for 32-bit timer mode */
	TIMER0_TAMR_R = 0x00000002;               /* configure for periodic mode, default down-count settings */
	TIMER0_TAILR_R = period-1;                /* reload value */
	NVIC_PRI4_R |= 0x20000000; 	 	            /* configure Timer0A interrupt priority as 1 */
	NVIC_EN0_R |= 0x00080000;     	          /* enable interrupt 19 in NVIC (Timer0A) */
	TIMER0_IMR_R |= 0x00000001;               /* arm timeout interrupt */
	TIMER0_CTL_R |= 0x00000001;               /* enable timer0A */
}

void Timer0A_Handler(void){				          /* Interrupt handler for Timer0A */
	TIMER0_ICR_R |= 0x00000001; 	            /* acknowledge flag for Timer0A */
	/* ================== Make a pulse ======================*/	
	GPIO_PORTN_DATA_R |=0x04;		
	SysCtlDelay(53);				                  /* to create 10us delay between pulses */
	GPIO_PORTN_DATA_R &=~0x04;
	UART_OutUHex(0xB);				                /* Print a character on UART */
}

/* Interrupt handler */
void GPIOPortN_Handler(void){	
	GPIO_PORTN_ICR_R |= 0x08;		              /* acknowledge flag for PN3 */
	while((GPIO_PORTN_DATA_R&0x08)!=0){			  /* if port N pin 3 become high */
		count++; 											          /* increment count */
	}
	UART_OutChar(LF);
	UART_OutUDec(count);
	if(count < 2000){
		GPIO_PORTN_DATA_R &= ~0x01;	          	/* blue led off (no person) PN0 */
	}
	else{
		GPIO_PORTN_DATA_R |= 0x01;	          	/* blue led on (person present) PN0 */
	}
	count = 0; 							                  /* make count 0 */
}

int main(void){
	unsigned long period = 960000; 	          /* reload value to Timer0A to generate 60ms delay */
	PortFunctionInit();				                /* initialize the GPIO ports */
	UART_Init();              		            /* initialize UART */
	Interrupt_Init();				                  /* Initialize hardware interrupt on PN3*/
	Timer0A_Init(period);			                /* initialize Timer0A and configure the interrupt */
	IntGlobalEnable();        		            /* globally enable interrupt */
    while(1)						                    /* Loop forever */
    {
		                                        /* Do nothing or anything of your choice */
    }
}
