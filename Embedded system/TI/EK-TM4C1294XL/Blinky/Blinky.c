//-----------------------------------------------------------------------*/

#include <stdio.h>
#include "TM4C129.h"                    // Device header
#include "Serial.h"
#include "LED.h"
#include "BTN.h"

//GPIO_PORTA ADDRESS DEFINE
#define SYSCTL_RCGCGPIO_R0      0x00000001  // GPIO Port J Run Mode Clock 
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRGPIO_R0      0x00000001
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x40058000))//PORT REGISTER=DATA 
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40058400))//DIRECTION
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40058420))//Alternate function select
#define GPIO_PORTA_PUR_R        (*((volatile uint32_t *)0x40058510))//pull-up select
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4005851C))//digital enable
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40058528))//analog mode select
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4005852C))//post control
//#define PA6       (*((volatile uint32_t *)0x4005801C))
//#define PA7       (*((volatile uint32_t *)0x40058020))
#include <stdint.h>

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

int buttons;
volatile uint32_t msTicks; 

void SysTick_Handler(void) {
  msTicks++;
}

void Delay (uint32_t dlyTicks) {//interrupt
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }//msTick2=0 beginning, curTick
}

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 8.333 nsec for 120 MHz clock)
void SysTick_Wait(uint32_t delay){
  volatile uint32_t elapsedTime;
  uint32_t startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}
// Time delay using busy wait.
// This assumes 120 MHz system clock.
void SysTick_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(1000000);  // wait 10ms (assumes 120 MHz clock)
  }
}
void SysTick_Init(void);

// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 8.333 nsec for 120 MHz clock)
void SysTick_Wait(uint32_t delay);

// Time delay using busy wait.
// This assumes 120 MHz system clock.
void SysTick_Wait10ms(uint32_t delay);
//--------------------------------------------------
volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/

void PortA_Init(void){
                                   // activate clock for Port A
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
                                   // allow time for clock to stabilize
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R0) == 0){};
  GPIO_PORTA_DIR_R = 0;            // make PA1-0 in (PJ1-0 built-in SW2-1)
  GPIO_PORTA_AFSEL_R = 0;          // disable alt funct on PJ1-0
  GPIO_PORTA_PUR_R |= 0xC0;                 //0xc0/0x03;         // enable pull-up on PA6,7//1-0
  GPIO_PORTA_DEN_R |= 0xC0;         // enable digital I/O on PA6,7//PJ1-0
  GPIO_PORTA_PCTL_R = 0;           // configure /PA6,7/PJ1-0 as GPIO
  GPIO_PORTA_AMSEL_R = 0;          // disable analog functionality on PJ1-0
}
void Port_Init(void){
	
//------------- port E 0-6 for output---------------------------------------------------                                   // activate clock for Port N  SYSCTL->PRGPIO |= (1UL << 4); 
  SYSCTL_RCGCGPIO_R |=  SYSCTL->RCGCGPIO |= (1UL << 4); 
                                   // allow time for clock to stabilize
  while((SYSCTL_PRGPIO_R&(SYSCTL->PRGPIO |= (1UL << 4))) == 0){};
  GPIOE_AHB->DIR  |= 0x3F;         // make PN1-0 out (PN1-0 built-in LED1-2)
  GPIOE_AHB->AFSEL &= ~0x3F;;          // disable alt funct on PN1-0
	GPIOE_AHB->PUR = 0x00; 
  GPIOE_AHB->DEN |= 0x3F;         // enable digital I/O on PN1-0
  GPIOE_AHB->PCTL = 0;           // configure PN1-0 as GPIO
  GPIOE_AHB->AMSEL &= ~0x3F;          // disable analog functionality on PN1-0
//---------------- port B 2-3 for inout   -------------------------------------------		
SYSCTL_RCGCGPIO_R |=  SYSCTL->RCGCGPIO |= (1UL << 1); 
                                   // allow time for clock to stabilize
  while((SYSCTL_PRGPIO_R&(SYSCTL->PRGPIO |= (1UL << 1))) == 0){};
  GPIOB_AHB->DIR  &= ~0x0C;         // make PE2-3 in (PN1-0 built-in LED1-2)
  GPIOB_AHB->AFSEL &= ~0x0C;;          // disable alt funct on PE23
	GPIOB_AHB->PUR = 0x0C; 
  GPIOB_AHB->DEN = 0x0C;         // enable digital I/O on PE-23
  GPIOB_AHB->PCTL = 0;           // configure PE23 as GPIO
  GPIOB_AHB->AMSEL &= ~0x0C;          // disable analog functionality on PE23	
}

void Port_Interupt_Init(void){
			/* Interrupt on Port E Pin 2/3 Initialization*/
	NVIC_EnableIRQ(GPIOB_IRQn); // Enable GPIOB
//	GPIOA_AHB->AFSEL |= (1UL << 5);
	//GPIOB_AHB->IS   &=  (1UL << 2);//((1UL << 2)|(1UL << 3));
	GPIOB_AHB->IS   |=  (1UL << 2);//((1UL << 2)|(1UL << 3));
	GPIOB_AHB->IBE  &= ~(1UL << 2);//((1UL << 2)|(1UL << 3));
	GPIOB_AHB->IEV  |=  (1UL << 2);//((1UL << 2)|(1UL << 3));
	GPIOB_AHB->IM		|=  (1UL << 2);//((1UL << 2)|(1UL << 3));
	GPIOB_AHB->ICR  |=  (1UL << 2);//((1UL << 2)|(1UL << 3));
}
 void GPIOB_Handler(void)
 {
	 	GPIOB_AHB->ICR  |=  ((1UL << 2));//|(1UL << 3));/*Clear the interrupt*/
		buttons=1;//(GPIOB_AHB->DATA&0x0C)>>2;
	 
 }
//------------PortA_Input------------
// Read and return the status of the switches.
// Input: none
// Output: 0x02 if only Switch 1 is pressed
//         0x01 if only Switch 2 is pressed
//         0x00 if both switches are pressed
//         0x03 if no switches are pressed


struct State {
  uint32_t Out;            // 6-bit output
  uint32_t Time;           // 10 ms
  uint32_t Next[4];};// depends on 2-bit input
typedef const struct State STyp;
#define goN   0	//&FSM[0]
#define waitN 1	//&FSM[1]
#define goE   2	//&FSM[2]
#define waitE 3	//&FSM[3]
STyp FSM[4]={
 {0x21,80,{goN,waitN,goN,waitN}},//100001
 {0x22, 40,{goE,goE,goE,goE}},//010010
 {0x0C,80,{goE,goE,waitE,waitE}},//001100
 {0x14,40,{goN,goN,goN,goN}}};//010100
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
  int main(void){
uint32_t Current_state;
  uint32_t Input;// 00 no cars in N E
								 // 01 car in E
								 // 10 car in N
								 // 11 car in N E
                 // configure for 120 MHz clock
  SysTick_Init();              // initialize SysTick timer
	Port_Init();
	Port_Interupt_Init();
	//SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  //SysTick_Config(SystemCoreClock / 1000ul);     /* Setup SysTick for 1 msec   */

  Current_state = goN;                    // initial state: Green north; Red east
  while(1){
		
		if (buttons==1){
			buttons=0;
			Current_state = goE; 
			GPIOE_AHB->DATA = FSM[Current_state].Out; 
			SysTick_Wait10ms(3*FSM[Current_state].Time);
			Current_state++;
			//Delay(3*FSM[Current_state].Time);
		}
		/*
		if (buttons==2){
			buttons=0;
			Current_state = goE; 
			GPIOE_AHB->DATA = FSM[Current_state].Out; 
			SysTick_Wait10ms(FSM[Current_state].Time);
		 // Delay(FSM[Current_state].Time);
		}
		*/
		GPIOE_AHB->DATA = FSM[Current_state].Out;  
		SysTick_Wait10ms(FSM[Current_state].Time);//  * current state's Time value
    
		//Delay(FSM[Current_state].Time);	
		Input = (GPIOB_AHB->DATA&0x0C)>>2;            // get new input from car detectors
    Current_state = FSM[Current_state].Next[Input];      // transition to next state
  }
}


	