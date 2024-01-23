// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "PLL.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// INPUT REGISTERS
// Port E
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002401C)) // bits 2-0
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SENSOR                  (*((volatile unsigned long *)0x4002401C))

// OUTPUT REGISTERS
// Port B
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control
#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
// Port F
#define GPIO_PORTF_OUT          (*((volatile unsigned long *)0x400250FC)) // bits 5-0
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define PED_LIGHT               (*((volatile unsigned long *)0x40025028))	// PF bit3,1

// General Clock
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

// ***** 3. Subroutines Section *****
// Linked data structure
struct State {
  unsigned long tOut;
	unsigned long pOut;	
  unsigned long Time;  
  unsigned long Next[8];}; 

typedef const struct State STyp;
#define goW   	0
#define waitW 	1
#define goS   	2
#define waitS 	3
#define goP			4
#define flash1	5
#define wait1		6
#define flash2	7
#define wait2		8
#define flash3	9
#define wait3		10
	
STyp FSM[11]={
{0x0C,0x2,300,{goW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},
{0x14,0x2,50,{goS,goS,goS,goS,goP,goP,goS,goS}},
{0x21,0x2,300,{goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},
{0x22,0x2,50,{goW,goW,goW,goW,goP,goP,goP,goP}},
{0x24,0x8,300,{goP,wait1,wait1,wait1,goP,wait1,wait1,wait1}},
{0x24,0x2,50,{wait1,wait1,wait1,wait1,wait1,wait1,wait1,wait1}},
{0x24,0x0,50,{flash2,flash2,flash2,flash2,flash2,flash2,flash2,flash2}},
{0x24,0x2,50,{wait2,wait2,wait2,wait2,wait2,wait2,wait2,wait2}},
{0x24,0x0,50,{flash3,flash3,flash3,flash3,flash3,flash3,flash3,flash3}},
{0x24,0x2,50,{wait3,wait3,wait3,wait3,wait3,wait3,wait3,wait3}},
{0x24,0x0,50,{wait3,goW,goS,goW,goP,goW,goS,goW}}
};

unsigned long S;  // index to the current state 
unsigned long Input; 
int main(void){ volatile unsigned long delay;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210, ScopeOff);
	PLL_Init();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
  
	SYSCTL_RCGC2_R |= 0x32;      // 1) F E B
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
	GPIO_PORTF_AMSEL_R &= ~0x0A; // 3) disable analog function on PF3-1
  GPIO_PORTF_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
	GPIO_PORTF_DIR_R |= 0x0A;     // 6) outputs on PF3, PF1
  GPIO_PORTF_AFSEL_R &= ~0x0A; // 7) regular function on PF3-1
  GPIO_PORTF_DEN_R |= 0x0A;    // 8) enable digital on PF3-1
  
	GPIO_PORTE_AMSEL_R &= ~0x07; // 9) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 10) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 11) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 12) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 13) enable digital on PE2-0
  
	GPIO_PORTB_AMSEL_R &= ~0x3F; // 14) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 15) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 16) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 17) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 18) enable digital on PB5-0
  S = goW;  
  while(1){
    LIGHT = FSM[S].tOut;  // set lights
		PED_LIGHT = FSM[S].pOut; // set Pedestrian lights
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];  
  }
}
