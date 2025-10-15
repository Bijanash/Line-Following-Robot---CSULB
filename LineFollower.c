// LineFollower.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Motors.h"
#include "Sensor.h"
#include "LED.h"
#include <stdint.h>

//#define IRSensor 		(*((volatile uint32_t *)0x40007040))
//safe_to_move = true;

// Function prototypes
void System_Init(void);
//void IRSensor_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  uint16_t delay;              // Delay in ms
  uint8_t next[9];   					 // next state // make 9 STATES
};
typedef const struct State State_t;

enum states {Center1,CenterStop,Left1,Left2,LeftStop,Right1,Right2,RightStop,Stop}; //1 declares f 2 medium 3 slow

State_t linefollower_fsm[]={
	//TODO: Fill out FSM
  // Straight
	// Left
	// Right
  // Stop
	{FORWARD, 4,  {CenterStop,Right1,Left1,Stop} },	//Center
	{FORWARDSTOP, 300,  {Center1,Right1,Left1,Stop} },	//Center
	
	{TURN_LEFT,    3,  {Center1,Left2,Right1,Stop} },   //LEFT fastest
	{LEFTSTOP,    250,  {Center1,LeftStop,Right1,Stop} },   //LEFT medium speed
	{TURN_LEFT,    1,  {Center1,Left1,Right1,Stop} },   //LEFT slow
	
	{TURN_RIGHT,  3,  {Center1,Left1,Right2,Stop} },   //RIGHT
	{RIGHTSTOP,  250,  {Center1,Left1,RightStop,Stop} },   //RIGHT
	{TURN_RIGHT,  1,  {Center1,Left1,Right1,Stop} },   //RIGHT
	
	{STOP,   40,  {Center1,Left1,Right2,Stop} }       //STOP         //STOP
};

enum states curr_s;   // Initial state
uint8_t Input;

int main(void){
	uint8_t Input;
	//Sensor_Init();
	//IR_Sensor_Init()
	//SysTick_Init();
	//Motor_Init();
	System_Init();
	//Input = Sensor_CollectData();  // Only use PE1 and PE0 bits
	//curr_s = linefollower_fsm[curr_s].next[Input];
	//safe_to_move = true;
//Enable_Interrupts();
	
	
	//TODO: Fill out starting state
	
	//curr_s = Stop;
	
	while (1) {

		//TODO: Fill out FSM Engine	
			MOTORS = linefollower_fsm[curr_s].motors;
			Wait_N_MS(linefollower_fsm[curr_s].delay);
			Input = Sensor_CollectData();  // Only use PE1 and PE0 bits
			curr_s = linefollower_fsm[curr_s].next[Input];
	}
	}


void System_Init(void){
	Sensor_Init();
	Motor_Init();
	LED_Init();
	SysTick_Init();
	//IRSensor_Init();
	
}
/*void IRSensor_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD;
    while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGC2_GPIOD) !=SYSCTL_RCGC2_GPIOD){}
        
        GPIO_PORTD_DIR_R &= ~0x40;
        GPIO_PORTD_AFSEL_R &= ~0x40;
        GPIO_PORTD_DEN_R |= 0x40;
        GPIO_PORTD_PCTL_R &=0x0F000000;
        GPIO_PORTD_AMSEL_R = 0;
        GPIO_PORTD_PUR_R |= 0x40;

        //interrupt logic
        GPIO_PORTD_IS_R &= ~0x40;
        GPIO_PORTD_IBE_R |= 0x40;
        GPIO_PORTD_IM_R |= 0x40;
        GPIO_PORTD_ICR_R |= 0x40;
        NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|0x60000000; //setting this to priority 3 , prio bits = 31-29
        NVIC_EN0_R |= 0x00000008; //interrupt number: 3
}

	void GPIOPortD_Handler(void) 
{
    GPIO_PORTD_ICR_R |= 0x40; //acknowledge flag
    //check port_data_r register to check rising or falling edge  
    safe_to_move = true;
		}
*/