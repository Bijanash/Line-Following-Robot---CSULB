// Motors.h
// Line Following Robot Starter program 

// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach

// 4/25/2025

#include "tm4c123gh6pm.h"
#include <stdint.h>

// Define motor outputs for different motions.
// PB7 for right motor and PB6 for left motor.
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
// SLP|DIR: 11: forward
// SLP|DIR: 10: backward
// TODO: find the constant values
#define FORWARD 		0xFC   // PWM(bits7,6):11, direction:0x3C
#define FORWARDSTOP 0x00
#define TURN_LEFT 	0xBC	  // PWM(bits7,6):10, direction:0x3C
#define LEFTSTOP    0x00
#define TURN_RIGHT 	0x7C   // PWM(bits7,6):01, direction:0x3C
#define RIGHTSTOP   0x00
#define STOP   			0x00


// bit address definitions for port data registers
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
// TODO: find the bit addresses
#define MOTORS (*((volatile uint32_t *)0x40005300)) 
#define DIRECTION (*((volatile uint32_t *)0x400050F0))
	
static uint8_t pwm;  // two PWM signals on bits 7,6

#define PWM_LEFT  0x80
#define PWM_RIGHT 0x40


// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(void);

void Motor_Start(void);
	
