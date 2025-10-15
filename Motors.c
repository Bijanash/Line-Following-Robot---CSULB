// Motor.c
// Line Following Robot Starter program 

// February 2025
// CECS 346 SPRING 2025
// California State University, Long Beach

// 4/25/2025


#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "Motors.h"
#include "Systick.h"

// bits 6,7 are PWM outputs connected to DC Motor: bit 6->left motor, bit 7->right motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
#define MOTOR_PINS  0x00



// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor


void Motor_Init(void){

  SYSCTL_RCGCGPIO_R |= 0x02;            // Enable clock for Port B
  while ((SYSCTL_RCGCGPIO_R & 0x02) == 0) {} // Wait for Port B to be ready

  GPIO_PORTB_AMSEL_R &= ~0xFC;          // Disable analog on PB7-PB2
  GPIO_PORTB_PCTL_R &= ~0xFFFFFF00;     // Clear PCTL (GPIO on PB7-PB2)
  GPIO_PORTB_DIR_R |= 0xFC;             // Set PB7-PB2 as outputs
  GPIO_PORTB_AFSEL_R &= ~0xFC;          // Disable alt functions on PB7-PB2
  GPIO_PORTB_DEN_R |= 0xFC;             // Enable digital I/O on PB7-PB2
		
  //MOTORS &= ~0xC0;                      // Initially set PWM (PB7,6) low
  //DIRECTION &= ~0x3C;                   // Initially clear direction bits (PB5–PB2)
		
//	DIRECTION = FORWARD;
//	pwm = PWM_RIGHT|PWM_LEFT;
}


