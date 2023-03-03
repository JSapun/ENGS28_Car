/* 
 * HBridge.c
 * Justin Sapun 
 * Lab 5 - 02/02/2023
 * Device driver c file for the H-Bridge chip ___ for use with controlling motors.
 * Engs 28
 * B4 --> AIN2
 * B3 --> AIN1
 * B2 --> PWM B
 * B1 --> PWM A
 */

#include <avr/io.h>		// All the port definitions are here
#include "tb6612.h"		

#define PWM_TIMER_MAX	  1250		         // 1.6kHz with prescale 8 	

volatile uint8_t TB6612_AIN1;
volatile uint8_t TB6612_AIN2;			
volatile uint8_t failsafe = 0;

// Initialize Timer 1 for use with Pulse Width Modulation
static void PWMtimer_init(void) {
 	TCCR1A |= (1 << WGM11); 				// fast pwm, using ICR1 as TOP
  	TCCR1B |= (1 << WGM12) | (1 << WGM13); 
	TCCR1B |= (1 << CS11); 					// /8 prescale --> 2MHz clock
	ICR1    = PWM_TIMER_MAX;  				// TOP --> 1.6kHz PWM frequency
	TCCR1A |= (1 << COM1A1); 				// clear on compare match, set at bottom
	OCR1A   = 0;    						// set it to stopped, initially 
	DDRB   |= (1 << DDB1); 					// set PB1/OC1A to output 
}

// Set Timer 1 B for use with updating the Pulse Width on PORTB2
static void PWMtimer_set(uint16_t value) {
// Safety first: Allow no pulse values outside the (0, PWM_TIMER_MAX-1 range!
	if (value>PWM_TIMER_MAX-1) 
		value = PWM_TIMER_MAX-1;
	else if (value < 0)
		value = 0;		
	OCR1A = value;
}

static void PWMtimer2_init(void) {
 	TCCR1A |= (1 << WGM11); 				// fast pwm, using ICR1 as TOP
  	TCCR1B |= (1 << WGM12) | (1 << WGM13); 
	TCCR1B |= (1 << CS11); 					// /8 prescale --> 2MHz clock
	ICR1    = PWM_TIMER_MAX;  				// TOP --> 1.6kHz PWM frequency

	TCCR1A |= (1 << COM1B1); 				// clear on compare match, set at bottom
	OCR1B   = 0;    						// set it to stopped, initially 
	DDRB   |= (1 << DDB2); 					// set PB2/OC1B to output 
}

// Set Timer 1 B for use with updating the Pulse Width on PORTB1
static void PWMtimer2_set(uint16_t value) {
// Safety first: Allow no pulse values outside the (0, PWM_TIMER_MAX-1 range!
	if (value>PWM_TIMER_MAX-1) 
		value = PWM_TIMER_MAX-1;
	else if (value < 0)
		value = 0;		
	OCR1B = value;
}

// Initialize the H-Bridge Chip for Motor (Set ports for PWMA, PWMB, IN1, IN2 (STBY?))
void motor_init(void){	             
  	DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4);	// Outputs to driver
	PWMtimer1_init(); 
	PWMtimer2_init();						

	PORTB = (PORTB & ~((1 << PB3) | (1 << PB4))) | (0 << PB4) | (0 << PB3); 
	PWMtimer1_set(0);						// STOP mode
	PWMtimer2_set(0);						// STOP mode
}

void motor_mode(direction_t direction){           // FWD, REV, BRAKE, STOP (STBY?), and a failsafe for FWD⟷REV?
	switch(direction) {
		case FWD:
			if (failsafe != 2){		// FAILSAFE, will pass if braked previously
				TB6612_AIN1 = 1;
				TB6612_AIN2 = 0;	// CW
				failsafe = 1;
			}
			break;
		case REV:
			if (failsafe != 1){
				TB6612_AIN1 = 0;
				TB6612_AIN2 = 1;	// CCW
				failsafe = 2;
			}
			break;
		case BRAKE:
			TB6612_AIN1 = 1;
			TB6612_AIN2 = 1;		// brake
			failsafe = 0;
			break;
		case STOP:
			PWMtimer1_setA(0);
			PWMtimer1_setB(0);		
			TB6612_AIN1 = 0;
  			TB6612_AIN2 = 0;		// STOP mode
			break;
	}
	PORTB = (PORTB & ~((1 << PB3) | (1 << PB4))) | (TB6612_AIN2 << PB4) | (TB6612_AIN1 << PB3); // Clear and set Bits
}

// Set speed of motor (Interacts with Timer1)
void motor_speed(uint16_t pwm_mag){ 
	PWMtimer1_set(pwm_mag);		// Update the PWM A
}

// Set speed of motor (Interacts with Timer1)
void motor_speed(uint16_t pwm_mag){ 
	PWMtimer2_set(pwm_mag);		// Update the PWM B
}