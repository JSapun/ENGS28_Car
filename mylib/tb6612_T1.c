/* 
 * SevenSeg.c
 * Justin Sapun 
 * Device driver module for the tb6612 h-bridge motor controller.
 * This driver enables dual motor control.
 */

#include "tb6612_T1.h"

static uint8_t motor1_init() { // Timer 1 Compare A

    TCCR1A |= (1 << WGM11); 					// fast pwm, using ICR1 as TOP
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS12); 		                // 256 prescale --> 2MHz clock
    ICR1    = 1025;  				            // TOP --> 1.6kHz PWM frequency
    TCCR1A |= (1 << COM1A1); 					// clear on compare match, set at bottom
    OCR1A   = 0;    							// set it to stopped, initially 
    DDRB   |= (1 << DDB1); 					    // set PB1/OC1A to output

    return 1;
}

static uint8_t motor2_init() { // Timer 2 Compare B

	TCCR1A |= (1 << COM1B1); 				// clear on compare match, set at bottom
	OCR1B   = 0;    						// set it to stopped, initially 
	DDRB   |= (1 << DDB2); 					// set PB2/OC1B to output

    return 1;
}

uint8_t motors_init(){

    DDRD |= (1 << AIN1) | (1 << AIN2);	// Outputs to driver
    DDRB |= (1 << DDB1) | (1 << DDB2);
    motor1_init(); 
	motor2_init();

    PORTD = (PORTD & ~((1 << AIN2) | (1 << AIN1))) | (0 << AIN1) | (0 << AIN2); // Stop Mode
	motor1_speed(0);						                                // STOP mode
	motor2_speed(0);						                                // STOP mode

    return 1;
}

uint8_t motors_mode(uint8_t direction) {

    uint8_t TB6612_AIN1 = 0;
    uint8_t TB6612_AIN2 = 0;
    uint8_t failsafe = 0;

    switch(direction) {
		case FWD:
			if (failsafe != 2){		// FAILSAFE, will pass if braked previously
				TB6612_AIN1 = 0;
				TB6612_AIN2 = 1;	// CW
				failsafe = 1;
			}
			break;
		case REV:
			if (failsafe != 1){
				TB6612_AIN1 = 1;
				TB6612_AIN2 = 0;	// CCW
				failsafe = 2;
			}
			break;
		case BRAKE:
			TB6612_AIN1 = 1;
			TB6612_AIN2 = 1;		// brake
			failsafe = 0;
			break;
		case STOP:
			motor1_speed(0);
			motor2_speed(0);		
			TB6612_AIN1 = 0;
  			TB6612_AIN2 = 0;		// STOP mode
			break;
        default:
            return 0;
            break;
	}
	PORTD = (PORTD & ~((1 << AIN2) | (1 << AIN1))) | (TB6612_AIN2 << AIN1) | (TB6612_AIN1 << AIN2); // Clear and set Bits
    return 1;
}

// Set motor speed (PWM) for Timer 1 Compare A
uint8_t motor1_speed(uint16_t value) {

    if (value > PWM_TIMER1_MAX-1) 
		value = PWM_TIMER1_MAX-1;
	else if (value < 0)
		value = 0;		
	OCR1A = value;

    return 1;
}

// Set motor speed (PWM) for Timer 1 Compare B
uint8_t motor2_speed(uint16_t value) {

    if (value > PWM_TIMER1_MAX-1) 
		value = PWM_TIMER1_MAX-1;
	else if (value < 0)
		value = 0;		
	OCR1B = value;

    return 1;
}