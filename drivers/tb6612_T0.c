#include "tb6612_T0.h"

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

    TCCR2A |= (1<<COM2B1) | (1 << WGM22) | (1<<WGM21) | (1<<WGM20);
    TCCR2B |= (1<<CS22) | (1 << CS21) | (1 << CS20);
    OCR2B |= 0;
    DDRD |= (1 << DDD3);

    return 1;
}

uint8_t motors_init(){

    DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4);	// Outputs to driver
    DDRB |= (1 << DDB1);
    motor1_init(); 
	motor2_init();

    PORTD = (PORTD & ~((1 << PD4) | (1 << PD2))) | (0 << PD2) | (0 << PD4); // Stop Mode
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
			motor1_speed(0);
			motor2_speed(0);		
			TB6612_AIN1 = 0;
  			TB6612_AIN2 = 0;		// STOP mode
			break;
        default:
            return 0;
            break;
	}
	PORTD = (PORTD & ~((1 << PD4) | (1 << PD2))) | (TB6612_AIN2 << PD2) | (TB6612_AIN1 << PD4); // Clear and set Bits
    return 1;
}

// Set motor speed (PWM) for Timer 1 Compare A
uint8_t motor1_speed(uint16_t value) {

    if (value>PWM_TIMER1_MAX-1) 
		value = PWM_TIMER1_MAX-1;
	else if (value < 0)
		value = 0;		

    value = (CLOCK * value) / 256 * 15;                         // Prescale the ON Time
	OCR1A = value;

    return 1;
}

// Set motor speed (PWM) for Timer 2 Compare B
uint8_t motor2_speed(uint16_t value) {

    if (value>PWM_TIMER2_MAX-1) 
		value = PWM_TIMER2_MAX-1;
	else if (value < 0)
		value = 0;		

    value = (CLOCK * value) / 256 * 15;                         // Prescale the ON Time
	OCR2B = value;

    return 1;
}