#include "tb6612.h"

// Set ports for PWM, IN1, IN2 (STBY?) 
uint8_t motor1_init() {

    DDRB |= (1 << DDB1) | (1 << DDB0);
    DDRD |= (1 << DDD6);

    TCCR1A |= (1 << WGM11); 					// fast pwm, using ICR1 as TOP
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS12); 		                // 256 prescale --> 2MHz clock
    ICR1    = 1025;  				    // TOP --> 1.6kHz PWM frequency
    TCCR1A |= (1 << COM1A1); 					// clear on compare match, set at bottom
    OCR1A   = 0;    							// set it to stopped, initially 
    DDRB   |= (1 << DDB1); 					// set PB1/OC1A to output

    return 1;

}

uint8_t motor2_init() {

    DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD3);	// Outputs to driver

    TCCR2A |= (1<<COM2B1) | (1 << WGM22) | (1<<WGM21) | (1<<WGM20);
    TCCR2B |= (1<<CS22) | (1 << CS21) | (1 << CS20);
    OCR2B |= 0;
    DDRD |= (1 << DDD3);

    return 1;

}

uint8_t motor1_mode(uint8_t direction) {

    uint8_t TB6612_AIN1 = 0;
    uint8_t TB6612_AIN2 = 0;

    switch(direction) {
		case FWD:
			TB6612_AIN1 = 1;
			TB6612_AIN2 = 0;	// CW
			break;
		case REV:
			TB6612_AIN1 = 0;
			TB6612_AIN2 = 1;	// CCW
			break;
		case STOP:
			TB6612_AIN1 = 1;
  			TB6612_AIN2 = 1;		// STOP mode
			break;
        default:
            return 0;
            break;
    }
    PORTB = (PORTB & ~(1 << PB0)) | (TB6612_AIN1 << PB0); // Clear and set Bits
	PORTD = (PORTD & ~(1 << PD6)) | (TB6612_AIN2 << PD6); // Clear and set Bits
    return 1;
}

uint8_t motor2_mode(uint8_t direction) {

    uint8_t TB6612_AIN1 = 0;
    uint8_t TB6612_AIN2 = 0;

    switch(direction) {
		case FWD:
			TB6612_AIN1 = 1;
			TB6612_AIN2 = 0;	// CW
			break;
		case REV:
			TB6612_AIN1 = 0;
			TB6612_AIN2 = 1;	// CCW
			break;
		case STOP:
			TB6612_AIN1 = 1;
  			TB6612_AIN2 = 1;		// STOP mode
			break;
        default:
            return 0;
            break;
    }
	PORTD = (PORTD & ~((1 << PD4) | (1 << PD2))) | (TB6612_AIN2 << PD2) | (TB6612_AIN1 << PD4); // Clear and set Bits
    return 1;
}

// Include a failsafe for FWD⟷REV? 
uint8_t motor1_speed(uint16_t speed) {

    // Hold the pulse time
    uint16_t onTime = 0;

    uint16_t PWM_timer_MAX = 1250;

    // Control check for speed
    if (speed < 0 || speed > 1024) {
        // Out of bounds
        return 0; // End the function

    } else if (speed < REV_LIMIT) {
        // Potentiometer is on the left side of the treshold
        motor1_mode(REV);                                                    // Motor movement REVERSE
        onTime = (REV_LIMIT - (uint32_t)speed) * PWM_timer_MAX / REV_LIMIT;       // Motor ON Time
        
    } else if (speed > FWD_LIMIT) {
        // Potentiometer is on the right side of the treshold
        motor1_mode(FWD);                                                    // Motor movement FORWARD
        onTime = ((uint32_t)speed - FWD_LIMIT) * PWM_timer_MAX / FWD_LIMIT; // Motor ON Time

    } else {
        // Potentiometer is on the treshold
        motor1_mode(STOP);                                                   // Motor movement STOP
        onTime = 0;                                                         // Motor ON Time
    }


    onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time
        
    OCR1A = onTime;

    return 1;
}

// Include a failsafe for FWD⟷REV? 
uint8_t motor2_speed(uint16_t speed) {

    // Hold the pulse time
    uint16_t onTime = 0;

    uint16_t PWM_timer_MAX = 256;

    // Control check for speed
    if (speed < 0 || speed > 1024) {
        // Out of bounds
        return 0; // End the function

    } else if (speed < REV_LIMIT) {
        // Potentiometer is on the left side of the treshold
        motor1_mode(REV);                                                    // Motor movement REVERSE
        onTime = (REV_LIMIT - (uint32_t)speed) * PWM_timer_MAX / REV_LIMIT;       // Motor ON Time
        
    } else if (speed > FWD_LIMIT) {
        // Potentiometer is on the right side of the treshold
        motor1_mode(FWD);                                                    // Motor movement FORWARD
        onTime = ((uint32_t)speed - FWD_LIMIT) * PWM_timer_MAX / FWD_LIMIT; // Motor ON Time

    } else {
        // Potentiometer is on the treshold
        motor1_mode(STOP);                                                   // Motor movement STOP
        onTime = 0;                                                         // Motor ON Time
    }


    onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time
	
    OCR2B = onTime;

    return 1;
}