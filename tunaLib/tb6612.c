#include "tb6612.h"

// Set ports for PWM, IN1, IN2 (STBY?) 
uint8_t motor_init(uint8_t timer) {

    if (timer == TIMER0) {

        DDRD |= (1 << PWM_0) | (1 << IN1_0) | (1 << IN2_0);	// Outputs to driver

        TCCR0A |= (1 << COM0A1);                   // clear on compare match, set at bottom
        TCCR0A |= (1 << WGM02) | (1 << WGM01) | (1 << WGM00); // fast pwm, using OCR0A as TOP
        TCCR0A |= (1 << CS01) | (1 << CS00);       // 64 Prescaler
        OCR0A = 0;
        DDRD |= (1 << PWM_0);

        return 1;

    } else if (timer == TIMER1) {

        DDRB |= (1 << PWM_1) | (1 << IN1_1) | (1 << IN2_1);	// Outputs to driver

        TCCR1A |= (1 << WGM11); 					// fast pwm, using ICR1 as TOP
        TCCR1B |= (1 << WGM12) | (1 << WGM13);
        TCCR1B |= (1 << CS12); 		                // 256 prescale --> 2MHz clock
        ICR1    = PWM_TIMER_MAX;  				    // TOP --> 1.6kHz PWM frequency
        TCCR1A |= (1 << COM1A1); 					// clear on compare match, set at bottom
        OCR1A   = 0;    							// set it to stopped, initially 
        DDRB   |= (1 << PWM_1); 					// set PB1/OC1A to output

        return 1;

    } else if (timer == TIMER2) {

        DDRD |= (1 << PWM_2) | (1 << IN1_2) | (1 << IN2_2);	// Outputs to driver

        TCCR2A = (1<<COM2B1) | (1 << WGM22) | (1<<WGM21) | (1<<WGM20);
        TCCR2B = (1<<CS22);
        OCR2B = 0;
        DDRD |= (1 << PWM_2);

        return 1;

    } else {
        return 0;
    }

}

// FWD, REV, BRAKE, STOP (STBY?) 
uint8_t motor_mode(uint8_t mode, uint8_t timer) {

    if (timer == TIMER0) {

        // Reset the IN1 and IN2 Pins
        PORTD &= ~((1 << PD7) | (1 << PD5));

        // Select the mode
        if (mode == REV) {
            PORTD |= REV_0;           // Set PORTD to 0b10000000 => Turn ON IN1, Turn OFF IN2
        } else if (mode == STOP) {
            PORTD |= STOP_0;          // Set PORTD to 0b10100000 => Turn ON IN1, Turn ON IN2
        } else if (mode == FWD) {
            PORTD |= FWD_0;           // Set PORTD to 0b00100000 => Turn OFF IN1, Turn ON IN2
        } else {
            return 0;
        }

    } else if (timer == TIMER1) {

        // Reset the IN1 and IN2 Pins
        PORTB &= ~((1 << PB2) | (1 << PB3));

        // Select the mode
        if (mode == REV) {
            PORTB |= REV_1;           // Set PORTB to 0b00001000 => Turn ON IN1, Turn OFF IN2
        } else if (mode == STOP) {
            PORTB |= STOP_1;          // Set PORTB to 0b00001100 => Turn ON IN1, Turn ON IN2
        } else if (mode == FWD) {
            PORTB |= FWD_1;           // Set PORTB to 0b00000100 => Turn OFF IN1, Turn ON IN2
        } else {
            return 0;
        }

    } else if (timer == TIMER2) {

        // Reset the IN1 and IN2 Pins
        PORTD &= ~((1 << PD4) | (1 << PD2));

        // Select the mode
        if (mode == REV) {
            PORTD |= REV_2;           // Set PORTD to 0b00010000 => Turn ON IN1, Turn OFF IN2
        } else if (mode == STOP) {
            PORTD |= STOP_2;          // Set PORTD to 0b00010100 => Turn ON IN1, Turn ON IN2
        } else if (mode == FWD) {
            PORTD |= FWD_2;           // Set PORTD to 0b00000100 => Turn OFF IN1, Turn ON IN2
        } else {
            return 0;
        }
    }

    return 1;
    
}

// Include a failsafe for FWD⟷REV? 
uint8_t motor_speed(uint16_t ADC_value, uint8_t timer){

    // Hold the pulse time
    uint16_t onTime = 0;

    uint16_t PWM_timer_MAX = 0;

    if (timer == TIMER0 || timer == TIMER2) {
        PWM_timer_MAX = 256;
    } else if (timer == TIMER1) {
        PWM_timer_MAX = PWM_TIMER_MAX;
    }

    // Control check for ADC_value
    if (ADC_value < 0 || ADC_value > 1024) {
        // Out of bounds
        return 0; // End the function

    } else if (ADC_value < REV_LIMIT) {
        // Potentiometer is on the left side of the treshold
        motor_mode(REV, timer);                                                    // Motor movement REVERSE
        onTime = (REV_LIMIT - (uint32_t)ADC_value) * PWM_timer_MAX / REV_LIMIT;       // Motor ON Time
        
    } else if (ADC_value > FWD_LIMIT) {
        // Potentiometer is on the right side of the treshold
        motor_mode(FWD, timer);                                                    // Motor movement FORWARD
        onTime = ((uint32_t)ADC_value - FWD_LIMIT) * PWM_timer_MAX / FWD_LIMIT; // Motor ON Time

    } else {
        // Potentiometer is on the treshold
        motor_mode(STOP, timer);                                                   // Motor movement STOP
        onTime = 0;                                                         // Motor ON Time
    }

    if (onTime>PWM_TIMER_MAX-1) {
        onTime = PWM_TIMER_MAX-1;
    } else if (onTime < 0) {
        onTime = 0;		
    }

    if (timer == TIMER0) {

        onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time
	
        OCR0A = onTime;

    } else if (timer == TIMER1) {

        onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time
        
        OCR1A = onTime;

    } else if (timer == TIMER2) {

        onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time
	
        OCR2B = onTime;

    } else {
        return 0;
    }

    return 1;
    
}