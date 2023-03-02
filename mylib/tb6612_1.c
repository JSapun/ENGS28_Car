/*#include "tb6612.h"

// Set ports for PWM, IN1, IN2 (STBY?) 
void motor_init(void) {

    DDRB |= (1 << PWM) | (1 << IN1) | (1 << IN2);	// Outputs to driver

    TCCR1A |= (1 << WGM11); 					// fast pwm, using ICR1 as TOP
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS12); 		                // 256 prescale --> 2MHz clock
    ICR1    = PWM_TIMER_MAX;  				    // TOP --> 1.6kHz PWM frequency
    TCCR1A |= (1 << COM1A1); 					// clear on compare match, set at bottom
    OCR1A   = 0;    							// set it to stopped, initially 
    DDRB   |= (1 << PWM); 					    // set PB1/OC1A to output
}

// FWD, REV, BRAKE, STOP (STBY?) 
void motor_mode(uint8_t mode) {

    // Reset the IN1 and IN2 Pins
    PORTB &= ~((1 << PB2) | (1 << PB3));

    // Select the mode
    if (mode == REV) {
        PORTB |= REV;           // Set PORTB to 0b00001000 => Turn ON IN1, Turn OFF IN2
    } else if (mode == STOP) {
        PORTB |= STOP;          // Set PORTB to 0b00001100 => Turn ON IN1, Turn ON IN2
    } else {
        PORTB |= FWD;           // Set PORTB to 0b00000100 => Turn OFF IN1, Turn ON IN2
    }
    
}

// Include a failsafe for FWD⟷REV? 
uint16_t motor_speed(uint16_t pwm_value){

    // Hold the pulse time
    uint16_t onTime = 0;

    // Control check for pwm_value
    if (pwm_value < 0 || pwm_value > 1024) {
        // Out of bounds
        return 0; // End the function

    } else if (pwm_value < REV_LIMIT) {
        // Potentiometer is on the left side of the treshold
        motor_mode(REV);                                                    // Motor movement REVERSE
        onTime = (REV_LIMIT - (uint32_t)pwm_value) * PWM_TIMER_MAX / REV_LIMIT;       // Motor ON Time
        
    } else if (pwm_value > FWD_LIMIT) {
        // Potentiometer is on the right side of the treshold
        motor_mode(FWD);                                                    // Motor movement FORWARD
        onTime = ((uint32_t)pwm_value - FWD_LIMIT) * PWM_TIMER_MAX / FWD_LIMIT; // Motor ON Time

    } else {
        // Potentiometer is on the treshold
        motor_mode(STOP);                                                   // Motor movement STOP
        onTime = 0;                                                         // Motor ON Time
    }

    onTime = (CLOCK * onTime) / 256 * 15;                         // Prescale the ON Time

    if (onTime>PWM_TIMER_MAX-1) 
		onTime = PWM_TIMER_MAX-1;
	else if (onTime < 0)
		onTime = 0;		
	OCR1A = onTime;

    return onTime;
}*/

#include "tb6612.h"

// Set ports for PWM, IN1, IN2 (STBY?) 
void motor_init(void) {
    DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD2);	// Outputs to driver

    TCCR2A = (1<<COM2B1) | (1<<COM2B0) | (1<<WGM21) | (1<<WGM20);
    TCCR2B = (1<<CS22) | (1<<CS20);
    OCR2B = 0;
    DDRD |= (1 << DDD3);

}

// FWD, REV, BRAKE, STOP (STBY?) 
void motor_mode(uint8_t mode) {

    // Reset the IN1 and IN2 Pins
    PORTD &= ~((1 << PD4) | (1 << PD2));

    // Select the mode
    if (mode == REV) {
        PORTB |= 0x10;           // Set PORTB to 0b00010000 => Turn ON IN1, Turn OFF IN2
    } else if (mode == STOP) {
        PORTB |= 0x14;          // Set PORTB to 0b00010100 => Turn ON IN1, Turn ON IN2
    } else {
        PORTB |= 0x04;           // Set PORTB to 0b00000100 => Turn OFF IN1, Turn ON IN2
    }
    
}

// Include a failsafe for FWD⟷REV? 
uint16_t motor_speed(uint16_t pwm_value){

    // Hold the pulse time
    uint16_t onTime = 0;

    // Control check for pwm_value
    if (pwm_value < 0 || pwm_value > 1024) {
        // Out of bounds
        return 0; // End the function

    } else if (pwm_value < REV_LIMIT) {
        // Potentiometer is on the left side of the treshold
        motor_mode(REV);                                                    // Motor movement REVERSE
        onTime = (REV_LIMIT - (uint32_t)pwm_value) * PWM_TIMER_MAX / REV_LIMIT;       // Motor ON Time
        
    } else if (pwm_value > FWD_LIMIT) {
        // Potentiometer is on the right side of the treshold
        motor_mode(FWD);                                                    // Motor movement FORWARD
        onTime = ((uint32_t)pwm_value - FWD_LIMIT) * PWM_TIMER_MAX / FWD_LIMIT; // Motor ON Time

    } else {
        // Potentiometer is on the treshold
        motor_mode(STOP);                                                   // Motor movement STOP
        onTime = 0;                                                         // Motor ON Time
    }

    onTime = (CLOCK * onTime) / 64;                         // Prescale the ON Time

    if (onTime>PWM_TIMER_MAX-1) 
		onTime = PWM_TIMER_MAX-1;
	else if (onTime < 0)
		onTime = 0;		
	OCR2B = onTime;

    return onTime;
}