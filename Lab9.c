
/*
 * Name:		Justin Sapun
 * Assignment:	ENGS 28 Lab6 -- Varible PWM control of a TT motor with potentiometer
 *
 * Program name: Lab6.c
 * Date created: 02/12/2023
 * 
 * Description:	 	
 *	
 * 	This file/lab aims to control the speed of a DC permanent magnet motor with a PWM signal.
 *	Also, we want to measure the speed of a DC permanent magnet motor with an optical sensor,
 *	and increase proficiency with device drivers, timers, and interrupts.
 *
 * I/O pins:  A0 --> Analog Input for Potentiometer
 *			  D7 --> Speed Sensor	  
 *            B3 --> AIN1  		  
 *			  B2 --> AIN2 		  
 *			  B1 --> PWMA 	
 *            SDA & SCL for I2C comm with Display
 * 
 * Additional Feature -->> Displayed the RPM, positive and negative, on the seven-segment display. 
 */

/* INCLUDE FILES */
#include <avr/io.h>				// All the port definitions are here
#include <avr/interrupt.h>
#include <stdlib.h>				// for abs()
#include <USARTE28.h>
#include <ioE28.h>
#include <ADC.h>
#include <tb6612.h>

void set_speed(uint16_t calc_speed, uint16_t pot_turn);

/* MAIN */
int main(void) { // MotorA --> left, MotorB --> right ------.>>> need max ADC value for motor turn

	uint16_t pot_speed = 300;         		// ADC Potentiometer value
	uint16_t pot_turn = 512;         		// ADC Potentiometer value


	motor_init();					// Initalize driver for motor
  	USART_Init();   				// Initalize USUART COM for Screen
  	ADC_Init();						// Initalize ADC

  
 	while(1) {

		pot_speed = ADC_getValue();
		ADC_setChannel(1);
		pot_turn = ADC_getValue();
		ADC_setChannel(0);

		printf("Pot Speed: %d, \tPot Turn: %d\n\r", pot_speed, pot_turn);
 
		// Combined Speed (Pot 1)

		if (pot_speed > 562){						// CW - Positive
			motor_mode(FWD);
			set_speed((pot_speed-512)*2, pot_turn)	;
		}
		else if (pot_speed < 462){			 		// CCW - Negative 
			motor_mode(REV);
			set_speed(abs((pot_speed-512)*2), pot_turn);
		}
		else {		 		 		 				// Center - Brake & Stop
			motor_mode(BRAKE); 						
			motor_mode(STOP); 	
		}
	}
	return 0;		// This line is never reached
}

void set_speed(uint16_t calc_speed, uint16_t pot_turn){

	uint16_t speedA = calc_speed;
	uint16_t speedB = calc_speed;
	// First we check for pot_turn
	if (pot_turn > 562){						// CW - Positive --> turn right
		speedB = 1124 - calc_speed;	// Stop B
	}
	else if (pot_turn < 462){			 		// CCW - Negative --> turn left
		speedA = 1124 - calc_speed;	// Stop A
	}

	motor_speedA(speedA);
	motor_speedB(speedB);
}