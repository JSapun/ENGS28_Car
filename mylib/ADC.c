/* 
 * ADC.c
 * Justin Sapun 
 * Device driver module for ADC onboard the Atmega328P.
 */

#include <avr/io.h>		// All the port definitions are here
#include "ADC.h"		


void ADC_setChannel(uint8_t channel){// Set the ADC input pins (0-5)
	if ((channel < 0) || (channel > 5))
		channel = 0; // set channel 0 if non valid argument presented
	ADMUX  &= 0xF0; // reset channel
	ADMUX |= channel; // ORs it with ADMUX register (0b11110000 for A0, 0b11110001 for A1)
}

void ADC_setReference(uint8_t Vref){// Set the ADC voltage reference (Accepts 1 for 1.1 V or 5 for 5 V)
	if (Vref == 1)
		ADMUX |= (1 << REFS1); // set bit -->> set Vref to 1.1 V
	else{
		ADMUX &= ~(1 << REFS1); // clears bit -->> set Vref to 5 V
	}	
}

void ADC_Init(void){			// Initialize the ADC
	ADMUX  |= (1 << REFS0); 						  // AVCC reference
	ADMUX  &= 0xF0;         						  // Use analog channel 0
	ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2); // Prescale by 128
	ADCSRA |= (1 << ADEN);  						  // Enable ADC
}

uint16_t ADC_getValue(void){	// Initiate conversion, return result
	ADCSRA |= (1 << ADSC); 				  // Start conversion
    while ((ADCSRA & (1 << ADSC)) != 0){} // Wait for completion
    uint16_t value = ADC;       		  // Read the result --- (Change to 'ADCH' for 8 bits resolution, but this library isn't meant for that)
    return value;
}