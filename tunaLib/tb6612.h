#include <avr/io.h>

#define CLOCK               16

#define FWD                   0
#define REV                   1
#define STOP                  2

#define FWD_LIMIT           524
#define REV_LIMIT           500

uint8_t motor1_init();   // Set ports for PWM, IN1, IN2 (STBY?) 
uint8_t motor2_init();   // Set ports for PWM, IN1, IN2 (STBY?) 

uint8_t motor1_mode(uint8_t mode); // FWD, REV, BRAKE, STOP (STBY?) 
uint8_t motor2_mode(uint8_t mode); // FWD, REV, BRAKE, STOP (STBY?) 

uint8_t motor1_speed(uint16_t ADC_value); 
uint8_t motor2_speed(uint16_t ADC_value); 