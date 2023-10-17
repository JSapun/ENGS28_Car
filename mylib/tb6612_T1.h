 /* 
 * SevenSeg.c
 * Justin Sapun 
 * Device driver header for the tb6612 h-bridge motor controller.
 */

#include <avr/io.h>

typedef enum{FWD, REV, BRAKE, STOP} direction_t;


#define PWM_TIMER1_MAX      1250

#define CLOCK               16

#define FWD_LIMIT           524
#define REV_LIMIT           500

#define AIN1                DDD5
#define AIN2                DDD6

uint8_t motors_init();   
uint8_t motors_mode(uint8_t mode); // FWD, REV, BRAKE, STOP 
uint8_t motor1_speed(uint16_t ADC_value); 
uint8_t motor2_speed(uint16_t ADC_value); 