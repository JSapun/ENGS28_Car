/* 
 * HBridge.h
 * Justin Sapun 
 * Lab 5 - 02/02/2023
 * Device driver header file for the H-Bridge chip ___ for use with controlling motors.
 * Engs 28
 */

typedef enum{FWD, REV, BRAKE, STOP} direction_t;


void motor_init(void);	                        // Initialize the H-Bridge Chip for Motor 
void motor_mode(direction_t direction);         // Set the speed desired
void motor_speed(uint16_t pwm_mag);             // Set the speed desired
