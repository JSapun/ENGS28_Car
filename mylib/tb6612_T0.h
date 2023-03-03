/* 
 * HBridge.h
 * Justin Sapun 
 * Lab 5 - 02/02/2023
 * Device driver header file for the H-Bridge chip ___ for use with controlling 2 motors on Timer 1 A, and Timer 0.
 * Engs 28
 */

#define PWM_2               DDD3            // Port B Pin 1
#define IN2               DDD5            // Port D Pin 5
#define IN1               DDD7            // Port D Pin 7
#define PWM_1               DDB1            // Port D Pin 6


typedef enum{FWD, REV, BRAKE, STOP} direction_t;


void motor_init(void);	                        // Initialize the H-Bridge Chip for Motor 
void motor_mode(direction_t direction);         // Set the speed desired
void motor_speedA(uint16_t pwm_mag);             // Set the speed desired
void motor_speedB(uint16_t pwm_mag);             // Set the speed desired

