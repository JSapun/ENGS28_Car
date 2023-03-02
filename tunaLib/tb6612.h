#include <avr/io.h>

#define PWM_0               DDD6            // Port D Pin 6
#define IN1_0               DDD7            // Port D Pin 7
#define IN2_0               DDD5            // Port D Pin 5

#define PWM_1               DDB1            // Port B Pin 9
#define IN1_1               DDB2            // Port B Pin 10
#define IN2_1               DDB3            // Port B Pin 11

#define PWM_2               DDD3            // Port D Pin 3
#define IN1_2               DDD4            // Port D Pin 4
#define IN2_2               DDD2            // Port D Pin 2

#define PWM_TIMER_MAX       1250		    // 1.6kHz with prescale 8
#define SPEED_UPDATE        2000		    // 2 seconds
#define CLOCK               16

#define FWD                   0
#define REV                   1
#define STOP                  2

#define FWD_0                 0x20            // Set PORTD to 0b00100000 => Turn OFF IN1, Turn ON IN2
#define REV_0                 0x80            // Set PORTD to 0b10000000 => Turn ON IN1, Turn OFF IN2
#define STOP_0                0xA0            // Set PORTD to 0b10100000 => Turn ON IN1, Turn ON IN2

#define FWD_1                 0x04            // Set PORTB to 0b00000100 => Turn OFF IN1, Turn ON IN2
#define REV_1                 0x08            // Set PORTB to 0b00001000 => Turn ON IN1, Turn OFF IN2
#define STOP_1                0x0C            // Set PORTB to 0b00001100 => Turn ON IN1, Turn ON IN2

#define FWD_2                 0x04            // Set PORTD to 0b00000100 => Turn OFF IN1, Turn ON IN2
#define REV_2                 0x10            // Set PORTD to 0b00010000 => Turn ON IN1, Turn OFF IN2
#define STOP_2                0x14            // Set PORTD to 0b00010100 => Turn ON IN1, Turn ON IN2

#define FWD_LIMIT           524
#define REV_LIMIT           500

#define TIMER0              0
#define TIMER1              1
#define TIMER2              2



uint8_t motor_init(uint8_t timer);   // Set ports for PWM, IN1, IN2 (STBY?) 

uint8_t motor_mode(uint8_t mode, uint8_t timer); // FWD, REV, BRAKE, STOP (STBY?) 

uint8_t motor_speed(uint16_t ADC_value, uint8_t timer); 