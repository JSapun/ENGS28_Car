#include <avr/io.h>				// All the port definitions are here
#include <util/delay.h>			// For the _delay_ms macro
#include "ioE28.h"
#include "USARTE28.h"
#include "wifiDrv.h"
#include "AdafruitIODrv.h"
#include <tb6612_T0.h>
#include <SevenSeg.h>

#include "mySecrets.h"

char networkName[] = MY_WLAN_SSID;      // your network SSID (name)
char networkPwd[]  = MY_WLAN_PASS;      // your network password (use for WPA, or use as key for WEP)

char aio_key[] = IO_KEY;             // your AdafruitIO key
char aio_usr[] = IO_USERNAME;        // your AdafruitIO username

//! [Credentials]
 
int main(void) {	

	//! [Initialize]

	//char aio_feed1[] = "ignition";     // name of the AdafruitIO feed you are querying
	//char aio_feed2[] = "headlights";     // name of the AdafruitIO feed you are querying
	char aio_feed[] = "drive";     // name of the AdafruitIO feed you are querying

	uint32_t hex;
	uint16_t prev_speed = 1000;
	//int8_t headlights;
	//int8_t ignition;
	
	uint16_t onTime1 = 0;
	uint16_t onTime2 = 0;

	uint16_t display_buffer_brake[HT16K33_NBUF]; 	// Array for brake display
	display_buffer_brake[0] = 0b00111001;
	display_buffer_brake[1] = 0b00001111;
	display_buffer_brake[2] = 0b00000000;
	display_buffer_brake[3] = 0b00111001;
	display_buffer_brake[4] = 0b00001111;

	DDRD |= (1 << PORTD6); // Set HeadLights as Output
	DDRB |= (1 << PORTB0);
	PORTD |= (1 << PORTD6); // Set bit --> Turn HeadLights on	 
	PORTB |= (1 << PORTB0);

	i2cInit();
	SevenSeg_init();
	SevenSeg_dim(1);
	SevenSeg_write(display_buffer_brake);

	USART_Init();
	SPIinit();	
	motors_init();

	printf("\n\r connecting to wifi network %s.\n\r", networkName);

	if (!WifiInit(networkName, networkPwd)) {
		return -1;
	}

	//printWifiStatus();

	while (1) {
		if (AdafruitIOConnect() != ESP32_CONNECT_SUCCESS) {
            //printf("Unable to connect to server.\r\n");
			ESP32setLEDs(100, 100, 100);
        } else if (AdafruitIOGetUInt32(aio_usr, aio_key, aio_feed, &hex)) {
				
            //printf("\n\rcolor = %x\n\r", hex);
                
			//uint16_t red = (hex >> 16) & 0xFF;
			uint16_t green = (hex >> 8) & 0xFF; // extract the green component
			uint16_t blue = hex & 0xFF; // extract the blue component

			//printf("G: %u, B: %u\n\r", green, blue); // output the values

			uint16_t direction = (uint32_t)green * 1024 / 255;
			uint16_t speed = (uint32_t)blue * 1024 / 255; 

			uint16_t motor1_value = 0;
			uint16_t motor2_value = 0;

			if (direction < speed/2) {
				motor1_value = speed - (speed/2 - direction);
				motor2_value = speed;
			} else if (direction > speed/2) {
				motor1_value = speed;
				motor2_value = speed - (direction - speed/2);
			}

			printf("Speed: %d, \tPrev speed: %d", motor1_value, prev_speed);

			if ((motor1_value < prev_speed-10)){
				SevenSeg_dim(15);
				printf("\t--> brake\n\r");
			}
			else {
				SevenSeg_dim(4);
				printf("\t--> here\n\r");
			}			
			prev_speed = motor1_value;


			printf("Dir: %d\t Speed: %d\t Motor1: %d\t Motor2: %d\n\r", direction, speed, motor1_value, motor2_value);

			onTime1 = 0;
			onTime2 = 0;
			
			if (speed < REV_LIMIT) { // Potentiometer is on the left side of the treshold --> Reverse
        		motors_mode(REV);                                                    
	    		onTime1 = (REV_LIMIT - (uint32_t)motor1_value) * PWM_TIMER1_MAX / REV_LIMIT;       
				onTime2 = (REV_LIMIT - (uint32_t)motor2_value) * PWM_TIMER2_MAX / REV_LIMIT;       
        
    		} else if (speed > FWD_LIMIT) { // Potentiometer is on the right side of the treshold --> Forward
        		motors_mode(FWD);                                                    
    			onTime1 = ((uint32_t)motor1_value - FWD_LIMIT) * PWM_TIMER1_MAX / FWD_LIMIT; 
				onTime2 = ((uint32_t)motor2_value - FWD_LIMIT) * PWM_TIMER2_MAX / FWD_LIMIT; 
    		} else { // Potentiometer is on the treshold
       			motors_mode(BRAKE);                                                   
       			motors_mode(STOP);                                                                                                          
    		}

			motor1_speed(onTime1);
			motor2_speed(onTime2);
		}
		_delay_ms(500);
       }
	//! [MainLoop]

	return 0;		/* never reached */
}