#include <avr/io.h>				// All the port definitions are here
#include <util/delay.h>			// For the _delay_ms macro
//#include "ADC.h"
#include "ioE28.h"
#include "USARTE28.h"
#include "wifiDrv.h"
#include "AdafruitIODrv.h"
#include "tb6612.h"

#include "mySecrets.h"

char networkName[] = MY_WLAN_SSID;      // your network SSID (name)
char networkPwd[]  = MY_WLAN_PASS;      // your network password (use for WPA, or use as key for WEP)

char aio_key[] = IO_KEY;             // your AdafruitIO key
char aio_usr[] = IO_USERNAME;        // your AdafruitIO username

//! [Credentials]

int main(void) {	

	//! [Initialize]

	char aio_feed[] = "robot";     // name of the AdafruitIO feed you are querying
	uint32_t hex;
	
	USART_Init();
	SPIinit();	
	motor1_init();
	motor2_init();

	//motor_mode(FWD, TIMER2);
	
	//! [Initialize]


	printf("\n\r connecting to wifi network %s.\n\r", networkName);

	//! [Netconnect]

	if (!WifiInit(networkName, networkPwd)) {
		return -1;
	}

	//! [Netconnect]

	printWifiStatus();

	//! [MainLoop]
	while (1) {
        
		if (AdafruitIOConnect() != ESP32_CONNECT_SUCCESS) {
            printf("Unable to connect to server.\r\n");
        } else if (AdafruitIOGetUInt32(aio_usr, aio_key, aio_feed, &hex)) {
            printf("\n\rcolor = %x\n\r", hex);
                
			//uint16_t red = (hex >> 16) & 0xFF;
			uint16_t green = (hex >> 8) & 0xFF; // extract the green component
			uint16_t blue = hex & 0xFF; // extract the blue component

			printf("G: %u, B: %u\n\r", green, blue); // output the values

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

			printf("Dir: %d\t Speed: %d\t Motor1: %d\t Motor2: %d\n\r", direction, speed, motor1_value, motor2_value);

			motor1_speed(motor1_value);
			motor2_speed(motor2_value);
			

        }
        _delay_ms(500);

	} 
	//! [MainLoop]

	return 0;		/* never reached */
}