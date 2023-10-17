/**
 *
 * AdafruitIO Driver
 * This library makes connecting to AdafruitIO 
 * less cumbersome by hiding connections steps
 * in easy-to-use functions.
 * 
 * created January 25, 2023
 * by Petra Bonfert-Taylor.
 */

#include <avr/io.h>				// All the port definitions are here
#include <util/delay.h>			// For the _delay_ms macro
#include "ioE28.h"
#include "USARTE28.h"
#include "wifiSpiDrv.h"
#include "wifiDrv.h"
#include <string.h>
#include <stdlib.h>
#include "AdafruitIODrv.h"

// Adafruit data
const char servername[] =  "io.adafruit.com";    // name address for adafruit test
char path[] = "/api/v2/";
#define SERVERPORT 443

// maximum allowable length for html response body - keep this small as it eats variable
// memory of which the Uno doesn't have a whole lot.
#define MAXBODY 100

wifiObject_t server = {.name = servername, .sock = NO_SOCKET_AVAIL, .port = SERVERPORT};
wifiObject_t client = {.sock = NO_SOCKET_AVAIL};
// then in other files that need these: - this is accomplished by AdafruitIODrv.h:
// extern wifiObject_t server;
// extern wifiObject_t client;


esp32_connect_status AdafruitIOConnect() {
   server.sock = NO_SOCKET_AVAIL;
   server.port = SERVERPORT;
   client.sock = NO_SOCKET_AVAIL;
   return(ESP32connectSSL(&client, &server));
}

uint8_t AdafruitIOGetInt(char *aio_usr, char *aio_key, char *aio_feed, int8_t *returnValue) {
	uint32_t tmp = 0;
	uint8_t status = AdafruitIOGetUInt32(aio_usr, aio_key, aio_feed, &tmp);
	*returnValue = (int8_t)tmp;
	return status;
}

uint8_t AdafruitIOGetUInt32(char *aio_usr, char *aio_key, char *aio_feed, uint32_t *returnValue) {
	char body[MAXBODY];
	uint8_t bodyLength = 0;


	httpGETRequest(client, aio_usr, aio_key, aio_feed, "value");
	//printf("Parsing response...\r\n");    
    if (!skipHeader(client)) {
		printf("Unable to skip header.\r\n");
		ESP32stop(&client);
		return 0;
	} else if ((bodyLength = readBodyLength(client)) == 0) {
		printf("Error reading length of http body.\r\n");
		ESP32stop(&client);
		return 0;
	} else if (!readBody(client, body, bodyLength)) {
		printf("Error reading json body.\r\n");
		ESP32stop(&client);
		return 0;
	} else if (!parseInt(body, "value", returnValue)){
		printf("Error parsing body: \r\n%s\r\n", body);
		ESP32stop(&client);
		return 0;
	}

	ESP32stop(&client);
	return 1;
}

void AdafruitIOPostInt(char *aio_usr, char *aio_key, char *aio_feed, int8_t value, uint8_t response) {
	
	httpPOSTRequest(client, aio_usr, aio_key, aio_feed, value);
	
	if (response) {
		uint8_t connected = 1;
		printf("Server response:\r\n");
		while(connected) {
		  	while (ESP32dataAvailable(client)) {
				char c = ESP32read(client);
				printf("%c",c);
	  		}

	  		// if the server's disconnected, stop the client:
	  		if (!ESP32connected(client)) {
				connected = 0;
				printf("\r\n");
				printf("disconnecting from server.\r\n");
				ESP32stop(&client);
			}
		}
	} else {
		ESP32stop(&client);
	}
}

const char *AdafruitIOGetServerName( void ) {
	// This was moved here to allow us to contain the client and server
	//  (particularly the server) structure in this file without needing to
	// directly expose it to the "outside" world.
	return server.name;
}

uint8_t parseInt(char *body, char *key, uint32_t *returnInt) {
	char *p;
	if ((p = strstr(body, key)) != NULL  && (p+strlen(key)+3 < body + strlen(body))) {
		p += strlen(key) + 3;    // A bit of magic numbering related to the format of the line we are parsing.

		if ( *p == '#' ) {
			p++;
			*returnInt = strtol(p, NULL, 16 );  // hex
		} else {
			*returnInt = strtol(p, NULL, 10 );  // dec
		}
		return 1;
	} else {
		return 0;
	}
}


uint8_t WifiInit(char *ssid, char *pass){
	int status = WL_IDLE_STATUS;
    while (ESP32getConnectionStatus() == WL_NO_MODULE) {
    	printf("ESP32 communication error!\n\r");
   	    _delay_ms(1000);
  	}

    const char* fv = ESP32getFwVersion();
    if (*fv < '1')
    {
       printf("Upgrade firmware\r\n");
       return 0;
    } else {
       printf("Firmware version %s.\r\n", fv);
    }

    // attempt to connect to Wifi network:
    // printf("Connecting to %s\r\n", ssid);
    // Connect to WPA/WPA2 network. 
    uint8_t timeout = 0;
    do {
      status = ESP32begin(ssid, pass);
      _delay_ms(100);     // wait until connection is ready!
      timeout++;
    } while (status != WL_CONNECTED && timeout < 50);

	if (status != WL_CONNECTED) {
		return 0;
	} else {
	    //printf("Connected to wifi\r\n");
    	//printWifiStatus();
		return 1;
	}
}
	
void httpGETRequest(wifiObject_t client, char *aio_usr, char *aio_key, char *feed, char *fields) {
/* Make a HTTP GET request to get the most recent value of the indicated field(s):
   Fields must be comma-separated with no spaces. Keep it short as the Arduino Uno
   does not have enough memory to process a long response.

GET https://io.adafruit.com/api/v2/{username}/feeds/{feed_key}/data?limit=1&include={fields}&x-aio-key={aio_key} HTTP/1.1
Host: io.adafruit.com
Connection: close
*/
       ESP32printString(client, "GET "); 
       ESP32printString(client, path); 
       ESP32printString(client, aio_usr); 
       ESP32printString(client, "/feeds/"); 
       ESP32printString(client, feed); 
       ESP32printString(client, "/data?limit=1&include=");
       ESP32printString(client, fields);
       ESP32printString(client, "&x-aio-key=");
       ESP32printString(client, aio_key);
       ESP32printString(client, " HTTP/1.1\r\n");
       ESP32printString(client, "Host: "); 
       ESP32printString(client, servername);
       ESP32printString(client, "\r\n");
       ESP32printString(client, "Connection: close\r\n\r\n");   
}

void httpPOSTRequest(wifiObject_t client, char *aio_usr, char *aio_key, char *feed, int8_t value) {
/* Make a HTTP POST request to add a value to a feed.

POST https://io.adafruit.com/api/v2/{username}/feeds/{feed_key}/data?x-aio-key={aio_key} HTTP/1.1
Host: io.adafruit.com
Content-Type: application/x-www-form-urlencoded
Content-Length: {calculated length of body}

value={value}
*/

       char body[20] = "value=";
       char valuestr[10];
       
       itoa(value, valuestr, 10);
       strcat(body, valuestr);
       itoa(strlen(body), valuestr, 10);
//     printf("Body: %s, length=%s.\r\n", body, valuestr);
       
       // Make a HTTP POST request:
       ESP32printString(client, "POST "); 
       ESP32printString(client, path); 
       ESP32printString(client, aio_usr); 
       ESP32printString(client, "/feeds/"); 
       ESP32printString(client, feed); 
       ESP32printString(client, "/data?x-aio-key=");
       ESP32printString(client, aio_key);
       ESP32printString(client, " HTTP/1.1\r\n");
       ESP32printString(client, "Host: "); 
       ESP32printString(client, servername);
       ESP32printString(client, "\r\nContent-Type: application/x-www-form-urlencoded\r\n");
       ESP32printString(client, "Content-Length: ");
       ESP32printString(client, valuestr);
       ESP32printString(client, "\r\n\r\n");
       ESP32printString(client, body);
       ESP32printString(client, "\r\n");
} 



uint8_t readBody(wifiObject_t client, char body[], uint8_t bodyLength) {
// Read the body of the server http response. Body needs to be short - hence the 
// restrictions on fields and only the most recent value in the GET request.
	uint8_t i = 0;	
	if (bodyLength<MAXBODY) {
		while (i<bodyLength && ESP32dataAvailable(client)) {
			body[i++] = ESP32read(client);
		}
		if (i==bodyLength) {
			body[i] = '\0';
			return 1;
		} else {
			printf("Error body read.\r\n");
			return 0;
		}
	} else {
		printf("Body too large.\r\n");
		while (ESP32dataAvailable(client)) {
			char c = ESP32read(client);
			printf("%c", c);
		}
		return 0;
	}
}

int8_t skipHeader(wifiObject_t client) {
// Ignore the header of the HTTP server response
	uint8_t currentLineIsBlank = 1;
	uint8_t continueReading = 10;
	char c;
		
	while (continueReading) {
		if (ESP32dataAvailable(client)) {
			c = ESP32read(client);
			if (c == '\n' && currentLineIsBlank) {
				return 1;
			} else if (c == '\n') {
				// you're starting a new line
				currentLineIsBlank = 1;
			} else if (c != '\r') {
				// you've gotten a character on the current line
				currentLineIsBlank = 0;
			}
		} else {
			continueReading--;
			_delay_ms(100);
		}
	}
	printf("Header timeout.\r\n");
	return 0;
}

uint8_t readBodyLength(wifiObject_t client) {
// Having skipped the header, find the length of the http body response
	uint8_t length = 0;
	uint8_t continueReading = 1;
	char lengthstr[10];
	uint8_t i = 0;
	
	while (continueReading && ESP32connected(client)) {
		if (ESP32dataAvailable(client)) {
			char c = ESP32read(client);
			if (c == '\n') {
				continueReading = 0;
				lengthstr[i] = '\0';
			} else if (c != '\r'){
				lengthstr[i++] = c;
				if (i==9) {
					lengthstr[i] = '\0';
					continueReading = 0;
				}
			} 
		}
	}
	char *ptr;
	length = strtol(lengthstr, &ptr, 16);
	return length;
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  printf("SSID: %s\r\n", ESP32getCurrentSSID());

  // print your board's IP address:
  uint32_t ip;
  ESP32getIpAddress(&ip);
  printf("IP Address: ");
  printf("%d.%d.%d.%d \r\n\r\n", (uint16_t) (ip >> 24), (uint16_t) (ip >> 16 & 0xff), (uint16_t) (ip >> 8 & 0xff), (uint16_t) ip & 0xff);
}


