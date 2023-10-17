/** @file AdafruitIODrv.h
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

/* function prototypes */

/**
 * @ingroup Http
 *
 * @fn int8_t skipHeader(wifiObject_t)
 * @brief Helper function for response parsing
 *
 * This function skips the header material in an HTTP response.
 *
 * @param client
 * @return 1 on success, 0 on failure
 */
int8_t skipHeader(wifiObject_t client);

/**
 * @ingroup Http
 *
 * @fn uint8_t readBodyLength(wifiObject_t)
 * @brief Helper function to extract body length from an HTTP response.
 *
 * Note: \ref skipHeader must have already run before calling this.
 *
 * @param client
 * @return Length in bytes of the response body.
 */
uint8_t readBodyLength(wifiObject_t client);

/**
 * @ingroup Http
 *
 * @fn uint8_t readBody(wifiObject_t, char[], uint8_t)
 * @brief Extract the body from an HTTP response
 *
 * \ref skipHeader and \ref readBodyLength must have already been
 * run before calling this.
 *
 * @param client
 * @param body
 * @param bodyLength
 * @return
 */
uint8_t readBody(wifiObject_t client, char body[], uint8_t bodyLength);

/**
 * @fn void printWifiStatus()
 * @brief Diagnostic/informative network message.
 *
 * This function retrieves the name of the current network and
 * the currently assigned IP Address.
 *
 */
void printWifiStatus(void);


/**
 * @ingroup Http
 *
 * @fn void httpGETRequest(wifiObject_t, char*, char*, char*, char*)
 * @brief Form an http GET request to fetch information from the specified feed.
 *
 * The client must already be connected (see eg \ref AdafruitIOConnect).  This
 * method will form a GET request for the specified feed, requesting the latest
 * values for the specified fields.
 *
 * The fields must be comma separated and properly URL escaped.  If that last bit
 * doesn't mean anything to you, the translation is "just use normal characters"...
 *
 * This method fetches only the most recent value for the fields specified.
 *
 * @param client
 * @param aio_usr
 * @param aio_key
 * @param feed
 * @param fields
 */
void httpGETRequest(wifiObject_t client, char *aio_usr, char *aio_key, char *feed, char *fields);

/**
 * @ingroup Http
 *
 * @fn void httpPOSTRequest(wifiObject_t, char*, char*, char*, int8_t)
 * @brief
 *
 * @brief Form an http POST request to send information to the specified feed.
 *
 * The client must already be connected (see eg \ref AdafruitIOConnect).  This
 * method will form a POST request for the specified feed.   The body will contain
 * the updated value for the feed.
 *
 * @param client
 * @param aio_usr
 * @param aio_key
 * @param feed
 * @param value
 */
void httpPOSTRequest(wifiObject_t client, char *aio_usr, char *aio_key, char *feed, int8_t value);

/**
 *
 * @ingroup Html
 *
 * @fn void AdafruitIOPostInt(char*, char*, char*, int8_t, uint8_t)
 * @brief Send a new value to the specified feed.
 *
 * usr, key, and feed specify the target.
 *
 * value is the datum to be sent.
 *
 * response is a flag.  Pass a non-zero value to print the response from the server.  (Useful
 * for debugging)
 *
 * @param aio_usr
 * @param aio_key
 * @param aio_feed
 * @param value
 * @param response
 */
void AdafruitIOPostInt(char *aio_usr, char *aio_key, char *aio_feed, int8_t value, uint8_t response); 


/**
 * @ingroup Wifi
 *
 * @fn uint8_t WifiInit(char*, char*)
 * @brief Connect to the specified wireless network.
 *
 * This function first checks the firmware version on the Airlift Shield, confirming
 * that it is at least 1.x.
 *
 * It then proceed to attempt a connection to the specified network.  At present,
 * only WPA and WPA2 encryption is supported.
 *
 * This routine will try for 5 seconds to connect.  If it has not successfully
 * connected by then, it returns an error.
 *
 * @param ssid Network Name
 * @param pass Network Password (WPA/WPA2)
 * @return 1 if successful 0 on error
 */
uint8_t WifiInit(char *ssid, char *pass);

/**
 * @ingroup Http
 *
 * @fn uint8_t parseInt(char*, char*, uint32_t*)
 * @brief Extract an integer value from the JSON body of the response.
 *
 * Extract a single integer from response.  It is assumed that the key name
 * is on the lead line in the body.
 *
 * This method always returns a 32 bit integer.  Convenience routines can
 * be used to return fewer bits if needed for space reasons.
 *
 * This function can parse decimal or hexadecimal integers.  Hexadecimal
 * values must be preceded by '#'
 *
 * @param body
 * @param key
 * @param returnInt
 * @return
 */
uint8_t parseInt(char *body, char *key, uint32_t *returnInt);

/**
 * @ingroup Wifi
 * @fn esp32_connect_status AdafruitIOConnect()
 * @brief Connect to adafruit.io
 *
 * This function fills in the server information and calls \ref ESP32connectSSL
 * to establish an SSL connection to adafruit.io.
 *
 * @return
 */
esp32_connect_status AdafruitIOConnect(void);

/**
 * @ingroup Html
 * @fn uint8_t AdafruitIOGetInt(char[], char[], char[], int8_t*)
 * @brief Retrieve an integer value from a feed
 *
 * This function uses \ref httpGETRequest to request the latest value of the
 * specified feed.  It then processes the received HTML response.
 *
 * user and key, are the adafruit io credentials for the feed.
 *
 * returnInt must be the address of the int8_t or else a microscopic black hole
 * will be formed which will slowly travel back and forth through the center of
 * the earth, gradually accreting matter as the unknowing world marches slowly
 * toward the event horizon.
 *
 * You have been warned.
 *
 * @param aio_usr
 * @param aio_key
 * @param aio_feed
 * @param returnInt
 * @return
 */
uint8_t AdafruitIOGetInt(char aio_usr[], char aio_key[], char aio_feed[], int8_t *returnInt);

/**
 * @fn uint32_t AdafruitIOGetInt(char[], char[], char[], uint32_t*)
 * @brief see \ref AdafruitIOGetInt.  This function only differs by pointer type.
 *
 * @param aio_usr
 * @param aio_key
 * @param aio_feed
 * @param returnInt
 * @return
 */
uint8_t AdafruitIOGetUInt32(char aio_usr[], char aio_key[], char aio_feed[], uint32_t *returnInt);


/**
 * @ingroup Html
 *
 * This function simply returns the name of the currently configured server.
 * Handy for debug messages.
 *
 * @return A pointer to the name
 */
const char *AdafruitIOGetServerName( void );
