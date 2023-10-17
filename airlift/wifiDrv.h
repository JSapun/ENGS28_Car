/**
 * @file wifiDrv.h - Library for Arduino Wifi shield.
 *
 * @brief C translation of Adafruit IO library for Uno only
 *
 * Translated into C and JUST for Adruino Uno by Petra Bonfert-Taylor
 *
 * Copyright (c) 2018 Arduino SA. All rights reserved.
 * Copyright (c) 2011-2014 Arduino.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef WIFIDRVE28_H
#define WIFIDRVE28_H


#include <stdlib.h>
#include "wifiSpiDrv.h"

/**
 * @def KEY_IDX_LEN
 * @brief Unused
 *
 */
#define KEY_IDX_LEN     1


/**
 * @def WL_DELAY_START_CONNECTION
 * @brief Time (in ms) allowed for a connection to be established before timing out.
 *
 */
#define WL_DELAY_START_CONNECTION 5000

/**
 * @def WL_FW_VER_LENGTH
 * @brief Length of the firmware version string.
 *
 */
#define WL_FW_VER_LENGTH 6

/**
 * @def WL_SSID_MAX_LENGTH
 * @brief Allowed length of network SSID
 *
 */
#define WL_SSID_MAX_LENGTH 32

/**
 * @def WL_WPA_KEY_MAX_LENGTH
 * @brief WPA maximum key size
 *
 */
#define WL_WPA_KEY_MAX_LENGTH 63

/**
 * @def WL_WEP_KEY_MAX_LENGTH
 * @brief WEP maximum key size
 *
 */
#define WL_WEP_KEY_MAX_LENGTH 13

/**
 * @def WL_MAC_ADDR_LENGTH
 * @brief Size of a MAC-address or BSSID
 *
 */
#define WL_MAC_ADDR_LENGTH 6

/**
 * @def WL_IPV4_LENGTH
 * @brief Number of bytes in an IPV4 addreess
 *
 */
#define WL_IPV4_LENGTH 4

/**
 * @def WL_NETWORKS_LIST_MAXNUM
 * @brief Number of stored network names allowed on the ESP32
 *
 */
#define WL_NETWORKS_LIST_MAXNUM	10

/**
 * @def WIFI_MAX_SOCK_NUM
 * @brief Number of sockets supported by the ESP32
 *
 */
#define	WIFI_MAX_SOCK_NUM	10

/**
 * @def WL_MAX_ATTEMPT_CONNECTION
 * @brief Limit number of failed connection attempts
 *
 */
#define WL_MAX_ATTEMPT_CONNECTION	10

/**
 * @typedef wl_status_t
 * @brief General status indications
 *
 */
typedef enum wl_status_e {
	WL_NO_SHIELD = 255,             /**< WL_NO_SHIELD */
	WL_NO_MODULE = WL_NO_SHIELD,    /**< WL_NO_MODULE */
	WL_IDLE_STATUS = 0,             /**< WL_IDLE_STATUS */
	WL_NO_SSID_AVAIL,               /**< WL_NO_SSID_AVAIL */
	WL_SCAN_COMPLETED,              /**< WL_SCAN_COMPLETED */
	WL_CONNECTED,                   /**< WL_CONNECTED */
	WL_CONNECT_FAILED,              /**< WL_CONNECT_FAILED */
	WL_CONNECTION_LOST,             /**< WL_CONNECTION_LOST */
	WL_DISCONNECTED,                /**< WL_DISCONNECTED */
	WL_AP_LISTENING,                /**< WL_AP_LISTENING */
	WL_AP_CONNECTED,                /**< WL_AP_CONNECTED */
	WL_AP_FAILED                    /**< WL_AP_FAILED */
} wl_status_t;

/**
 * @enum wl_enc_type
 *
 * Encryption Suites
 *
 * Values map to 802.11 encryption suites.
 */
enum wl_enc_type {
        ENC_TYPE_WEP  = 5,    /**< ENC_TYPE_WEP Wired Equivalent Privacy */
        ENC_TYPE_TKIP = 2,    /**< ENC_TYPE_TKIP Temporal Key Integrity Protocol */
        ENC_TYPE_CCMP = 4,    /**< ENC_TYPE_CCMP Cypher Mode Block Chaining MAC */
        /* ... except these two, 7 and 8 are reserved in 802.11-2007 */
        ENC_TYPE_NONE = 7,    /**< ENC_TYPE_NONE - Reserved in 802.11 */
        ENC_TYPE_AUTO = 8,    /**< ENC_TYPE_AUTO - Reserved in 802.11 */

        ENC_TYPE_UNKNOWN = 255/**< ENC_TYPE_UNKNOWN - ??? */
};

/**
 * @typedef wl_error_code_t
 * @brief Return value from several ESP32 functions
 *
 */
typedef enum {
        WL_FAILURE = -1,/**< Failure */
        WL_SUCCESS = 1, /**< Success */
} wl_error_code_t;

/**
 * @enum wl_auth_mode
 * @brief Different authentication modes available
 *
 */
enum wl_auth_mode {
	AUTH_MODE_INVALID,    /**< AUTH_MODE_INVALID */
	AUTH_MODE_AUTO,       /**< Figure it out automatically */
	AUTH_MODE_OPEN_SYSTEM,/**< No auth needed */
	AUTH_MODE_SHARED_KEY, /**< AUTH_MODE_SHARED_KEY */
	AUTH_MODE_WPA,        /**< AUTH_MODE_WPA */
	AUTH_MODE_WPA2,       /**< AUTH_MODE_WPA2 */
	AUTH_MODE_WPA_PSK,    /**< AUTH_MODE_WPA_PSK */
	AUTH_MODE_WPA2_PSK    /**< AUTH_MODE_WPA2_PSK */
};

typedef enum {
  WL_PING_DEST_UNREACHABLE = -1,
  WL_PING_TIMEOUT = -2,
  WL_PING_UNKNOWN_HOST = -3,
  WL_PING_ERROR = -4
} wl_ping_result_t;

/**
 * \struct wifiObject_t
 *
 * This structure is use to hold the specifics of a particular WiFi connection.
 */
typedef struct {
	const char *name;              	/**< Name of the server (eg io.adafruit.com) */
	uint8_t ip[WL_IPV4_LENGTH];		/**< 4 bytes of IP address                   */
	uint32_t ip32;					/**< 32bit version of ip address             */
	uint8_t sock;				  	/**< Currently allocated socket              */
	uint16_t port;					/**< Current listening port                  */
} wifiObject_t;

/**
 * @enum eProtMode
 * @brief Possible connection protocols.
 *
 * Only TCP and TLS are currently supported
 *
 */
typedef enum eProtMode {
	TCP_MODE,         /**< Transmission Control */
	UDP_MODE,         /**< User Datagram  */
	TLS_MODE,         /**< Transport Layer Security */
	UDP_MULTICAST_MODE/**< Multicast UDP */
} tProtMode;

/**
 *
 */
typedef enum {
	ESP32_CONNECT_SUCCESS = 0,  /**< Connection successfully achieved */
	ESP32_CONNECT_CLIENT_IN_USE,/**< Attempted re-use of an already connected client */
	ESP32_CONNECT_FAILED,       /**< Generic failure to connect see \ref ESP32startClient for possible reasons */
	ESP32_CONNECT_NOSOCK,       /**< All available sockets on the Wifi chip are in use */
} esp32_connect_status;

/**
 * @def INPUT
 * @brief ESP32 pin mode
 *
 */
#define INPUT 0x0

/**
 * @def OUTPUT
 * @brief ESP32 pin mode
 *
 */
#define OUTPUT 0x1


/**
 * @ingroup ESPFunctions
 *
 * @fn uint8_t ESP32getConnectionStatus()
 * @brief Check to see if the Airlift shield is present.
 *
 * Busy wait until the BUSY pin (D7) on the shield is deasserted.
 *
 * Once the shield is ready, query the connection status register
 * over SPI.  Return the result.
 *
 * Returns a wl_status_t enum.
 *
 * WL_NO_MODULE is returned is there is no ESP32 in the system.
 *
 * @return (wl_status_t)
 */
uint8_t ESP32getConnectionStatus();

/**
 * @ingroup ESPFunctions
 *
 * @brief Attempt an ssl connection
 *
 * This function sends the configured client and server infomation
 * to the wireless chip.
 *
 * If there is an error in the process, the return value will indicate
 * the reason.  (See \ref esp32_connect_status)
 *
 * @param client
 * @param server
 * @return ESP32_CONNECT_SUCCESS or relevant error
 */
esp32_connect_status ESP32connectSSL(wifiObject_t *client, wifiObject_t *server);


/**
 * @ingroup ESP32Functions
 *
 * @fn void ESP32setLEDs(uint8_t, uint8_t, uint8_t)
 * @brief Activate the RGB LED on the Airlift Shield.
 *
 * @param red    0-255
 * @param green  0-255
 * @param blue   0-255
 */
void ESP32setLEDs(uint8_t red, uint8_t green, uint8_t blue);


/**
 * @ingroup ESP32Functions
 *
 * @fn void ESP32startClient(wifiObject_t*, wifiObject_t*, uint8_t)
 * @brief Attempt to connect to the host on behalf of the client.
 *
 * This function communicates with the ESP32 hardware over SPI.  It sends the
 * START_CLIENT_TCP_CMD command to the Wifi Engine using the IP address and Port
 * specified by the host parameter and uses the socket specified by the client.
 *
 * protMode must be either TCP_MODE or TLS_MODE.
 *
 * If the protMode is incorrect, the client socekt is set to 255 to indicate
 * the error.
 * *
 * @param client
 * @param host
 * @param protMode
 */
void ESP32startClient(wifiObject_t *client, wifiObject_t *host, uint8_t protMode);

/**
 * @ingroup ESP32Functions
 *
 * @fn const char ESP32getFwVersion()
 * @brief Retrieve the firmware version from the ESP32
 *
 * @return The firmware version string (nullstring on error)
 */
const char* ESP32getFwVersion();

/**
 * @ingroup ESP32Functions
 *
 * @fn void ESP32pinMode(uint8_t, uint8_t)
 * @brief USe the SPI command interface to set the mode of a GPIO on the ESP32
 *
 * @param pin
 *
 * The pin number of interest
 *
 * @param mode
 *
 * Either INPUT or OUTPUT
 */
void ESP32pinMode(uint8_t pin, uint8_t mode);

/**
 * @ingroup ESPFunctions
 *
 * @fn void ESP32analogWrite(uint8_t, uint8_t)
 * @brief Use the SPI command interface to drive an anolog (PWM)
 * value on the specified pin.
 *
 * @param pin
 *
 * Pin number of the pin of interest.
 *
 * @param value
 *
 * uint8_t PWM pulse width.
 *
 */
void ESP32analogWrite(uint8_t pin, uint8_t value);

/**
 * @ingroup ESPFunctions
 *
 * @fn int8_t ESP32scanNetworks()
 * @brief Scan for wireless networks
 *
 * send \ref ESP32startScanNetworks wait for the scan to happen and call
 * \ref ES32getScanNetworks to retrieve the data.
 *
 * @return The number of networks or \ref WL_FAILURE
 */
int8_t ESP32scanNetworks();

/**
 * @ingroup ESPFunctions
 *
 * @fn int8_t ESP32startScanNetworks()
 * @brief Send the start scan command to the ESP32
 *
 * @return return \ref WL_FAILURE or \WL_SUCCESS
 */
int8_t ESP32startScanNetworks();

/**
 * @ingroup ESPFunctions
 *
 * @fn uint8_t ESP32getScanNetworks()
 * @brief Fetch the results of a previously started scan.
 *
 * Store the SSIDs in a private variable accessible through \ref ESP32getSSIDNetworks
 *
 * @return The number of networks
 */
uint8_t ESP32getScanNetworks();

/**
 * @ingroup ESPFunctions
 *
 * @fn const char ESP32getSSIDNetworks*(uint8_t)
 * @brief Get an SSID from a previously completed scan.
 *
 * Be warned!  No checking is done on the bounds, so you may, if
 * you are not careful, cause the Higgs to drop to its low energy
 * state, and then, well...  You know - poof!
 *
 * @param networkItem - index
 * @return String representing the name.
 */
const char* ESP32getSSIDNetworks(uint8_t networkItem);

/**
 * @ingroup ESPFunctions
 *
 * @fn const char ESP32getCurrentSSID*()
 * @brief Get the name of the currently connected Wifi
 *
 * @return
 */
const char* ESP32getCurrentSSID();


int ESP32begin(const char* ssid, const char *passphrase);
int8_t ESP32setPassphrase(const char* ssid, uint8_t ssid_len, const char *passphrase, const uint8_t len);
void ESP32getNetworkData(uint8_t *ip, uint8_t *mask, uint8_t *gwip);
void ESP32getIpAddress(uint32_t* ip);
int ESP32getHostByName(const char* aHostname, uint8_t* aResult);
int ESP32connect(wifiObject_t *client, wifiObject_t *host);
uint8_t ESP32getSocket();
uint8_t ESP32connected(wifiObject_t object);
uint8_t ESP32status(uint8_t sock);
uint8_t ESP32getClientState(uint8_t sock);
int ESP32dataAvailable(wifiObject_t object);
int ESP32getDataBuf(uint8_t sock, uint8_t *_data, uint16_t *_dataLen);
uint16_t ESP32sendData(uint8_t sock, const char *data, uint16_t len);
uint8_t ESP32checkDataSent(uint8_t sock);
size_t ESP32write(wifiObject_t object, const char *buf, size_t size);
size_t ESP32printString(wifiObject_t object, const char *buf);
int ESP32read(wifiObject_t client);
int ESP32bufferAvailable(int socket);
int ESP32bufferRead(int socket, uint8_t* data, size_t length);
void ESP32stop(wifiObject_t *object);
void ESP32socketBufferClose(int socket);

#endif // WIFIDRVE28_H
