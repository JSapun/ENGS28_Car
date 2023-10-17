/*
  spi_drv.h - Library for Arduino Wifi shield.
  Copyright (c) 2018 Arduino SA. All rights reserved.
  Copyright (c) 2011-2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/** \file wifiSpiDrv.h
 * @brief Declarations and enums used in the SPI portion of the Wifi
 * driver.
 *
 */
#ifndef WIFISPIDRVE28_H
#define WIFISPIDRVE28_H

#include <avr/io.h>
#include <util/delay.h>			// For the _delay_ms macro
#include <stddef.h> // for size_t datatype
//#include <string.h>

/* SPI mode defines */
#define SPI_SS           PB2
#define SPI_SS_PORT      PORTB
#define SPI_SS_PIN       PINB
#define SPI_SS_DDR       DDRB

#define SPI_MOSI         PB3
#define SPI_MOSI_PORT    PORTB
#define SPI_MOSI_PIN     PINB
#define SPI_MOSI_DDR     DDRB

#define SPI_MISO         PB4
#define SPI_MISO_PORT    PORTB
#define SPI_MISO_PIN     PINB
#define SPI_MISO_DDR     DDRB

#define SPI_SCK          PB5
#define SPI_SCK_PORT     PORTB
#define SPI_SCK_PIN      PINB
#define SPI_SCK_DDR      DDRB

#define ESP32_SLAVEREADY  		  PD7
#define ESP32_SLAVEREADY_PORT      PORTD
#define ESP32_SLAVEREADY_PIN       PIND
#define ESP32_SLAVEREADY_DDR       DDRD

#define ESP32_SLAVERESET  		  PD5
#define ESP32_SLAVERESET_PORT      PORTD
#define ESP32_SLAVERESET_PIN       PIND
#define ESP32_SLAVERESET_DDR       DDRD

/* Which pin selects ESP32 as slave? */
#define SLAVE_SELECT    (SPI_SS_PORT &= ~(1 << SPI_SS))
#define SLAVE_DESELECT  (SPI_SS_PORT |= (1 << SPI_SS))

#define SPI_START_CMD_DELAY 	10

#define NO_LAST_PARAM   0
#define LAST_PARAM      1

#define DUMMY_DATA  0xFF

#define CMD_FLAG        0
#define REPLY_FLAG      1<<7
#define DATA_FLAG 		0x40

#define WIFI_SPI_ACK        1
#define WIFI_SPI_ERR        0xFF

#define TIMEOUT_CHAR    1000

//#define	MAX_SOCK_NUM		4	/**< Maxmium number of socket  */
#define NO_SOCKET_AVAIL     255

#define START_CMD   0xE0
#define END_CMD     0xEE
#define ERR_CMD   	0xEF
#define CMD_POS			1		// Position of Command OpCode on SPI stream
#define PARAM_LEN_POS 	2		// Position of Param len on SPI stream

enum {
	SET_NET_CMD 		= 0x10,
	SET_PASSPHRASE_CMD	= 0x11,
	SET_KEY_CMD	        = 0x12,
//	TEST_CMD	        = 0x13,
	SET_IP_CONFIG_CMD	= 0x14,         /**< SET_IP_CONFIG_CMD */
	SET_DNS_CONFIG_CMD	= 0x15,        /**< SET_DNS_CONFIG_CMD */
	SET_HOSTNAME_CMD	= 0x16,          /**< SET_HOSTNAME_CMD */
	SET_POWER_MODE_CMD	= 0x17,        /**< SET_POWER_MODE_CMD */
	SET_AP_NET_CMD		= 0x18,           /**< SET_AP_NET_CMD */
	SET_AP_PASSPHRASE_CMD = 0x19,     /**< SET_AP_PASSPHRASE_CMD */
	SET_DEBUG_CMD		= 0x1A,            /**< SET_DEBUG_CMD */
	GET_TEMPERATURE_CMD = 0x1B,       /**< GET_TEMPERATURE_CMD */

	GET_CONN_STATUS_CMD	= 0x20,       /**< GET_CONN_STATUS_CMD */
	GET_IPADDR_CMD		= 0x21,           /**< GET_IPADDR_CMD */
	GET_MACADDR_CMD		= 0x22,          /**< GET_MACADDR_CMD */
	GET_CURR_SSID_CMD	= 0x23,         /**< GET_CURR_SSID_CMD */
	GET_CURR_BSSID_CMD	= 0x24,        /**< GET_CURR_BSSID_CMD */
	GET_CURR_RSSI_CMD	= 0x25,         /**< GET_CURR_RSSI_CMD */
	GET_CURR_ENCT_CMD	= 0x26,         /**< GET_CURR_ENCT_CMD */
	SCAN_NETWORKS		= 0x27,            /**< SCAN_NETWORKS */
	START_SERVER_TCP_CMD= 0x28,       /**< START_SERVER_TCP_CMD */
	GET_STATE_TCP_CMD   = 0x29,       /**< GET_STATE_TCP_CMD */
	DATA_SENT_TCP_CMD	= 0x2A,         /**< DATA_SENT_TCP_CMD */
    AVAIL_DATA_TCP_CMD	= 0x2B,     /**< AVAIL_DATA_TCP_CMD */
    GET_DATA_TCP_CMD	= 0x2C,       /**< GET_DATA_TCP_CMD */
    START_CLIENT_TCP_CMD= 0x2D,    /**< START_CLIENT_TCP_CMD */
    STOP_CLIENT_TCP_CMD = 0x2E,    /**< STOP_CLIENT_TCP_CMD */
    GET_CLIENT_STATE_TCP_CMD= 0x2F,/**< GET_CLIENT_STATE_TCP_CMD */
    DISCONNECT_CMD		= 0x30,        /**< DISCONNECT_CMD */
//	GET_IDX_SSID_CMD	= 0x31,
	GET_IDX_RSSI_CMD	= 0x32,          /**< GET_IDX_RSSI_CMD */
	GET_IDX_ENCT_CMD	= 0x33,          /**< GET_IDX_ENCT_CMD */
	REQ_HOST_BY_NAME_CMD= 0x34,       /**< REQ_HOST_BY_NAME_CMD */
	GET_HOST_BY_NAME_CMD= 0x35,       /**< GET_HOST_BY_NAME_CMD */
	START_SCAN_NETWORKS	= 0x36,       /**< START_SCAN_NETWORKS */
	GET_FW_VERSION_CMD	= 0x37,        /**< GET_FW_VERSION_CMD */
//	GET_TEST_CMD		= 0x38,
	SEND_DATA_UDP_CMD	= 0x39,         /**< SEND_DATA_UDP_CMD */
	GET_REMOTE_DATA_CMD = 0x3A,       /**< GET_REMOTE_DATA_CMD */
	GET_TIME_CMD		= 0x3B,             /**< GET_TIME_CMD */
	GET_IDX_BSSID		= 0x3C,            /**< GET_IDX_BSSID */
	GET_IDX_CHANNEL_CMD = 0x3D,       /**< GET_IDX_CHANNEL_CMD */
	PING_CMD			= 0x3E,                /**< PING_CMD */
	GET_SOCKET_CMD		= 0x3F,           /**< GET_SOCKET_CMD */

    // All command with DATA_FLAG 0x40 send a 16bit Len

	SEND_DATA_TCP_CMD		= 0x44,        /**< SEND_DATA_TCP_CMD */
    GET_DATABUF_TCP_CMD		= 0x45,   /**< GET_DATABUF_TCP_CMD */
    INSERT_DATABUF_CMD		= 0x46,    /**< INSERT_DATABUF_CMD */

    // regular format commands
    SET_PIN_MODE		= 0x50,          /**< SET_PIN_MODE */
    SET_DIGITAL_WRITE	= 0x51,      /**< SET_DIGITAL_WRITE */
    SET_ANALOG_WRITE	= 0x52,       /**< SET_ANALOG_WRITE */
};

/**
 * @enum wl_tcp_state
 * @brief Current state of the tcp connection.
 *
 * See https://en.wikipedia.org/wiki/Transmission_Control_Protocol for more information
 * on the state transitions used in TCP.
 *
 */
enum wl_tcp_state {
  CLOSED      = 0,/**< CLOSED */
  LISTEN      = 1,/**< LISTEN */
  SYN_SENT    = 2,/**< SYN_SENT */
  SYN_RCVD    = 3,/**< SYN_RCVD */
  ESTABLISHED = 4,/**< ESTABLISHED */
  FIN_WAIT_1  = 5,/**< FIN_WAIT_1 */
  FIN_WAIT_2  = 6,/**< FIN_WAIT_2 */
  CLOSE_WAIT  = 7,/**< CLOSE_WAIT */
  CLOSING     = 8,/**< CLOSING */
  LAST_ACK    = 9,/**< LAST_ACK */
  TIME_WAIT   = 10/**< TIME_WAIT */
};


enum numParams{
    PARAM_NUMS_0,
    PARAM_NUMS_1,
    PARAM_NUMS_2,
    PARAM_NUMS_3,
    PARAM_NUMS_4,
    PARAM_NUMS_5,
    MAX_PARAM_NUMS
};

#define MAX_PARAMS MAX_PARAM_NUMS-1
#define PARAM_LEN_SIZE 1

typedef struct  __attribute__((__packed__))
{
	uint8_t     paramLen;
	char*	    param;
}tParam;

void SPIinit(void);
uint8_t SPItransfer(uint8_t data);
int SPIwaitChar(unsigned char waitChar);
char SPIreadChar();
void SPIgetParam(uint8_t* param);
int SPIwaitResponse(uint8_t cmd, uint8_t* numParamRead, uint8_t** params, uint8_t maxNumParams);
uint8_t SPIreadParamLen8(uint8_t* param_len);
int SPIreadAndCheckChar(char checkChar, char* readChar);
uint16_t SPIreadParamLen16(uint16_t* param_len);
int SPIwaitResponseCmd(uint8_t cmd, uint8_t numParam, uint8_t* param, uint8_t* param_len);
void SPIsendCmd(uint8_t cmd, uint8_t numParam);
void SPIsendParam(uint8_t* param, uint8_t param_len, uint8_t lastParam);
void SPIsendParam16(uint16_t param, uint8_t lastParam);
void SPIsendParamLen8(uint8_t param_len);
void SPIsendParamLen16(uint16_t param_len);
int SPIwaitResponseParams(uint8_t cmd, uint8_t numParam, tParam* params);
void SPIsendBuffer(uint8_t* param, uint16_t param_len, uint8_t lastParam);
int SPIwaitResponseData16(uint8_t cmd, uint8_t* param, uint16_t* param_len);
int SPIwaitResponseData8(uint8_t cmd, uint8_t* param, uint8_t* param_len);

#endif // WIFISPIDRVE28_H