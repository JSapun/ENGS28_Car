/*
 wifiDrv.c - Library for Arduino Wifi shield.
 Translated into C and JUST for Adruino Uno by Petra Bonfert-Taylor

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


#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>	
#include <stdlib.h>
#include <ioE28.h>
#include <string.h>

#include "wifiSpiDrv.h"
#include "wifiDrv.h"

// Array of data to cache the information related to the networks discovered
static char _networkSsid[WL_NETWORKS_LIST_MAXNUM][WL_SSID_MAX_LENGTH] = {
		{ "1" }, { "2" }, { "3" }, { "4" }, { "5" } };
;
// firmware version string in the format a.b.c
static char fwVersion[WL_FW_VER_LENGTH] = { 0 };

// Cached values of retrieved data
static char _ssid[WL_SSID_MAX_LENGTH] = { 0 };
static uint8_t _localIp[WL_IPV4_LENGTH] = { 0 };
static uint8_t _subnetMask[WL_IPV4_LENGTH] = { 0 };
static uint8_t _gatewayIp[WL_IPV4_LENGTH] = { 0 };

static struct {
	uint8_t *data;
	uint8_t *head;
	int length;
} _buffers[WIFI_MAX_SOCK_NUM];

#define WIFI_SOCKET_NUM_BUFFERS (sizeof(_buffers) / sizeof(_buffers[0]))
#define WIFI_SOCKET_BUFFER_SIZE 64


uint8_t ESP32getConnectionStatus() {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(GET_CONN_STATUS_CMD, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = WL_NO_MODULE;
	uint8_t _dataLen = 0;
	SPIwaitResponseCmd(GET_CONN_STATUS_CMD, PARAM_NUMS_1, &_data, &_dataLen);

	SLAVE_DESELECT;

	return _data;
}


const char* ESP32getFwVersion() {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(GET_FW_VERSION_CMD, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseCmd(GET_FW_VERSION_CMD, PARAM_NUMS_1,
			(uint8_t*) fwVersion, &_dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;
	return fwVersion;
}


void ESP32setLEDs(uint8_t red, uint8_t green, uint8_t blue) {
	ESP32pinMode(25, OUTPUT);
	ESP32pinMode(26, OUTPUT);
	ESP32pinMode(27, OUTPUT);
	ESP32analogWrite(25, green);
	ESP32analogWrite(26, red);
	ESP32analogWrite(27, blue);
}


void ESP32pinMode(uint8_t pin, uint8_t mode) {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(SET_PIN_MODE, PARAM_NUMS_2);
	SPIsendParam((uint8_t*) &pin, 1, NO_LAST_PARAM);
	SPIsendParam((uint8_t*) &mode, 1, LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = 0;
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseCmd(SET_PIN_MODE, PARAM_NUMS_1, &_data, &_dataLen)) {
		printf("error waitResponse\n\r");
		_data = WL_FAILURE;
	}
	SLAVE_DESELECT;
}


void ESP32analogWrite(uint8_t pin, uint8_t value) {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(SET_ANALOG_WRITE, PARAM_NUMS_2);
	SPIsendParam((uint8_t*) &pin, 1, NO_LAST_PARAM);
	SPIsendParam((uint8_t*) &value, 1, LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = 0;
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseCmd(SET_ANALOG_WRITE, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\n\r");
		_data = WL_FAILURE;
	}
	SLAVE_DESELECT;
}

int8_t ESP32scanNetworks() {
	uint8_t attempts = 10;
	uint8_t numOfNetworks = 0;

	if (ESP32startScanNetworks() == WL_FAILURE)
		return WL_FAILURE;
	do {
		_delay_ms(2000);
		numOfNetworks = ESP32getScanNetworks();
	} while ((numOfNetworks == 0) && (--attempts > 0));

	return numOfNetworks;
}

int8_t ESP32startScanNetworks() {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(START_SCAN_NETWORKS, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = 0;
	uint8_t _dataLen = 0;

	if (!SPIwaitResponseCmd(START_SCAN_NETWORKS, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
		_data = WL_FAILURE;
	}

	SLAVE_DESELECT;

	return ((int8_t) _data == WL_FAILURE) ? _data : (int8_t) WL_SUCCESS;
}

uint8_t ESP32getScanNetworks() {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(SCAN_NETWORKS, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t ssidListNum = 0;
	SPIwaitResponse(SCAN_NETWORKS, &ssidListNum, (uint8_t**) _networkSsid,
			WL_NETWORKS_LIST_MAXNUM);

	SLAVE_DESELECT;

	return ssidListNum;
}

const char* ESP32getSSIDNetworks(uint8_t networkItem) {
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return (char*) NULL;

	return _networkSsid[networkItem];
}

const char* ESP32getCurrentSSID() {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(GET_CURR_SSID_CMD, PARAM_NUMS_1);

	uint8_t _dummy = DUMMY_DATA;
	SPIsendParam(&_dummy, 1, LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	memset(_ssid, 0x00, sizeof(_ssid));

	// Wait for reply
	uint8_t _dataLen = 0;
	SPIwaitResponseCmd(GET_CURR_SSID_CMD, PARAM_NUMS_1, (uint8_t*) _ssid,
			&_dataLen);

	SLAVE_DESELECT;

	return _ssid;
}

int ESP32begin(const char *ssid, const char *passphrase) {
	uint8_t status = WL_IDLE_STATUS;
	uint8_t attempts = WL_MAX_ATTEMPT_CONNECTION;

	// set passphrase
	if (ESP32setPassphrase(ssid, strlen(ssid), passphrase, strlen(passphrase))
			!= WL_FAILURE) {
		do {
			_delay_ms(WL_DELAY_START_CONNECTION);
			status = ESP32getConnectionStatus();
		} while (((status == WL_IDLE_STATUS) || (status == WL_NO_SSID_AVAIL)
				|| (status == WL_SCAN_COMPLETED)) && (--attempts > 0));
	} else {
		status = WL_CONNECT_FAILED;
	}
	return status;
}

int8_t ESP32setPassphrase(const char *ssid, uint8_t ssid_len,
		const char *passphrase, const uint8_t len) {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(SET_PASSPHRASE_CMD, PARAM_NUMS_2);
	SPIsendParam((uint8_t*) ssid, ssid_len, NO_LAST_PARAM);
	SPIsendParam((uint8_t*) passphrase, len, LAST_PARAM);

	// pad to multiple of 4
	int commandSize = 6 + ssid_len + len;
	while (commandSize % 4) {
		SPIreadChar();
		commandSize++;
	}

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = 0;
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseCmd(SET_PASSPHRASE_CMD, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
		_data = WL_FAILURE;
	}
	SLAVE_DESELECT;
	return _data;
}

void ESP32getNetworkData(uint8_t *ip, uint8_t *mask, uint8_t *gwip) {
	tParam params[PARAM_NUMS_3] = { { 0, (char*) ip }, { 0, (char*) mask }, { 0,
			(char*) gwip } };

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(GET_IPADDR_CMD, PARAM_NUMS_1);

	uint8_t _dummy = DUMMY_DATA;
	SPIsendParam(&_dummy, sizeof(_dummy), LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	SPIwaitResponseParams(GET_IPADDR_CMD, PARAM_NUMS_3, params);

	SLAVE_DESELECT;
}

void ESP32getIpAddress(uint32_t *ip) {
	ESP32getNetworkData(_localIp, _subnetMask, _gatewayIp);
	*ip = ((uint32_t) _localIp[0] << 24) + ((uint32_t) _localIp[1] << 16)
			+ ((uint32_t) _localIp[2] << 8) + ((uint32_t) _localIp[3]);
}

int ESP32connect(wifiObject_t *client, wifiObject_t *host) {

	ESP32getHostByName(host->name, host->ip);
	host->ip32 = ((uint32_t) host->ip[3] << 24) + ((uint32_t) host->ip[2] << 16)
			+ ((uint32_t) host->ip[1] << 8) + ((uint32_t) host->ip[0]);

	if (client->sock != NO_SOCKET_AVAIL) {
		//stop();
		printf("No socket.\r\n");
		return 0;
	}

	client->sock = ESP32getSocket();
	if (client->sock != NO_SOCKET_AVAIL) {
		ESP32startClient(client, host, TCP_MODE);

		uint32_t i = 0;

		// wait 4 second for the connection to close
		while (!ESP32connected(*client) && i++ < 4000)
			_delay_ms(1);

		if (!ESP32connected(*client)) {
			client->sock = NO_SOCKET_AVAIL;
			return 0;
		}
	} else {
		printf("No Socket available\r\n");
		return 0;
	}
	return 1;
}


esp32_connect_status ESP32connectSSL(wifiObject_t *client, wifiObject_t *host) {
	if (client->sock != NO_SOCKET_AVAIL) {
		printf("No socket.\r\n");
		return ESP32_CONNECT_CLIENT_IN_USE;
	}

	client->sock = ESP32getSocket();
	if (client->sock != NO_SOCKET_AVAIL) {
		ESP32startClient(client, host, TLS_MODE);

		uint32_t i = 0;

		// wait 4 second for the connection to close
		while (!ESP32connected(*client) && i++ < 4000)
			_delay_ms(1);

		if (!ESP32connected(*client)) {
			printf("no connection\r\n");
			client->sock = NO_SOCKET_AVAIL;
			return ESP32_CONNECT_FAILED;
		}
	} else {
		printf("No Socket available\r\n");
		return ESP32_CONNECT_NOSOCK;
	}
	return ESP32_CONNECT_SUCCESS;
}

uint8_t ESP32getSocket() {
	uint8_t _data = -1;
	uint8_t _dataLen = 0;

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(GET_SOCKET_CMD, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	SPIwaitResponseCmd(GET_SOCKET_CMD, PARAM_NUMS_1, &_data, &_dataLen);

	SLAVE_DESELECT;

	return _data;
}

int ESP32getHostByName(const char *aHostname, uint8_t *aResult) {
	int result = 0;
	uint8_t _data = 0;
	uint8_t _dataLen = 0;

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Send Command
	SPIsendCmd(REQ_HOST_BY_NAME_CMD, PARAM_NUMS_1);
	SPIsendParam((uint8_t*) aHostname, strlen(aHostname), LAST_PARAM);

	// pad to multiple of 4
	int commandSize = 5 + strlen(aHostname);
	while (commandSize % 4) {
		SPIreadChar();
		commandSize++;
	}

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	result = SPIwaitResponseCmd(REQ_HOST_BY_NAME_CMD, PARAM_NUMS_1, &_data,
			&_dataLen);

	SLAVE_DESELECT;

	if (result) {
		result = (_data == 1);
	}

	if (!result) {
		return 0;
	}

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(GET_HOST_BY_NAME_CMD, PARAM_NUMS_0);

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	_dataLen = 0;
	if (!SPIwaitResponseCmd(GET_HOST_BY_NAME_CMD, PARAM_NUMS_1, aResult,
			&_dataLen)) {
		printf("error waitResponse\r\n");
	} else {
		result = (aResult[0] != 0xFF || aResult[1] != 0xFF || aResult[2] != 0xFF
				|| aResult[3] != 0xFF);
	}
	SLAVE_DESELECT;
	return result;
}

uint8_t ESP32connected(wifiObject_t object) {
	uint8_t sock = object.sock;
	if (sock == NO_SOCKET_AVAIL) {
		return 0;
	} else if (ESP32dataAvailable(object)) {
		return 1;
	} else {
		uint8_t s = ESP32status(sock);

		uint8_t result = !(s == LISTEN || s == CLOSED || s == FIN_WAIT_1
				|| s == FIN_WAIT_2 || s == TIME_WAIT || s == SYN_SENT
				|| s == SYN_RCVD || (s == CLOSE_WAIT));

		if (result == 0) {
			if (_buffers[sock].data) {
				free(_buffers[sock].data);
				_buffers[sock].data = _buffers[sock].head = NULL;
				_buffers[sock].length = 0;
			}
		}

		return result;
	}
}

uint8_t ESP32status(uint8_t sock) {
	if (sock == NO_SOCKET_AVAIL) {
		return CLOSED;
	} else {
		return ESP32getClientState(sock);
	}
}

uint8_t ESP32getClientState(uint8_t sock) {
	uint8_t _data = 0;
	uint8_t _dataLen = 0;

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(GET_CLIENT_STATE_TCP_CMD, PARAM_NUMS_1);
	SPIsendParam(&sock, sizeof(sock), LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	if (!SPIwaitResponseCmd(GET_CLIENT_STATE_TCP_CMD, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;

	return _data;
}

int ESP32dataAvailable(wifiObject_t object) {
	uint8_t sock = object.sock;
	if (sock != NO_SOCKET_AVAIL) {
		if (_buffers[sock].length == 0) {
			if (_buffers[sock].data == NULL) {
				_buffers[sock].data = _buffers[sock].head = (uint8_t*) malloc(
						WIFI_SOCKET_BUFFER_SIZE);
				_buffers[sock].length = 0;
			}

			// sizeof(size_t) is architecture dependent
			// but we need a 16 bit data type here
			uint16_t size = WIFI_SOCKET_BUFFER_SIZE;
			if (ESP32getDataBuf(sock, _buffers[sock].data, &size)) {
				_buffers[sock].head = _buffers[sock].data;
				_buffers[sock].length = size;
			}
		}
		return _buffers[sock].length;
	}

	return 0;
}

int ESP32getDataBuf(uint8_t sock, uint8_t *_data, uint16_t *_dataLen) {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(GET_DATABUF_TCP_CMD, PARAM_NUMS_2);
	SPIsendBuffer(&sock, sizeof(sock), NO_LAST_PARAM);
	SPIsendBuffer((uint8_t*) _dataLen, sizeof(*_dataLen), LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	if (!SPIwaitResponseData16(GET_DATABUF_TCP_CMD, _data, _dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;
	if (*_dataLen != 0) {
		return 1;
	}
	return 0;
}

void ESP32startClient(wifiObject_t *client, wifiObject_t *host, uint8_t protMode) {
	uint32_t ipAddress = host->ip32;
	uint16_t port = host->port;
	uint8_t sock = client->sock;
	uint8_t _data = 0;
	uint8_t _dataLen = 0;

	if (protMode == TCP_MODE) {
		while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
			; // wait for slave ready
		SLAVE_SELECT;

		// Send Command	
		SPIsendCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_4);
		SPIsendParam((uint8_t*) &ipAddress, sizeof(ipAddress), NO_LAST_PARAM);
		SPIsendParam16(port, NO_LAST_PARAM);
		SPIsendParam(&sock, 1, NO_LAST_PARAM);
		SPIsendParam(&protMode, 1, LAST_PARAM);

		SLAVE_DESELECT;
	} else if (protMode == TLS_MODE) {
		ipAddress = 0;
		uint8_t host_len = strlen(host->name);
		while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
			; // wait for slave ready
		SLAVE_SELECT;

		// Send Command    
		SPIsendCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_5);
		SPIsendParam((uint8_t*) host->name, host_len, NO_LAST_PARAM);
		SPIsendParam((uint8_t*) &ipAddress, sizeof(ipAddress), NO_LAST_PARAM);
		SPIsendParam16(port, NO_LAST_PARAM);
		SPIsendParam(&sock, 1, NO_LAST_PARAM);
		SPIsendParam(&protMode, 1, LAST_PARAM);

		// pad to multiple of 4
		int commandSize = 17 + host_len;
		while (commandSize % 4) {
			SPIreadChar();
			commandSize++;
		}

		SLAVE_DESELECT;
	} else {
		printf("Unknown protMode.\r\n");
		client->sock = NO_SOCKET_AVAIL;
		return;
	}
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	if (!SPIwaitResponseCmd(START_CLIENT_TCP_CMD, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;
}

uint16_t ESP32sendData(uint8_t sock, const char *data, uint16_t len) {
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(SEND_DATA_TCP_CMD, PARAM_NUMS_2);
	SPIsendBuffer(&sock, sizeof(sock), NO_LAST_PARAM);
	SPIsendBuffer((uint8_t*) data, len, LAST_PARAM);

	// pad to multiple of 4
	int commandSize = 9 + len;
	while (commandSize % 4) {
		SPIreadChar();
		commandSize++;
	}

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint16_t _data = 0;
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseData8(SEND_DATA_TCP_CMD, (uint8_t*) &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;

	return _data;
}

uint8_t ESP32checkDataSent(uint8_t sock) {
	const uint16_t TIMEOUT_DATA_SENT = 25;
	uint16_t timeout = 0;
	uint8_t _data = 0;
	uint8_t _dataLen = 0;

	do {
		while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
			; // wait for slave ready
		SLAVE_SELECT;
		// Send Command
		SPIsendCmd(DATA_SENT_TCP_CMD, PARAM_NUMS_1);
		SPIsendParam(&sock, sizeof(sock), LAST_PARAM);

		// pad to multiple of 4
		SPIreadChar();
		SPIreadChar();

		SLAVE_DESELECT;
		//Wait the reply elaboration
		while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
			; // wait for slave ready
		SLAVE_SELECT;

		// Wait for reply
		if (!SPIwaitResponseCmd(DATA_SENT_TCP_CMD, PARAM_NUMS_1, &_data,
				&_dataLen)) {
			printf("error waitResponse isDataSent\r\n");
		}
		SLAVE_DESELECT;

		if (_data)
			timeout = 0;
		else {
			++timeout;
			_delay_ms(100);
		}

	} while ((_data == 0) && (timeout < TIMEOUT_DATA_SENT));
	return (timeout == TIMEOUT_DATA_SENT) ? 0 : 1;
}

size_t ESP32write(wifiObject_t object, const char *buf, size_t size) {
	uint8_t sock = object.sock;
	if (sock == NO_SOCKET_AVAIL) {
		printf("Write Error\r\n");
		return 0;
	}
	if (size == 0) {
		printf("Write Error\r\n");
		return 0;
	}

	size_t written = ESP32sendData(sock, (char*) buf, size);
	if (!written) {
		printf("Write Error\r\n");
		return 0;
	}
	if (!ESP32checkDataSent(sock)) {
		printf("Write Error\r\n");
		return 0;
	}
	return written;
}

size_t ESP32printString(wifiObject_t object, const char *buf) {
	printf("%s", buf);
	return ESP32write(object, buf, strlen(buf));
}

int ESP32read(wifiObject_t client) {
	if (!ESP32dataAvailable(client)) {
		return -1;
	}

	uint8_t b;

	ESP32bufferRead(client.sock, &b, sizeof(b));

	return b;
}

int ESP32bufferAvailable(int socket) {
	if (_buffers[socket].length == 0) {
		if (_buffers[socket].data == NULL) {
			_buffers[socket].data = _buffers[socket].head = (uint8_t*) malloc(
					WIFI_SOCKET_BUFFER_SIZE);
			_buffers[socket].length = 0;
		}

		// sizeof(size_t) is architecture dependent
		// but we need a 16 bit data type here
		uint16_t size = WIFI_SOCKET_BUFFER_SIZE;
		if (ESP32getDataBuf(socket, _buffers[socket].data, &size)) {
			_buffers[socket].head = _buffers[socket].data;
			_buffers[socket].length = size;
		}
	}

	return _buffers[socket].length;
}

int ESP32bufferRead(int socket, uint8_t *data, size_t length) {
	int avail = ESP32bufferAvailable(socket);

	if (!avail) {
		return 0;
	}

	if (avail < (int) length) {
		length = avail;
	}

	memcpy(data, _buffers[socket].head, length);
	_buffers[socket].head += length;
	_buffers[socket].length -= length;

	return length;
}

void ESP32stop(wifiObject_t *object) {

	uint8_t sock = object->sock;
	if (sock == NO_SOCKET_AVAIL)
		return;

	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;
	// Send Command
	SPIsendCmd(STOP_CLIENT_TCP_CMD, PARAM_NUMS_1);
	SPIsendParam(&sock, 1, LAST_PARAM);

	// pad to multiple of 4
	SPIreadChar();
	SPIreadChar();

	SLAVE_DESELECT;
	//Wait the reply elaboration
	while (!(ESP32_SLAVEREADY_PIN & (1 << ESP32_SLAVEREADY)) == 0)
		; // wait for slave ready
	SLAVE_SELECT;

	// Wait for reply
	uint8_t _data = 0;
	uint8_t _dataLen = 0;
	if (!SPIwaitResponseCmd(STOP_CLIENT_TCP_CMD, PARAM_NUMS_1, &_data,
			&_dataLen)) {
		printf("error waitResponse\r\n");
	}
	SLAVE_DESELECT;

	int count = 0;
	// wait maximum 5 secs for the connection to close
	while (sock != NO_SOCKET_AVAIL && ESP32getClientState(sock) != CLOSED && ++count < 50)
		_delay_ms(100);

	ESP32socketBufferClose(sock);
	object->sock = NO_SOCKET_AVAIL;
}

void ESP32socketBufferClose(int socket) {
	if (_buffers[socket].data) {
		free(_buffers[socket].data);
		_buffers[socket].data = _buffers[socket].head = NULL;
		_buffers[socket].length = 0;
	}
}

