/*
 wifiSpiDrv.c - Library for Arduino Wifi shield.
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

#include <avr/io.h>
#include <util/delay.h>
#include "wifiSpiDrv.h"
#include "wifiDrv.h"
#include "ioE28.h"
#include "USARTE28.h"

uint8_t SpiDrvInitialized = 0;

/**
 * @fn void SPIinit(void)
 * @brief Initialize the SPI interface.
 *
 * Sets the SPI interface to 250KHz, master mode, clock idle
 * low, data driven on falling edge.
 *
 * The internal pullup on the Spi input data is enabled.
 *
 */
void SPIinit(void) {
	SPI_SS_DDR |= (1 << SPI_SS); /* set SS output */
	SPI_SS_PORT |= (1 << SPI_SS); /* start off not selected (high) */

	SPI_MOSI_DDR |= (1 << SPI_MOSI); /* output on MOSI */
	SPI_MISO_PORT |= (1 << SPI_MISO); /* pullup on MISO */
	SPI_SCK_DDR |= (1 << SPI_SCK); /* output on SCK */

	/* Don't have to set phase, polarity b/c
	 * default works with 25LCxxx chips */
	SPCR |= (1 << SPR1); /* div 64, safer for breadboards */
	SPCR |= (1 << MSTR); /* clockmaster */
	SPCR &= ~(1 << DORD); /* MSB first data order */
	SPCR |= (1 << SPE); /* enable */

	/* initialize control pins */
	ESP32_SLAVERESET_DDR |= (1 << ESP32_SLAVERESET); /* set SLAVERESET output */
	ESP32_SLAVEREADY_DDR &= ~(1 << ESP32_SLAVEREADY); /* set SLAVEREADY input */

	/* Reset ESP32 */
	ESP32_SLAVERESET_PORT &= ~(1 << ESP32_SLAVERESET);
	_delay_ms(10);
	ESP32_SLAVERESET_PORT |= (1 << ESP32_SLAVERESET);
	_delay_ms(750);
}

/**
 * @fn uint8_t SPItransfer(uint8_t)
 * @brief Transmit/receive one byte.
 *
 * Immediately begin shifting one byte of data.  Data is simultaneously
 * shifted in from the peripheral device.
 *
 * This function is blocking.
 *
 * @param data - the byte to send
 * @return the received data
 */
uint8_t SPItransfer(uint8_t data) {
	SPDR = data; /* SPI starts sending immediately */
	while (!(SPSR & (1 << SPIF)))
		; /* wait until done */
	return SPDR; /* return the received byte */
}

int SPIwaitChar(unsigned char waitChar) {
	int timeout = TIMEOUT_CHAR;
	unsigned char _readChar = 0;
	do {
		_readChar = SPIreadChar(); //get data byte
		if (_readChar == ERR_CMD) {
			printf("Err cmd received\n\r");
			return -1;
		}
	} while ((timeout-- > 0) && (_readChar != waitChar));
	return (_readChar == waitChar);
}

char SPIreadChar() {
	uint8_t readChar = 0;
	SPIgetParam(&readChar);
	return readChar;
}

void SPIgetParam(uint8_t *param) {
	// Get Params data
	*param = SPItransfer(DUMMY_DATA);
}

void SPIsendParam(uint8_t *param, uint8_t param_len, uint8_t lastParam) {
	int i = 0;
	// Send Spi paramLen
	SPIsendParamLen8(param_len);

	// Send Spi param data
	for (i = 0; i < param_len; ++i) {
		SPItransfer(param[i]);
	}

	// if lastParam==1 Send Spi END CMD
	if (lastParam == 1)
		SPItransfer(END_CMD);
}

void SPIsendParam16(uint16_t param, uint8_t lastParam) {
	// Send Spi paramLen
	SPIsendParamLen8(2);

	SPItransfer((uint8_t) ((param & 0xff00) >> 8));
	SPItransfer((uint8_t) (param & 0xff));

	// if lastParam==1 Send Spi END CMD
	if (lastParam == 1)
		SPItransfer(END_CMD);
}

void SPIsendParamLen8(uint8_t param_len) {
	// Send Spi paramLen
	SPItransfer(param_len);
}

void SPIsendParamLen16(uint16_t param_len) {
	// Send Spi paramLen
	SPItransfer((uint8_t) ((param_len & 0xff00) >> 8));
	SPItransfer((uint8_t) (param_len & 0xff));
}

int SPIwaitResponse(uint8_t cmd, uint8_t *numParamRead, uint8_t **params,
		uint8_t maxNumParams) {
	char _data = 0;
	int i = 0, ii = 0;
	uint8_t numParam;

	char *index[WL_SSID_MAX_LENGTH];

	for (i = 0; i < WL_NETWORKS_LIST_MAXNUM; i++)
		index[i] = (char*) params + WL_SSID_MAX_LENGTH * i;

	if (!SPIwaitChar(START_CMD)) {
		printf("Error waiting START_CMD\n\r");
		return 0;
	} else {
		if (!SPIreadAndCheckChar(cmd | REPLY_FLAG, &_data)) {
			printf("Reply error\n\r");
			return 0;
		}
		numParam = SPIreadChar();
		if (numParam > maxNumParams) {
			numParam = maxNumParams;
		}
		*numParamRead = numParam;
		if (numParam != 0) {
			for (i = 0; i < numParam; ++i) {
				uint8_t paramLen = SPIreadParamLen8(NULL);
				for (ii = 0; ii < paramLen; ++ii) {
					index[i][ii] = (uint8_t) SPItransfer(DUMMY_DATA);
				}
				index[i][ii] = 0;
			}
		} else {
			printf("Error numParams == 0\n\r");
			SPIreadAndCheckChar(END_CMD, &_data);
			return 0;
		}
		SPIreadAndCheckChar(END_CMD, &_data);
	}
	return 1;
}

int SPIreadAndCheckChar(char checkChar, char *readChar) {
	SPIgetParam((uint8_t*) readChar);

	return (*readChar == checkChar);
}

int SPIwaitResponseCmd(uint8_t cmd, uint8_t numParam, uint8_t *param,
		uint8_t *param_len) {
	char _data = 0;
	int ii = 0;

	if (!SPIwaitChar(START_CMD)) {
		printf("Error waiting START_CMD\n\r");
		return 0;
	} else {
		if (!SPIreadAndCheckChar(cmd | REPLY_FLAG, &_data)) {
			printf("Reply 1 error\n\r");
			return 0;
		}
		if (!SPIreadAndCheckChar(numParam, &_data)) {
			printf("Expected %d and received %d.\r\n", numParam, _data);
			printf("Reply 2 error\n\r");
			return 0;
		} else {
			SPIreadParamLen8(param_len);
			for (ii = 0; ii < (*param_len); ++ii) {
				SPIgetParam(&param[ii]);
			}
		}
		SPIreadAndCheckChar(END_CMD, &_data);
	}

	return 1;
}

void SPIsendCmd(uint8_t cmd, uint8_t numParam) {
	// Send Spi START CMD
	SPItransfer(START_CMD);

	// Send Spi C + cmd
	SPItransfer(cmd & ~(REPLY_FLAG));

	// Send Spi totLen
	//spiTransfer(totLen);

	// Send Spi numParam
	SPItransfer(numParam);

	// If numParam == 0 send END CMD
	if (numParam == 0)
		SPItransfer(END_CMD);

}

int SPIwaitResponseParams(uint8_t cmd, uint8_t numParam, tParam *params) {
	char _data = 0;
	int i = 0, ii = 0;
	uint8_t _numParam;

	if (!SPIwaitChar(START_CMD)) {
		printf("Error waiting START_CMD\n\r");
		return 0;
	} else {
		if (!SPIreadAndCheckChar(cmd | REPLY_FLAG, &_data)) {
			printf("Reply error\n\r");
			return 0;
		}

		_numParam = SPIreadChar();
		if (_numParam != 0) {
			for (i = 0; i < _numParam; ++i) {
				params[i].paramLen = SPIreadParamLen8(NULL);
				for (ii = 0; ii < params[i].paramLen; ++ii) {
					// Get Params data
					params[i].param[ii] = SPItransfer(DUMMY_DATA);
				}
			}
		} else {
			printf("Error numParam == 0\n\r");
			return 0;
		}

		if (numParam != _numParam) {
			printf("Mismatch numParam\n\r");
			return 0;
		}

		SPIreadAndCheckChar(END_CMD, &_data);
	}
	return 1;
}

int SPIwaitResponseData16(uint8_t cmd, uint8_t *param, uint16_t *param_len) {
	char _data = 0;
	uint16_t ii = 0;
	uint8_t numParam;

	if (!SPIwaitChar(START_CMD)) {
		printf("Error waiting START_CMD\n\r");
		return 0;
	} else {
		if (!SPIreadAndCheckChar(cmd | REPLY_FLAG, &_data)) {
			printf("Reply error\n\r");
			return 0;
		}
		numParam = SPIreadChar();
		if (numParam != 0) {
			SPIreadParamLen16(param_len);
			for (ii = 0; ii < (*param_len); ++ii) {
				// Get Params data
				param[ii] = SPItransfer(DUMMY_DATA);
			}
		}

		SPIreadAndCheckChar(END_CMD, &_data);
	}

	return 1;
}

int SPIwaitResponseData8(uint8_t cmd, uint8_t *param, uint8_t *param_len) {
	char _data = 0;
	int ii = 0;
	uint8_t numParam;

	if (!SPIwaitChar(START_CMD)) {
		printf("Error waiting START_CMD\n\r");
		return 0;
	} else {
		if (!SPIreadAndCheckChar(cmd | REPLY_FLAG, &_data)) {
			printf("Reply error\n\r");
			return 0;
		}
		numParam = SPIreadChar();
		if (numParam != 0) {
			SPIreadParamLen8(param_len);
			for (ii = 0; ii < (*param_len); ++ii) {
				// Get Params data
				param[ii] = SPItransfer(DUMMY_DATA);
			}
		}

		SPIreadAndCheckChar(END_CMD, &_data);
	}

	return 1;
}

void SPIsendBuffer(uint8_t *param, uint16_t param_len, uint8_t lastParam) {
	uint16_t i = 0;

	// Send Spi paramLen
	SPIsendParamLen16(param_len);

	// Send Spi param data
	for (i = 0; i < param_len; ++i) {
		SPItransfer(param[i]);
	}

	// if lastParam==1 Send Spi END CMD
	if (lastParam == 1)
		SPItransfer(END_CMD);
}

uint8_t SPIreadParamLen8(uint8_t *param_len) {
	uint8_t _param_len = SPItransfer(DUMMY_DATA);
	if (param_len != NULL) {
		*param_len = _param_len;
	}
	return _param_len;
}

uint16_t SPIreadParamLen16(uint16_t *param_len) {
	uint16_t _param_len = SPItransfer(DUMMY_DATA) << 8
			| (SPItransfer(DUMMY_DATA) & 0xff);
	if (param_len != NULL) {
		*param_len = _param_len;
	}
	return _param_len;
}
