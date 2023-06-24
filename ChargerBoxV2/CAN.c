/*
 * CAN.c
 *
 * Created: 23/06/2023 4:05:09 PM
 *  Author: Michael Jones
 */ 

#include "SPI.h"

// Instruction codes
#define RESET_INSTRUC 0xC0
#define READ_INSTRUC 0x03
#define WRITE_INSTRUC 0x02
// Request to send (nnn are last 3 bits, 1 for each transmit buffer)
#define RTS_INSTRUC(nnn) (0x80 | (nnn & 0x07))
#define READ_STATUS_INSTRUC 0xA0
#define RX_STATUS_INSTRUC 0xB0
#define BIT_MODIFY 0x05

void setupCAN() {
	
}

static void reset() {
	char data[1] = {RESET_INSTRUC};
	writeSPI(data, 1);
}

char readRegister(char address) {
	char result;
	char data[2] = {READ_INSTRUC, address};
	readSPI(data, 2, &result, 1);
	return result;
}

static void writeRegister(char address, char data) {
	char writeData[3] = {WRITE_INSTRUC, address, data};
	writeSPI(writeData, 3);
}