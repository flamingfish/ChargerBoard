/*
 * CAN.c
 *
 * Created: 23/06/2023 4:05:09 PM
 *  Author: Michael Jones
 */ 

#include "SPI.h"
#include "REGS2515.h"
#include "util/delay.h"
#include <stdio.h>
#include "avr/interrupt.h"

#include "USB.h"

// Instruction codes
#define RESET_INSTRUC 0xC0
#define READ_INSTRUC 0x03
#define WRITE_INSTRUC 0x02
// Request to send (nnn are last 3 bits, 1 for each transmit buffer)
#define RTS_INSTRUC(nnn) (0x80 | (nnn & 0x07))
#define READ_STATUS_INSTRUC 0xA0
#define RX_STATUS_INSTRUC 0xB0
#define BIT_MODIFY 0x05

#define BMS_INFO_MESSAGE_ID 0x1806E5F4L

static void reset() {
	uint8_t data[1] = {RESET_INSTRUC};
	writeSPI(data, 1);
}

static uint8_t readRegister(uint8_t address) {
	uint8_t result;
	uint8_t data[2] = {READ_INSTRUC, address};
	readSPI(data, 2, &result, 1);
	return result;
}

void writeRegister(uint8_t address, uint8_t data) {
	uint8_t writeData[3] = {WRITE_INSTRUC, address, data};
	writeSPI(writeData, 3);
}

void setupCAN() {
	// Reset the MCP2515 (also initialises it to configuration mode)
	reset();
	_delay_ms(1);
	
	// Clear masks to RX all messages
	writeRegister(RXM0SIDH, 0x00);
	writeRegister(0x20, 0x00);
	writeRegister(RXM0SIDL, 0x00);
	
	// Clear filter
	writeRegister(RXF0SIDL, 0x00);
	
	
	// Timing based on 8 TQ (time quantum) per bit time
	// 1 For sync segment (fixed)
	// 1 For propagation segment (programmable)
	// 3 For phase segment 1 (PS1) (programmable)
	//     (Sample point in here)
	// 3 for phase segment 2 (PS2) (programmable)
	
	// Set CNF1
	// Synchronisation jump width length bits = 1 TQ
	// Baud rate prescaler gives fastest possible (F_OSC/2 = 8MHz)
	writeRegister(CNF1, 0x00);
	
	// Set CNF2
	// Propagation segment = 1TQ
	// Phase segment 1 (PS1) = 3TQ : PHSEG1 = 2 (0b010)
	// BTLMODE = 1 (PS2 is determined in CNF3 register)
	// SAM = 0 (sampled once at sampling point)
	writeRegister(CNF2, (1 << 4) | (1 << 7));
	
	// Set CNF3
	// Phase segment 2 (PS2) = 3TQ : PHSEG2 = 2 (0b010)
	// WAKFIL = 0 (wakeup filter disabled)
	// SOF = 0 (CLKOUT pin enabled for clock out function)
	writeRegister(CNF3, (1 << 1));
	
	// Interrupt on RXB0 full - CANINTE
	// Also on TX0IE empty
	//writeRegister(CANINTE, RX0IE | TX0IE);
	writeRegister(CANINTE, RX0IE | RX1IE);
	
	// Set NORMAL mode
	// One-shot mode disabled (will attempt to retransmit message)
	// CLKOUT pin is disabled
	//writeRegister(CANCTRL, 0x00);
	
	// Verify device entered normal mode
	//uint8_t dummy = readRegister(CANSTAT);
	//printf("CANSTAT: 0x%02x\r\n", dummy);
	//if (OPMODE_NORMAL != (dummy & 0xe0)) {
		//printf("device did not enter normal mode, retrying...\r\n");
		//writeRegister(CANCTRL, 0x00);
	//}
	
	// Enter loopback mode for testing
	writeRegister(CANCTRL, REQOP_LOOPBACK);
	uint8_t dummy = readRegister(CANSTAT);
	if (OPMODE_NORMAL != (dummy & 0xe0)) {
		printf("device did not enter loopback mode, retrying...\r\n");
		writeRegister(CANCTRL, REQOP_LOOPBACK);
	}
	
	
	// Setup Atmega32u2 hardware
	// Make PD7 an input (should already be default)
	DDRD &= ~(1 << ISC71);
	// Enable interrupt on INT7 (PD7)
	EIMSK |= (1 << INT7);
	// Active low interrupt on INT7
	EICRB |= (1 << 7);
	// Ensure interrupt flag is cleared
	EIFR = (1 << INTF7);
	
	printf("inside setupCAN\r\n");
	updateUSB();
}

// Uses extended Identifier
void writeCAN(uint32_t canID, uint8_t* data, uint8_t dataLen) {
	printf("writing data: %s\r\n", (char*) data);
	// Transmit buffer 0 control register
	// writeRegister(TXB0CTRL, 0x03); // highest priority // done at end
	
	// The following writes may be able to be done contiguously in a single SPI message.
	
	// AVR is little endian
	// EXIDE_SET sets use of extended identifier
	// EID is extended id (18 bits)
	// SID is standard id (11 bits)
	uint8_t* part = (uint8_t*) &canID;
	writeRegister(TXB0EID0, part[0]);
	writeRegister(TXB0EID8, part[1]);
	writeRegister(TXB0SIDL, EXIDE_SET | (part[2] & 0x03) | ((part[2] & 0x1c) << 3));
	writeRegister(TXB0SIDH, ((part[2] & 0xe0) >> 5) | ((part[3] & 0x1f) << 3));
	
	// Indicate data frame and not remote transmit request frame
	// also specify data length (data length code bits)
	writeRegister(TXB0DLC, dataLen & 0x07);
	
	for (uint8_t i = 0; i < dataLen; i++) {
		writeRegister(TXB0D0 + i, data[i]);
	}
	//writeRegister(TXB0D0, (uint8_t) 'H');
	//writeRegister(TXB0D1, (uint8_t) 'i');
	//writeRegister(TXB0D2, (uint8_t) '\0');
	
	// Request to send
	writeRegister(TXB0CTRL, TXP_HIGHEST | TXREQ_SET);
}

void updateCAN() {
	
}

ISR(INT7_vect) {
	uint8_t interruptFlags = readRegister(CANINTF);
	printf("Received CAN interrupt! flags: 0x%02x\r\n", interruptFlags);
	if (RX0IF_SET == (interruptFlags & RX0IF)) {
		// A message is received in RX buffer 0
		uint32_t canID = 0;
		uint8_t* part = (uint8_t*) &canID;
		uint8_t value;
		part[0] = readRegister(RXB0EID0);
		part[1] = readRegister(RXB0EID8);
		value = readRegister(RXB0SIDL);
		part[2] = (value & 0x3) | ((value & 0xe0) >> 3);
		uint8_t ide = !!(value & (1 << 3)); // extended identifier flag
		uint8_t ssr = !!(value & (1 << 4)); // standard frame remote transmit request bit (valid only if ide = 0)
		value = readRegister(RXB0SIDH);
		part[2] |= (value & 0x03) << 5;
		part[3] = (value & 0xf8) >> 3;
		
		// Get DLC (data length)
		value = readRegister(RXB0DLC);
		uint8_t dataLen = value & 0x0f;
		char data[dataLen];
		
		for (uint8_t i = 0; i < dataLen; i++) {
			data[i] = (char) readRegister(RXB0D0 + i);
		}
		printf("Received data in RXB0: %s\r\n", data);
	} else if (RX1IF_SET == (interruptFlags & RX1IF)) {
		// A message is received in RX buffer 0
		uint32_t canID = 0;
		uint8_t* part = (uint8_t*) &canID;
		uint8_t value;
		part[0] = readRegister(RXB1EID0);
		part[1] = readRegister(RXB1EID8);
		value = readRegister(RXB1SIDL);
		part[2] = (value & 0x3) | ((value & 0xe0) >> 3);
		uint8_t ide = !!(value & (1 << 3)); // extended identifier flag
		uint8_t ssr = !!(value & (1 << 4)); // standard frame remote transmit request bit (valid only if ide = 0)
		value = readRegister(RXB1SIDH);
		part[2] |= (value & 0x03) << 5;
		part[3] = (value & 0xf8) >> 3;
			
		// Get DLC (data length)
		value = readRegister(RXB1DLC);
		uint8_t dataLen = value & 0x0f;
		char data[dataLen];
		//char data[3];
		//data[0] = readRegister(RXB1D0);
		//data[1] = readRegister(RXB1D1);
		//data[2] = readRegister(RXB1D2);
			
		for (uint8_t i = 0; i < dataLen; i++) {
			data[i] = (char) readRegister(RXB1D0 + i);
		}
		//printf("data0: %d\r\n", data[0]);
		//printf("data1: %d\r\n", data[1]);
		//printf("data2: %d\r\n", data[2]);
		
		printf("Received data: %s with CAN id: 0x%lx in RXB1 with length: %d\r\n", data, canID, dataLen);
	}
}