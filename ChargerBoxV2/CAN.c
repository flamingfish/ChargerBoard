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
#include <stdbool.h>
#include "avr/interrupt.h"
#include <avr/pgmspace.h>
#include "logger.h"

#include "USB.h"

// For READ_RX_BUFFER instruction
#define START_AT_RXBnSIDH 0
#define START_AT_RXBnD0 1

// Instruction codes
#define RESET_INSTRUC 0xC0
#define READ_INSTRUC 0x03
// Read receive buffer (n for buffer num, m=0 to start at RXBnSIDH, m=1 to start at RXBnD0)
#define READ_RX_BUFFER_INSTRUC(n, m) (0x90 | ((n & 0x01) << 2) | ((m & 0x01) << 1))
#define WRITE_INSTRUC 0x02
// Request to send (nnn are last 3 bits, 1 for each transmit buffer)
#define RTS_INSTRUC(nnn) (0x80 | (nnn & 0x07))
#define READ_STATUS_INSTRUC 0xA0
#define RX_STATUS_INSTRUC 0xB0
#define BIT_MODIFY_INSTRUC 0x05

#define BMS_INFO_MESSAGE_ID 0x1806E5F4L

typedef struct {
	uint8_t data[8];
	uint8_t len;
	bool flag;
	uint32_t canID;
	uint8_t ide; // extended identifier flag
	uint8_t ssr; // standard frame remote transmit request bit (valid only if ide = 0)
} RXBuffer;

// Buffers to hold received data;
static RXBuffer rxBuffers[2] = {0}; // initialise everything to 0
	
static volatile bool tx0empty = true;
static volatile bool tx1empty = true;
static volatile bool tx2empty = true;

static void reset() {
	uint8_t data[1] = {RESET_INSTRUC};
	writeSPI(data, 1);
}

 uint8_t readRegister(uint8_t address) {
	uint8_t result;
	uint8_t data[2] = {READ_INSTRUC, address};
	readSPI(data, 2, &result, 1);
	return result;
}

static void writeRegister(uint8_t address, uint8_t data) {
	uint8_t writeData[3] = {WRITE_INSTRUC, address, data};
	writeSPI(writeData, 3);
}

//static void asyncReadRegister(uint8_t address, uint8_t* readData, uint8_t readLen, void (*callback)(uint8_t*)) {
	//uint8_t writePreamble[2] = {READ_INSTRUC, address};
	//asyncReadSPI(writePreamble, 2, readLen, callback);
//}
//
//static void asyncWriteRegister(uint8_t address, uint8_t* data, uint8_t dataLen) {
	//uint8_t sendData[dataLen + 2] = {WRITE_INSTRUC, address};
	//for (uint8_t i = 0; i < dataLen; i++) {
		//sendData[i + 2] = data[i];
	//}
	//asyncWriteSPI(sendData, dataLen, 0);
//}

//static void requestToSend(uint8_t bufferNum) {
	//// Look at schematic:
	//// PD3 -> TX0RTS
	//// PD2 -> TX1RTS
	//// PD1 -> TX2RTS
	//uint8_t pinNum = 3 - bufferNum;
	//
	//// Pulse the RTS line low
	//PORTD &= ~(1 << pinNum);
	//PORTD |= (1 << pinNum);
//}

static void requestToSendBuffer0() {
	//printf_P(PSTR("Inside request to send buffer 0\r\n"));
	CAN_DEBUG_LOG("Requesting to send buffer TX0\r\n");
	// Pulse the line low
	PORTD &= ~(1 << 3);
	PORTD |= (1 << 3);
}

static void requestToSendBuffer1() {
	// Pulse the line low
	PORTD &= ~(1 << 2);
	PORTD |= (1 << 2);
}

static void requestToSendBuffer2() {
	PORTD &= ~(1 << 1);
	PORTD |= (1 << 1);
}

void setupCAN() {
	// Reset the MCP2515 (also initialises it to configuration mode)
	reset();
	_delay_ms(1);
	
	// Clear masks to RX all messages
	writeRegister(RXM0SIDL, 0x00);
	writeRegister(RXM0SIDH, 0x00);
	writeRegister(RXM0EID0, 0x00);
	writeRegister(RXM0EID8, 0x00);
	writeRegister(RXM1SIDL, 0x00);
	writeRegister(RXM1SIDH, 0x00);
	writeRegister(RXM1EID0, 0x00);
	writeRegister(RXM1EID8, 0x00);
	
	// Clear filters (but set to receive extended IDs)
	writeRegister(RXF0SIDL, EXIDE_SET);
	writeRegister(RXF0SIDH, 0x00);
	writeRegister(RXF0EID0, 0x00);
	writeRegister(RXF0EID8, 0x00);
	writeRegister(RXF1SIDL, EXIDE_SET);
	writeRegister(RXF1SIDH, 0x00);
	writeRegister(RXF1EID0, 0x00);
	writeRegister(RXF1EID8, 0x00);
	
	
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
	//writeRegister(CANINTE, RX0IE | RX1IE);
	
	// Interrupt on Error:
	// No need to interrupt on RX buffer full - they have individual interrupt lines
	writeRegister(CANINTE, ERRIE);
	
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
	
	// Setup Atmega32u2 hardware
	// Set PD1, PD2, PD3 as high
	PORTD |= (1 << 1) | (1 << 2) | (1 << 3);
	// make PD1, PD2, PD3 outputs (for request to send buffers)
	DDRD |= (1 << 1) | (1 << 2) | (1 << 3);
	// Make PD7 and PD4, PD6 an input (should already be default)
	DDRD &= ~(1 << 7) & ~(1 << 4) & ~(1 << 6);
	
	// Falling edge interrupt on INT7, INT6 and INT5
	EICRB |= (1 << ISC71) | (1 << ISC61) | (1 << ISC51);
	// Ensure interrupt flag is cleared
	EIFR = (1 << INTF7) | (1 << INTF6) | (1 << INTF5);
	// Enable interrupt on INT7 (PD7), INT6 (PD6) and INT5 (PD4)
	EIMSK |= (1 << INT7) | (1 << INT6) | (1 << INT5);
	
	// Enable request to send pins for all transmission buffers
	writeRegister(TXRTSCTRL, 0x07);
	
	// Enable receive interrupt pins
	writeRegister(BFPCTRL, B0BFM | B1BFM | B0BFE | B1BFE);
	
	// Enter loopback mode for testing
	writeRegister(CANCTRL, REQOP_LOOPBACK);
	uint8_t dummy = readRegister(CANSTAT);
	if (OPMODE_NORMAL != (dummy & 0xe0)) { // This is wrong for loopback mode (correct for normal mode)
		printf("device did not enter loopback mode, retrying...\r\n");
		writeRegister(CANCTRL, REQOP_LOOPBACK);
	}
	
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
	writeRegister(TXB0DLC, dataLen & 0x0f);
	
	for (uint8_t i = 0; i < dataLen; i++) {
		writeRegister(TXB0D0 + i, data[i]);
	}
	//writeRegister(TXB0D0, (uint8_t) 'H');
	//writeRegister(TXB0D1, (uint8_t) 'i');
	//writeRegister(TXB0D2, (uint8_t) '\0');
	
	// Request to send
	writeRegister(TXB0CTRL, TXP_HIGHEST | TXREQ_SET);
}

#ifdef CAN_DEBUG_LOGGING
void printResult(uint8_t* data) {
	CAN_DEBUG_LOG("Data received: 0x%02x 0x%02x 0x%02x 0x%02x\r\n", data[0], data[1], data[2], data[3]);
}
#endif

void setCANID(uint8_t bufferNum, uint32_t id) {
	uint8_t* part = (uint8_t*) &id;
	uint8_t buffer[6] = {WRITE_INSTRUC};
	
	// Set the address for the right buffer
	switch (bufferNum) {
	case 0:
		buffer[1] = TXB0SIDH;
		break;
	case 1:
		buffer[1] = TXB1SIDH;
		break;
	case 2:
		buffer[1] = TXB2SIDH;
		break;
	default:
		return;
	}
	
	// TXBnSIDH, TXBnSIDL, TXBnEID8 and TXBnEID0 are all contiguous in memory
	// Therefore, can use a single SPI write
	// TXBnEID0:
	buffer[5] = part[0];
	// TXBnEID8
	buffer[4] = part[1];
	// TXBnSIDL (extended identifier set)
	buffer[3] = EXIDE_SET | (part[2] & 0x03) | ((part[2] & 0x1c) << 3);
	// TXBnSIDH
	buffer[2] = ((part[2] & 0xe0) >> 5) | ((part[3] & 0x1f) << 3);
	asyncWriteSPI(buffer, 6, 0);
	
	//asyncWriteRegister()
//#ifdef DEBUG_LOGGING
	//uint8_t result[6];
	//uint8_t preamble[2] = {READ_INSTRUC, buffer[1]};
	//asyncReadSPI(preamble, 2, 6, printResult);
//#endif
}


void sendCANMessage(uint8_t bufferNum, uint8_t* data, uint8_t dataLen) {
	uint8_t buffer[11] = {WRITE_INSTRUC};
		
	// Set the address for the right buffer
	switch (bufferNum) {
	case 0:
		tx0empty = false;
		buffer[1] = TXB0DLC;
		break;
	case 1:
		tx1empty = false;
		buffer[1] = TXB1DLC;
		break;
	case 2:
		tx2empty = false;
		buffer[1] = TXB2DLC;
		break;
	default:
		return;
	}
	
	// Indicate data frame and not remote transmit request frame
	// also specify data length (data length code bits)
	buffer[2] = dataLen & 0x0f;
	
	
	for (uint8_t i = 0; i < dataLen; i++) {
		buffer[i + 3] = data[i];
	}
	
	void (*callback)(void) = 0;
	switch (bufferNum) {
	case 0:
		callback = requestToSendBuffer0;
		break;
	case 1:
		callback = requestToSendBuffer1;
		break;
	case 2:
		callback = requestToSendBuffer2;
		break;
	}
	
	asyncWriteSPI(buffer, 11, callback);
}


void updateCAN() {
	for (uint8_t i = 0; i < 2; i++) {
		if (rxBuffers[i].flag) {
			DEBUG_LOG("Received data from RX%d: %s\r\n", i, rxBuffers[i].data);
			rxBuffers[i].flag = false;
		}
	}
}

static void extractRXBuffer(uint8_t bufferNum, uint8_t* data) {
	RXBuffer* buffer = &rxBuffers[bufferNum];
	
	// AVR is little endian
	uint8_t* canIDPart = (uint8_t*) &buffer->canID;
	// RXBnEID0
	canIDPart[0] = data[3];
	// RXBnEID8
	canIDPart[1] = data[2];
	// RXBnSIDL = data[2] and RXBnSIDH = data[1]
	canIDPart[2] = (data[1] & 0x03) | ((data[1] & 0xe0) >> 3) | ((data[0] & 0x03) << 5);
	canIDPart[3] = ((data[0] & 0xf8) >> 3);
	
	// RXBnDLC
	buffer->len = data[4] & 0x0f;
	
	// Now extract the data
	for (uint8_t i = 0; i < buffer->len; i++) {
		buffer->data[i] = data[i + 5];
	}
	
	buffer->flag = true;
}

static void onReceiveRX0(uint8_t* data) {
	extractRXBuffer(0, data);
}

static void onReceiveRX1(uint8_t* data) {
	extractRXBuffer(1, data);
}

#define RX1OVR 0x80 // RX1 overflow
#define RX0OVR 0x40 // RX0 overflow
#define TXBO 0x20 // Bus-Off Error Flag Bit
#define TXEP 0x10 // Transmit error-passive flag bit
#define RXEP 0x08 // Receive error-passive flag bit
#define TXWAR 0x04 // Transmit Error Warning Flag bit
#define RXWAR 0x02 // Receive Error Warning Flag bit
#define EWARN 0x01 // Error Warning Flag bit == TXWAR | RXWAR

// When the INT pin is driven low
static void onInterrupt(uint8_t* data) {
	uint8_t canintf = data[0];
	uint8_t eflg = data[1];
	if (canintf & ERRIF) {
		// Error interrupt flag is set
		if (eflg & RX0OVR) {
			CAN_DEBUG_LOG("RX0 buffer has overflown\r\n");
			// RX0 buffer has overflown
		}
		if (eflg & RX1OVR) {
			CAN_DEBUG_LOG("RX1 buffer has overflown\r\n");
			// RX1 buffer has overflown
		}
		if (eflg & TXBO) {
			CAN_DEBUG_LOG("CAN in Bus-Off state (nothing connected)\r\n");
			// Bus-Off state - nothing connected to CAN
		}
		if (eflg & TXEP) {
			CAN_DEBUG_LOG("CAN in transmit error-passive state\r\n");
		}
		if (eflg & RXEP) {
			CAN_DEBUG_LOG("CAN in receive error-passive state\r\n");
		}
		if (eflg & TXWAR) {
			CAN_DEBUG_LOG("CAN in transmit error warning state\r\n");
		}
		if (eflg & RXWAR) {
			CAN_DEBUG_LOG("CAN in receive error warning state\r\n");
		}
		if (eflg & EWARN) {
			CAN_DEBUG_LOG("CAN in error warning state\r\n");
		}
	}
	
	uint8_t canintfMask = 0x00;
	if (canintf & TX0IF) {
		// TX0 buffer is empty and ready to send another message
		tx0empty = true;
		canintfMask |= TX0IF;
	}
	if (canintf & TX1IF) {
		// TX1 buffer is empty and ready to send another message
		tx1empty = true;
		canintfMask |= TX1IF;
	}
	if (canintf & TX2IF) {
		// TX2 buffer is empty and ready to send another message
		tx2empty = true;
		canintf |= TX2IF;
	}
	if (canintfMask) { // If there is actually a newly emptied buffer:
		uint8_t writeData[3] = {BIT_MODIFY_INSTRUC, canintfMask, TX0IF | TX1IF | TX2IF};
		asyncWriteSPI(writeData, 3, NULL); // actually want to set the tx-empty flags in clalback here
	}
}

// RX0 buffer is full (received message)
ISR(INT6_vect) {
	CAN_DEBUG_LOG("Received message from RX0\r\n");
	//uint8_t preamble[2] = {READ_INSTRUC, RXB0CTRL};
	//asyncReadSPI(preamble, 2, 14, onReceiveRX0);
	uint8_t preamble[1] = {READ_RX_BUFFER_INSTRUC(0, START_AT_RXBnSIDH)};
	//asyncReadSPI(preamble, 1, 13, onReceiveRX0);
}

// RX1 buffer is full (received message)
ISR(INT5_vect) {
	CAN_DEBUG_LOG("Received message from RX1\r\n");
	//uint8_t preamble[2] = {READ_INSTRUC, RXB1CTRL};
	//asyncReadSPI(preamble, 2, 14, onReceiveRX1);
	uint8_t preamble[1] = {READ_RX_BUFFER_INSTRUC(1, START_AT_RXBnSIDH)};
	asyncReadSPI(preamble, 1, 13, onReceiveRX1);
}

ISR(INT7_vect) {
	CAN_DEBUG_LOG("Received CAN interrupt!\r\n");
	uint8_t preamble[2] = {READ_INSTRUC, CANINTF}; // CANINFT and EFLG are next to each other
	asyncReadSPI(preamble, 2, 2, onInterrupt);
	//uint8_t interruptFlags = readRegister(CANINTF);
	//uint8_t rxb0ctrl = readRegister(RXB0CTRL);
	//uint8_t rxb1ctrl = readRegister(RXB1CTRL);
	//uint8_t eflg = readRegister(EFLG);
	//CAN_DEBUG_LOG("Received CAN interrupt!\r\nCANINTF: 0x%02x\r\nRXB0CTRL: 0x%02x\r\nRXB1CTRL: 0x%02x\r\nEFLG: 0x%02x\r\n", interruptFlags, rxb0ctrl, rxb1ctrl, eflg);

}