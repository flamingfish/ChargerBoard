/*
 * PCComms.c
 *
 * Created: 24/07/2023 3:23:58 PM
 *  Author: Michael Jones
 */ 

#include "PCComms.h"
#include <stdio.h>
#include "logger.h"

#define COMMS_START_BYTE 0xff
#define COMMS_END_BYTE 0x00

typedef enum PCCommsState {
	COMMS_IDLE,
	COMMS_GET_ID,
	COMMS_GET_TYPE,
	COMMS_GET_LENGTH,
	COMMS_GET_CHECKSUM,
	COMMS_GET_DATA,
	COMMS_GET_END_BYTE,
	COMMS_RECEIVED,
	COMMS_ERROR
} PCCommsState;

static PCCommsMessage receiveMessage;

static PCCommsState state;
static uint8_t idCounter;
static uint8_t checksumCounter;
static uint8_t dataCounter;

void setupPCComms() {
	state = COMMS_IDLE;
	idCounter = 0;
	checksumCounter = 0;
	dataCounter = 0;
}


void updatePCComms() {
	if (state == COMMS_RECEIVED) {
		// If a message is received but not yet read, don't do getchar()
		// -> leave it in the internal buffer for later.
		return;
	}
	
	int16_t result;
	uint8_t* part = 0; // Used to point to multi-byte metadata
	while ((result = getchar()) >= 0) {
		switch (state) {
		case COMMS_ERROR: // fall through
		case COMMS_IDLE:
			if (result == COMMS_START_BYTE) {
				state = COMMS_GET_ID;
			} else {
				state = COMMS_ERROR;
			}
			break;
			//state = COMMS_GET_ID;
			// Fall through
		case COMMS_GET_ID:
			part = (uint8_t*) &receiveMessage.id;
			part[3 - idCounter] = (uint8_t) result; // AVR is little endian
			if (++idCounter == 4) {
				idCounter = 0;
				state = COMMS_GET_TYPE;
			}
			break;
		case COMMS_GET_TYPE:
			receiveMessage.type = (uint8_t) result;
			state = COMMS_GET_LENGTH;
			break;
		case COMMS_GET_LENGTH:
			receiveMessage.length = (uint8_t) result;
			state = COMMS_GET_CHECKSUM;
			break;
		case COMMS_GET_CHECKSUM:
			part = (uint8_t*) &receiveMessage.checksum;
			part[1 - checksumCounter] = (uint8_t) result;
			if (++checksumCounter == 2) {
				checksumCounter = 0;
				state = COMMS_GET_DATA;
			}
			break;
		case COMMS_GET_DATA:
			receiveMessage.data[dataCounter] = (uint8_t) result;
			if (++dataCounter == receiveMessage.length) {
				dataCounter = 0;
				state = COMMS_GET_END_BYTE;
			}
			break;
		case COMMS_GET_END_BYTE:
			if (result == COMMS_END_BYTE) {
				// Check the checksum here
				PC_COMMS_DEBUG_LOG("Received PC message:\r\n"
						"\tid:0x%08lx\r\n\ttype:0x%02x\r\n\tlength:0x%02x\r\n\tchecksum:0x%04x",
						receiveMessage.id, receiveMessage.type, receiveMessage.length, receiveMessage.checksum);
#ifdef PC_COMMS_DEBUG_LOGGING
				for (uint8_t i = 0; i < 40; ++i) {
					if (i % 10 == 0) {
						printf("\r\n\t");
					}
					printf(" 0x%02x('%c')", receiveMessage.data[i], receiveMessage.data[i]);
				}
#endif
				state = COMMS_RECEIVED;
			} else {
				state = COMMS_ERROR;
			}
			break;
		case COMMS_RECEIVED:
			// do nothing, can't reach here.
			break;
		}
	}
}


const PCCommsMessage* receivePCMessage() {
	if (state == COMMS_RECEIVED) {
		state = COMMS_IDLE; // allow receiving new messages again
		return &receiveMessage;
	} else {
		return 0;
	}
}


void sendPCMessage(const PCCommsMessage* message) {
	uint8_t* part = (uint8_t*) &message->id;
	putchar(part[3]); // AVR is little endian
	putchar(part[2]);
	putchar(part[1]);
	putchar(part[0]);
	putchar(message->type);
	putchar(message->length);
	part = (uint8_t*) &message->checksum;
	putchar(part[1]);
	putchar(part[0]);
	for (uint8_t i = 0; i < message->length; ++i) {
		putchar(message->data[i]);
	}
}