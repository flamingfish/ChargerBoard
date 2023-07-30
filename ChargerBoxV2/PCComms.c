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

//typedef enum PCCommsState {
	//COMMS_IDLE,
	//COMMS_GET_ID,
	//COMMS_GET_TYPE,
	//COMMS_GET_LENGTH,
	//COMMS_GET_CHECKSUM,
	//COMMS_GET_DATA,
	//COMMS_GET_END_BYTE,
	//COMMS_RECEIVED,
	//COMMS_ERROR
//} PCCommsState;

typedef enum PCCommsState {
	COMMS_IDLE,
	COMMS_GET_ID,
	COMMS_GET_LENGTH,
	COMMS_GET_DATA,
	COMMS_GET_END_BYTE,
	COMMS_RECEIVED,
	COMMS_ERROR
} PCCommsState;

static PCCommsRawMessage rawReceiveMessage;
static PCCommsMessage receiveMessage;
static PCCommsRawMessage rawSendMessage;

static PCCommsState state;
static uint8_t dataCounter;


static void extractMessage() {
	receiveMessage.id = rawReceiveMessage.id;
	switch (rawReceiveMessage.id) {
	case PC_MSG_START_CHARGE_ID:
		// AVR is little endian
		receiveMessage.startCharge.maxChargingVoltage = ((uint16_t) rawReceiveMessage.data[0] >> 8) | ((uint16_t) rawReceiveMessage.data[1]);
		receiveMessage.startCharge.maxChargingCurrent = ((uint16_t) rawReceiveMessage.data[2] >> 8) | ((uint16_t) rawReceiveMessage.data[3]);
		break;
	}
}


static void extractByte(uint8_t byte) {
	switch(state) {
	case COMMS_ERROR: // fall through
	case COMMS_IDLE:
		if (byte == COMMS_START_BYTE) {
			state = COMMS_GET_ID;
			} else {
			state = COMMS_ERROR;
		}
		break;
	case COMMS_GET_ID:
		rawReceiveMessage.id = byte;
		state = COMMS_GET_LENGTH;
		break;
	case COMMS_GET_LENGTH:
		rawReceiveMessage.length = byte;
		state = COMMS_GET_DATA;
		break;
	case COMMS_GET_DATA:
		rawReceiveMessage.data[dataCounter] = byte;
		if (++dataCounter == rawReceiveMessage.length) {
			dataCounter = 0;
			state = COMMS_GET_END_BYTE;
		}
		break;
	case COMMS_GET_END_BYTE:
		if (byte == COMMS_END_BYTE) {
			PC_COMMS_DEBUG_LOG("Received PC message:\r\n"
					"\tid:0x%02x\r\n\tlength:0x%02x",
					rawReceiveMessage.id, rawReceiveMessage.length);
#ifdef PC_COMMS_DEBUG_LOGGING
			for (uint8_t i = 0; i < 40; ++i) {
				if (i % 10 == 0) {
					printf("\r\n\t");
				}
				printf(" 0x%02x('%c')", rawReceiveMessage.data[i], rawReceiveMessage.data[i]);
			}
			printf("\r\n");
#endif
			extractMessage();
			state = COMMS_RECEIVED;
			// If a single byte was dropped - the current message is lost, but the next one can still be salvaged
		} else if (byte == COMMS_START_BYTE) {
			state = COMMS_GET_ID;
		} else {
			state = COMMS_ERROR; // may get overwritten below
			// Iterate backwards through data to see if there was any start byte
			// -> try to see if the next message can be salvaged
			//uint8_t byte; // just overwrite byte parameter
			for (uint8_t i = rawReceiveMessage.length - 1; i >= 0; --i) {
				byte = rawReceiveMessage.data[i];
				if (byte == COMMS_START_BYTE) {
					// Start extracting next message
					state = COMMS_GET_ID;
					for (++i; i < rawReceiveMessage.length; ++i) {
						byte = rawReceiveMessage.data[i];
						extractByte(byte);
					}
					break;
				}
			}
		}
		break;
	case COMMS_RECEIVED:
		break; // do nothing - should never be here
	}
}


void setupPCComms() {
	state = COMMS_IDLE;
	dataCounter = 0;
}


void updatePCComms() {
	if (state == COMMS_RECEIVED) {
		// If a message is received but not yet read, don't do getchar()
		// -> leave it in the internal buffer for later.
		return;
	}
	
	int16_t result;
	while ((result = getchar()) >= 0) {
		extractByte((uint8_t) result);
	}
}


const PCCommsRawMessage* receiveRawPCMessage() {
	if (state == COMMS_RECEIVED) {
		state = COMMS_IDLE;
		return &rawReceiveMessage;
	}
	return 0; // NULL
}


void sendRawPCMessage(const PCCommsRawMessage* message) {
	putchar(COMMS_START_BYTE);
	putchar(message->id);
	putchar(message->length);
	for (uint8_t i = 0; i < message->length; ++i) {
		putchar(message->data[i]);
	}
	putchar(COMMS_END_BYTE);
}


const PCCommsMessage* receivePCMessage() {
	if (state == COMMS_RECEIVED) {
		state = COMMS_IDLE;
		return &receiveMessage;
	}
	return 0; // NULL
}


void sendPCMessage(const PCCommsMessage* message) {
	rawSendMessage.id = message->id;
	switch (message->id) {
	case PC_MSG_HEARTBEAT:
		break;
	default:
		rawSendMessage.length = 0;
		break;
	}
	sendRawPCMessage(&rawSendMessage);
}