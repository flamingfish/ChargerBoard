/*
 * PCComms.h
 *
 * Created: 24/07/2023 3:23:42 PM
 *  Author: Michael Jones
 */ 


#ifndef PCCOMMS_H_
#define PCCOMMS_H_

#include <stdint.h>

typedef struct PCCommsMessage {
	uint32_t id;
	uint8_t type;
	uint8_t length;
	uint16_t checksum;
	uint8_t data[40];
} PCCommsMessage;

/**
 * Set up the communication protocol with the PC (over USB).
 *
 * USB must also be initialised.
 */
void setupPCComms();

/**
 * Must be called each update loop to receive messages from PC.
 */
void updatePCComms();

/**
 * Receive any pending messages from the PC.
 *
 * Returns NULL pointer if no (completely reconstructed) message has been received.
 */
const PCCommsMessage* receivePCMessage();

/**
 * Send message to the PC.
 */
void sendPCMessage(const PCCommsMessage* message);

#endif /* PCCOMMS_H_ */