/*
 * PCComms.h
 *
 * Created: 24/07/2023 3:23:42 PM
 *  Author: Michael Jones
 */ 


#ifndef PCCOMMS_H_
#define PCCOMMS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ControlLoop.h"

#define PC_MSG_START_CHARGE_ID 0x01
#define PC_MSG_STOP_CHARGE_ID 0x02
#define PC_MSG_RUN_TEST 0x03
#define PC_MSG_HEARTBEAT 0x04
#define PC_MSG_CHARGER_STATUS 0x05

typedef struct PCCommsStartChargeMessage {
	uint16_t maxChargingVoltage;
	uint16_t maxChargingCurrent;
} PCCommsStartChargeMessage;

typedef struct PCCommsBoardHeartbeatMessage {
	OperatingMode boardMode;
	bool chargerConnected;
	bool accumConnected;
} PCCommsBoardHeartbeatMessage;

typedef struct PCCommsMessage {
	uint8_t id;
	union {
		PCCommsStartChargeMessage startCharge;
		PCCommsBoardHeartbeatMessage boardHeartbeat;
	};
} PCCommsMessage;

typedef struct PCCommsRawMessage {
	uint8_t id;
	uint8_t length;
	uint8_t data[40];
} PCCommsRawMessage;

//typedef struct OldPCCommsRawMessage {
	//uint32_t id;
	//uint8_t type;
	//uint8_t length;
	//uint16_t checksum;
	//uint8_t data[40];
//} OldPCCommsRawMessage;

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
 * Receive any pending messages from the PC in the structured message format.
 *
 * Returns NULL pointer if no (completely reconstructed) message has been received.
 */
const PCCommsMessage* receivePCMessage();

/**
 * Send message to the PC using the structured message format.
 */
void sendPCMessage(const PCCommsMessage* message);

/**
 * Receive any pending messages from the PC in the raw message format.
 *
 * Returns NULL pointer if no (completely reconstructed) message has been received.
 */
const PCCommsRawMessage* receiveRawPCMessage();

/**
 * Send message to the PC using the raw message format.
 */
void sendRawPCMessage(const PCCommsRawMessage* message);

#endif /* PCCOMMS_H_ */