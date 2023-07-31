/*
 * main.c
 *
 * Created: 30/05/2023 7:54:32 PM
 *  Author: Michael Jones
 */ 

#include "gpio.h"
#include "USB.h"
#include "CAN.h"
#include "SPI.h"
#include <stdint.h>
#include <stdio.h>
#include "util/delay.h"
#include <stdbool.h>
#include "timer.h"
#include <avr/pgmspace.h>
#include "logger.h"
#include "PCComms.h"
#include "ControlLoop.h"

//char readRegister(char address);
//void writeRegister(uint8_t address, uint8_t data);
void testLoop();

static bool amsError;
static OperatingMode operatingMode;
static uint16_t setChargeVoltage;
static uint16_t setChargeCurrent;

void controlSetup() {
	amsError = false;
	operatingMode = OPERATING_IDLE;
}

void controlLoop() {
	static const PCCommsMessage* receiveMessage;
	// ===== Check for CAN messages:
	
	// ===== Check for PC messages:
	receiveMessage = receivePCMessage();
	if (receiveMessage) { // If not NULL pointer
		switch (receiveMessage->id) {
		case PC_MSG_START_CHARGE_ID:
			DEBUG_LOG("Trying to start charge\r\n");
			switch (operatingMode) {
			case OPERATING_IDLE:
				setChargeVoltage = receiveMessage->startCharge.maxChargingVoltage;
				setChargeCurrent = receiveMessage->startCharge.maxChargingCurrent;
				// close enable relay
				setGpioState(ENABLE_CHARGE_SIGNAL, ON);
				// send confirmation back to PC
				operatingMode = OPERATING_CHARGING;
				break;
			case OPERATING_CHARGING:
				DEBUG_LOG("Already in charging mode\r\n");
				// Update the voltage and current with new values
				setChargeVoltage = receiveMessage->startCharge.maxChargingVoltage;
				setChargeCurrent = receiveMessage->startCharge.maxChargingCurrent;
				break;
			case OPERATING_ERROR:
				DEBUG_LOG("Unable to start charge since in ERROR mode\r\n");
				break;
			case OPERATING_TESTING:
				DEBUG_LOG("Unable to start charge since in TESTING mode\r\n");
				break;
			}
			break;
		}
	}
	
	// if "start charging" (comes with voltage and current):
	// if mode == IDLE:
	// close enable relay, change mode to OPERATING_CHARGING (which will send heartbeat), send confirmation back to PC
	// if mode == CHARGING:
	// update voltage and current, send confirmation back to PC
	// if mode == ERROR:
	// send message back to PC explaining error
	// if mode == TESTING:
	// send message back to PC explaining why
	
	
	// if "stop charging":
	// send heartbeat message with charging set to OFF, then open enable relay after confirmation from charger, then send confirmation back to PC
	
	// if "heartbeat":
	
	//if "run test"
	
	// ===== Check for buttons/switches (e.g. enable switch)
	
	// Perform necessary tasks
	switch (operatingMode) {
	case OPERATING_IDLE:
		break;
	case OPERATING_CHARGING:
		// send charging message at 1Hz
		break;
	case OPERATING_ERROR:
		break;
	case OPERATING_TESTING:
		break;
	}
	
	// send PC status info at 1Hz
}

static inline void setup() {
	setupTimer();
	setupUSB();
	_delay_ms(500);
	setupGpios();
	setupSPI();
	setupCAN();
	setupPCComms();
	controlSetup();
}

static inline void update() {
	updateUSB();
	updateCAN();
	updatePCComms();
	testLoop();
	controlLoop();
}

int main() {
	setup();
	while(1) {
		update();
	}
	
	return 0;
}

void startBlinkLEDs() {
	
}

char receiveBuffer[20];

void testLoop() {
	// Static varaibles
	static uint32_t lastTime = 0;
	static uint8_t count = 0;
	static GpioState chargingState = OFF;
	
	// Get current time at beginning of each loop:
	uint32_t currentTime = millis();
	
	
	if (count == 0) {
		if (currentTime < 2000) {
			//printf("Milliseconds: %lu\r\n", currentTime);
			return; // equivalent to continue if this where a while loop
		} else {
			lastTime = currentTime;
			++count;
			//printf("Starting program\r\n");
			DEBUG_LOG("Starting program\r\n");
		}
	}
	
	if (count <= 3) {
		if (currentTime - lastTime > 500) {
			lastTime = currentTime;
			chargingState = !chargingState;
			setGpioState(CHARGING_LED, chargingState);
			printf("Setting Charging LED to ");
			printf(chargingState ? "ON" : "OFF");
			printf("\r\n");
			++count;
		}
	//} else if (count == 4) {
		////printf("Before SPI\r\n");
		////updateUSB();
		////char data[1] = {0xaa};
		////writeSPI(data, 1);
		////printf("After SPI\r\n");
		////updateUSB();
		//
		//printf("Before result\r\n");
		//updateUSB();
		//writeRegister(0x20, 0x00);
		////char result = readRegister(0xff);
		////printf("Result: 0x%02x\r\n", result);
		////updateUSB();
	} else if (count == 4) {
		//writeCAN(0x1806E5F4L, (uint8_t*) "Hello", 6);
		setCANID(0, 0x1806E5F4L);
		sendCANMessage(0, (uint8_t*) "Hello", 6);
		sendCANMessage(0, (uint8_t*) "Hello", 6);
		++count;
	} else {
		if (currentTime - lastTime > 2000) {
			lastTime = currentTime;
			printf("Inside main loop\r\n");
			//char* result = fgets(receiveBuffer, 20, stdin);
			//if (result == 0) {
				//printf_P(PSTR("No bytes received\r\n"));
			//} else {
				//printf_P(PSTR("Received bytes: "));
				//printf("%s\r\n", result);
			//}
			const PCCommsRawMessage* receivedMessage = receiveRawPCMessage();
			if (receivedMessage) {
				printf("Received message!\r\n");
			}
		}
	}
}

