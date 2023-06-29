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

//char readRegister(char address);
//void writeRegister(uint8_t address, uint8_t data);
void testLoop();

static inline void setup() {
	setupTimer();
	setupUSB();
	_delay_ms(500);
	setupGpios();
	setupSPI();
	setupCAN();
}

static inline void update() {
	updateUSB();
	testLoop();
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
		++count;
	} else {
		if (currentTime - lastTime > 2000) {
			lastTime = currentTime;
			printf("Inside main loop\r\n");
		}
	}
}

