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

char readRegister(char address);
void testLoop();

static inline void setup() {
	setupGpios();
	setupUSB();
	setupTimer();
	setupSPI();
	setupCAN();
}

static inline void update() {
	updateUSB();
	testLoop();
}

int main() {
	setup();
		//setGpioState(CHARGING_LED, ON);
		//_delay_ms(500);
		//setGpioState(CHARGING_LED, OFF);
		//_delay_ms(500);
		//setGpioState(CHARGING_LED, ON);
		//_delay_ms(500);
		//setGpioState(CHARGING_LED, OFF);
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
			printf("Milliseconds: %lu\r\n", currentTime);
			return; // equivalent to continue if this where a while loop
		} else {
			lastTime = currentTime;
			++count;
			printf("Starting program\r\n");
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
	} else if (count == 4) {
		//printf("Before SPI\r\n");
		//updateUSB();
		//char data[1] = {0xaa};
		//writeSPI(data, 1);
		//printf("After SPI\r\n");
		//updateUSB();
		
		printf("Before result\r\n");
		updateUSB();
		char result = readRegister(0xff);
		printf("Result: 0x%02x\r\n", result);
		updateUSB();
	} else {
		if (currentTime - lastTime > 2000) {
			lastTime = currentTime;
			printf("Inside main loop\r\n");
		}
	}
}

int main2() {
	setupGpios();
	setupUSB();
	//setupSPI();
	//setupCAN();
	
	setGpioState(CHARGING_LED, ON);
	_delay_ms(500);
	setGpioState(CHARGING_LED, OFF);
	_delay_ms(500);
	setGpioState(CHARGING_LED, ON);
	_delay_ms(500);
	setGpioState(CHARGING_LED, OFF);
	
	char data[1] = {0xaa};
	writeSPI(data, 1);
	return 0;
	
	//_delay_ms(10000);
	//printf("Before result\r\n");
	//char result = readRegister(0xff);
	//printf("Result: 0x%x\r\n", 1);
	
	uint16_t i = 0;
	while (1) {
		updateUSB();
		
		if (i == 50) {
			printf("Before result\r\n");
			setGpioState(CHARGING_LED, ON);
			updateUSB();
			_delay_ms(500);
			updateUSB();
			_delay_ms(500);
			updateUSB();
			char result = readRegister(0xff);
			printf("Result: 0x%x\r\n", 1);
			updateUSB();
		} else if (i > 50) {
			printf("%s", "Hello there buddy\r\n");
		}
		

		//for (uint32_t i = 0; i < 1000000; i++) {
			//// do nothing
		//}
		_delay_ms(200);
		i++;
	}
	
	return 0;
}