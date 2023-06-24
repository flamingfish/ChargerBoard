/*
 * Leds.c
 *
 * Created: 21/04/2023 1:39:09 PM
 *  Author: Michael Jones
 */ 

#include "gpio.h"
#include <avr/io.h>

void setupGpios() {
	// Set PB4, PB5, PB6 as output
	DDRB |= (1 << 4) | (1 << 5) | (1 << 6);
	// Set PC4, PC6 as output
	DDRC |= (1 << 4) | (1 << 6);
	// Set PB4, PB5, PB6 to 0
	PORTB &= ~(1 << 4) & ~(1 << 5) & ~(1 << 6);
	// Set PC4, PC6 to 0
	PORTC &= ~(1 << 4) & ~(1 << 4);
	
	// Set PC5, PB7 as input
	DDRC &= ~(1 << 5);
	DDRB &= ~(1 << 7);
	
	// Turn off internal pull-up resistors for PC5, PB7
	PORTC &= ~(1 << 5);
	PORTB &= ~(1 << 7);
}

void setGpioState(GpioName name, GpioState state) {
	switch (name) {
	case CHARGING_LED:
		if (state) { // If state == ON
			PORTB |= (1 << 4);
		} else {
			PORTB &= ~(1 << 4);
		}
		break;
	case CHARGER_FAULT_LED:
		if (state) { // If state == ON
			PORTB |= (1 << 5);
		} else {
			PORTB &= ~(1 << 5);
		}
		break;
	case AMS_FAULT_LED:
		if (state) { // If state == ON
			PORTB |= (1 << 6);
		} else {
			PORTB &= ~(1 << 6);
		}
		break;
	case ENABLE_CHARGE_SIGNAL:
		if (state) { // If state == ON
			PORTC |= (1 << 4);
		} else {
			PORTC &= ~(1 << 4);
		}
		break;
	case GENERIC_SHUTDOWN_SIGNAL:
		if (state) { // If state == ON
			PORTC |= (1 << 6);
		} else {
			PORTC &= ~(1 << 6);
		}
		break;
	default:
		// Ignore all other cases
		break;
	}
}

GpioState getGpioState(GpioName name) {
	switch (name) {
	case IMD_FEEDBACK:
		return !!(PINC & (1 << 5));
	case AMS_FEEDBACK:
		return !!(PINB & (1 << 7));
	default:
		return OFF;
	}
}