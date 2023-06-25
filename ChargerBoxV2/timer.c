/*
 * timer.c
 *
 * Created: 24/06/2023 11:46:19 AM
 *  Author: Michael Jones
 */ 

#include "timer.h"
#include <util/atomic.h>

#if F_CPU != 16000000
#	error "Must have CPU frequency set to 16MHz"
#endif

static volatile uint32_t milliseconds = 0;

void setupTimer() {
	milliseconds = 0L;
	
	// Clear the timer
	TCNT0 = 0;
	
	// Set the output compare value to be 124 if F_CPU=8MHz
	OCR0A = 249;
	
	// Timer counter control registers
	// Set to CTC mode (clear timer on compare match)
	// and divide clock by 64
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	
	// Enable interrupt on timer output compare match.
	TIMSK0 |= (1 << OCIE0A);
	
	// Ensure the interrupt flag is cleared by writing a 1.
	TIFR0 = (1 << OCF0A);
	
	// Turn on global interrupts
	sei();
}

uint32_t millis() {
	uint32_t returnValue;
	// Ensure milliseconds isn't updating in the middle of reading it.
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		returnValue = milliseconds;
	}
	return returnValue;
}


ISR(TIMER0_COMPA_vect) {
	// Increment tick count
	++milliseconds;
}