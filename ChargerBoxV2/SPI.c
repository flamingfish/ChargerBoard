/*
 * SPI.c
 *
 * Created: 23/06/2023 2:27:45 PM
 *  Author: Michael Jones
 */ 

#include "SPI.h"
#include <avr/io.h>
#include <avr/common.h>

#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3


void setupSPI() {
	// Set as output:
	DDRB |= (1 << MOSI) | (1 << SCK) | (1 << SS);
	// Set as input:
	DDRB &= ~(1 << MISO);
	// Set SS high
	PORTB |= (1 << SS);
	
	// Enable SPI, not SPI interrupt, MSB first, master SPI mode, SCK low when idle,
	// leading edge sample trailing edge setup, fastest speed
	SPCR = (0 << SPIE) | (1 << SPE) | (1 << MSTR);
	SPSR &= ~(1 << SPI2X);
	
	// Turn on global interrupts
	//sei();
}

//void _writeSPI(char data) {
	//char flushBuffer;
	//// Set SS low
	//PORTB &= !(1 << SS);
	//SPDR = data; // Write data to SPI data register
	//while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
	//flushBuffer = SPDR; // Flush received data
	//// SPIF flag is cleared after reading from SPSR and accessing SPDR
	//// Set SS high
	//PORTB |= (1 << SS);
//}

// Note: the SPIF flag in SPSR is cleared either when the interrupt handling
// vector is called, or if SPSR is read with SPIF set followed by accessing SPDR.

void writeSPI(uint8_t* data, uint8_t dataLength) {
	uint8_t flushBuffer;
	// Set SS low
	PORTB &= !(1 << SS);
	for (uint8_t i = 0; i < dataLength; i++) {
		SPDR = data[i]; // Write data to SPI data register
		while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
		flushBuffer = SPDR;
	}
	// Set SS high
	PORTB |= (1 << SS);
}

void readSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t* readData, uint8_t readLength) {
	uint8_t flushBuffer;
	// Set SS low
	PORTB &= !(1 << SS);
	for (uint8_t i = 0; i < writeLength; i++) {
		SPDR = writePreamble[i]; // Write data to SPI data register
		while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
		flushBuffer = SPDR;
	}
	
	for (uint8_t i = 0; i < readLength; i++) {
		// Write dummy data to SPDR to generate SCK for reception
		SPDR = 0xff;
		while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
		readData[i] = SPDR;
	}
	
	// Set SS high
	PORTB |= (1 << SS);
}