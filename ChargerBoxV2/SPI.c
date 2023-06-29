/*
 * SPI.c
 *
 * Created: 23/06/2023 2:27:45 PM
 *  Author: Michael Jones
 */ 

#include "SPI.h"
#include <avr/io.h>
#include <avr/common.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "logger.h"
#include "USB.h"

#include "gpio.h"

#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3

#define SPI_ASYNC_DATA_BUF_LEN (3 * SPI_ASYNC_QUEUE_LEN)
#define SPI_ASYNC_MAX_READ_LEN 16

typedef struct {
	uint8_t* data; // Need to be careful when iterating through data - may wrap around in data buffer
	uint8_t writeLen;
	uint8_t readLen;
	void (*callback)(void);
} SPIItem;

// Circular buffer
static uint8_t spiDataBuffer[3 * SPI_ASYNC_QUEUE_LEN];
static uint8_t spiDataBufferTail = 0; // index of next data to insert
static uint8_t spiDataBufferHead = 0; // index of oldest data inserted
static uint8_t spiDataBufferLen = 0; // num of data items in queue
// Circular buffer
static SPIItem spiItemBuffer[SPI_ASYNC_QUEUE_LEN];
static uint8_t spiItemBufferTail = 0; // index of next item to insert
static uint8_t spiItemBufferHead = 0; // index of oldest item inserted
static uint8_t spiItemBufferLen = 0; // num of items in queue

// If a synchronous transmission is occurring (or waiting to occur),
// asynchronous transmissions should be queued.
static volatile bool synchronousTransmission = false;
static volatile bool asyncTransmission = false;

// Keep track of which data byte you're up to when transmitting
static volatile uint8_t asyncWriteIndex = 0;
static volatile uint8_t asyncReadIndex = 0;
static volatile uint8_t* currentWriteData = 0;
//static volatile uint8_t currentWriteLen = 0;
////static volatile uint8_t currentReadLen = 0;
//static volatile uint8_t currentDataLen = 0;
//static volatile uint8_t* currentData = 0;
//static volatile void (*currentCallback)(void) = 0;
static uint8_t currentReadData[SPI_ASYNC_MAX_READ_LEN];

//static uint8_t spiItemBufferTail() {
	//return (spiItemBufferHead + SPI_ASYNC_QUEUE_LEN - spiItemBufferLen) % SPI_ASYNC_QUEUE_LEN;
//}

/**
 * Returns 1 if buffer is full. Fills newData with pointer value within buffer.
 */
static uint8_t addDataToBuffer(uint8_t* data, uint8_t dataLength, uint8_t** newData) {
	if (spiDataBufferLen + dataLength > SPI_ASYNC_DATA_BUF_LEN) {
#ifdef SPI_DEBUG_LOGGING
		setGpioState(CHARGER_FAULT_LED, ON); // for debugging purposes
#endif
		SPI_DEBUG_LOG("ERROR: data buffer is full\r\n");
		return 1;
	}
	*newData = &spiDataBuffer[spiDataBufferTail];
	spiDataBufferLen += dataLength;
	for (uint8_t i = 0; i < dataLength; i++) {
		spiDataBuffer[spiDataBufferTail] = data[i];
		spiDataBufferTail = (spiDataBufferTail + 1) % SPI_ASYNC_DATA_BUF_LEN;
	}
	
	return 0;
}

/**
 * Returns 1 if trying to remove more than what buffer has.
 */
static uint8_t removeDataFromBuffer(uint8_t count) {
	if (count > spiDataBufferLen) {
#ifdef SPI_DEBUG_LOGGING
		setGpioState(CHARGER_FAULT_LED, ON); // for debugging purposes
#endif
		SPI_DEBUG_LOG("ERROR: cannot remove data from data buffer as already empty\r\n");
		return 1;
	}
	spiDataBufferLen -= count;
	spiDataBufferHead = (spiDataBufferHead + count) % SPI_ASYNC_DATA_BUF_LEN;
	
	return 0;
}

/**
 * Returns 1 if item buffer is full. Returns 2 if data buffer is full.
 */
static uint8_t addItemToBuffer(uint8_t* writeData, uint8_t writeLen, uint8_t readLen, void (*callback)(void)) {
	if (spiItemBufferLen == SPI_ASYNC_QUEUE_LEN) {
#ifdef SPI_DEBUG_LOGGING
		setGpioState(CHARGER_FAULT_LED, ON); // for debugging purposes
#endif
		SPI_DEBUG_LOG("ERROR: Item buffer is full\r\n");
		return 1;
	}
	uint8_t* newData;
	if (addDataToBuffer(writeData, writeLen, &newData)) {
		//setGpioState(CHARGER_FAULT_LED, ON); // for debugging purposes
		return 2;
	}
	
	spiItemBuffer[spiItemBufferTail].data = newData;
	spiItemBuffer[spiItemBufferTail].writeLen = writeLen;
	spiItemBuffer[spiItemBufferTail].readLen = readLen;
	spiItemBuffer[spiItemBufferTail].callback = callback;
	
	++spiItemBufferLen;
	spiItemBufferTail = (spiItemBufferTail + 1) % SPI_ASYNC_QUEUE_LEN;
	
	return 0;
}

/**
 * Returns 1 if buffer is already emtpy.
 */
static uint8_t removeItemFromBuffer() {
	if (spiItemBufferLen == 0) {
#ifdef SPI_DEBUG_LOGGING
		setGpioState(CHARGER_FAULT_LED, ON); // for debugging purposes
#endif
		SPI_DEBUG_LOG("ERROR: cannot remove item from buffer as already empty\r\n");
		return 1;
	}
	removeDataFromBuffer(spiItemBuffer[spiItemBufferHead].writeLen);
	--spiItemBufferLen;
	spiItemBufferHead = (spiItemBufferHead + 1) % SPI_ASYNC_QUEUE_LEN;
	
	return 0;
}

// Forward declaration
static void processAsyncSPIItem();

static void startAsyncSPIItem() {
	SPI_DEBUG_LOG("startAsyncSPIItem\r\n");
	asyncTransmission = true;
	SPIItem* currentItem = &spiItemBuffer[spiItemBufferHead];
	currentWriteData = currentItem->data;
	asyncWriteIndex = 0;
	asyncReadIndex = 0;
	// Set SS low
	PORTB &= !(1 << SS);
	processAsyncSPIItem();
}

static void processAsyncSPIItem() {
	SPIItem* currentItem = &spiItemBuffer[spiItemBufferHead];
	SPI_DEBUG_LOG(
		"processAsyncSPIItem: [writeLen: %d, readLen: %d, callback: 0x%04x, writeIndex: %d, readIndex: %d, bufferLen: %d, bufferTail: %d, bufferHead: %d]\r\n\t",
		currentItem->writeLen,
		currentItem->readLen,
		currentItem->callback,
		asyncWriteIndex,
		asyncReadIndex,
		spiItemBufferLen,
		spiItemBufferTail,
		spiItemBufferHead
	);
#ifdef SPI_DEBUG_LOGGING
	updateUSB();
#endif
	if (asyncWriteIndex < currentItem->writeLen) {
		SPI_DEBUG_LOG("Writing data: 0x%02x ('%c')\r\n", *currentWriteData, *currentWriteData);
		SPDR = *currentWriteData++;
		if (currentWriteData - spiDataBuffer == SPI_ASYNC_DATA_BUF_LEN) {
			currentWriteData = spiDataBuffer; // wrap around to start of buffer if necessary
		}
		++asyncWriteIndex;
		return;
	}
	if (asyncWriteIndex == currentItem->writeLen && currentItem->readLen == 0) {
		// This async spi write item is now done.
		(void) SPDR; // Read buffer to flush, discard value
		// Set SS high
		PORTB |= (1 << SS);
		SPI_DEBUG_LOG("Finished writing data.");
		if (currentItem->callback) {
			SPI_DEBUG_LOG(" Calling write callback.\r\n");
			currentItem->callback();
		}
		goto endOfSpiItem;
	}
	if (asyncWriteIndex == currentItem->writeLen) {
		SPI_DEBUG_LOG("Beginning reading data.\r\n");
		// Write dummy data to SPDR to generate SCK to receive next byte
		SPDR = 0xff;
		++asyncWriteIndex;
		return;
	}
	if (asyncReadIndex < currentItem->readLen - 1) {
		currentReadData[asyncReadIndex++] = SPDR;
		SPI_DEBUG_LOG("Read data: 0x%02x ('%c')\r\n", currentReadData[asyncReadIndex - 1], currentReadData[asyncReadIndex - 1]);
		// Write dummy data to SPDR to generate SCK to receive next byte
		SPDR = 0xff;
		return;
	}
	currentReadData[asyncReadIndex++] = SPDR;
	// Set SS high
	PORTB |= (1 << SS);
	if (currentItem->callback) {
		SPI_DEBUG_LOG("Finished reading data: ");
#ifdef SPI_DEBUG_LOGGING
		for (uint8_t i = 0; i < currentItem->readLen; i++) {
			SPI_DEBUG_LOG("%c", currentReadData[i]);
		}
#endif
		((void (*)(uint8_t*)) currentItem->callback)(currentReadData);
	}
endOfSpiItem:
	// Remove current item from buffer
	removeItemFromBuffer();
	if (synchronousTransmission) {
		SPI_DEBUG_LOG(" Returning to synchronous.\r\n");
		asyncTransmission = false;
		return;
	}
	if (spiItemBufferLen) {
		SPI_DEBUG_LOG(" Starting next item.");
		startAsyncSPIItem();
		return;
	}
	// No more items to process!
	SPI_DEBUG_LOG(" Finished processing items.\r\n");
	asyncTransmission = false;
}

void setupSPI() {
	// Set as output:
	DDRB |= (1 << MOSI) | (1 << SCK) | (1 << SS);
	// Set as input:
	DDRB &= ~(1 << MISO);
	// Set SS high
	PORTB |= (1 << SS);
	
	// Enable SPI, SPI interrupt, MSB first, master SPI mode, SCK low when idle,
	// leading edge sample trailing edge setup, fastest speed = F_CPU/4 = 4MHz
	SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR);
	SPSR &= ~(1 << SPI2X);
	
	// Turn on global interrupts
	sei();
}

// Note: the SPIF flag in SPSR is cleared either when the interrupt handling
// vector is called, or if SPSR is read with SPIF set followed by accessing SPDR.

void writeSPI(uint8_t* data, uint8_t dataLength) {
	//uint8_t flushBuffer;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		synchronousTransmission = true;
	}
	while(asyncTransmission); // Wait until ready for transmission
	SPCR &= ~(1 << SPIE); // Turn off SPI interrupt
	// Set SS low
	PORTB &= !(1 << SS);
	for (uint8_t i = 0; i < dataLength; i++) {
		SPDR = data[i]; // Write data to SPI data register
		while(!(SPSR & (1 << SPIF))); // Wait until transmission complete
		//flushBuffer = SPDR;
		(void) SPDR; // Read buffer to flush, discard value
	}
	// Set SS high
	PORTB |= (1 << SS);
	SPCR |= (1 << SPIE); // Turn SPI interrupt back on.
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		synchronousTransmission = false;
	}
	
	// If there are async items to process, do that.
	if (spiItemBufferLen) {
		asyncTransmission = true;
		processAsyncSPIItem();
	}
}

void readSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t* readData, uint8_t readLength) {
	//uint8_t flushBuffer;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		synchronousTransmission = true;
	}
	while(asyncTransmission); // Wait until ready for transmission
	SPCR &= ~(1 << SPIE); // Turn off SPI interrupt
	// Set SS low
	PORTB &= !(1 << SS);
	for (uint8_t i = 0; i < writeLength; i++) {
		SPDR = writePreamble[i]; // Write data to SPI data register
		while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
		//flushBuffer = SPDR;
		(void) SPDR; // Read buffer to flush, discard value
	}
	
	for (uint8_t i = 0; i < readLength; i++) {
		// Write dummy data to SPDR to generate SCK for reception
		SPDR = 0xff;
		while(!(SPSR & (1 << SPIF))); // Wait till transmission complete
		readData[i] = SPDR;
	}
	
	// Set SS high
	PORTB |= (1 << SS);
	SPCR |= (1 << SPIE); // Turn SPI interrupt back on.
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		synchronousTransmission = false;
	}
	
	if (spiItemBufferLen) {
		asyncTransmission = true;
		processAsyncSPIItem();
	}
}

uint8_t asyncWriteSPI(uint8_t* data, uint8_t dataLength, void (*callback)(void)) {
	// Add to queue
	uint8_t result;
	bool isAsync;
	// must be restorestate since this can be called inside another ISR
	// And inside ISRs, global interrupts are disabled by default
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		result = addItemToBuffer(data, dataLength, 0, callback);
		isAsync = asyncTransmission;
	}
	if (result) {
		setGpioState(CHARGER_FAULT_LED, ON); // for debug purposes
		return result;
	}
	if (synchronousTransmission || isAsync) {
		return 0;
	}
	// Otherwise, start SPI transmission now
	startAsyncSPIItem();
	
	return 0;
}

uint8_t asyncReadSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t readLength, void (*callback)(uint8_t*)) {
	// Add to queue
	uint8_t result;
	bool isAsync;
	// must be restorestate since this can be called inside another ISR
	// And inside ISRs, global interrupts are disabled by default
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		result = addItemToBuffer(writePreamble, writeLength, readLength, (void (*)(void)) callback);
		isAsync = asyncTransmission;
	}
	if (result) {
		setGpioState(CHARGER_FAULT_LED, ON); // for debug purposes
		return result;
	}
	if (synchronousTransmission || isAsync) {
		return 0;
	}
	// Otherwise, start SPI transmission now
	startAsyncSPIItem();
	
	return 0;
}

ISR(SPI_STC_vect) {
	processAsyncSPIItem();
}