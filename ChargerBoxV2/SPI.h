/*
 * SPI.h
 *
 * Created: 23/06/2023 2:27:29 PM
 *  Author: Michael Jones
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

// How many SPI call requests to queue
#define SPI_ASYNC_QUEUE_LEN 10

/**
 * Set up hardware for SPI communication.
 */
void setupSPI();

/**
 * Synchronously perform SPI transmission of data.
 */
void writeSPI(uint8_t* data, uint8_t dataLength);

/**
 * Synchronously retrieve data over SPI.
 */
void readSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t* readData, uint8_t readLength);

/**
 * Asynchronously perform SPI transmission of data.
 *
 * The callback function will be executed once the SPI transmission has completed.
 * Make the callback short as it will be executed in an ISR.
 * SPI calls will be queued if they can't be dispatched immediately. If the queue
 * is full, a 1 will be returned to indicate the error condition. If the underlying
 * data buffer is full, a 2 will be returned. Returns 0 otherwise.
 */
uint8_t asyncWriteSPI(uint8_t* data, uint8_t dataLength, void (*callback)(void));

/**
 * Asynchronously retrieve data over SPI.
 * 
 * The callback function will be executed once the SPI transmission has completed.
 * The received data will be stored in an internal buffer and passed to the callback.
 * The data is not guaranteed to persist after the callback has returned. The user must
 * copy the data if necessary.
 * Make the callback short as it will be executed in an ISR.
 * SPI calls will be queued if they can't be dispatched immediately. If the queue
 * is full, a 1 will be returned to indicate the error condition. If the underlying
 * data buffer is full, a 2 will be returned. Returns 0 otherwise.
 */
uint8_t asyncReadSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t readLength, void (*callback)(uint8_t*));


#endif /* SPI_H_ */