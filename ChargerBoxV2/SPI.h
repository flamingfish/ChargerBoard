/*
 * SPI.h
 *
 * Created: 23/06/2023 2:27:29 PM
 *  Author: Michael Jones
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

void setupSPI();
void writeSPI(uint8_t* data, uint8_t dataLength);
void readSPI(uint8_t* writePreamble, uint8_t writeLength, uint8_t* readData, uint8_t readLength);


#endif /* SPI_H_ */