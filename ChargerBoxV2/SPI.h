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
void writeSPI(char* data, uint8_t dataLength);
void readSPI(char* writePreamble, uint8_t writeLength, char* readData, uint8_t readLength);


#endif /* SPI_H_ */