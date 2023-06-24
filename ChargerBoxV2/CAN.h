/*
 * CAN.h
 *
 * Created: 23/06/2023 4:04:53 PM
 *  Author: Michael Jones
 */ 


#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

void setupCAN();
void writeCAN(char* data, uint8_t dataLen);



#endif /* CAN_H_ */