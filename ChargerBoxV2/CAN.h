/*
 * CAN.h
 *
 * Created: 23/06/2023 4:04:53 PM
 *  Author: Michael Jones
 */ 


#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

#define CAN_1Mbps 0
#define CAN_500kbps 1
#define CAN_250kbps 3
#define CAN_125kbps 7

/**
 * Set up CAN communication with MCP2515.
 *
 * SPI must be set up for this to work.
 */
void setupCAN();
void writeCAN(uint32_t canID, uint8_t* data, uint8_t dataLen);

/**
 * Set the CAN ID for one of the transmit buffers (1 to 3).
 */
void setCANID(uint8_t bufferNum, uint32_t id);

/**
 * Send a CAN message from one of the transmit buffers (1 to 3).
 *
 * Data length must be no greater than 8 bytes
 */
void sendCANMessage(uint8_t bufferNum, uint8_t* data, uint8_t dataLen);

/**
 * Must be called in main update loop.
 */
void updateCAN();



#endif /* CAN_H_ */