/*
 * logger.h
 *
 * Created: 29/06/2023 1:12:03 PM
 *  Author: micha
 */ 


#ifndef LOGGER_H_
#define LOGGER_H_

// =========== Debug loggers ===========
// Define the types of logging you want
#define DEBUG_LOGGING
#define SPI_DEBUG_LOGGING
#define CAN_DEBUG_LOGGING
#define PC_COMMS_DEBUG_LOGGING
// =====================================

#ifdef DEBUG_LOGGING
#	include <stdio.h>
#	include <avr/pgmspace.h>
#	define DEBUG_LOG(x, ...) printf_P(PSTR(x), ##__VA_ARGS__)
#	ifdef SPI_DEBUG_LOGGING
#		define SPI_DEBUG_LOG(x, ...) DEBUG_LOG(x, ##__VA_ARGS__)
#	else
#		define SPI_DEBUG_LOG(x, ...) // do nothing
#	endif
#	ifdef CAN_DEBUG_LOGGING
#		define CAN_DEBUG_LOG(x, ...) DEBUG_LOG(x, ##__VA_ARGS__)
#	else
#		define CAN_DEBUG_LOG(x, ...) // do nothing
#	endif
#	ifdef PC_COMMS_DEBUG_LOGGING
#		define PC_COMMS_DEBUG_LOG(x, ...) DEBUG_LOG(x, ##__VA_ARGS__)
#	else
#		define PC_COMMS_DEBUG_LOG(x, ...) // do nothing
#	endif
# else
#	define DEBUG_LOG(x, ...) // do nothing
#	define SPI_DEBUG_LOG(x, ...) // do nothing
#	define CAN_DEBUG_LOG(x, ...) // do nothing
#endif



#endif /* LOGGER_H_ */