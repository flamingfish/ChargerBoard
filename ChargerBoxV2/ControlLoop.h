
/*
 * ControlLoop.h
 *
 * Created: 30/07/2023 11:36:28 AM
 *  Author: Michael Jones
 */ 

#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_


typedef enum OperatingMode {
	OPERATING_IDLE,
	OPERATING_CHARGING,
	OPERATING_ERROR,
	OPERATING_TESTING
	// may need new state for STOPPING_CHARGING
} OperatingMode;

#endif /* CONTROL_LOOP_H_ */