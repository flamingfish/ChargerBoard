/*
 * timer.h
 *
 * Created: 24/06/2023 11:46:08 AM
 *  Author: Michael Jones
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

/**
 * Set up timer for millis() function.
 */
void setupTimer();

/**
 * Return the number of milliseconds since startup.
 */
uint32_t millis();

#endif /* TIMER_H_ */