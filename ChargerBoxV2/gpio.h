/*
 * Leds.h
 *
 * Created: 21/04/2023 1:39:39 PM
 *  Author: Michael Jones
 */ 


#ifndef GPIO_H_
#define GPIO_H_

typedef enum {
	OFF = 0, ON = 1
} GpioState;

typedef enum {
	CHARGING_LED,
	CHARGER_FAULT_LED,
	AMS_FAULT_LED,
	ENABLE_CHARGE_SIGNAL,
	GENERIC_SHUTDOWN_SIGNAL,
	IMD_FEEDBACK,
	AMS_FEEDBACK
} GpioName;

/** Set up all GPIO pin registers */
void setupGpios();

/** Set the state of an output GPIO pin. Has no effect on input pins. */
void setGpioState(GpioName name, GpioState state);

/** Get the state of an input GPIO pin. Returns OFF for output pins. */
GpioState getGpioState(GpioName name);


#endif /* LEDS_H_ */