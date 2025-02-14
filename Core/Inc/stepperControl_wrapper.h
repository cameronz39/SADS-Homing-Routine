/*
 * stepperControl_wrapper.h
 *
 *  Created on: Feb 11, 2025
 *      Author: camer
 */

#ifndef INC_STEPPERCONTROL_WRAPPER_H_
#define INC_STEPPERCONTROL_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Initialize the stepper control system (this calls the C++ initialization function).
void stepperControl_init_wrapper(void);
void stepperControl_home_wrapper(void);

// Set the desired position for motor number motorIndex (0–3).
// For example, motorIndex 0 corresponds to motor1, 1 to motor2, etc.
void setStepperDesiredPos(uint8_t motorIndex, int pos);

// Get the current position for motor number motorIndex (0–3).
int getStepperCurrentPos(uint8_t motorIndex);

#ifdef __cplusplus
}
#endif

#endif /* INC_STEPPERCONTROL_WRAPPER_H_ */
