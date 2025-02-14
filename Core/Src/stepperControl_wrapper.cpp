// stepperControl_wrapper.cpp
#include "stepperControl_wrapper.h"
#include "stepperControl.hpp"

// Declare the global stepper motor objects
extern StepperMotor motor1;
extern StepperMotor motor2;
extern StepperMotor motor3;
extern StepperMotor motor4;

static StepperMotor* motors[4] = { &motor1, &motor2, &motor3, &motor4 };

extern "C" {

void stepperControl_init_wrapper(void) {
    stepperControl_init();
}

void stepperControl_home_wrapper(void) {
	home();
}

void setStepperDesiredPos(uint8_t motorIndex, int pos) {
    if (motorIndex < 4) {
        motors[motorIndex]->desiredPos = pos;
    }
}

int getStepperCurrentPos(uint8_t motorIndex) {
    if (motorIndex < 4) {
        return motors[motorIndex]->currentPos;
    }
    return 0;
}

}
