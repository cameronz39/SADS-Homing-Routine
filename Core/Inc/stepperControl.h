#ifndef INC_STEPPERCONTROL_H_
#define INC_STEPPERCONTROL_H_

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

static const int STEPS_PER_REV = 800;
static const int HOMING_STEPS_REQ = STEPS_PER_REV*9;


void stepperControl_init();
void home();
void manualControl();

typedef struct {
    // GPIO information
    GPIO_TypeDef *stepPort;
    uint16_t      stepPin;
    GPIO_TypeDef *dirPort;
    uint16_t      dirPin;
    uint8_t toggleCount;
    // Motion state
    volatile int currentPos;   // current step count
    volatile int desiredPos;   // target step count
    char active;
    // Timer for stepping (assume your Timer library supports a context pointer)
    // Timer *stepTimer;
} StepperMotor;

extern StepperMotor motor1;
extern StepperMotor motor2;
extern StepperMotor motor3;
extern StepperMotor motor4;

#endif
