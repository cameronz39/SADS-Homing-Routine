#ifndef INC_STEPPERCONTROL_H_
#define INC_STEPPERCONTROL_H_

#include "main.h"
#include "TimerArrayInc/STM32TimerArray.hpp"

#include <cstring>
#include <cstdio>



const int STEPS_PER_REV = 800;
const int HOMING_STEPS_REQ = STEPS_PER_REV*9;

// Function prototypes and variables can use C++ linkage now.
void stepperControl_init();
void home();

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
    // Timer for stepping (assume your Timer library supports a context pointer)
    Timer *stepTimer;
} StepperMotor;

#endif
