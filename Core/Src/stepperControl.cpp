#include "stepperControl.hpp"
// In this example we use the CPU's frequency (what was set in CubeMX by default).
uint32_t timerInputFrequency = F_CPU;
// tick generation running at 10kHz
uint32_t frequencyDivision = timerInputFrequency/100000;
uint32_t timerCounterBits = 16;
char UART_buffer2[50];
TimerArrayControl control(&htim2, timerInputFrequency, frequencyDivision, timerCounterBits);
StepperMotor motor4, motor3, motor2, motor1;

char homingState;
bool homing;
char MOTOR1_READY;
char MOTOR2_READY;
char MOTOR3_READY;
char MOTOR4_READY;

void motor4_callback(StepperMotor *motor);
ContextTimer<StepperMotor> motor4_timer(16, true, &motor4, motor4_callback);

void motor3_callback(StepperMotor *motor);
ContextTimer<StepperMotor> motor3_timer(16, true, &motor3, motor3_callback);

void motor2_callback(StepperMotor *motor);
ContextTimer<StepperMotor> motor2_timer(16, true, &motor2, motor2_callback);

void motor1_callback(StepperMotor *motor);
ContextTimer<StepperMotor> motor1_timer(16, true, &motor1, motor1_callback);

void doStep(void *context) {
	StepperMotor *motor = (StepperMotor *)context;
	if(motor->currentPos < motor->desiredPos) {
		HAL_GPIO_WritePin(motor->dirPort,motor->dirPin,GPIO_PIN_SET);
		HAL_GPIO_TogglePin(motor->stepPort,motor->stepPin);
		if (motor->toggleCount == 1) {
			motor->currentPos++;
		}

		if (motor->toggleCount == 0) {
			motor->toggleCount = 1;
		} else {
			motor->toggleCount = 0;
		}
	} else if (motor->currentPos > motor->desiredPos) {
		HAL_GPIO_WritePin(motor->dirPort,motor->dirPin,GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(motor->stepPort,motor->stepPin);
		if (motor->toggleCount == 1) {
			motor->currentPos--;
		}

		if (motor->toggleCount == 0) {
			motor->toggleCount = 1;
		} else {
			motor->toggleCount = 0;
		}
	}
}

void motor4_callback(StepperMotor *motor) {
	doStep(motor);
}
void motor3_callback(StepperMotor *motor) {
	doStep(motor);
}
void motor2_callback(StepperMotor *motor) {
	doStep(motor);
}
void motor1_callback(StepperMotor *motor) {
	doStep(motor);
}

void stepperControl_init(){

	motor4.stepPort = STEP4_GPIO_Port;
	motor4.stepPin  = STEP4_Pin;
	motor4.dirPort  = DIR4_GPIO_Port;
	motor4.dirPin   = DIR4_Pin;
	motor4.currentPos = 0;
	motor4.desiredPos = 0;
	motor4.stepTimer = &motor4_timer;
	motor4.toggleCount = 0;
    // control.attachTimer(&motor4_timer);

	motor3.stepPort = STEP3_GPIO_Port;
	motor3.stepPin  = STEP3_Pin;
	motor3.dirPort  = DIR3_GPIO_Port;
	motor3.dirPin   = DIR3_Pin;
	motor3.currentPos = 0;
	motor3.desiredPos = 0;
	motor3.stepTimer = &motor3_timer;
	motor3.toggleCount = 0;
    // control.attachTimer(&motor3_timer);

    motor2.stepPort = STEP2_GPIO_Port;
	motor2.stepPin  = STEP2_Pin;
	motor2.dirPort  = DIR2_GPIO_Port;
	motor2.dirPin   = DIR2_Pin;
	motor2.currentPos = 0;
	motor2.desiredPos = 0;
	motor2.stepTimer = &motor2_timer;
	motor2.toggleCount = 0;
	// control.attachTimer(&motor2_timer);

	motor1.stepPort = STEP1_GPIO_Port;
	motor1.stepPin  = STEP1_Pin;
	motor1.dirPort  = DIR1_GPIO_Port;
	motor1.dirPin   = DIR1_Pin;
	motor1.currentPos = 0;
	motor1.desiredPos = 0;
	motor1.stepTimer = &motor1_timer;
	motor1.toggleCount = 0;
	// control.attachTimer(&motor1_timer);

	control.begin();
}

void home(){

	MOTOR1_READY = 0;
	MOTOR2_READY = 0;
	homing = 1;

	motor1.desiredPos = 1000000;
	control.attachTimer(&motor1_timer);
	motor2.desiredPos = 1000000;
	control.attachTimer(&motor2_timer);
	while((!MOTOR1_READY) || (!MOTOR2_READY)) {

	}

	int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Finished Homing, begin in 10 seconds...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);

	HAL_Delay(10000);
	control.detachTimer(&motor1_timer);
	control.detachTimer(&motor2_timer);

	homing = 0;

	HAL_Delay(2000);
	control.changeTimerDelay(&motor1_timer,3);
	control.changeTimerDelay(&motor2_timer,3);
	control.attachTimer(&motor1_timer);
	control.attachTimer(&motor2_timer);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(!homing) {
	    if ((GPIO_Pin == LIMIT_SWITCH1_Pin) || (GPIO_Pin == LIMIT_SWITCH2_Pin)) {  // Ensure LIMIT_SWITCH_Pin is defined correctly
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);

	        control.detachTimer(&motor1_timer);
	        control.detachTimer(&motor2_timer);
	        control.detachTimer(&motor3_timer);
	        control.detachTimer(&motor4_timer);

	        motor1.currentPos = 0;
	        motor2.currentPos = 0;
	        motor3.currentPos = 0;
	        motor4.currentPos = 0;
	    } else {
	    	__NOP();
	    }
	}


	if(homing) {
		if ((GPIO_Pin == LIMIT_SWITCH1_Pin) && (!MOTOR1_READY)) {
			MOTOR1_READY = 1;
			motor1.currentPos = HOMING_STEPS_REQ;
			motor1.desiredPos = 0;
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 1!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);

		}
		if ((GPIO_Pin == B1_Pin) && (!MOTOR2_READY)) {
			MOTOR2_READY = 1;
			motor2.currentPos = HOMING_STEPS_REQ;
			motor2.desiredPos = 0;
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 2!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);
		}

	}
}






