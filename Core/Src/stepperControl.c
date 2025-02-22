#include <stepperControl.h>

char UART_buffer2[128];
StepperMotor motor4, motor3, motor2, motor1;
StepperMotor* motors[4] = { &motor1, &motor2, &motor3, &motor4 };

char homingState;
char homing;
char MOTOR1_READY;
char MOTOR2_READY;
char MOTOR3_READY;
char MOTOR4_READY;

void doStep() {

	for (int i = 0; i < 4; i++) {
		StepperMotor *motor = motors[i];
		if (motor->active) {
			if (motor->currentPos < motor->desiredPos) {
				// Set direction for forward motion
				motor->dirPort->BSRR = motor->dirPin;
				if (motor->toggleCount == 0) { // rising edge: set STEP high
					motor->stepPort->BSRR = motor->stepPin;
					motor->toggleCount = 1;
				} else { // falling edge: set STEP low and update position
					motor->stepPort->BSRR = (motor->stepPin << 16);
					motor->toggleCount = 0;
					motor->currentPos++;
//					int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Motor %d At Position: %d\n",i,motor->currentPos);
//					HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);
				}
			} else if (motor->currentPos > motor->desiredPos) {
				// Set direction for reverse motion using DIR clear
				motor->dirPort->BSRR = (motor->dirPin << 16);
				if (motor->toggleCount == 0) { // rising edge: set STEP high
					motor->stepPort->BSRR = motor->stepPin;
					motor->toggleCount = 1;
				} else { // falling edge: set STEP low and update position
					motor->stepPort->BSRR = (motor->stepPin << 16);
					motor->toggleCount = 0;
					motor->currentPos--;
				}
			}
		}
	}

}

void stepperControl_init(){

	motor4.stepPort = STEP4_GPIO_Port;
	motor4.stepPin  = STEP4_Pin;
	motor4.dirPort  = DIR4_GPIO_Port;
	motor4.dirPin   = DIR4_Pin;
	motor4.currentPos = 0;
	motor4.desiredPos = 0;
	motor4.toggleCount = 0;
	motor4.active = 0;

	motor3.stepPort = STEP3_GPIO_Port;
	motor3.stepPin  = STEP3_Pin;
	motor3.dirPort  = DIR3_GPIO_Port;
	motor3.dirPin   = DIR3_Pin;
	motor3.currentPos = 0;
	motor3.desiredPos = 0;
	motor3.toggleCount = 0;
	motor3.active = 0;

    motor2.stepPort = STEP3_GPIO_Port;
	motor2.stepPin  = STEP3_Pin;
	motor2.dirPort  = DIR3_GPIO_Port;
	motor2.dirPin   = DIR3_Pin;
	motor2.currentPos = 0;
	motor2.desiredPos = 0;
	motor2.toggleCount = 0;
	motor2.active = 0;

	motor1.stepPort = STEP4_GPIO_Port;
	motor1.stepPin  = STEP4_Pin;
	motor1.dirPort  = DIR4_GPIO_Port;
	motor1.dirPin   = DIR4_Pin;
	motor1.currentPos = 0;
	motor1.desiredPos = 0;
	motor1.toggleCount = 0;
	motor1.active = 0;
}

void home(){

	MOTOR1_READY = 0;
	MOTOR2_READY = 0;
	homing = 1;

	motor1.desiredPos = 1000000;
	motor1.active = 1;
	motor2.desiredPos = 1000000;
	motor2.active = 1;
	int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Motor 1 Active and Desired Pos Set\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);
	while((!MOTOR1_READY) || (!MOTOR2_READY)) {

	}

	len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Finished Homing, beginning next stage in 10 seconds...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);

	HAL_Delay(10000);
	motor1.active = 0;
	motor2.active = 0;

	homing = 0;

	HAL_Delay(2000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(!homing) {
	    if ((GPIO_Pin == LIMIT_SWITCH1_Pin) || (GPIO_Pin == LIMIT_SWITCH2_Pin)) {  // Ensure LIMIT_SWITCH_Pin is defined correctly
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer2, len, 10000);

	        motor1.active = 0;
	        motor2.active = 0;
	        motor3.active = 0;
	        motor4.active = 0;

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        doStep();
    }
}





