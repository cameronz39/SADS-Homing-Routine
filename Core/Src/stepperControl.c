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

char printingRollPitch = 0;

enum Status {
	STANDBY,
	HOMING,
	MANUAL_CONTROL,
	SAFE_MODE
};

enum Status currentStatus;

UART_HandleTypeDef *selected_huart;

void togglePrintMode();
void sendPosCommand(int,int);
void sendSinglePosCommand(char,int);
void sendSingleRelPosCommand(char,int);
void sendVelocityCommand(int);
void sendPosUpdate(char,int);
void resetPositions();
void emergencyStop();
void resetSystem();

void stepperControl_init(UART_HandleTypeDef *huart){
	currentStatus = STANDBY;
	resetSystem();
	selected_huart = huart;

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

    motor2.stepPort = STEP2_GPIO_Port;
	motor2.stepPin  = STEP2_Pin;
	motor2.dirPort  = DIR2_GPIO_Port;
	motor2.dirPin   = DIR2_Pin;
	motor2.currentPos = 0;
	motor2.desiredPos = 0;
	motor2.toggleCount = 0;
	motor2.active = 0;

	motor1.stepPort = STEP1_GPIO_Port;
	motor1.stepPin  = STEP1_Pin;
	motor1.dirPort  = DIR1_GPIO_Port;
	motor1.dirPin   = DIR1_Pin;
	motor1.currentPos = 0;
	motor1.desiredPos = 0;
	motor1.toggleCount = 0;
	motor1.active = 0;
}

void home(UART_HandleTypeDef *huart){
	resetPositions();
	sendVelocityCommand(STEPS_PER_REV);
	currentStatus = HOMING;
	MOTOR3_READY = 0;
	MOTOR4_READY = 0;
	homing = 1;

	sendSinglePosCommand(3,200000);
	sendSinglePosCommand(4,200000); // I2C call is blocking, spamming buffer okay
	motor3.desiredPos = 100000;
	motor3.active = 1;
	motor4.desiredPos = 100000;
	motor4.active = 1;
	int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Motor 3 and 4 Active and Desired Pos Set\n");
	HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 10000);
	while((!MOTOR3_READY) || (!MOTOR4_READY)) {

	}

	len = snprintf(UART_buffer2, sizeof(UART_buffer2), "MOT 3 and 4 done, beginning next stage in 10 seconds...\r\n");
	HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 10000);
	HAL_Delay(10000);

	motor4.active = 0;
	motor3.active = 0;

	sendSinglePosCommand(1,200000);
	sendSinglePosCommand(2,200000); // I2C call is blocking, spamming buffer okay

	MOTOR1_READY = 0;
	MOTOR2_READY = 0;

	motor1.desiredPos = 100000;
	motor1.active = 1;
	motor2.desiredPos = 100000;
	motor2.active = 1;

	len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Motor 1 and 2 Active and Desired Pos Set\n");
	HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 10000);
	while((!MOTOR1_READY) || (!MOTOR2_READY)) {

	}



	len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Homing routine finished\n");
	HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 10000);
	HAL_Delay(2000);
	currentStatus = STANDBY;
	homing = 0;
}

void manualControl(UART_HandleTypeDef *huart) {
	currentStatus = MANUAL_CONTROL;
	while(1) {

		enum axis {
			X_AXIS,
			Y_AXIS
		};

		enum axis userAxis;
		char inputBuffer[256];
		uint8_t rxChar;
		int index = 0;

		// Prompt the user for input
		char txBuffer[256];

		int len = snprintf(txBuffer, sizeof(txBuffer), "Select an axis 'x' or 'y', or press 'b' to return\r\n");
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);

		while (1) {
			// Blocking call to receive one character
			if (HAL_UART_Receive(huart, &rxChar, 1, HAL_MAX_DELAY) == HAL_OK) {
				if (rxChar == 'x') {
					userAxis = X_AXIS;
					break;

				} else if(rxChar == 'y') {
					userAxis = Y_AXIS;
					break;

				} else if(rxChar == 'b' ) {
					return;
				} else if(rxChar == 'p' ){
					togglePrintMode();
				} else {
					len = snprintf(txBuffer, sizeof(txBuffer), "Invalid keystroke\r\n");
					HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
				}
			}
		}

		len = snprintf(txBuffer, sizeof(txBuffer), "%d, Enter a step number and press Return:\r\n", userAxis);
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);


		// Loop to receive one character at a time
		while (1) {
			// Blocking call to receive one character
			if (HAL_UART_Receive(huart, &rxChar, 1, HAL_MAX_DELAY) == HAL_OK) {
				// Optionally echo the character back to the terminal
				HAL_UART_Transmit(huart, &rxChar, 1, 100);

				// Check if the character is a newline or carriage return (end of input)
				if (rxChar == '\n' || rxChar == '\r') {
					inputBuffer[index] = '\0';  // Null-terminate the string
					break;
				} else if (rxChar == 8){
					int len = snprintf(txBuffer, sizeof(txBuffer), "\nInput buffer cleared, re-enter number:\n");
					HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
					index = 0;
				} else {
					// Only store the character if there is still room in the buffer
					if (index < 256 - 1) {
						inputBuffer[index++] = rxChar;
					} else {
						// If the buffer is full, null-terminate and break out
						inputBuffer[index] = '\0';
						break;
					}
				}
			}
		}

		// Convert the accumulated string to an integer
		int userSteps = atoi(inputBuffer);

		// Send back the result
		len = snprintf(txBuffer, sizeof(txBuffer), "Waiting %d milliseconds to step...\n",(int)(abs(userSteps) * 0.3));
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);

		if(userAxis == X_AXIS) {
			motor1.currentPos = motor1.currentPos - userSteps;
			motor2.currentPos = motor2.currentPos + userSteps;

			sendSingleRelPosCommand(1,-userSteps);
			sendSingleRelPosCommand(2,+userSteps);
			HAL_Delay((int)(abs(userSteps) * 0.3));

		} else if(userAxis == Y_AXIS) {
			motor3.currentPos = motor3.currentPos - userSteps;
			motor4.currentPos = motor4.currentPos + userSteps;
			sendSingleRelPosCommand(3,-userSteps);
			sendSingleRelPosCommand(4,+userSteps);
			HAL_Delay((int)(abs(userSteps) * 0.3));
		}
		len = snprintf(txBuffer, sizeof(txBuffer), "Mot X Pos: %d, Mot Y Pos: %d\n",motor2.currentPos,motor4.currentPos);
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if((currentStatus == MANUAL_CONTROL)) {
	    if ((GPIO_Pin == LIMIT_SWITCH3_Pin) || (GPIO_Pin == LIMIT_SWITCH4_Pin) || (GPIO_Pin == LIMIT_SWITCH1_Pin)|| (GPIO_Pin == LIMIT_SWITCH2_Pin)) {
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered, entering safemode. Restart program when ready\n");
			HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 100);

			emergencyStop();

	        currentStatus = SAFE_MODE;
	    } else {
	    	__NOP();
	    }
	}

	if(currentStatus == HOMING) {
		if ((GPIO_Pin == LIMIT_SWITCH3_Pin) && (!MOTOR3_READY)) {\
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 3!\r\n");
			HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 100);
			MOTOR3_READY = 1;
			motor3.currentPos = 0;
			sendPosUpdate(3,HOMING_STEPS_REQ);
			sendSinglePosCommand(3,0);


		}
		if ((GPIO_Pin == LIMIT_SWITCH4_Pin) && (!MOTOR4_READY)) {
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 4!\r\n");
			HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 100);
			MOTOR4_READY = 1;
			motor4.currentPos = 0;
			sendPosUpdate(4,HOMING_STEPS_REQ);
			sendSinglePosCommand(4,0);

		}
		if ((GPIO_Pin == LIMIT_SWITCH1_Pin) && (!MOTOR1_READY)) {
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 1!\r\n");
			HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 100);
			MOTOR1_READY = 1;
			motor1.currentPos = 0;
			sendPosUpdate(1,HOMING_STEPS_REQ);
			sendSinglePosCommand(1,0);

		}
		if ((GPIO_Pin == LIMIT_SWITCH2_Pin) && (!MOTOR2_READY)) {\
			int len = snprintf(UART_buffer2, sizeof(UART_buffer2), "Interuppt triggered on switch 2!\r\n");
			HAL_UART_Transmit(selected_huart, (uint8_t *)UART_buffer2, len, 100);
			MOTOR2_READY = 1;
			motor2.currentPos = 0;
			sendPosUpdate(2,HOMING_STEPS_REQ);
			sendSinglePosCommand(2,0);
		}
	}
}

void togglePrintMode() {
	if(printingRollPitch) {
		HAL_TIM_Base_Stop_IT(&htim1);
		printingRollPitch = 0;
	} else {
		HAL_TIM_Base_Start_IT(&htim1);
		printingRollPitch = 1;
	}

}

void sendPosCommand(int posX, int posY) {
	int32_t packet[4] = {posX,posY,0x01,0};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void sendSinglePosCommand(char motorNumber, int pos) {
	int32_t packet[4] = {pos,0,0x05,motorNumber};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void sendSingleRelPosCommand(char motorNumber, int del_pos) {
	int32_t packet[4] = {del_pos,0,0x07,motorNumber};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void sendVelocityCommand(int velocity) {
	int32_t packet[4] = {0,0,0x02,velocity};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void sendPosUpdate(char motorNumber, int pos) {
	int32_t packet[4] = {pos,0,0x03,motorNumber};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void emergencyStop() {
	int32_t packet[4] = {0,0,0x04,0};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void resetPositions() {
	int32_t packet[4] = {0,0,0x06,0};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}

void resetSystem() {
	int32_t packet[4] = {0,0,0x08,0};
	char txBuffer[64];
	uint8_t teensyTxBuffer[1 + sizeof(packet)];
	teensyTxBuffer[0] = 0x00; // starting register
	memcpy(&teensyTxBuffer[1], packet, sizeof(packet));
	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
		HAL_UART_Transmit(selected_huart, (uint8_t *)txBuffer, len, 100);
	}
}




