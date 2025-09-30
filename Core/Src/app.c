/*
 * app.c
 *
 *  Created on: Mar 17, 2025
 *      Author: camer
 */


#include "app.h"

#define TEENSY_ADDR       0x42      /* 7-bit address             */
#define REG_OFFSET        0x00      /* Teenys writable block     */

char txBuffer[256];


void app_start(UART_HandleTypeDef *huart) {
	uint8_t rxChar;
	while (1) {
		// Blocking call to receive one character
		int len = snprintf(txBuffer, sizeof(txBuffer), "------ SADS Balance Configuration Tool -----\n");
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
		len = snprintf(txBuffer, sizeof(txBuffer), "Press 'h' to home system\nPress 'm' to go into manual mode\nPress 'i' to configure IMU sample rate\n");
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);

		if (HAL_UART_Receive(huart, &rxChar, 1, HAL_MAX_DELAY) == HAL_OK) {
			if (rxChar == 'h') {
				len = snprintf(txBuffer, sizeof(txBuffer), "Entering homing routine in 5 seconds, ensure 24V power is connected...\n");
				HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
				HAL_Delay(5000);
				home(huart);

			} else if(rxChar == 'm') {
				manualControl(huart);
			} else if(rxChar == 'i') {
				MTi_manual_init(huart);
			} else {
				len = snprintf(txBuffer, sizeof(txBuffer), "Invalid keystroke\n");
				HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
			}
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM3) {
    	MTi_step();
//    	int gain = 200;
//
//    	int32_t positionCommands[4] = {(int32_t)gain*roll, (int32_t)gain*pitch, 0x01, 123};
//    	uint8_t teensyTxBuffer[1 + sizeof(positionCommands)];
//    	teensyTxBuffer[0] = REG_OFFSET;
//    	memcpy(&teensyTxBuffer[1], positionCommands, sizeof(positionCommands));
//    	if(HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDR << 1, teensyTxBuffer, sizeof(teensyTxBuffer), 100) != HAL_OK) {;
//			int len = snprintf(txBuffer, sizeof(txBuffer), "I2C Communication with Teensy Failed\n");
//			HAL_UART_Transmit(&huart3, (uint8_t *)txBuffer, len, 100);
//    	}
    }

    if(htim->Instance == TIM1) {
    	int len = snprintf(txBuffer, sizeof(txBuffer), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f      Quat: [ %.2f %.2f %.2f %.2f ]\n",roll,pitch,yaw,q0,q1,q2,q3);
    	HAL_UART_Transmit(user_huart, (uint8_t *)txBuffer, len, 10000);
    }
}


