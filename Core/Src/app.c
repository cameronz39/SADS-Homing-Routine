/*
 * app.c
 *
 *  Created on: Mar 17, 2025
 *      Author: camer
 */


#include "app.h"

void app_start(UART_HandleTypeDef *huart) {
	uint8_t rxChar;

	char txBuffer[256];
	// uint8_t sampleRate = 0x40;

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
				home();

			} else if(rxChar == 'm') {
				manualControl(&huart5);
			} else if(rxChar == 'i') {
				MTi_manual_init(&huart5);
			} else {
				len = snprintf(txBuffer, sizeof(txBuffer), "Invalid keystroke\n");
				HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
			}
		}

	}
}
