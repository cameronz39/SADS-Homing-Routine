/*
 * MTi.h
 *
 *  Created on: Feb 11, 2025
 *      Author: camer
 */

#ifndef INC_MTI_H_
#define INC_MTI_H_

#include "main.h"
#include "xbus.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

void MTi_manual_init(UART_HandleTypeDef *huart);
void MTi_init(uint8_t sampleRate, UART_HandleTypeDef *huart);
void MTi_goToMeasurement();
void MTi_step(UART_HandleTypeDef *huart);
void MTi_test_init();
#endif /* INC_MTI_H_ */
