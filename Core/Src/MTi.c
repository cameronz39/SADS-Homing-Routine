/*
 * MTi.c
 *
 *  Created on: Feb 11, 2025
 *      Author: camer
 */

#include "MTi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

char UART_buffer[256];
int len = 0;

uint8_t state;
uint8_t m_dataBuffer[256]; // Buffer for incoming messages


uint8_t m_xbusTxBuffer[256]; // Buffer for outgoing messages

uint8_t buffer[128]; // Helper buffer for creating outgoing messages
size_t rawLength;

float roll_mounting_error = -1.35;
float pitch_mounting_error = -1.11;

uint16_t notificationMessageSize;
uint16_t measurementMessageSize;
uint8_t status[4];

float roll, pitch, yaw;

float q0, q1, q2, q3;

void readAndPrintNotification(UART_HandleTypeDef *huart);
void MTi_init(uint8_t sampleRate, UART_HandleTypeDef *huart) {
	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;
	state = WAITING_FOR_WAKEUP;

	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);

	while(state != READY) {
		// HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);
		if(checkDataReadyLineMain()) {
			HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_PIPE_STATUS, 1, status, sizeof(status), 100);
			notificationMessageSize = status[0] | (status[1] << 8);
			measurementMessageSize = status[2] | (status[3] << 8);
		}

		if ((notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer)) ) {
			if(checkDataReadyLineMain()) {
				if(HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_NOTIFICATION_PIPE, 1, &m_dataBuffer[2], notificationMessageSize, 1000) != HAL_OK) {
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Failed to connect to MTi\n");
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

					break;
				}
				// 3) User xbus.h helper to read the message ID and enter a new program state if needed
				if (Xbus_getMessageId(m_dataBuffer) == XMID_Wakeup && state == WAITING_FOR_WAKEUP)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got Wakeup\n");
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);

					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = WAITING_FOR_ID;
				}

				if (Xbus_getMessageId(m_dataBuffer) == XMID_DeviceId && state == WAITING_FOR_ID)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got Device ID\n");
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

					// IMPORTANT: be sure to change the length of this message depending on which
					// meausurements you include
					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_SetOutputConfig, 16);
					// Set Output mode: RotMatrix (0x2020)
					Xbus_getPointerToPayload(m_xbusTxBuffer)[0] = 0x20;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[1] = 0x20;
					// Set Output rate
					Xbus_getPointerToPayload(m_xbusTxBuffer)[2] = 0x00;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[3] = sampleRate;

					// Set Output mode: Quaternion (0x2010)
					Xbus_getPointerToPayload(m_xbusTxBuffer)[4] = 0x20;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[5] = 0x10;
					// Set Output rate
					Xbus_getPointerToPayload(m_xbusTxBuffer)[6] = 0x00;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[7] = sampleRate;

					// Set Output mode: Body Rates (0x8020)
					Xbus_getPointerToPayload(m_xbusTxBuffer)[8] = 0x80;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[9] = 0x20;
					// Set Output rate
					Xbus_getPointerToPayload(m_xbusTxBuffer)[10] = 0x00;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[11] = sampleRate;

					// Set Output mode: Euler Angles (0x2030)
					Xbus_getPointerToPayload(m_xbusTxBuffer)[12] = 0x20;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[13] = 0x30;
					// Set Output rate
					Xbus_getPointerToPayload(m_xbusTxBuffer)[14] = 0x00;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[15] = sampleRate;


					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = WAITING_FOR_CONFIG_ACK;
				}

				// note: the config ack message is just the output config itself
				if(Xbus_getMessageId(m_dataBuffer) == XMID_OutputConfig && state == WAITING_FOR_CONFIG_ACK)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got config ACK\n");
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

					uint8_t buffer[2];
					HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_PROTOCOL_INFO, 1, buffer, sizeof(buffer), 100);

					uint8_t version = buffer[0];
					uint8_t dataReadyConfig = buffer[1];

					len = snprintf(UART_buffer, sizeof(UART_buffer), "Version: %d\nData Ready Pin configuartion: %d\n",version,dataReadyConfig);
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

					len = snprintf(UART_buffer, sizeof(UART_buffer), "MTi was successfully configured!\n");
					HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);

//					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
//					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
//					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = READY;
				}
			}
		}
	}
}

void MTi_manual_init(UART_HandleTypeDef *huart) {
	uint8_t rxChar;
	char txBuffer[256];
	char inputBuffer[256];
	int index = 0;
	int len = snprintf(txBuffer, sizeof(txBuffer), "Enter a sample rate less than 100:\r\n");
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
	int userSampleRate = atoi(inputBuffer);
	if (userSampleRate > 100) {
		int len = snprintf(txBuffer, sizeof(txBuffer), "Error: ensure sample rate is less than 100\n");
		HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
		return;
	}
	len = snprintf(txBuffer, sizeof(txBuffer), "Setting IMU Sample Rate to %d Hz\n",userSampleRate);
	HAL_UART_Transmit(huart, (uint8_t *)txBuffer, len, 100);
	MTi_init(userSampleRate,huart);
}


void MTi_goToMeasurement() {
	size_t rawLength;
	uint8_t m_xbusTxBuffer[256]; // Buffer for outgoing messages
	uint8_t buffer[128]; // Helper buffer for creating outgoing messages
	Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
	rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
	HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);
}

void readAndPrintNotification(UART_HandleTypeDef *huart) {
	int len;
	HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_MEASUREMENT_PIPE, 1, &m_dataBuffer[2], measurementMessageSize, 100);

	if(HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_NOTIFICATION_PIPE, 1, &m_dataBuffer[2], notificationMessageSize, 1000) != HAL_OK) {
	    // Print the hex data, excluding any extra zeros
	    for (int i = 2; i < notificationMessageSize + 2; i++) {
	        if (m_dataBuffer[i] != 0) {  // Only print non-zero values
	        	len = snprintf(UART_buffer, sizeof(UART_buffer), "0x%02X ", m_dataBuffer[i]);
	        	HAL_UART_Transmit(huart, (uint8_t *)UART_buffer, len, 10000);
	        }
	    }
	}
}

void MTi_step() {
	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	// 1) Read pipe status and save incoming message sizes
	//	len = snprintf(UART_buffer, sizeof(UART_buffer), "Notif: %d, Msg: %d \n", notificationMessageSize,measurementMessageSize);
	//	HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 100);
	if(checkDataReadyLineMain()) {
		HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_PIPE_STATUS, 1, status, sizeof(status), 100);

		notificationMessageSize = status[0] | (status[1] << 8);
		measurementMessageSize = status[2] | (status[3] << 8);


		if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer)) {
			HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_MEASUREMENT_PIPE, 1, &m_dataBuffer[2], measurementMessageSize, 100);

			if(Xbus_getMessageId(m_dataBuffer) == XMID_MtData2) {
                int index = 4;  // Start index for reading the payload

                uint16_t dataId   = extractUint16(m_dataBuffer, &index);
                uint8_t  dataSize = extractUint8(m_dataBuffer, &index);

                float rotMatrixBuffer[9];
                if (dataId == 0x2020) {
                	rotMatrixBuffer[0] = extractFloat(m_dataBuffer, &index);
                	rotMatrixBuffer[1] = extractFloat(m_dataBuffer, &index);
                	rotMatrixBuffer[2] = extractFloat(m_dataBuffer, &index);
                	rotMatrixBuffer[3] = extractFloat(m_dataBuffer, &index); // roll
                	rotMatrixBuffer[4] = extractFloat(m_dataBuffer, &index); // pitch
                	rotMatrixBuffer[5] = extractFloat(m_dataBuffer, &index); // yaw
                	rotMatrixBuffer[6] = extractFloat(m_dataBuffer, &index); // roll
                	rotMatrixBuffer[7] = extractFloat(m_dataBuffer, &index); // pitch
                	rotMatrixBuffer[8] = extractFloat(m_dataBuffer, &index); // yaw
                }

                dataId   = extractUint16(m_dataBuffer, &index);
				dataSize = extractUint8(m_dataBuffer, &index);
				float quatBuffer[4];
				if (dataId == 0x2010) {
					quatBuffer[0] = extractFloat(m_dataBuffer, &index);
					quatBuffer[1] = extractFloat(m_dataBuffer, &index);
					quatBuffer[2] = extractFloat(m_dataBuffer, &index);
					quatBuffer[3] = extractFloat(m_dataBuffer, &index);
				}
				q0 = quatBuffer[0];
				q1 = quatBuffer[1];
				q2 = quatBuffer[2];
				q3 = quatBuffer[3];

				dataId   = extractUint16(m_dataBuffer, &index);
				dataSize = extractUint8(m_dataBuffer, &index);
				float bodyRateBuffer[3];
				if (dataId == 0x8020) {
					bodyRateBuffer[0] = extractFloat(m_dataBuffer, &index);
					bodyRateBuffer[1] = extractFloat(m_dataBuffer, &index);
					bodyRateBuffer[2] = extractFloat(m_dataBuffer, &index);
				}
				dataId   = extractUint16(m_dataBuffer, &index);
				dataSize = extractUint8(m_dataBuffer, &index);

				float eulerAngleBuffer[3];
				if (dataId == 0x2030) {
					eulerAngleBuffer[0] = extractFloat(m_dataBuffer, &index);
					eulerAngleBuffer[1] = extractFloat(m_dataBuffer, &index);
					eulerAngleBuffer[2] = extractFloat(m_dataBuffer, &index);
				}

				roll = eulerAngleBuffer[0] - roll_mounting_error;
				pitch = eulerAngleBuffer[1]- pitch_mounting_error;
				yaw = eulerAngleBuffer[2];


//                len = snprintf(UART_buffer, sizeof(UART_buffer), "Rotation Matrix:\n  %.2f %.2f %.2f\n %.2f %.2f %.2f\n %.2f %.2f %.2f\n ",rotMatrixBuffer[0],rotMatrixBuffer[1],rotMatrixBuffer[2],rotMatrixBuffer[3],rotMatrixBuffer[4],rotMatrixBuffer[5],rotMatrixBuffer[6],rotMatrixBuffer[7],rotMatrixBuffer[8]);
//                HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
//
//				len = snprintf(UART_buffer, sizeof(UART_buffer), "Quaternion: %.2f %.2f %.2f %.2f\n",quatBuffer[0],quatBuffer[1],quatBuffer[2],quatBuffer[3]);
//				HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
//
//				len = snprintf(UART_buffer, sizeof(UART_buffer), "g in Body: %.2f %.2f %.2f\n",-9.81*rotMatrixBuffer[2],-9.81*rotMatrixBuffer[5],-9.81*rotMatrixBuffer[8]);
//				HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
//
//				len = snprintf(UART_buffer, sizeof(UART_buffer), "Body Rates: %.2f %.2f %.2f\n",bodyRateBuffer[0],bodyRateBuffer[1],bodyRateBuffer[2]);
//				HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
//
//				len = snprintf(UART_buffer, sizeof(UART_buffer), "Euler Angles: %.2f %.2f %.2f\n",eulerAngleBuffer[0],eulerAngleBuffer[1],eulerAngleBuffer[2]);
//				HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
			}
		}
	}
}


