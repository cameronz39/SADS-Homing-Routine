/*
 * MTi.c
 *
 *  Created on: Feb 11, 2025
 *      Author: camer
 */

#include "MTi.h"

char UART_buffer[100];
int len = 0;

uint8_t state = WAITING_FOR_WAKEUP;
uint8_t m_dataBuffer[256]; // Buffer for incoming messages


uint8_t m_xbusTxBuffer[256]; // Buffer for outgoing messages

uint8_t buffer[128]; // Helper buffer for creating outgoing messages
size_t rawLength;


uint16_t notificationMessageSize;
uint16_t measurementMessageSize;
uint8_t status[4];

void MTi_init(uint8_t sampleRate) {
	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);


	while(state != READY) {
		if(checkDataReadyLineMain()) {
			HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_PIPE_STATUS, 1, status, sizeof(status), 100);
			notificationMessageSize = status[0] | (status[1] << 8);
			measurementMessageSize = status[2] | (status[3] << 8);
		}

		if ((notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer)) ) {
			if(checkDataReadyLineMain()) {
				HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_NOTIFICATION_PIPE, 1, &m_dataBuffer[2], notificationMessageSize, 100);


				// 3) User xbus.h helper to read the message ID and enter a new program state if needed
				if (Xbus_getMessageId(m_dataBuffer) == XMID_Wakeup && state == WAITING_FOR_WAKEUP)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got Wakeup\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);

					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);

					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = WAITING_FOR_ID;
				}

				if (Xbus_getMessageId(m_dataBuffer) == XMID_DeviceId && state == WAITING_FOR_ID)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got Device ID\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);

					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_SetOutputConfig, 12);
					// Set Output mode: Euler angles (0x2030)
					Xbus_getPointerToPayload(m_xbusTxBuffer)[0] = 0x20;
					Xbus_getPointerToPayload(m_xbusTxBuffer)[1] = 0x30;
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

					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = WAITING_FOR_CONFIG_ACK;
				}

				// note: the config ack message is just the output config itself
				if(Xbus_getMessageId(m_dataBuffer) == XMID_OutputConfig && state == WAITING_FOR_CONFIG_ACK)
				{
					len = snprintf(UART_buffer, sizeof(UART_buffer), "Got config ACK\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);

					uint8_t buffer[2];
					HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), XBUS_PROTOCOL_INFO, 1, buffer, sizeof(buffer), 100);

					uint8_t version = buffer[0];
					uint8_t dataReadyConfig = buffer[1];

					len = snprintf(UART_buffer, sizeof(UART_buffer), "Version: %d\nData Ready Config: %d\n",version,dataReadyConfig);
					HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);

//					Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
//					rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
//					HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);

					state = READY;
				}
			}
		}

	}
	// END WHILE LOOP
}

void MTi_goToMeasurement() {
	size_t rawLength;
	uint8_t m_xbusTxBuffer[256]; // Buffer for outgoing messages
	uint8_t buffer[128]; // Helper buffer for creating outgoing messages
	Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
	rawLength = Xbus_createRawMessageHelper(buffer, m_xbusTxBuffer);
	HAL_I2C_Master_Transmit(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1), (uint8_t*)buffer, rawLength, 100);
}

void MTi_step(float *anglesBuffer, size_t bufferLength) {
    // Ensure the provided buffer is large enough (needs at least 3 floats).
    if (bufferLength < 3) {
        // Optionally, you can handle the error here.
        int len = snprintf(UART_buffer, sizeof(UART_buffer), "Error\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
        return;
    }

    // Check if new data is available.
    if (checkDataReadyLineMain()) {
        HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1),
                           XBUS_PIPE_STATUS, 1, status, sizeof(status), 100);
        notificationMessageSize = status[0] | (status[1] << 8);
        measurementMessageSize    = status[2] | (status[3] << 8);
    }

    if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer)) {
        if (checkDataReadyLineMain()) {
            // Read the measurement data into the buffer (starting at offset 2)
            HAL_I2C_Mem_Read(&hi2c1, (MTI_I2C_DEVICE_ADDRESS << 1),
                               XBUS_MEASUREMENT_PIPE, 1, &m_dataBuffer[2],
                               measurementMessageSize, 100);

            // Check that the message is of type XMID_MtData2
            if (Xbus_getMessageId(m_dataBuffer) == XMID_MtData2) {
//            	int len = snprintf(UART_buffer, sizeof(UART_buffer), "Took a meausrement\n");
//            	HAL_UART_Transmit(&huart2, (uint8_t *)UART_buffer, len, 10000);
                int index = 4;  // Start index for reading the payload

                uint16_t dataId   = extractUint16(m_dataBuffer, &index);
                uint8_t  dataSize = extractUint8(m_dataBuffer, &index);
                if (dataId == 0x2030 && dataSize == 12) {
                    // Extract Euler angles: roll, pitch, yaw
                    anglesBuffer[0] = extractFloat(m_dataBuffer, &index); // roll
                    anglesBuffer[1] = extractFloat(m_dataBuffer, &index); // pitch
                    anglesBuffer[2] = extractFloat(m_dataBuffer, &index); // yaw
                }
                // Optionally, handle the case when dataId/dataSize are not as expected.
            }
        }
    }
}
