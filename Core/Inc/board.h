
//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef BOARD_H
#define BOARD_H

/*!	\file board.h
	Defines macro's for GPIO pins and ports for the STM32F401NUCLEO board
*/

#define DEBUG_PORT					GPIOC
#define DEBUG_PIN_1					GPIO_PIN_10
#define DEBUG_PIN_2					GPIO_PIN_11
#define DEBUG_PIN_3					GPIO_PIN_12

#define RESET_PORT					GPIOB
#define RESET_PIN					GPIO_PIN_5

#define CHIP_SELECT_PORT			GPIOB
#define CHIP_SELECT_PIN				GPIO_PIN_6

#define DATA_READY_PORT				GPIOB
#define DATA_READY_PIN				GPIO_PIN_3
#define DATA_READY_IRQ_NR			EXTI3_IRQn

#define PSEL0_PORT					GPIOC
#define PSEL0_PIN					GPIO_PIN_7

#define PSEL1_PORT					GPIOA
#define PSEL1_PIN					GPIO_PIN_9

#define USART_TX_PIN				GPIO_PIN_2
#define USART_TX_GPIO_PORT			GPIOA
#define USART_RX_PIN				GPIO_PIN_3
#define USART_RX_GPIO_PORT			GPIOA

#define I2C_ADD_PORT				GPIOA
#define I2C_ADD0_PIN				GPIO_PIN_7
#define I2C_ADD1_PIN				GPIO_PIN_6
#define I2C_ADD2_PIN				GPIO_PIN_5

#define XBUS_PROTOCOL_INFO (0x01)
#define XBUS_CONFIGURE_PROTOCOL (0x02)
#define XBUS_CONTROL_PIPE (0x03)
#define XBUS_PIPE_STATUS (0x04)
#define XBUS_NOTIFICATION_PIPE (0x05)
#define XBUS_MEASUREMENT_PIPE (0x06)

enum XsMessageId
{
	XMID_Wakeup              = 0x3E,
	XMID_WakeupAck           = 0x3F,
	XMID_ReqDid              = 0x00,
	XMID_DeviceId            = 0x01,
	XMID_GotoConfig          = 0x30,
	XMID_GotoConfigAck       = 0x31,
	XMID_GotoMeasurement     = 0x10,
	XMID_GotoMeasurementAck  = 0x11,
	XMID_MtData2             = 0x36,
	XMID_ReqOutputConfig     = 0xC0,
	XMID_SetOutputConfig     = 0xC0,
	XMID_OutputConfig        = 0xC1,
	XMID_Reset               = 0x40,
	XMID_ResetAck            = 0x41,
	XMID_Error               = 0x42,
	XMID_ToggleIoPins        = 0xBE,
	XMID_ToggleIoPinsAck     = 0xBF,
	XMID_FirmwareUpdate      = 0xF2,
	XMID_GotoBootLoader      = 0xF0,
	XMID_GotoBootLoaderAck   = 0xF1,
	XMID_ReqFirmwareRevision = 0x12,
	XMID_FirmwareRevision    = 0x13


};


#endif
