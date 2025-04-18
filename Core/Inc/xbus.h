
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

#ifndef XBUS_H
#define XBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "board.h"


#ifdef __cplusplus
extern "C" {
#endif

#define OFFSET_TO_PREAMBLE		0
#define OFFSET_TO_BID			1
#define OFFSET_TO_MID			2
#define OFFSET_TO_LEN			3
#define OFFSET_TO_LEN_EXT_HI	4
#define OFFSET_TO_LEN_EXT_LO	5
#define OFFSET_TO_PAYLOAD		4
#define OFFSET_TO_PAYLOAD_EXT	6
#define XBUS_CHECKSUM_SIZE		1
#define LENGTH_EXTENDER_BYTE	0xFF
#define XBUS_PREAMBLE			0xFA
#define XBUS_MASTERDEVICE		0xFF
#define XBUS_EXTENDED_LENGTH	0xFF

#define MTI_I2C_DEVICE_ADDRESS 0x6B
#define WAITING_FOR_WAKEUP 0
#define WAITING_FOR_ID 1
#define WAITING_FOR_CONFIG_ACK 2
#define WAITING_FOR_MEASUREMENT_ACK 3
#define READY 4


bool Xbus_checkPreamble(const uint8_t* xbusMessage);

int  Xbus_getBusId(const uint8_t* xbusMessage);
void Xbus_setBusId(uint8_t* xbusMessage, uint8_t busId);

int  Xbus_getMessageId(const uint8_t* xbusMessage);
void Xbus_setMessageId(uint8_t* xbusMessage, uint8_t messageId);

int  Xbus_getPayloadLength(const uint8_t* xbusMessage);
void Xbus_setPayloadLength(uint8_t* xbusMessage, uint16_t payloadLength);

void Xbus_message(uint8_t* xbusMessage, uint8_t bid, uint8_t mid, uint16_t len);

int Xbus_getRawLength(const uint8_t* xbusMessage);

uint8_t* Xbus_getPointerToPayload(uint8_t* xbusMessage);
uint8_t const* Xbus_getConstPointerToPayload(uint8_t const* xbusMessage);


void Xbus_insertChecksum(uint8_t* xbusMessage);
bool Xbus_verifyChecksum(const uint8_t* xbusMessage);


bool checkDataReadyLineMain();
size_t Xbus_createRawMessageHelper(uint8_t* dest, uint8_t const* message);
uint8_t extractUint8(const uint8_t* data, int *index);
uint16_t extractUint16(const uint8_t* data, int *index);
uint32_t extractUint32(const uint8_t* data, int *index);
float extractFloat(const uint8_t* data, int *index);

#ifdef __cplusplus
}
//}
#endif


#endif
