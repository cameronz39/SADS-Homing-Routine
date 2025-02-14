
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

#include "xbus.h"


/*!	\brief Returns true if the preamble equeals 0xFA, false othersise
*/
bool Xbus_checkPreamble(const uint8_t* xbusMessage)
{
	return xbusMessage[OFFSET_TO_PREAMBLE] == XBUS_PREAMBLE;
}

/*! \brief Returns xbus Bus identifier
*/
int Xbus_getBusId(const uint8_t* xbusMessage)
{
	return (xbusMessage[OFFSET_TO_BID] & 0xff);
}

/*! \brief Sets xbus Bus identifier
*/
void Xbus_setBusId(uint8_t* xbusMessage, uint8_t busId)
{
	xbusMessage[OFFSET_TO_BID] = busId & 0xff;
}

/*! \brief Returns xbus Message identifier
*/
int Xbus_getMessageId(const uint8_t* xbusMessage)
{
	return (xbusMessage[OFFSET_TO_MID] & 0xff);
}

/*! \brief Sets xbus Message identifier
*/
void Xbus_setMessageId(uint8_t* xbusMessage, uint8_t messageId)
{
	xbusMessage[OFFSET_TO_MID] = messageId & 0xff;
}

/*! \brief Returns xbus message (payload) length
*/
int Xbus_getPayloadLength(const uint8_t* xbusMessage)
{
	int length = xbusMessage[OFFSET_TO_LEN] & 0xff;
	if (length != LENGTH_EXTENDER_BYTE)
		return length;
	else
	{
		int result = (xbusMessage[OFFSET_TO_LEN + 2] & 0xff);
		result += (xbusMessage[OFFSET_TO_LEN + 1] & 0xff) << 8;
		return result;
	}
}

/*! \brief Sets xbus message (payload) length
*/
void Xbus_setPayloadLength(uint8_t* xbusMessage, uint16_t payloadLength)
{
	if (payloadLength < 255)
		xbusMessage[OFFSET_TO_LEN] = payloadLength & 0xff;
	else
	{
		xbusMessage[OFFSET_TO_LEN] = LENGTH_EXTENDER_BYTE;
		xbusMessage[OFFSET_TO_LEN + 1] = (payloadLength >> 8) & 0xff;
		xbusMessage[OFFSET_TO_LEN + 2] = payloadLength & 0xff;
	}
}

/*! \brief Initialize a xbus message with BID, MID and Length
*/
void Xbus_message(uint8_t* xbusMessage, uint8_t bid, uint8_t mid, uint16_t len)
{
	xbusMessage[0] = 0xFA;
	Xbus_setBusId(xbusMessage, bid);
	Xbus_setMessageId(xbusMessage, mid);
	Xbus_setPayloadLength(xbusMessage, len);
}

/*! \brief Returns total length of xbus message (header + payload + checksum)
*/
int Xbus_getRawLength(const uint8_t* xbusMessage)
{
	int rtrn = Xbus_getPayloadLength(xbusMessage);

	if ((xbusMessage[OFFSET_TO_LEN] & 0xff) == LENGTH_EXTENDER_BYTE)
		rtrn += 7;
	else
		rtrn += 5;
	return rtrn;
}

/*! \brief Returns pointer to payload of an xbus message
*/
uint8_t* Xbus_getPointerToPayload(uint8_t* xbusMessage)
{
	if ((xbusMessage[OFFSET_TO_LEN] & 0xff) == LENGTH_EXTENDER_BYTE)
		return xbusMessage + OFFSET_TO_PAYLOAD_EXT;
	else
		return xbusMessage + OFFSET_TO_PAYLOAD;
}

/*! \brief Returns a const pointer to payload of an xbus message
*/
uint8_t const* Xbus_getConstPointerToPayload(uint8_t const* xbusMessage)
{
	return Xbus_getPointerToPayload((uint8_t*)xbusMessage);
}


/*! \brief Inserts the correct checksum in xbus message
*/
void Xbus_insertChecksum(uint8_t* xbusMessage)
{
	int nBytes = Xbus_getRawLength(xbusMessage);

	uint8_t checksum = 0;
	for (int i = 0; i < nBytes - 2; i++)
		checksum -= xbusMessage[1 + i];

	xbusMessage[nBytes - 1] = checksum;
}

/*! \brief Verifies the checksum of aon xbus message
*/
bool Xbus_verifyChecksum(const uint8_t* xbusMessage)
{
	int nBytes =  Xbus_getRawLength(xbusMessage);
	uint8_t checksum = 0;
	for (int n = 1; n < nBytes; n++)
		checksum += (xbusMessage[n] & 0xff);
	checksum &= 0xff;
	return (checksum == 0);
}

bool checkDataReadyLineMain()
{
	return HAL_GPIO_ReadPin(DATA_READY_PORT, DATA_READY_PIN) == GPIO_PIN_SET;
}

size_t Xbus_createRawMessageHelper(uint8_t* dest, uint8_t const* message)
{
	int n;
	uint8_t checksum;
	uint16_t length;
	uint8_t* dptr = dest;

	length = Xbus_getPayloadLength(message);

	*dptr++ = XBUS_CONTROL_PIPE;

	checksum = 0;
	checksum -= XBUS_MASTERDEVICE;

	*dptr = Xbus_getMessageId(message);
	checksum -= *dptr++;

	if (length < XBUS_EXTENDED_LENGTH)
	{
		*dptr = length;
		checksum -= *dptr++;
	}
	else
	{
		*dptr = XBUS_EXTENDED_LENGTH;
		checksum -= *dptr++;
		*dptr = length >> 8;
		checksum -= *dptr++;
		*dptr = length & 0xFF;
		checksum -= *dptr++;
	}

	for (n = 0; n < length; n++)
	{
		*dptr = Xbus_getConstPointerToPayload(message)[n];
		checksum -= *dptr++;
	}

	*dptr++ = checksum;

	return dptr - dest;
}

uint8_t extractUint8(const uint8_t *data, int *index) {
    uint8_t result = data[*index];
    (*index)++;
    return result;
}

/* Extract a 16-bit unsigned integer from data in big-endian order.
   The first byte becomes the high 8 bits and the second byte the low 8 bits. */
uint16_t extractUint16(const uint8_t *data, int *index) {
    uint16_t result = 0;
    result |= ((uint16_t)data[*index]) << 8;
    (*index)++;
    result |= ((uint16_t)data[*index]);
    (*index)++;
    return result;
}

/* Extract a 32-bit unsigned integer from data in big-endian order.
   The first byte is shifted to the most significant position, etc. */
uint32_t extractUint32(const uint8_t *data, int *index) {
    uint32_t result = 0;
    result |= ((uint32_t)data[*index]) << 24;
    (*index)++;
    result |= ((uint32_t)data[*index]) << 16;
    (*index)++;
    result |= ((uint32_t)data[*index]) << 8;
    (*index)++;
    result |= ((uint32_t)data[*index]);
    (*index)++;
    return result;
}

/* Extract a float by first reading 4 bytes as a 32-bit unsigned integer
   and then copying them into a float. */
float extractFloat(const uint8_t *data, int *index) {
    uint32_t temp = extractUint32(data, index);
    float result;
    memcpy(&result, &temp, sizeof(result));
    return result;
}


