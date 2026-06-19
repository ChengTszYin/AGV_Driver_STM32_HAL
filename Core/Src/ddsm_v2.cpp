/*
 * ddsm_v2.cpp
 *
 *  Created on: May 18, 2026
 *      Author: user
 */

#include <ddsm_v2.hpp>

DDSM115::DDSM115(uint8_t _id)
{
	id = _id;
}

void DDSM115::sort(short* array, int size)
{
	for(int i = 0; i < size - 1; i++)
	{
		for(int j = i + 1; j < size; j++)
		{
			if(array[i] > array[j])
			{
				short temp = array[i];
				array[i] = array[j];
				array[j] = temp;
			}
		}
	}
}

short DDSM115::mid_filter(short velocity, short* history, short *index)
{
	history[*index] = velocity;
	*index = (*index + 1) % FILTER_SIZE;			//ensure index will not exceed FILTER_SIZE

	short temp[FILTER_SIZE];
	for(int i=0; i < FILTER_SIZE; i++)
	{
		temp[i] = history[i];
	}
	sort(temp, FILTER_SIZE);
	return temp[FILTER_SIZE / 2];
}

uint8_t DDSM115::checkCRC(uint8_t *Buffer)
{
	if (crc8(Buffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT) == Buffer[9]){
			return 1;
	}
	else if(Buffer[9]==0x00){
			return 0;
		}
	else return 0;
}

void DDSM115::send(uint8_t crc)
{
	if(crc)
	{
		commandBuffer[9] = crc8(commandBuffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT);
	}
	HAL_UART_Transmit(&huart2, commandBuffer, sizeof(commandBuffer),10);
}

void DDSM115::setID(uint8_t id)
{
	uint8_t buf[] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
	HAL_UART_Transmit(&huart2, buf, sizeof(buf),10);
}

uint8_t DDSM115::setVelocity(int16_t velocity, uint8_t acceleration)
{
	if(velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
	if(velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
	uint16_t velocityRecalc = abs(velocity);
	if(velocity < 0 && velocity != 0) velocityRecalc = 0xFFFF - velocityRecalc + 1;
	uint8_t velocityHighByte = (uint8_t)(velocityRecalc >> 8) & 0xFF;
	uint8_t velocityLowByte = (uint8_t) (velocityRecalc) & 0xFF;
	uint8_t buf[] = {id, 0x64, velocityHighByte, velocityLowByte, 0, 0, acceleration, 0, 0, 0};
	for(int i = 0; i < 10; i++)
	{
		commandBuffer[i] = buf[i];
	}
	send(1);
	//receive();
	//parse(DDSM115_PROTOCOL_V2);
	return 0;
}


void DDSM115::parse_buffer_DMA(uint8_t* responseBuffer, uint8_t connected)
{
	if(!connected)
	{
		if(responseBuffer && responseBuffer[0] == id)
		{
			id = responseBuffer[0];
			mode = (ddsm115_mode_t)responseBuffer[1];
			uint16_t current = (uint16_t)(responseBuffer[2]) << 8 | (uint16_t)(responseBuffer[3]);
			short currentR = current;
			if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
			if (currentR >= 0)
			{
				current = (float)currentR * (float)MAX_CURRENT / 32767.0;
			} else {
				current = (float)currentR * (float)MIN_CURRENT / -32767.0;
			}
			uint16_t _velocity = (uint16_t)(responseBuffer[4] << 8 | (uint16_t)(responseBuffer[5]));
			if (_velocity > MAX_VELOCITY){ _velocity -= 0xFFFF; _velocity--; }
			velocity = mid_filter(_velocity, VelocityHistory, &_index);
			winding_temp = responseBuffer[6];
			angle = round((float)responseBuffer[7] * (float)MAX_ANGLE / 255.0);
			error = responseBuffer[8];
		}
	}

	else
	{
		velocity = 0;
		winding_temp = 0;
		angle = 0;
		error = 0;
	}
}
