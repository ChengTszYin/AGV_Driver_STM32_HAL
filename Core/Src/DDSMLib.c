/*
 * DDSMLib.c
 *
 *  Created on: Apr 14, 2024
 *      Author: chengty
 */
#include "DDSMLib.h"
#include "CRC.h"
#include "usart.h"


uint8_t CRC8_MAXIM_REFIN = 1;
uint8_t CRC8_MAXIM_REFOUT = 1;

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
short velocityHR;
short velocityHL;
short velocityLR;
short velocityLL;

uint8_t responseBuffer[45];
uint8_t responseBufferHL[10];
uint8_t responseBufferHR[10];
uint8_t responseBufferLL[10];
uint8_t responseBufferLR[10];
struct motor_sensor_t wheelsensor;
uint8_t commandBuffer[10];

short HLeftVelocityHistory[FILTER_SIZE] = {0};
short HRightVelocityHistory[FILTER_SIZE] = {0};
short LLeftVelocityHistory[FILTER_SIZE] = {0};
short LRightVelocityHistory[FILTER_SIZE] = {0};
int HLeftIndex = 0;
int HRightIndex = 0;
int LLeftIndex = 0;
int LRightIndex = 0;

void sort(short* array, int size)
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

short mid_filter(short velocity, short* history, short *index)
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


uint8_t checkCRC(uint8_t *Buffer)
{
	if (crc8(Buffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT) == Buffer[9]){
			return 1;
	}
	else if(Buffer[9]==0x00){
			return 0;
		}
	else return 0;
}

void receiveFromBuffer()
{
	HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	Parse_DMA_All(&wheelsensor);
//	HAL_Delay(500);
}

void send(uint8_t crc)
{
	if(crc)
	{
		commandBuffer[9] = crc8(commandBuffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT);
	}
	HAL_UART_Transmit(&huart2, commandBuffer, sizeof(commandBuffer),10);
}

void setID(uint8_t id)
{
	uint8_t buf[] = {0xAA, 0x55, 0x53, id, 0, 0, 0, 0, 0, 0};
	HAL_UART_Transmit(&huart2, buf, sizeof(buf),10);
}

void setMode(uint8_t id, ddsm115_mode_t mode){
	uint8_t buf[] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
	HAL_UART_Transmit(&huart2, buf, sizeof(buf),10);
}


void Parse_DMA_All(struct motor_sensor_t* sensor, uint8_t connected)
{
	if (!connected)
	{
		if(sizeof(responseBufferHL)>0)
			{
				sensor->Hleftii = responseBufferHL[0];
				sensor->HleftMode = (ddsm115_mode_t)responseBufferHL[1];
				uint16_t current = (uint16_t)(responseBufferHL[2]) << 8 | (uint16_t)(responseBufferHL[3]);
				short currentR = current;
				if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
				if (currentR >= 0) {
					sensor->HleftCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
				} else {
					sensor->HleftCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
				}
				uint16_t velocity = (uint16_t)(responseBufferHL[4] << 8 | (uint16_t)(responseBufferHL[5]));
				velocityHL = velocity;
				if (velocityHL  > MAX_VELOCITY){ velocityHL -= 0xFFFF; velocityHL--; }
				short filteredHLeftVelocity = mid_filter(velocityHL, HLeftVelocityHistory, &HLeftIndex);
				sensor->HLeftVelocity = filteredHLeftVelocity;
				sensor->HLeftwinding_temp = responseBufferHL[6];
				sensor->HLeftangle = round((float)responseBufferHL[7] * (float)MAX_ANGLE / 255.0);
				sensor->HLefterror = responseBufferHL[8];
			}
		if(sizeof(responseBufferHR)>0)
		{
			sensor->Hrightii = responseBufferHR[0];
			sensor->HrightMode = (ddsm115_mode_t)responseBufferHR[1];
			uint16_t current = (uint16_t)(responseBufferHR[2]) << 8 | (uint16_t)(responseBufferHR[3]);
			short currentR = current;
			if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
			if (currentR >= 0) {
				sensor->HrightCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
			} else {
				sensor->HrightCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
			}
			uint16_t velocity = (uint16_t)(responseBufferHR[4] << 8 | (uint16_t)(responseBufferHR[5]));
			velocityHR = velocity;
			if (velocityHR  > MAX_VELOCITY){ velocityHR -= 0xFFFF; velocityHR--; }
			short filteredHRightVelocity = mid_filter(velocityHR, HRightVelocityHistory, &HRightIndex);
			sensor->HRightVelocity = filteredHRightVelocity;
			sensor->HRightwinding_temp = responseBufferHR[6];
			sensor->HRightangle = round((float)responseBufferHR[7] * (float)MAX_ANGLE / 255.0);
			sensor->HRighterror = responseBufferHR[8];
		}

		if(sizeof(responseBufferLL)>0)
		{
			sensor->Lleftii = responseBufferHL[0];
			sensor->LleftMode = (ddsm115_mode_t)responseBufferLL[1];
			uint16_t current = (uint16_t)(responseBufferLL[2]) << 8 | (uint16_t)(responseBufferLL[3]);
			short currentR = current;
			if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
			if (currentR >= 0) {
				sensor->LleftCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
			} else {
				sensor->LleftCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
			}
			uint16_t velocity = (uint16_t)(responseBufferLL[4] << 8 | (uint16_t)(responseBufferLL[5]));
			velocityLL = velocity;
			if (velocityLL  > MAX_VELOCITY){ velocityLL -= 0xFFFF; velocityLL--; }
			short filteredLLeftVelocity = mid_filter(velocityLL, LLeftVelocityHistory, &LLeftIndex);
			sensor->LLeftVelocity = filteredLLeftVelocity;
			sensor->LLeftwinding_temp = responseBufferLL[6];
			sensor->LLeftangle = round((float)responseBufferLL[7] * (float)MAX_ANGLE / 255.0);
			sensor->LLefterror = responseBufferLL[8];
		}
		if(sizeof(responseBufferLR)>0)
		{
			sensor->Lrightii = responseBufferLR[0];
			sensor->LrightMode = (ddsm115_mode_t)responseBufferLR[1];
			uint16_t current = (uint16_t)(responseBufferLR[2]) << 8 | (uint16_t)(responseBufferLR[3]);
			short currentR = current;
			if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
			if (currentR >= 0) {
				sensor->LrightCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
			} else {
				sensor->LrightCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
			}
			uint16_t velocity = (uint16_t)(responseBufferLR[4] << 8 | (uint16_t)(responseBufferLR[5]));
			velocityLR = velocity;
			if (velocityLR  > MAX_VELOCITY){ velocityLR -= 0xFFFF; velocityLR--; }
			short filteredLRightVelocity = mid_filter(velocityLR, LRightVelocityHistory, &LRightIndex);
			sensor->LRightVelocity = filteredLRightVelocity;
			sensor->LRightwinding_temp = responseBufferLR[6];
			sensor->LRightangle = round((float)responseBufferLR[7] * (float)MAX_ANGLE / 255.0);
			sensor->LRighterror = responseBufferLR[8];
		}
	}
	else
	{
		sensor->HLeftVelocity = 0;
		sensor->HRightVelocity = 0;
		sensor->LLeftVelocity = 0;
		sensor->LRightVelocity = 0;
	}


}

uint8_t setVelocity(uint8_t id, int16_t velocity, uint8_t acceleration)
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

