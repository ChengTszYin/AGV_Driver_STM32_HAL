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
short velocity[MAX_MOTORS];

uint8_t responseBuffer[65];
uint8_t responseBuffer_module[MAX_MOTORS][10];

uint8_t commandBuffer[10];

short velocityHistory[MAX_MOTORS][FILTER_SIZE] = {0};
int   velocityIndex[MAX_MOTORS] = {0};


uint8_t motor_count = 0;

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


void Parse_DMA_All(motor_sensor_t* motors, uint8_t connected)
{
	if (!connected)
	{
		for(int i = 0; i < MAX_MOTORS; ++i)
		{
			if(sizeof(responseBuffer_module[i]) > 0)
			{
				motors[i].id = responseBuffer_module[i][0];
				motors[i].mode = responseBuffer_module[i][1];
				uint16_t current = (uint16_t)(responseBuffer_module[i][2]) << 8 | (uint16_t)(responseBuffer_module[i][3]);
				short currentR = current;
				if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
				if (currentR >= 0) {
					motors[i].current = (float)currentR * (float)MAX_CURRENT / 32767.0;
				} else {
					motors[i].current = (float)currentR * (float)MIN_CURRENT / -32767.0;
				}
				uint16_t _velocity = (uint16_t)(responseBuffer_module[i][4] << 8 | (uint16_t)(responseBuffer_module[i][5]));
				velocity[i] = _velocity;
				if (velocity[i]  > MAX_VELOCITY){ velocity[i] -= 0xFFFF; velocity[i]--; }
				short filteredVelocity = mid_filter(velocity[i], velocityHistory[i], &velocityIndex[i]);
				motors[i].velocity = filteredVelocity;
				motors[i].winding_temp = responseBuffer_module[i][6];
				motors[i].angle = round((float)responseBuffer_module[i][7] * (float)MAX_ANGLE / 255.0);
				motors[i].error = responseBuffer_module[i][8];
			}
		}
	}
	else
	{
		for(int i = 0; i < MAX_MOTORS; ++i)
		{
			motors[i].velocity = 0;
		}
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

