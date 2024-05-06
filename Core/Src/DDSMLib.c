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
short velocityR;
short velocityL;

uint8_t responseBuffer[25];
uint8_t responseBufferH[10];
uint8_t responseBufferL[10];
struct motor_sensor_t wheelsensor;
uint8_t commandBuffer[10];


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

void Parse_DMA_All(struct motor_sensor_t* sensor)
{
	if(sizeof(responseBufferH)>0)
	{
		sensor->leftii = responseBufferH[0];
		sensor->leftMode = (ddsm115_mode_t)responseBufferH[1];
		uint16_t current = (uint16_t)(responseBufferH[2]) << 8 | (uint16_t)(responseBufferH[3]);
		short currentR = current;
		if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
		if (currentR >= 0) {
			sensor->leftCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
		} else {
			sensor->leftCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
		}
		uint16_t velocity = (uint16_t)(responseBufferH[4] << 8 | (uint16_t)(responseBufferH[5]));
		velocityL = velocity;
		if (velocityL  > MAX_VELOCITY){ velocityL -= 0xFFFF; velocityL--; }
		sensor->LeftVelocity = velocityL;
		sensor->Leftwinding_temp = responseBufferH[6];
		sensor->Leftangle = round((float)responseBufferH[7] * (float)MAX_ANGLE / 255.0);
		sensor->Righterror = responseBufferH[8];
//		uint8_t mess[20];
//		sprintf(mess, "Left sensor: %d\n",sensor->LeftVelocity);
//		HAL_UART_Transmit(&huart1,mess,sizeof(mess),HAL_MAX_DELAY);
	}
	if(sizeof(responseBufferL)>0)
	{
		sensor->reightii = responseBufferL[0];
		sensor->rightMode = (ddsm115_mode_t)responseBufferL[1];
		uint16_t current = (uint16_t)(responseBufferL[2]) << 8 | (uint16_t)(responseBufferL[3]);
		short currentR = current;
		if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
		if (currentR >= 0) {
			sensor->rightCurrent = (float)currentR * (float)MAX_CURRENT / 32767.0;
		} else {
			sensor->rightCurrent = (float)currentR * (float)MIN_CURRENT / -32767.0;
		}
		uint16_t velocity = (uint16_t)(responseBufferL[4] << 8 | (uint16_t)(responseBufferL[5]));
		velocityR = velocity;
		if (velocityR  > MAX_VELOCITY){ velocityR -= 0xFFFF; velocityR--; }
		sensor->RightVelocity = velocityR;
		sensor->Rightwinding_temp = responseBufferL[6];
		sensor->Rightangle = round((float)responseBufferL[7] * (float)MAX_ANGLE / 255.0);
		sensor->Righterror = responseBufferL[8];
//		uint8_t mess[20];
//		sprintf(mess, "RIGHT sensor: %d\n",sensor->RightVelocity);
//		HAL_UART_Transmit(&huart1,mess,sizeof(mess),HAL_MAX_DELAY);
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

