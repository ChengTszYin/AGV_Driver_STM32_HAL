/*
 * ddsm_v2.hpp
 *
 *  Created on: May 18, 2026
 *      Author: user
 */

#ifndef INC_DDSM_V2_HPP_
#define INC_DDSM_V2_HPP_

#include "CRC.h"
#include <stdint.h>
#include <math.h>
#include "usart.h"

#define RxBuffer_Size 20
#define TxBuffer_Size 20

#define MAX_CURRENT 8
#define MIN_CURRENT -8
#define MAX_VELOCITY 330
#define MIN_VELOCITY -330
#define MAX_ANGLE 360
#define MIN_ANGLE -360

#define CRC8_MAXIM_REFIN 1
#define CRC8_MAXIM_REFOUT 1

#define CRC8_MAXIM_POLY 0x31
#define CRC8_MAXIM_INIT 0X00
#define CRC8_MAXIM_XOROUT 0x00
#define FILTER_SIZE 7

typedef enum{
  CURRENT_LOOP = 1,
  VELOCITY_LOOP = 2,
  POSITION_LOOP = 3,
} ddsm115_mode_t;

typedef enum{
	DDSM115_PROTOCOL_V1 = 1,
	DDSM115_PROTOCOL_V2 = 2,
}ddsm115_protocol_t;

class DDSM115
{
	private:
		uint8_t id;
		ddsm115_mode_t mode;
		uint8_t m_buffer[10];
		short current;
		short VelocityHistory[FILTER_SIZE] = {0};
		short _index = 0;
	public:
		DDSM115(uint8_t _id);
		void sort(short* array, int size);
		short mid_filter(short velocity, short* history, short *index);
		uint8_t checkCRC(uint8_t *Buffer);
		void send(uint8_t crc);
		void setID(uint8_t id);
		uint8_t setVelocity(int16_t velocity, uint8_t acceleration);
		void parse_buffer_DMA(uint8_t* responseBuffer, uint8_t connected);
		int16_t velocity;
		uint8_t winding_temp;
		int16_t angle;
		uint8_t error;
		uint8_t commandBuffer[10];
};



#endif /* INC_DDSM_V2_HPP_ */
