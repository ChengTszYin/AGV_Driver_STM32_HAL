/*
 * DDSMLib.h
 *
 *  Created on: Apr 14, 2024
 *      Author: chengty
 */

#ifndef INC_DDSMLIB_H_
#define INC_DDSMLIB_H_

#include "main.h"

#define RxBuffer_Size 20
#define TxBuffer_Size 20

#define MAX_CURRENT 8
#define MIN_CURRENT -8
#define MAX_VELOCITY 330
#define MIN_VELOCITY -330
#define MAX_ANGLE 360
#define MIN_ANGLE -360

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

struct motor_sensor_t{
	uint8_t Hleftii;
	ddsm115_mode_t HleftMode;
	float HleftCurrent;
	int16_t HLeftVelocity;
	uint8_t HLeftwinding_temp;
	int16_t HLeftangle;
	uint8_t HLefterror;

	uint8_t Hrightii;
	ddsm115_mode_t HrightMode;
	float HrightCurrent;
	int16_t HRightVelocity;
	uint8_t HRightwinding_temp;
	int16_t HRightangle;
	uint8_t HRighterror;

	uint8_t Lleftii;
	ddsm115_mode_t LleftMode;
	float LleftCurrent;
	int16_t LLeftVelocity;
	uint8_t LLeftwinding_temp;
	int16_t LLeftangle;
	uint8_t LLefterror;

	uint8_t Lrightii;
	ddsm115_mode_t LrightMode;
	float LrightCurrent;
	int16_t LRightVelocity;
	uint8_t LRightwinding_temp;
	int16_t LRightangle;
	uint8_t LRighterror;
};


#endif /* INC_DDSMLIB_H_ */
