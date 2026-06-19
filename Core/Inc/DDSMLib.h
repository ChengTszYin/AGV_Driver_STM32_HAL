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
#define MAX_MOTORS 6

#define FILTER_SIZE 7

typedef enum{
  CURRENT_LOOP = 1,
  VELOCITY_LOOP = 2,
  POSITION_LOOP = 3,
} ddsm115_mode_t;

typedef enum{
	DDSM115_PROTOCOL_V1 = 1,
	DDSM115_PROTOCOL_V2 = 2,
} ddsm115_protocol_t;

typedef struct {
	uint8_t  id;
	ddsm115_mode_t mode;
	float    current;
	int16_t  velocity;
	uint8_t  winding_temp;
	int16_t  angle;
	uint8_t  error;
} motor_sensor_t;

extern motor_sensor_t wheelsensor[MAX_MOTORS];

extern uint8_t motor_count;

#endif /* INC_DDSMLIB_H_ */
