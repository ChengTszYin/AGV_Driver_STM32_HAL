/*
 * DDSM115.h
 *
 *  Created on: Mar 21, 2024
 *      Author: chengty
 */

#ifndef INC_DDSM115_H_
#define INC_DDSM115_H_

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


typedef enum{
  CURRENT_LOOP = 1,
  VELOCITY_LOOP = 2,
  POSITION_LOOP = 3,
} ddsm115_mode_t;

typedef enum{
	DDSM115_PROTOCOL_V1 = 1,
	DDSM115_PROTOCOL_V2 = 2,
}ddsm115_protocol_t;

typedef enum{
	DDSM115_TROUBLESHOOTING = 0x10,
	DDSM115_STALL_ERROR = 0x08,
	DDSM115_PHASE_OVERCURRENT_ERROR = 0x04,
	DDSM115_OVERCURRENT_ERROR = 0x02,
	DDSM115_SENSOR_ERROR = 0x01,
}ddsm115_error_t;

struct Response{
	uint8_t id;
	ddsm115_mode_t mode;
	float current;
	int16_t velocity;
	int16_t angle;
	uint8_t winding_temp;
	int16_t position;
	uint8_t error;
};

#endif /* INC_DDSM115_H_ */
