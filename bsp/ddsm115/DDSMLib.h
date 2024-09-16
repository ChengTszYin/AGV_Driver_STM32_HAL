/*
 * DDSMLib.h
 *
 *  Created on: Apr 14, 2024
 *      Author: chengty
 */

#ifndef INC_DDSMLIB_H_
#define INC_DDSMLIB_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//  motor profile
#define MOTOR_MAX_CURRENT 8
#define MOTOR_MAX_RPM 330
#define MOTOR_MAX_ANGLE 360

typedef enum
{
	MOTOR_CUR_MODE = 1,
	MOTOR_VEL_MODE,
	MOTOR_POS_MODE,
} ddsm115_mode_t;

typedef struct
{
	uint8_t id;
	ddsm115_mode_t mode;
	int16_t current;
	int16_t rpm;
	uint16_t pos;
	int8_t temp;
	uint8_t error;
} ddsm115_motor_t;

extern ddsm115_motor_t left_motor;
extern ddsm115_motor_t right_motor;

uint8_t ddsm115_motor_init(void);

void set_motor_mode(ddsm115_motor_t *motor, ddsm115_mode_t mode);
void set_motor_current(ddsm115_motor_t *motor, float current);
void set_motor_rpm(ddsm115_motor_t *motor, float rpm);
void set_motor_pos(ddsm115_motor_t *motor, float pos);
void set_brake(ddsm115_motor_t *motor, bool enable);

ddsm115_mode_t get_motor_mode();
float get_motor_rpm(ddsm115_motor_t *motor);
float get_motor_pos(ddsm115_motor_t *motor);
float get_motor_current(ddsm115_motor_t *motor);
int8_t get_motor_temp(ddsm115_motor_t *motor);
uint8_t get_motor_error(ddsm115_motor_t *motor);

#endif /* INC_DDSMLIB_H_ */