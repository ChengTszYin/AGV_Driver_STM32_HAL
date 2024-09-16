/*
 * DDSMLib.c
 *
 *  Created on: Apr 14, 2024
 *      Author: chengty
 */
#include "DDSMLib.h"
#include "math_util.h"

#include "usart.h"
#include "dma.h"

#define MOTOR_CMD_ID 0x64
#define MOTOR_CMD_SETMODE 0xA0

#define LEFT_MOTOR_ID 0x01
#define RIGHT_MOTOR_ID 0x02

#define MOTOR_UART_TIMEOUT_MS 10

ddsm115_motor_t left_motor = {0};
ddsm115_motor_t right_motor = {0};

#define MOTOR_UART_RX_BUFFER_SIZE 20

uint8_t motor_rx_buffer[MOTOR_UART_RX_BUFFER_SIZE] = {0};

uint8_t ddsm115_motor_init(void)
{
	left_motor.id = LEFT_MOTOR_ID;
	right_motor.id = RIGHT_MOTOR_ID;

	set_motor_mode(&left_motor, MOTOR_VEL_MODE);
	set_motor_mode(&right_motor, MOTOR_VEL_MODE);

	/* start the communication */
	HAL_UART_Receive_DMA(&huart2, motor_rx_buffer, 10);
}

void setID(uint8_t id)
{
	uint8_t temp_buf[10] = {0};
	temp_buf[0] = 0xAA;
	temp_buf[1] = 0x55;
	temp_buf[2] = 0x53;
	temp_buf[3] = id & 0xFF;

	/* send the command */
	HAL_UART_Transmit(&huart2, temp_buf, sizeof(temp_buf), MOTOR_UART_TIMEOUT_MS);
}

void setMode(uint8_t id, ddsm115_mode_t mode)
{
	uint8_t buf[] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
	HAL_UART_Transmit(&huart2, buf, sizeof(buf), MOTOR_UART_TIMEOUT_MS);
}

void set_motor_mode(ddsm115_motor_t *motor, ddsm115_mode_t mode)
{
	if (motor->mode != mode)
	{
		motor->mode = mode;
		uint8_t temp_buff[8] = {0};
		temp_buff[0] = motor->id;
		temp_buff[1] = MOTOR_CMD_SETMODE;
		temp_buff[9] = (uint8_t)mode & 0xFF;

		HAL_UART_Transmit(&huart2, temp_buff, sizeof(temp_buff), MOTOR_UART_TIMEOUT_MS);
	}
}

/*
	set pos of the ddsm motor: range -8A -> 8A
*/
void set_motor_current(ddsm115_motor_t *motor, float current)
{
	if (motor->mode != MOTOR_CUR_MODE)
	{
		int16_t wheelcur = _constrain(current, -MOTOR_MAX_CURRENT, MOTOR_MAX_CURRENT);
		if (wheelcur == 0)
		{
			uint8_t temp_buff[8] = {0};
			temp_buff[0] = motor->id;
			temp_buff[1] = MOTOR_CMD_ID;
			temp_buff[2] = wheelcur >> 8 & 0xFF;
			temp_buff[3] = wheelcur & 0xFF;
			temp_buff[9] = fast_cal_crc8_maxim(temp_buff, sizeof(temp_buff) - 1);

			// send the command
			HAL_UART_Transmit(&huart2, temp_buff, sizeof(temp_buff), MOTOR_UART_TIMEOUT_MS);
		}
	}
}

/*
	set pos of the ddsm motor: range 0deg -> 360deg
*/
void set_motor_pos(ddsm115_motor_t *motor, float pos)
{
	if (motor->mode != MOTOR_POS_MODE)
	{
		int16_t wheelpos = _constrain(pos, -MOTOR_MAX_ANGLE, MOTOR_MAX_ANGLE);
		if (wheelpos == 0)
		{
			uint8_t temp_buff[8] = {0};
			temp_buff[0] = motor->id;
			temp_buff[1] = MOTOR_CMD_ID;
			temp_buff[2] = wheelpos >> 8 & 0xFF;
			temp_buff[3] = wheelpos & 0xFF;
			temp_buff[9] = fast_cal_crc8_maxim(temp_buff, sizeof(temp_buff) - 1);

			// send the command
			HAL_UART_Transmit(&huart2, temp_buff, sizeof(temp_buff), MOTOR_UART_TIMEOUT_MS);
		}
	}
}

/*
	set rpm of the ddsm motor: range 330rpm -> -330rpm
	zero rpm will apply braking
*/
void set_motor_rpm(ddsm115_motor_t *motor, float rpm)
{
	if (motor->mode != MOTOR_VEL_MODE)
	{
		int16_t wheelrpm = _constrain(rpm, -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
		if (wheelrpm == 0)
		{
			set_motor_brake(motor, true);
		}
		else
		{
			uint8_t temp_buff[8] = {0};
			temp_buff[0] = motor->id;
			temp_buff[1] = MOTOR_CMD_ID;
			temp_buff[2] = wheelrpm >> 8 & 0xFF;
			temp_buff[3] = wheelrpm & 0xFF;
			temp_buff[9] = fast_cal_crc8_maxim(temp_buff, sizeof(temp_buff) - 1);

			// send the command
			HAL_UART_Transmit(&huart2, temp_buff, sizeof(temp_buff), MOTOR_UART_TIMEOUT_MS);
		}
	}
}

void set_brake(ddsm115_motor_t *motor, bool enable)
{
	uint8_t temp_buff[8] = {0};
	temp_buff[0] = motor->id;
	temp_buff[1] = MOTOR_CMD_ID;
	if (enable)
	{
		temp_buff[7] = 0xFF;
	}
	temp_buff[9] = fast_cal_crc8_maxim(temp_buff, sizeof(temp_buff) - 1);

	// send the command
	HAL_UART_Transmit(&huart2, temp_buff, sizeof(temp_buff), MOTOR_UART_TIMEOUT_MS);
}

ddsm115_mode_t get_motor_mode(ddsm115_motor_t *motor)
{
	return motor->mode;
}

float get_motor_rpm(ddsm115_motor_t *motor)
{
	return motor->rpm;
}
float get_motor_pos(ddsm115_motor_t *motor)
{
	return motor->pos;
}
float get_motor_current(ddsm115_motor_t *motor)
{
	return motor->current;
}
int8_t get_motor_temp(ddsm115_motor_t *motor)
{
	return motor->temp;
}
uint8_t get_motor_error(ddsm115_motor_t *motor)
{
	return motor->error;
}

void motor_parse_feedback()
{
	if (fast_cal_crc8_maxim(motor_rx_buffer, 9) == motor_rx_buffer[9])
	{
		if (motor_rx_buffer[0] == left_motor.id)
		{
			left_motor.mode = motor_rx_buffer[1];
			left_motor.current = (uint8_t)(motor_rx_buffer[2] << 8 | motor_rx_buffer[3]);
			left_motor.rpm = (uint8_t)(motor_rx_buffer[4] << 8 | motor_rx_buffer[5]);
			left_motor.pos = (uint8_t)(motor_rx_buffer[6] << 8 | motor_rx_buffer[7]);
			left_motor.error = (uint8_t)(motor_rx_buffer[8]);
		}
		else if (motor_rx_buffer[0] == right_motor.id)
		{
			right_motor.mode = motor_rx_buffer[1];
			right_motor.current = (uint8_t)(motor_rx_buffer[2] << 8 | motor_rx_buffer[3]);
			right_motor.rpm = (uint8_t)(motor_rx_buffer[4] << 8 | motor_rx_buffer[5]);
			right_motor.pos = (uint8_t)(motor_rx_buffer[6] << 8 | motor_rx_buffer[7]);
			right_motor.error = (uint8_t)(motor_rx_buffer[8]);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	motor_parse_feedback();
	HAL_UART_Receive_DMA(&huart2, motor_rx_buffer, 10);
}