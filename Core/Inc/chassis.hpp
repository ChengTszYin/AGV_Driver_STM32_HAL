/*
 * chassis.h
 *
 *  Created on: May 17, 2026
 *      Author: user
 */

#ifndef INC_CHASSIS_HPP_
#define INC_CHASSIS_HPP_

#define NUM_WHEEL 6
#define NUM_PROX 4
#define MOTOR_BYTE_SIZE 10
#define receiveByteSize 20	//cmd byte size from host computer

#include "usart.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <ddsm_v2.hpp>
#include "gy95t.h"
#include "SR04.h"
#include <stdint.h>
#include <string.h>

class Chassis
{
	public:
		Chassis(uint8_t* _id, int num_motor);
		void HostMessageParse(uint8_t *receiveBytes);
		void MotorMessageSend();
		uint8_t checksum(uint8_t *data, uint8_t len);
		uint8_t m_checkCRC(uint8_t* bufferByte);
		void setMotors();
		void run_sensors();
		void run_imu();
		void d80nk_read();
		void writeBuffer(uint8_t id, uint8_t* responseBuffer);
		const short* get_C_Velocity() const;	//DEBUG FUNC FROM COMPUTER SIDE
		short m_velocty[NUM_WHEEL];
		gy my_95Q;
	private:
		uint8_t m_id[NUM_WHEEL];
		uint8_t m_responseBuffer[10];
		DDSM115* motor[NUM_WHEEL];
		short c_velocty[NUM_WHEEL];
		uint8_t d80nk_[4];
		uint16_t distance1 = 0;
		uint16_t distance2 = 0;
};

#endif /* INC_CHASSIS_HPP_ */
