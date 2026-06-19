/*
 * chassis.cpp
 *
 *  Created on: May 17, 2026
 *      Author: user
 */

#include <chassis.hpp>

Chassis::Chassis(uint8_t* _id, int num_motor)
{
	for(uint8_t i = 0; i < num_motor; ++i)
	{
		m_id[i] = 0x01 + i;
		c_velocty[i] = 0;
		m_velocty[i] = 0;
	}
	for(uint8_t j=0; j < num_motor; ++j)
	{
		motor[j] = new DDSM115(m_id[j]);
		LOG("Created motor[%d] = %p\r\n", j, motor[j]);
	}
};

uint8_t Chassis::checksum(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len - 1; i++)
	{
		crc += data[i];
	}
	return crc;
};

uint8_t Chassis::m_checkCRC(uint8_t* bufferByte)
{
	uint8_t _data[MOTOR_BYTE_SIZE];
	memcpy(_data, bufferByte, MOTOR_BYTE_SIZE);
	uint8_t check;
	for(uint8_t i = 0; i < NUM_WHEEL; i++)
	{
		if(_data[0] == m_id[i])
		{
			return motor[i]->checkCRC(_data);
		}

	}
	return 0;
}

void Chassis::HostMessageParse(uint8_t *receiveBytes)
{
    uint8_t data[receiveByteSize];
    memcpy(data, receiveBytes, receiveByteSize);

    // Correct CRC check: compute sum of first 19 bytes and compare with byte 19
    uint8_t computed_crc = 0;
    for (uint8_t i = 0; i < receiveByteSize - 1; i++) {
        computed_crc += data[i];
    }

    if (data[0] == 0x00 && computed_crc == data[19])   // valid header + CRC
    {
        m_id[0] = data[1];
        c_velocty[0] = (data[2] << 8) | data[3];
        m_id[1] = data[4];
        c_velocty[1] = (data[5] << 8) | data[6];
        m_id[2] = data[7];
        c_velocty[2] = (data[8] << 8) | data[9];
        m_id[3] = data[10];
        c_velocty[3] = (data[11] << 8) | data[12];
        m_id[4] = data[13];
        c_velocty[4] = (data[14] << 8) | data[15];
        m_id[5] = data[16];
        c_velocty[5] = (data[17] << 8) | data[18];

    }
    else
    {
        memset(c_velocty, 0, sizeof(c_velocty));
    }

//    LOG("Cmd Vel: %d %d %d %d %d %d \n", c_velocty[0], c_velocty[1], c_velocty[2], c_velocty[3], c_velocty[4], c_velocty[5]);

    memset(receiveBytes, 0, sizeof(receiveBytes));
}

void Chassis::MotorMessageSend()
{
	uint32_t tick_delay = pdMS_TO_TICKS(100);
	uint8_t sendData[48] = {0};
	sendData[0] = 0x00;
	sendData[1] = m_id[0] ;
	sendData[2] = (m_velocty[0] >> 8) & 0xFF;
	sendData[3] = m_velocty[0] & 0xFF;
	sendData[4] = m_id[1];
	sendData[5] = (m_velocty[1] >> 8) & 0xFF;
	sendData[6] = m_velocty[1] & 0xFF;
	sendData[7] = m_id[2];
	sendData[8] = (m_velocty[2] >> 8) & 0xFF;
	sendData[9] = m_velocty[2] & 0xFF;
	sendData[10] = m_id[3];
	sendData[11] = (m_velocty[3] >> 8) & 0xFF;
	sendData[12] = m_velocty[3] & 0xFF;
	sendData[13] = m_id[4];
	sendData[14] = (m_velocty[4] >> 8) & 0xFF;
	sendData[15] = m_velocty[4] & 0xFF;
	sendData[16] = m_id[5];
	sendData[17] = (m_velocty[5] >> 8) & 0xFF;
	sendData[18] = m_velocty[5] & 0xFF;
	sendData[19] = (my_95Q.Acc_x >> 8) & 0xFF;
	sendData[20] = my_95Q.Acc_x & 0XFF;
	sendData[21] = (my_95Q.Acc_y >> 8) & 0XFF;
	sendData[22] = my_95Q.Acc_y & 0xFF;
	sendData[23] = (my_95Q.Acc_z >> 8) & 0xFF;
	sendData[24] = my_95Q.Acc_z & 0xFF;
	sendData[25] = (my_95Q.Gyro_x >> 8) & 0XFF;
	sendData[26] = my_95Q.Gyro_x & 0xFF;
	sendData[27] = (my_95Q.Gyro_y >> 8) & 0XFF;
	sendData[28] = my_95Q.Gyro_y & 0xFF;
	sendData[29] = (my_95Q.Gyro_z >> 8) & 0XFF;
	sendData[30] = my_95Q.Gyro_z >> 8 & 0xFF;
	sendData[31] = (my_95Q.Q0 >> 8) & 0xFF;
	sendData[32] = my_95Q.Q0 & 0xFF;
	sendData[33] = (my_95Q.Q1 >> 8) & 0xFF;
	sendData[34] = my_95Q.Q1 & 0xFF;
	sendData[35] = (my_95Q.Q2 >> 8) & 0xFF;
	sendData[36] = my_95Q.Q2 & 0xFF;
	sendData[37] = (my_95Q.Q3 >> 8) & 0xFF;
	sendData[38] = my_95Q.Q3 & 0xFF;
	sendData[39] = (distance1 >> 8) & 0xFF;
	sendData[40] = (distance1) & 0xFF;
	sendData[41] = (distance2 >> 8) & 0xFF;
	sendData[42] = (distance2) & 0xFF;
	sendData[43] = d80nk_[0] & 0xFF;
	sendData[44] = d80nk_[1] & 0xFF;
	sendData[45] = d80nk_[2] & 0xFF;
	sendData[46] = d80nk_[3] & 0xFF;
	sendData[47] = checksum(sendData, 48);
	HAL_UART_Transmit(&huart1, sendData, 48, HAL_MAX_DELAY);
	vTaskDelay(tick_delay);
};

void Chassis::setMotors()
{
	uint32_t L_R_delay = pdMS_TO_TICKS(4);
	motor[0]->setVelocity(c_velocty[0], 0);
	vTaskDelay(L_R_delay * 5);
	motor[1]->setVelocity(c_velocty[1], 0);
	vTaskDelay(L_R_delay * 4);
	motor[2]->setVelocity(c_velocty[2], 0);
	vTaskDelay(L_R_delay * 3);
	motor[3]->setVelocity(c_velocty[3], 0);
	vTaskDelay(L_R_delay * 2);
	motor[4]->setVelocity(c_velocty[4], 0);
	vTaskDelay(L_R_delay * 1);
	motor[5]->setVelocity(c_velocty[5], 0);

//	HAL_UART_Receive_DMA(&huart2, responseBuffer, 65);
//	HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
};


void Chassis::writeBuffer(uint8_t id, uint8_t* responseBuffer)
{
    if (id >= NUM_WHEEL || motor[id] == nullptr || responseBuffer == nullptr) {
         LOG("ERROR: bad id %d\n", id);   // you can keep for debug
        return;
    }

    uint8_t buffers[10];
    memcpy(buffers, responseBuffer, 10);

    motor[id]->parse_buffer_DMA(buffers, 0);
    int16_t vel = motor[id]->velocity;
    if (vel > 32767) {
		vel = vel - 65536;
	}
    m_velocty[id] = vel;

//    LOG("m_velocty[%d]: %d\n", id, m_velocty[id]);
}

const short* Chassis::get_C_Velocity() const
{
    return c_velocty;
}

void Chassis::d80nk_read()
{
	GPIO_PinState pinStates[NUM_PROX];
	pinStates[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	pinStates[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	pinStates[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	pinStates[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	for (int i = 0; i < 4; i++)
	{
		if (pinStates[i] == GPIO_PIN_SET)
		{
			d80nk_[i] = '0';
		}
		else
		{
			d80nk_[i] = '1';
			//			sprintf(message,"Sensor ON\n");
			//			HAL_UART_Transmit(&huart3, message, sizeof(message), HAL_MAX_DELAY);
		}
	}
};

void Chassis::run_sensors()
{
	uint32_t send_delay = pdMS_TO_TICKS(100);
	distance1 = HCSR04_Read_l();
	distance2 = HCSR04_Read_r();
	d80nk_read();
	vTaskDelay(send_delay);
};

void Chassis::run_imu()
{
	uint32_t tick_delay = pdMS_TO_TICKS(200);
	uint8_t inited = 0;
	do {
		inited = gy95_Init(0);
		LOG("IMU not inited\n");
		vTaskDelay(tick_delay);
	} while (inited != 1);
	while(1)
	{
		LOG("IMU is inited\n");
		gy95_All(&my_95Q);
		vTaskDelay(tick_delay);
	}
};
