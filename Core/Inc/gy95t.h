/*
 * gh95t.h
 *
 *  Created on: Oct 16, 2024
 *      Author: chengty
 */

#ifndef INC_GY95T_H_
#define INC_GY95T_H_

// #include "gy95REG.h"

#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"
#include "usart.h"

#define IMU_BROCASE_REG 0x00
#define IMU_SET_BAUDRATE_REG 0x01
#define IMU_SET_UPDATE_RATE_REG 0x02
#define IMU_SET_DATA_OUTPUT_MODE 0x03
#define IMU_SET_OUTPUT_FORMAT_REG 0x04
#define IMU_CONFIG_REG 0x05
#define IMU_CALIBRATE_REG 0x06
#define IMU_SET_MODE_REG 0x07

#define IMU_FACTORY_RESET_CMD 0xAA
#define IMU_SAVE_CONFIG_CMD 0x55
#define IMU_ACC_CALIBRATE_CMD 0x57
#define IMU_MAG_CALIBRATE_START_CMD 0x58
#define IMU_MAG_CALIBRATE_STOP_CMD 0x59
#define IMU_MAG_CALIBRATE_SAVE_CMD 0x5A

#define IMU_SET_DATARATE_10HZ 0x00
#define IMU_SET_DATARATE_50Hz 0x01
#define IMU_SET_DATARATE_100Hz 0x02
#define IMU_SET_DATARATE_200Hz 0x03

#define ACC_X_L 0x08
#define ACC_X_H 0x09
#define ACC_Y_L 0x0a
#define ACC_Y_H 0x0b
#define ACC_Z_L 0x0c
#define ACC_Z_H 0x0d

#define GYRO_X_L 0x0e
#define GYRO_X_H 0x0f
#define GYRO_Y_L 0x10
#define GYRO_Y_H 0x11
#define GYRO_Z_L 0x12
#define GYRO_Z_H 0x13

#define ROLL_L 0x14
#define ROLL_H 0x15
#define PITCH_L 0x16
#define PITCH_H 0x17
#define YAW_L 0x18
#define YAW_H 0x19

#define TEMP_L 0x1B
#define TEMP_H 0x1C

#define MAG_X_L 0x1D
#define MAG_X_H 0x1E
#define MAG_Y_L 0x1F
#define MAG_Y_H 0x20
#define MAG_Z_L 0x21
#define MAG_Z_H 0x22

#define Q0_L 0x23
#define Q0_H 0x24
#define Q1_L 0x25
#define Q1_H 0x26
#define Q2_L 0x27
#define Q2_H 0x28
#define Q3_L 0x29
#define Q3_H 0x2A

typedef struct
{
    int16_t Acc_x;
    int16_t Acc_y;
    int16_t Acc_z;

    int16_t Gyro_x;
    int16_t Gyro_y;
    int16_t Gyro_z;

    int8_t Yaw;
    int8_t Pitch;
    int8_t Row;

    int16_t Q0;
    int16_t Q1;
    int16_t Q2;
    int16_t Q3;

    int8_t temp;
} gyro_data;

void gy95t_init(void);

bool gy95t_detect(void);
bool gy95_read_all(gyro_data *gy95t);

// void iic_read(uint8_t add, uint8_t *data, uint8_t len);

// void gy95_All(gyro_data *gy95t);

#endif /* INC_GY95T_H_ */
