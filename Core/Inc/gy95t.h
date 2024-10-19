/*
 * gy95t.h
 *
 *  Created on: Oct 19, 2024
 *      Author: chengty
 */

#ifndef INC_GY95T_H_
#define INC_GY95T_H_

#include "gy95REG.h"
#include <stdint.h>
#include "i2c.h"

#define iic_add  0xa4>>1

typedef struct
{

	int16_t Acc_x;
    int16_t Acc_y;
    int16_t Acc_z;
    int16_t Gyro_x;
    int16_t Gyro_y;
    int16_t Gyro_z;
    int16_t Q0;
    int16_t Q1;
    int16_t Q2;
    int16_t Q3;

} gy;


void iic_read(uint8_t add, uint8_t *data, uint8_t len);

void gy95_All(gy* gy95t);

uint8_t gy95_Init(uint8_t *data);


#endif /* INC_GY95T_H_ */
