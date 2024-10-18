/*
 * gy95t.c
 *
 *  Created on: Oct 16, 2024
 *      Author: chengty
 */

#include "gy95t.h"
#include "usart.h"



void iic_read(uint8_t add, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef ret;
    uint8_t mess[20];

    ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)iic_add << 1, &add, 1, 100);
    if (ret != HAL_OK) {
		sprintf(mess, "not transmit to I2C device\n");
		HAL_UART_Transmit(&huart1,mess,sizeof(mess),HAL_MAX_DELAY);
    }
    ret = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)iic_add << 1, data, len, 100);
    if (ret != HAL_OK) {
		sprintf(mess, "no receive from I2C device");
		HAL_UART_Transmit(&huart1,mess,sizeof(mess),HAL_MAX_DELAY);
    }
    // If everything is OK
//	sprintf(mess, "I2C Read OK\n");
//	HAL_UART_Transmit(&huart1, (uint8_t*)mess, strlen(mess), HAL_MAX_DELAY);

}


void gy95_All(gy* my_95Q)
{
	uint8_t data_L;
    uint8_t data_H;

    iic_read(ACC_X_L, &data_L, 1);
    iic_read(ACC_X_H, &data_H, 1);
    my_95Q->Acc_x = (int16_t)((data_H << 8) | data_L);

    iic_read(ACC_Y_L, &data_L, 1);
    iic_read(ACC_Y_H, &data_H, 1);
    my_95Q->Acc_y = (int16_t)((data_H << 8) | data_L);

    iic_read(ACC_Z_L, &data_L, 1);
    iic_read(ACC_Z_H, &data_H, 1);
    my_95Q->Acc_z = (int16_t)((data_H << 8) | data_L);

    iic_read(GYRO_X_L, &data_L, 1);
    iic_read(GYRO_X_H, &data_H, 1);
    my_95Q->Gyro_x = (int16_t)((data_H << 8) | data_L);

    iic_read(GYRO_Y_L, &data_L, 1);
    iic_read(GYRO_Y_H, &data_H, 1);
    my_95Q->Gyro_y = (int16_t)((data_H << 8) | data_L);

    iic_read(GYRO_Z_L, &data_L, 1);
    iic_read(GYRO_Z_H, &data_H, 1);
    my_95Q->Gyro_z = (int16_t)((data_H << 8) | data_L);

    iic_read(Q0_L, &data_L, 1);
    iic_read(Q0_H, &data_H, 1);
    my_95Q->Q0 = (int16_t)((data_H << 8) | data_L) / 1000;

    iic_read(Q1_L, &data_L, 1);
	iic_read(Q1_H, &data_H, 1);
	my_95Q->Q1 = (int16_t)((data_H << 8) | data_L) / 1000;

	iic_read(Q2_L, &data_L, 1);
	iic_read(Q2_H, &data_H, 1);
	my_95Q->Q2 = (int16_t)((data_H << 8) | data_L) / 1000;

	iic_read(Q3_L, &data_L, 1);
	iic_read(Q3_H, &data_H, 1);
	my_95Q->Q3 = (int16_t)((data_H << 8) | data_L) / 1000;
}
