/*
 * gy95t.c
 *
 *  Created on: Oct 16, 2024
 *      Author: chengty
 */

#include "gy95t.h"

#define GY95t_ADDR 0xa4

#define I2C_DEFAULT_TIMEOUT_MS (1000)

bool i2c_is_device_ready(uint8_t dev_addr)
{
  return (HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 3, I2C_DEFAULT_TIMEOUT_MS) == HAL_OK);
}

bool i2c_write_mem(uint8_t dev_addr, uint16_t mem_addr, uint8_t *buffer, uint16_t length)
{
  return (HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buffer, length, I2C_DEFAULT_TIMEOUT_MS) == HAL_OK);
}

bool i2c_read_mem(uint8_t dev_addr, uint16_t mem_addr, uint8_t *buffer, uint16_t length)
{
  return (HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buffer, length, I2C_DEFAULT_TIMEOUT_MS) == HAL_OK);
}

bool i2c_write_buf(uint8_t dev_addr, uint8_t *buffer, uint16_t length)
{
  return (HAL_I2C_Master_Transmit(&hi2c1, dev_addr, buffer, length, I2C_DEFAULT_TIMEOUT_MS) == HAL_OK);
}

bool i2c_read_buf(uint8_t dev_addr, uint8_t *buffer, uint16_t length)
{
  return (HAL_I2C_Master_Receive(&hi2c1, dev_addr, buffer, length, I2C_DEFAULT_TIMEOUT_MS) == HAL_OK);
}

static inline bool gy95t_write_reg(uint8_t reg, uint8_t value)
{
  return i2c_write_mem(GY95t_ADDR << 1, reg, &value, 1);
}

static inline bool gy95t_read_reg(uint8_t reg, uint8_t *buffer, uint16_t length)
{
  return i2c_read_mem(GY95t_ADDR << 1, reg, buffer, length);
}

bool gy95t_detect(void)
{
  uint8_t chip_id = 0;
  uint8_t i = 0;

  // broast and update the chip id
  while ((chip_id != GY95t_ADDR) && (i++ < 5))
  {
	  gy95t_read_reg(IMU_BROCASE_REG, &chip_id, 1);
    if ((i == 5) && (chip_id != GY95t_ADDR))
    {
      return false;
    }
  }
  return true;
}

void gy95t_init(void)
{
  if (gy95t_detect())
  {
    // reset the device
	  gy95t_write_reg(IMU_CONFIG_REG, IMU_FACTORY_RESET_CMD);
    // sys_delay_ms(200);

    // On calibrateions:
    // write_reg(IMU_CONFIG_REG, IMU_ACC_CALIBRATE_CMD);
    // sys_delay_ms(2200);

    // Configure the range and placements
	  gy95t_write_reg(IMU_SET_MODE_REG, 0b10100110);
//    sys_delay_ms(1);

    // wait some times for settle down the data
  }
  else
  {
    while (1)
    {
      // system hang  or do something
    }
  }
}

bool gy95t_read_acc(gyro_data *gy95t)
{
  uint8_t data[6] = {0};
  bool result = gy95t_read_reg(ACC_X_L, data, 6);
  int16_t x = (int16_t)((data[1] << 8) | data[0]);
  int16_t y = (int16_t)((data[3] << 8) | data[2]);
  int16_t z = (int16_t)((data[5] << 8) | data[4]);

  gy95t->Acc_x = x;
  gy95t->Acc_y = y;
  gy95t->Acc_z = z;

  return result;
}


bool gy95t_read_gyro(gyro_data *gy95t)
{
  uint8_t data[6] = {0};
  bool result = gy95t_read_reg(GYRO_X_L, data, 6);
  int16_t x = (int16_t)((data[1] << 8) | data[0]);
  int16_t y = (int16_t)((data[3] << 8) | data[2]);
  int16_t z = (int16_t)((data[5] << 8) | data[4]);

  gy95t->Gyro_x = x;
  gy95t->Gyro_y = y;
  gy95t->Gyro_z = z;

  return result;
}


bool gy95t_read_all(gyro_data *gy95t)
{
  gy95t_read_acc(gy95t);
  gy95t_read_gyro(gy95t);
}

// void iic_read(uint8_t add, uint8_t *data, uint8_t len)
// {
//   HAL_StatusTypeDef ret;
//   uint8_t mess[20];

//   ret = HAL_I2C_Master_Transmit(&&hi2c11, (uint16_t)iic_add << 1, &add, 1, 100);
//   if (ret != HAL_OK)
//   {
//     sprintf(mess, "not transmit to I2C device\n");
//     HAL_UART_Transmit(&huart1, mess, sizeof(mess), HAL_MAX_DELAY);
//   }
//   ret = HAL_I2C_Master_Receive(&&hi2c11, (uint16_t)iic_add << 1, data, len, 100);
//   if (ret != HAL_OK)
//   {
//     sprintf(mess, "no receive from I2C device");
//     HAL_UART_Transmit(&huart1, mess, sizeof(mess), HAL_MAX_DELAY);
//   }
//   // If everything is OK
//   //	sprintf(mess, "I2C Read OK\n");
//   //	HAL_UART_Transmit(&huart1, (uint8_t*)mess, strlen(mess), HAL_MAX_DELAY);
// }

// void gy95t_All(gy *my_95Q)
// {
//   uint8_t data_L;
//   uint8_t data_H;

//   iic_read(ACC_X_L, &data_L, 1);
//   iic_read(ACC_X_H, &data_H, 1);
//   my_95Q->Acc_x = (int16_t)((data_H << 8) | data_L);

//   iic_read(ACC_Y_L, &data_L, 1);
//   iic_read(ACC_Y_H, &data_H, 1);
//   my_95Q->Acc_y = (int16_t)((data_H << 8) | data_L);

//   iic_read(ACC_Z_L, &data_L, 1);
//   iic_read(ACC_Z_H, &data_H, 1);
//   my_95Q->Acc_z = (int16_t)((data_H << 8) | data_L);

//   iic_read(GYRO_X_L, &data_L, 1);
//   iic_read(GYRO_X_H, &data_H, 1);
//   my_95Q->Gyro_x = (int16_t)((data_H << 8) | data_L);

//   iic_read(GYRO_Y_L, &data_L, 1);
//   iic_read(GYRO_Y_H, &data_H, 1);
//   my_95Q->Gyro_y = (int16_t)((data_H << 8) | data_L);

//   iic_read(GYRO_Z_L, &data_L, 1);
//   iic_read(GYRO_Z_H, &data_H, 1);
//   my_95Q->Gyro_z = (int16_t)((data_H << 8) | data_L);

//   iic_read(Q0_L, &data_L, 1);
//   iic_read(Q0_H, &data_H, 1);
//   my_95Q->Q0 = (int16_t)((data_H << 8) | data_L) / 1000;

//   iic_read(Q1_L, &data_L, 1);
//   iic_read(Q1_H, &data_H, 1);
//   my_95Q->Q1 = (int16_t)((data_H << 8) | data_L) / 1000;

//   iic_read(Q2_L, &data_L, 1);
//   iic_read(Q2_H, &data_H, 1);
//   my_95Q->Q2 = (int16_t)((data_H << 8) | data_L) / 1000;

//   iic_read(Q3_L, &data_L, 1);
//   iic_read(Q3_H, &data_H, 1);
//   my_95Q->Q3 = (int16_t)((data_H << 8) | data_L) / 1000;
// }
