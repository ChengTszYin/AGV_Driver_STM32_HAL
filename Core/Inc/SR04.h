/*
 * SR04.h
 *
 *  Created on: Jun 9, 2026
 *      Author: user
 */

#ifndef INC_SR04_H_
#define INC_SR04_H_

#include "main.h"
#include "tim.h"
#include "stdio.h"

#include "chassis.hpp"
#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

typedef struct
{
	TIM_HandleTypeDef *htim;
	GPIO_TypeDef *port;
	uint16_t pin;
} sr04_t;


void delay_us(uint16_t time);
uint16_t HCSR04_Read_l();
uint16_t HCSR04_Read_r();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);



#endif /* INC_SR04_H_ */
