/*
 * sr04.h
 *
 *  Created on: Oct 23, 2024
 *      Author: chengty
 */

#ifndef INC_SR04_H_
#define INC_SR04_H_

#include "main.h"
#include "tim.h"
#include "stm32f1xx_hal.h"

typedef struct
{
	TIM_HandleTypeDef *htim;
	GPIO_TypeDef *port;
	uint16_t pin;
} sr04_t;


void delay_us(uint16_t time);

uint16_t HCSR04_Read();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_SR04_H_ */
