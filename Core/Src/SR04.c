/*
 * sr04.c
 *
 *  Created on: Oct 23, 2024
 *      Author: chengty
 */
#include "sr04.h"

uint8_t IS_First_CAPTURED =0;
uint16_t IC_value1=0;
uint16_t IC_value2=0;
uint16_t Difference =0;
uint16_t distance1 =0;

uint8_t _IS_First_CAPTURED =0;
uint16_t _IC_value1=0;
uint16_t _IC_value2=0;
uint16_t _Difference =0;
uint16_t distance2 =0;

void delay_us(uint16_t time)
{
  __HAL_TIM_SET_COUNTER(&htim2,0);
  while(__HAL_TIM_GET_COUNTER(&htim2)<time);
}

uint16_t HCSR04_Read(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	delay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);

	__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);

	__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
	//HAL_Delay(100);
	return distance1;
}

uint16_t _HCSR04_Read(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	delay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);

	__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);

	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
	//HAL_Delay(100);
	return distance2;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if(htim==&htim2 &&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(IS_First_CAPTURED==0)
		{
			IS_First_CAPTURED=1;
			IC_value1=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(IS_First_CAPTURED==1)
		{
			IS_First_CAPTURED=0;
			IC_value2=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim,0);

			//����
			if(IC_value1 < IC_value2)
			{
				Difference = IC_value2-IC_value1;
			}
			else if(IC_value1 > IC_value2)
			{
				Difference = 0xffff - IC_value1+IC_value2;
			}

			distance1 = Difference/58;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim,TIM_IT_CC1);

			}
	 }
  if(htim==&htim3 &&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
  	{
  		if(_IS_First_CAPTURED==0)
  		{
  			_IS_First_CAPTURED=1;
  			_IC_value1=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
  			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
  		}
  		else if(_IS_First_CAPTURED==1)
  		{
  			_IS_First_CAPTURED=0;
  			_IC_value2=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
  			__HAL_TIM_SET_COUNTER(htim,0);

  			if(_IC_value1 < _IC_value2)
  			{
  				_Difference = _IC_value2-_IC_value1;
  			}
  			else if(_IC_value1 > _IC_value2)
  			{
  				_Difference = 0xffff - _IC_value1+ _IC_value2;
  			}

  			distance2 = _Difference/58;

  			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
  			__HAL_TIM_DISABLE_IT(htim,TIM_IT_CC1);

  			}
  	 }
}

