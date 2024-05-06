
#include "SR04.h"

SR04_PulseType pulse;
SR04_PulseType pulse2;
SR04_PulseType pulse3;
SR04_PulseType pulse4;

void SR04_Init()
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);
}
void SR04_Start()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	pulse.rising_flag=1;
	pulse2.rising_flag=1;
	pulse3.rising_flag=1;
	pulse4.rising_flag=1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{
		if(pulse.rising_flag)
		{
			pulse.start = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			pulse.rising_flag = 0;
		}
		else
		{
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			pulse.end = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
			pulse.rising_flag = 1;
		}
	}
	if(htim==&htim3)
	{
		if(pulse2.rising_flag)
		{
			pulse2.start = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			pulse2.rising_flag = 0;
		}
		else
		{
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
			pulse2.end = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
			pulse2.rising_flag = 1;
		}
	}
	if(htim==&htim4)
	{
		if(pulse3.rising_flag)
		{
			pulse3.start = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			pulse3.rising_flag = 0;
		}
		else
		{
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
			pulse3.end = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
			pulse3.rising_flag = 1;
		}
	}
	if(htim==&htim5)
	{
		if(pulse4.rising_flag)
		{
			pulse4.start = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
			pulse4.rising_flag = 0;
		}
		else
		{
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
			pulse4.end = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);
			pulse4.rising_flag = 1;
		}
	}

}


void SR04_Calculate(SR04_PulseType *pulse)
{
	if(pulse->end > pulse->start)
		pulse->cnt = pulse->end - pulse->start;
	else
		pulse->cnt = SR04_COUNT_PERIOD + pulse->end - pulse->start;
	pulse->distance = pulse->cnt * SPEED *100 / 2.0f /1000.0f /1000.0f;
}


