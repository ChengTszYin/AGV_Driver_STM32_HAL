
#include "SR04.h"
#include "main.h"
#include "tim.h"

SR04_PulseType pulse = {0};
SR04_PulseType pulse2 = {0};

void SR04_Init()
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

void SR04_Read()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
	__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		if (pulse.Is_First_Captured==0) // if the first value is not captured
		{
			pulse.IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			pulse.Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (pulse.Is_First_Captured==1)   // if the first is already captured
		{
			pulse.IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (pulse.IC_Val2 > pulse.IC_Val1)
			{
				pulse.Difference = pulse.IC_Val2-pulse.IC_Val1;
			}

			else if (pulse.IC_Val1 > pulse.IC_Val2)
			{
				pulse.Difference = (0xffff - pulse.IC_Val1) + pulse.IC_Val2;
			}

			pulse.Distance = pulse.Difference * .034/2;
			pulse.Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
		}
	}
	else if (htim == &htim3)
	{
		if (pulse2.Is_First_Captured==0) // if the first value is not captured
		{
			pulse2.IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			pulse2.Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (pulse2.Is_First_Captured==1)   // if the first is already captured
		{
			pulse2.IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (pulse2.IC_Val2 > pulse2.IC_Val1)
			{
				pulse2.Difference = pulse2.IC_Val2-pulse.IC_Val1;
			}

			else if (pulse2.IC_Val1 > pulse2.IC_Val2)
			{
				pulse2.Difference = (0xffff - pulse2.IC_Val1) + pulse2.IC_Val2;
			}

			pulse2.Distance = pulse2.Difference * .034/2;
			pulse2.Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
		}
	}
}