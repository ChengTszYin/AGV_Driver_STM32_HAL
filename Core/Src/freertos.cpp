/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <chassis.hpp>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define receiveByteSize 20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t responseBuffer[65];
uint8_t receiveBytes[receiveByteSize];
uint8_t receiveBuff[receiveByteSize];

volatile uint8_t huart2Received = 0;
volatile uint32_t timerCounter = 0;

xTaskHandle Serial_Task_Handler;
xTaskHandle Sensor_Task_Handler;
xTaskHandle IMU_Task_Handler;
xTaskHandle Feedback_Task_Handler;

uint8_t motor_id[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
Chassis r2(motor_id, 6);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		memcpy(receiveBytes, receiveBuff, sizeof(receiveBuff));
		r2.HostMessageParse(receiveBytes);
		HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	}

	if (huart == &huart2)
	{
		huart2Received = 1;
		timerCounter = 0;
		size_t arraysz = sizeof(responseBuffer);
		for(int i = 0; i < arraysz; ++i)
		{
			uint8_t sigmentBuffer[10];
			memcpy(sigmentBuffer, &responseBuffer[i], 10);

			for (int j = 0; j < 6; ++j)
			{
				if (sigmentBuffer[0] == motor_id[j])
				{
					uint8_t checking = r2.m_checkCRC(sigmentBuffer);
					if(checking)
					{
						r2.writeBuffer(j, sigmentBuffer);
					}
					break;
				}
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)
	{
		if (huart2Received)
		{
			huart2Received = 0; // Reset the flag
			timerCounter = 0;	// Reset the timer counter
		}
		else
		{
			timerCounter++;
			if (timerCounter >= 2) // Adjust the value based on your timer period (e.g., 2 for 1 second if the timer period is 0.5 seconds)
			{
				timerCounter = 1;
			}
		}
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Serial_Task(void *argument);
void Sensor_Task(void *argument);
void IMU_Task(void *argument);
void Feedback_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(Serial_Task, "Serial_Task_", 128, NULL, 5, &Serial_Task_Handler);
  xTaskCreate(Sensor_Task, "Sensor_Task", 128, NULL, 4, &Sensor_Task_Handler);
  xTaskCreate(Feedback_Task, "Feedback_Task", 128, NULL, 3, &Feedback_Task_Handler);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Serial_Task(void *argument)
{
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	HAL_UART_Receive_DMA(&huart2, responseBuffer, sizeof(responseBuffer));
	while (1)
	{
		r2.setMotors();
		vTaskDelay(pdMS_TO_TICKS(5));
		HAL_UART_Receive_DMA(&huart2, responseBuffer, sizeof(responseBuffer));
		HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	}
}

void Feedback_Task(void *argument)
{
	while(1)
	{
		r2.MotorMessageSend();
	}

}

void Sensor_Task(void *argument)
{
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	while (1)
	{
		r2.run_sensors();
	}
}

void IMU_Task(void *argument)
{
	r2.run_imu();
}
/* USER CODE END Application */

