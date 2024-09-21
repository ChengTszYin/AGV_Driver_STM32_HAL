/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "DDSMLib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t LeftID;
	uint8_t RightID;
	short LeftSpeed;
	short RightSpeed;
} MotorControl;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t checksum(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len-1; i++) {
       crc += data[i];
    }
    return crc;
}

void HostMessageParse(uint8_t *receiveBytes, MotorControl* motors)
{
	uint8_t data[8];
	for(uint8_t i=0;i<8;i++)
	{
		data[i] = receiveBytes[i];
	}
	uint8_t checking = checksum(data,8);
	if(checking==data[7])
	{
		motors->LeftID = data[1];
		motors->LeftSpeed = (data[2] << 8) | data[3];
		motors->RightID = data[4];
		motors->RightSpeed = (data[5] << 8) | data[6];
	}
	memset(receiveBytes, 0, sizeof(receiveBytes));
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t receiveBytes[8];
uint8_t receiveBuff[8];

extern uint8_t responseBuffer[25];
extern uint8_t responseBufferH[10];
extern uint8_t responseBufferL[10];

extern uint8_t commandBuffer[10];
extern struct motor_sensor_t wheelsensor;

MotorControl motors;
uint32_t L_R_delay = pdMS_TO_TICKS(4);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

xTaskHandle Serial_Task_Handler;
xTaskHandle Motor_Task_Handler;

void Serial_Task(void *argument);
void Motor_Task(void *argument);
/* USER CODE END Variables */
//osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(Serial_Task, "Serial_Task_", 128, NULL, 2, &Serial_Task_Handler);
  xTaskCreate(Motor_Task, "Motor_Task", 128, NULL, 1,&Motor_Task_Handler);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		memcpy(receiveBytes, receiveBuff, sizeof(receiveBuff));
		HostMessageParse(receiveBytes, &motors);
		HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
	}

	if(huart == &huart2)
	{
		short len = strlen(responseBuffer);
		short arraysz=sizeof(responseBuffer)/sizeof(*responseBuffer);
		for(int i=0;i<arraysz;i++)
		{
			if(responseBuffer[i]==motors.LeftID)
			{
				uint8_t sigmentBuffer[10];
				memcpy(sigmentBuffer, &responseBuffer[i], 10);
				uint8_t checking = checkCRC(&sigmentBuffer);
				if(checking)
				{
					memcpy(responseBufferL, &responseBuffer[i], 10);
				}
			}
			else if(responseBuffer[i]==motors.RightID)
			{
				uint8_t sigmentBuffer[10];
				memcpy(sigmentBuffer, &responseBuffer[i], 10);
				uint8_t checking = checkCRC(&sigmentBuffer);
				if(checking)
				{
					memcpy(responseBufferH, &responseBuffer[i], 10);
				}
			}
		}
		memset(responseBuffer, 0, sizeof(responseBuffer));
		HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	}
}

void Serial_Task(void *argument)
{
	HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
	HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	while(1)
	{
	  setVelocity(motors.LeftID, motors.LeftSpeed, 0);
	  vTaskDelay(L_R_delay);
	  setVelocity(motors.RightID, motors.RightSpeed, 0);
	  receiveFromBuffer();
	  Parse_DMA_All(&wheelsensor, 0);
	  HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	  HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
	}
}

void Motor_Task(void *argument)
{
	while(1)
	{

	}
}
/* USER CODE END Application */

