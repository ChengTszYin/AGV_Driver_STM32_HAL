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
#include "i2c.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "DDSMLib.h"
#include "gy95t.h"
#include "sr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_MOTORS 6
#define HOSTMESSAGESZ 20

#define NUM_PROX 5

#define iic_add 0xa4 >> 1

typedef struct
{
	uint8_t m_id;
	short m_speed;
} MotorCmd_t;

uint8_t d80nk_[4];
extern uint16_t distance1;
extern uint16_t distance2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t checksum(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len - 1; i++)
	{
		crc += data[i];
	}
	return crc;
}

void HostMessageParse(uint8_t *receiveBytes, MotorCmd_t *m_motors)
{
	uint8_t data[HOSTMESSAGESZ];
	for (uint8_t i = 0; i < HOSTMESSAGESZ; i++)
	{
		data[i] = receiveBytes[i];
	}
	uint8_t checking = checksum(data, HOSTMESSAGESZ);
	if (checking == data[19])
	{
		m_motors[0].m_id = data[1];
		m_motors[0].m_speed = (data[2] << 8) | data[3];
		m_motors[1].m_id = data[4];
		m_motors[1].m_speed = (data[5] << 8) | data[6];
		m_motors[2].m_id = data[7];
		m_motors[2].m_speed = (data[8] << 8) | data[9];
		m_motors[3].m_id = data[10];
		m_motors[3].m_speed = (data[11] << 8) | data[12];
		m_motors[4].m_id = data[13];
		m_motors[4].m_speed = (data[14] << 8) | data[15];
		m_motors[5].m_id = data[16];
		m_motors[5].m_speed = (data[17] << 8) | data[18];
	}
	memset(receiveBytes, 0, sizeof(receiveBytes));
}

void d80nk_read()
{
	GPIO_PinState pinStates[NUM_PROX];
	pinStates[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	pinStates[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	pinStates[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	pinStates[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	for (int i = 0; i < 4; i++)
	{
		if (pinStates[i] == GPIO_PIN_SET)
		{
			d80nk_[i] = '0';
		}
		else
		{
			d80nk_[i] = '1';
			//			sprintf(message,"Sensor ON\n");
			//			HAL_UART_Transmit(&huart3, message, sizeof(message), HAL_MAX_DELAY);
		}
	}
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t receiveBytes[14];
uint8_t receiveBuff[14];


extern uint8_t responseBuffer[45];
extern uint8_t responseBuffer_module[MAX_MOTORS][10];

extern uint8_t commandBuffer[10];
motor_sensor_t wheelsensor[MAX_MOTORS];

MotorCmd_t m_motors[MAX_MOTORS];
uint32_t L_R_delay = pdMS_TO_TICKS(4);

gy my_95Q;
uint8_t td = 0;

volatile uint8_t huart2Received = 0;
volatile uint32_t timerCounter = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
xTaskHandle Serial_Task_Handler;
xTaskHandle Sensor_Task_Handler;
xTaskHandle IMU_Task_Handler;
xTaskHandle Feedback_Task_Handler;

void Serial_Task(void *argument);
void Sensor_Task(void *argument);
void IMU_Task(void *argument);
void Feedback_Task(void *argument);
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
  /* add threads, ... */
	xTaskCreate(Serial_Task, "Serial_Task_", 128, NULL, 5, &Serial_Task_Handler);
	xTaskCreate(Sensor_Task, "Sensor_Task", 128, NULL, 4, &Sensor_Task_Handler);
	//xTaskCreate(IMU_Task, "IMU_Task", 128, NULL, 3, IMU_Task_Handler);
	xTaskCreate(Feedback_Task, "Feedback_Task", 128, NULL, 3, Feedback_Task_Handler);
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
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		memcpy(receiveBytes, receiveBuff, sizeof(receiveBuff));
		HostMessageParse(receiveBytes, &m_motors);
		HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	}

	if (huart == &huart2)
	{
		huart2Received = 1;
		timerCounter = 0;
		short len = strlen(responseBuffer);
		short arraysz = sizeof(responseBuffer) / sizeof(*responseBuffer);
		for (int i = 0; i < arraysz; i++)
		{
			for(int j = 0; j < MAX_MOTORS; ++j)
			{
				if(responseBuffer[i] == wheelsensor[j].id)
				{
					uint8_t sigmentBuffer[10];
					memcpy(sigmentBuffer, &responseBuffer[i], 10);
					uint8_t checking = checkCRC(&sigmentBuffer);
					if(checking)
					{
						memcpy(responseBuffer_module[j], &responseBuffer[i], 10);
					}
				}
			}
		}

		memset(responseBuffer, 0, sizeof(responseBuffer));
		HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	}
}

void Serial_Task(void *argument)
{
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	uint32_t send_delay = pdMS_TO_TICKS(100);
	while (1)
	{
		setVelocity(m_motors[0].m_id, m_motors[0].m_speed, 0);
		vTaskDelay(L_R_delay * 5);
		setVelocity(m_motors[1].m_id, m_motors[1].m_speed, 0);
		vTaskDelay(L_R_delay * 4);
		setVelocity(m_motors[2].m_id, m_motors[2].m_speed, 0);
		vTaskDelay(L_R_delay * 3);
		setVelocity(m_motors[3].m_id, m_motors[3].m_speed, 0);
		vTaskDelay(L_R_delay * 2);
		setVelocity(m_motors[4].m_id, m_motors[4].m_speed, 0);
		vTaskDelay(L_R_delay * 1);
		setVelocity(m_motors[5].m_id, m_motors[5].m_speed, 0);
		receiveFromBuffer();
		Parse_DMA_All(&wheelsensor, timerCounter);
		vTaskDelay(send_delay);
//		uint8_t str[30];
//		sprintf(str, "HL: %d HR: %d LL: %d LR: %d\n", wheelsensor.HLeftVelocity,  wheelsensor.HRightVelocity, wheelsensor.LLeftVelocity,  wheelsensor.LRightVelocity);
//		HAL_UART_Transmit(&huart3, str, sizeof(str), HAL_MAX_DELAY);
		HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
		HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	}
}

void Feedback_Task(void *argument)
{
	uint32_t tick_delay = pdMS_TO_TICKS(200);
	while(1)
	{
		uint8_t sendData[48];
		sendData[0] = 0x00;
		sendData[1] = (wheelsensor[0].id) & 0xFF;
		sendData[2] = ((wheelsensor[0].velocity)>>8) & 0xFF;
		sendData[3] = wheelsensor[0].velocity & 0xFF;
		sendData[4] = wheelsensor[1].id & 0xFF;
		sendData[5] = ((wheelsensor[1].velocity)>>8) & 0xFF;
		sendData[6] = wheelsensor[1].velocity & 0xFF;
		sendData[7] = (wheelsensor[2].id) & 0xFF;
		sendData[8] = ((wheelsensor[2].velocity) >> 8) & 0xFF;
		sendData[9] = wheelsensor[2].velocity & 0xFF;
		sendData[10] = (wheelsensor[3].id) & 0xFF;
		sendData[11] = ((wheelsensor[3].velocity) >> 8) & 0xFF;
		sendData[12] = wheelsensor[3].velocity & 0xFF;
		sendData[13] = (wheelsensor[4].id) & 0xFF;
		sendData[14] = ((wheelsensor[4].velocity) >> 8) & 0xFF;
		sendData[15] = wheelsensor[4].velocity & 0xFF;
		sendData[16] = (wheelsensor[5].id) & 0xFF;
		sendData[17] = ((wheelsensor[5].velocity) >> 8) & 0xFF;
		sendData[18] = wheelsensor[5].velocity & 0xFF;
		sendData[19] = (my_95Q.Acc_x >> 8) & 0xFF;
		sendData[20] = my_95Q.Acc_x & 0XFF;
		sendData[21] = (my_95Q.Acc_y >> 8) & 0XFF;
		sendData[22] = my_95Q.Acc_y & 0xFF;
		sendData[23] = (my_95Q.Acc_z >> 8) & 0xFF;
		sendData[24] = my_95Q.Acc_z & 0xFF;
		sendData[25] = (my_95Q.Gyro_x >> 8) & 0XFF;
		sendData[26] = my_95Q.Gyro_x & 0xFF;
		sendData[27] = (my_95Q.Gyro_y >> 8) & 0XFF;
		sendData[28] = my_95Q.Gyro_y & 0xFF;
		sendData[29] = (my_95Q.Gyro_z >> 8) & 0XFF;
		sendData[30] = my_95Q.Gyro_z >> 8 & 0xFF;
		sendData[31] = (my_95Q.Q0 >> 8) & 0xFF;
		sendData[32] = my_95Q.Q0 & 0xFF;
		sendData[33] = (my_95Q.Q1 >> 8) & 0xFF;
		sendData[34] = my_95Q.Q1 & 0xFF;
		sendData[35] = (my_95Q.Q2 >> 8) & 0xFF;
		sendData[36] = my_95Q.Q2 & 0xFF;
		sendData[37] = (my_95Q.Q3 >> 8) & 0xFF;
		sendData[38] = my_95Q.Q3 & 0xFF;
		sendData[39] = (distance1 >> 8) & 0xFF;
		sendData[40] = (distance1) & 0xFF;
		sendData[41] = (distance2 >> 8) & 0xFF;
		sendData[42] = (distance2) & 0xFF;
		sendData[43] = d80nk_[0] & 0xFF;
		sendData[44] = d80nk_[1] & 0xFF;
		sendData[45] = d80nk_[2] & 0xFF;
		sendData[46] = d80nk_[3] & 0xFF;
		sendData[47] = checksum(sendData, 48);
		HAL_UART_Transmit(&huart1, sendData, 48, HAL_MAX_DELAY);
		vTaskDelay(tick_delay);
	}

}

void Sensor_Task(void *argument)
{
	uint32_t send_delay = pdMS_TO_TICKS(100);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	while (1)
	{
		HCSR04_Read();
		_HCSR04_Read();
		d80nk_read();
		vTaskDelay(send_delay);
	}
}

/// @brief IMU control and display task
/// @param argument

void IMU_Task(void *argument)
{
	uint32_t tick_delay = pdMS_TO_TICKS(200);
	uint8_t inited = 0;
	uint8_t debug[30];
	do {
		inited = gy95_Init(&td);
		sprintf(debug, "IMU not inited\n");
		HAL_UART_Transmit(&huart3, debug, sizeof(debug), HAL_MAX_DELAY);
		vTaskDelay(tick_delay);
	    } while (inited != 1);
	while(1)
	{
//		sprintf(debug, "IMU is inited\n");
//		HAL_UART_Transmit(&huart3, debug, sizeof(debug), HAL_MAX_DELAY);
		gy95_All(&my_95Q);
		vTaskDelay(tick_delay);
	}
}
/* USER CODE END Application */
