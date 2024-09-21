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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "DDSMLib.h"
#include "mpu6050.h"
#include "SR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_PROX 5

typedef struct
{
	uint8_t LeftID;
	uint8_t RightID;
	short LeftSpeed;
	short RightSpeed;
} MotorControl;

uint8_t d80nk_[4];
extern SR04_PulseType pulse;
extern SR04_PulseType pulse2;
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

void d80nk_read()
{
	GPIO_PinState pinStates[NUM_PROX];
	pinStates[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	pinStates[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	pinStates[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	pinStates[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	for(int i=0; i<4;i++)
	{
		if(pinStates[i] == GPIO_PIN_SET)
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

void distance_Calculate()
{
	SR04_Calculate(&pulse);
	SR04_Calculate(&pulse2);
	if(pulse.distance > 80.0) pulse.distance = 80.0;
	if(pulse2.distance > 80.0) pulse2.distance = 80.0;
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
MPU6050_t MPU6050;
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
  xTaskCreate(Serial_Task, "Serial_Task_", 128, NULL, 4, &Serial_Task_Handler);
  xTaskCreate(Sensor_Task, "Sensor_Task", 128, NULL, 3,&Sensor_Task_Handler);
  xTaskCreate(IMU_Task, "IMU_Task", 128, NULL, 3, IMU_Task_Handler);
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
//	  uint8_t str[20];
//	  sprintf(str, "speed: %d\n", (int)wheelsensor.LeftVelocity);
//	  HAL_UART_Transmit(&huart3, str, sizeof(str), HAL_MAX_DELAY);
	  HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	  HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
	}
}

void Sensor_Task(void *argument)
{
	SR04_Init();
	while(1)
	{
		SR04_Start();
		d80nk_read();
		distance_Calculate();
//		uint8_t str[20];
//		sprintf(str, "distance: %d\n", (int)pulse2.distance);
//		HAL_UART_Transmit(&huart3, str, sizeof(str), HAL_MAX_DELAY);
	}
}

void Feedback_Task(void *argument)
{
	uint32_t tick_delay = pdMS_TO_TICKS(200);
	while(1)
	{
		uint8_t sendData[30];
		sendData[0] = 0x00;
		sendData[1] = (wheelsensor.leftii) & 0xFF;
		sendData[2] = ((wheelsensor.LeftVelocity)>>8) & 0xFF;
		sendData[3] = wheelsensor.LeftVelocity & 0xFF;
		sendData[4] = wheelsensor.reightii & 0xFF;
		sendData[5] = ((wheelsensor.RightVelocity)>>8) & 0xFF;
		sendData[6] = wheelsensor.RightVelocity & 0xFF;
		sendData[7] = (MPU6050.Accel_X_RAW >> 8) & 0xFF;
		sendData[8] = MPU6050.Accel_X_RAW & 0XFF;
		sendData[9] = (MPU6050.Accel_Y_RAW >> 8) & 0XFF;
		sendData[10] = MPU6050.Accel_Y_RAW & 0xFF;
		sendData[11] = (MPU6050.Accel_Z_RAW >> 8) & 0xFF;
		sendData[12] = MPU6050.Accel_Z_RAW & 0xFF;
		sendData[13] = (MPU6050.Gyro_X_RAW >> 8) & 0XFF;
		sendData[14] = MPU6050.Gyro_X_RAW & 0xFF;
		sendData[15] = (MPU6050.Gyro_Y_RAW >> 8) & 0XFF;
		sendData[16] = MPU6050.Gyro_Y_RAW & 0xFF;
		sendData[17] = (MPU6050.Gyro_Z_RAW >> 8) & 0XFF;
		sendData[18] = MPU6050.Gyro_Z_RAW & 0xFF;
		sendData[19] = (((int)pulse.distance) >> 8) & 0xFF;
		sendData[20] = ((int)pulse.distance) & 0xFF;
		sendData[21] = (((int)pulse2.distance) >> 8) & 0xFF;
		sendData[22] = ((int)pulse2.distance) & 0xFF;
		sendData[23] = d80nk_[0] & 0xFF;
		sendData[24] = d80nk_[1] & 0xFF;
		sendData[25] = d80nk_[2] & 0xFF;
		sendData[26] = d80nk_[3] & 0xFF;
		sendData[27] = checksum(sendData, 28);
		HAL_UART_Transmit(&huart1, sendData, 28, HAL_MAX_DELAY);
		vTaskDelay(tick_delay);
	}
}

void IMU_Task(void *argument)
{
	uint32_t tick_delay = pdMS_TO_TICKS(500);
	while (MPU6050_Init(&hi2c1) == 1)
	{
	  uint8_t message[30];
	  sprintf(message,"Device not found. Retry...\n");
	  HAL_UART_Transmit(&huart3, message, sizeof(message), HAL_MAX_DELAY);
	  vTaskDelay(tick_delay);
	};
	while(1)
	{
		MPU6050_Read_All(&hi2c1, &MPU6050);

	}
}
/* USER CODE END Application */

