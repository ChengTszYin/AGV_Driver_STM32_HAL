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
// #include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "DDSMLib.h"
#include "gy95t.h"
#include "SR04.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_PROX 5
#define iic_add 0xa4 >> 1

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
uint8_t checksum(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len - 1; i++)
	{
		crc += data[i];
	}
	return crc;
}

void HostMessageParse(uint8_t *receiveBytes, MotorControl *motors)
{
	uint8_t data[8];
	for (uint8_t i = 0; i < 8; i++)
	{
		data[i] = receiveBytes[i];
	}
	uint8_t checking = checksum(data, 8);
	if (checking == data[7])
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

void distance_Calculate()
{
	SR04_Calculate(&pulse);
	SR04_Calculate(&pulse2);
	if (pulse.distance > 80.0)
		pulse.distance = 80.0;
	if (pulse2.distance > 80.0)
		pulse2.distance = 80.0;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t receiveBytes[8];
uint8_t receiveBuff[8];

extern i2c_tx_complete;
extern i2c_rx_complete;

uint8_t data_L;
uint8_t data_H;

extern uint8_t responseBuffer[25];
extern uint8_t responseBufferH[10];
extern uint8_t responseBufferL[10];

extern uint8_t commandBuffer[10];
extern struct motor_sensor_t wheelsensor;

MotorControl motors;
uint32_t L_R_delay = pdMS_TO_TICKS(4);

gyro_data imu_gy95t = {0};

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
// osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
	xTaskCreate(Sensor_Task, "Sensor_Task", 128, NULL, 3, &Sensor_Task_Handler);
	xTaskCreate(IMU_Task, "IMU_Task", 256, NULL, 3, IMU_Task_Handler);
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
// void StartDefaultTask(void const * argument)
//{
//   /* USER CODE BEGIN StartDefaultTask */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END StartDefaultTask */
// }

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

//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	if(hi2c == &hi2c1)
//	{
//		i2c_tx_complete = 1;
//	}
//}
//
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if(hi2c == &hi2c1)
//    {
//    	i2c_rx_complete = 1;
//    }
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		memcpy(receiveBytes, receiveBuff, sizeof(receiveBuff));
		HostMessageParse(receiveBytes, &motors);
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
			if (responseBuffer[i] == motors.LeftID)
			{
				uint8_t sigmentBuffer[10];
				memcpy(sigmentBuffer, &responseBuffer[i], 10);
				uint8_t checking = checkCRC(&sigmentBuffer);
				if (checking)
				{
					memcpy(responseBufferL, &responseBuffer[i], 10);
				}
			}
			else if (responseBuffer[i] == motors.RightID)
			{
				uint8_t sigmentBuffer[10];
				memcpy(sigmentBuffer, &responseBuffer[i], 10);
				uint8_t checking = checkCRC(&sigmentBuffer);
				if (checking)
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
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
	uint32_t send_delay = pdMS_TO_TICKS(100);
	while (1)
	{
		setVelocity(motors.LeftID, motors.LeftSpeed, 0);
		vTaskDelay(L_R_delay);
		setVelocity(motors.RightID, motors.RightSpeed, 0);
		receiveFromBuffer();
		Parse_DMA_All(&wheelsensor, timerCounter);
		vTaskDelay(send_delay);
		//	  uint8_t str[30];
		//	  sprintf(str, "%L: %d R: %d\n", wheelsensor.LeftVelocity,  wheelsensor.RightVelocity);
		//	  HAL_UART_Transmit(&huart3, str, sizeof(str), HAL_MAX_DELAY);
		HAL_UART_Receive_DMA(&huart2, responseBuffer, 25);
		HAL_UART_Receive_DMA(&huart1, receiveBuff, sizeof(receiveBuff));
	}
}

void Feedback_Task(void *argument)
{
	uint32_t tick_delay = pdMS_TO_TICKS(200);
	while (1)
	{
		uint8_t sendData[36];
		sendData[0] = 0x00;
		sendData[1] = (wheelsensor.leftii) & 0xFF;
		sendData[2] = ((wheelsensor.LeftVelocity) >> 8) & 0xFF;
		sendData[3] = wheelsensor.LeftVelocity & 0xFF;
		sendData[4] = wheelsensor.reightii & 0xFF;
		sendData[5] = ((wheelsensor.RightVelocity) >> 8) & 0xFF;
		sendData[6] = wheelsensor.RightVelocity & 0xFF;
		sendData[7] = (imu_gy95t.Acc_x >> 8) & 0xFF;
		sendData[8] = imu_gy95t.Acc_x & 0XFF;
		sendData[9] = (imu_gy95t.Acc_y >> 8) & 0XFF;
		sendData[10] = imu_gy95t.Acc_y & 0xFF;
		sendData[11] = (imu_gy95t.Acc_z >> 8) & 0xFF;
		sendData[12] = imu_gy95t.Acc_z & 0xFF;
		sendData[13] = (imu_gy95t.Gyro_x >> 8) & 0XFF;
		sendData[14] = imu_gy95t.Gyro_x & 0xFF;
		sendData[15] = (imu_gy95t.Gyro_y >> 8) & 0XFF;
		sendData[16] = imu_gy95t.Gyro_y & 0xFF;
		sendData[17] = (imu_gy95t.Gyro_z >> 8) & 0XFF;
		sendData[18] = imu_gy95t.Gyro_z >> 8 & 0xFF;
		sendData[19] = (imu_gy95t.Q0 >> 8) & 0xFF;
		sendData[20] = imu_gy95t.Q0 & 0xFF;
		sendData[21] = (imu_gy95t.Q1 >> 8) & 0xFF;
		sendData[22] = imu_gy95t.Q1 & 0xFF;
		sendData[23] = (imu_gy95t.Q2 >> 8) & 0xFF;
		sendData[24] = imu_gy95t.Q2 & 0xFF;
		sendData[25] = (imu_gy95t.Q3 >> 8) & 0xFF;
		sendData[26] = imu_gy95t.Q3 & 0xFF;
		sendData[27] = (((int)pulse.distance) >> 8) & 0xFF;
		sendData[28] = ((int)pulse.distance) & 0xFF;
		sendData[29] = (((int)pulse2.distance) >> 8) & 0xFF;
		sendData[30] = ((int)pulse2.distance) & 0xFF;
		sendData[31] = d80nk_[0] & 0xFF;
		sendData[32] = d80nk_[1] & 0xFF;
		sendData[33] = d80nk_[2] & 0xFF;
		sendData[34] = d80nk_[3] & 0xFF;
		sendData[35] = checksum(sendData, 36);
		HAL_UART_Transmit(&huart1, sendData, 36, HAL_MAX_DELAY);
		vTaskDelay(tick_delay);
	}
}

void Sensor_Task(void *argument)
{
	SR04_Init();
	while (1)
	{
		SR04_Start();
		d80nk_read();
		distance_Calculate();
	}
}

/// @brief IMU control and display task
/// @param argument

void IMU_Task(void *argument)
{
	uint8_t str[30];
	gy95t_init();
	sprintf(str, "IMU init sucess\r\n");
	HAL_UART_Transmit(&huart1, str, strlen(str), HAL_MAX_DELAY);
	while (1)
	{
		gy95t_read_all(imu_gy95t);
		// use uart to printf something
		sprintf(str, "IMU: ax: %d, ay: %d az: %d\r\n", imu_gy95t.Acc_x, imu_gy95t.Acc_y, imu_gy95t.Acc_z);
		HAL_UART_Transmit(&huart1, str, strlen(str), HAL_MAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
/* USER CODE END Application */
