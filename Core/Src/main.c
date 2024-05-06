/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
uint8_t receiveBytes[8];
uint8_t receiveBuff[8];

extern uint8_t commandBuffer[10];
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t responseBuffer[25];
extern uint8_t responseBufferH[10];
extern uint8_t responseBufferL[10];

uint8_t message[20];
extern struct motor_sensor_t wheelsensor;
MotorControl motors;
MPU6050_t MPU6050;
extern SR04_PulseType pulse;
extern SR04_PulseType pulse2;
extern SR04_PulseType pulse3;
extern SR04_PulseType pulse4;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void distance_Calculate()
{
	SR04_Calculate(&pulse);
	SR04_Calculate(&pulse2);
	SR04_Calculate(&pulse3);
	SR04_Calculate(&pulse4);
}

void buzzer()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

}

uint8_t checksum(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len-1; i++) {
       crc += data[i];
    }
    return crc;
}

void HostMessageParse(uint8_t *receiveBytes)
{
	uint8_t data[8];
	for(uint8_t i=0;i<8;i++)
	{
		data[i] = receiveBytes[i];
	}
	uint8_t checking = checksum(data,8);
	if(checking==data[7])
	{
		motors.LeftID = data[1];
		motors.LeftSpeed = (data[2] << 8) | data[3];
		motors.RightID = data[4];
		motors.RightSpeed = (data[5] << 8) | data[6];
	}
	memset(receiveBytes, 0, sizeof(receiveBytes));
}

void SendToHost(struct motor_sensor_t* wheelsensor)
{
	uint8_t sendData[30];
	sendData[0] = 0x00;
	sendData[1] = wheelsensor->leftii;
	sendData[2] = ((wheelsensor->LeftVelocity)>>8) & 0xFF;
	sendData[3] = wheelsensor->LeftVelocity & 0xFF;
	sendData[4] = wheelsensor->reightii;
	sendData[5] = ((wheelsensor->RightVelocity)>>8) & 0xFF;
	sendData[6] = wheelsensor->RightVelocity & 0xFF;
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
	sendData[23] = (((int)pulse3.distance) >> 8) & 0xFF;
	sendData[24] = ((int)pulse3.distance) & 0xFF;
	sendData[25] = (((int)pulse4.distance) >> 8) & 0xFF;
	sendData[26] = ((int)pulse4.distance) & 0xFF;
	sendData[27] = checksum(sendData, 28);
	HAL_UART_Transmit(&huart1, sendData, 28, HAL_MAX_DELAY);
	HAL_Delay(200);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		memcpy(receiveBytes, receiveBuff, sizeof(receiveBuff));
		HostMessageParse(receiveBytes);
		HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
	}
	else if(huart==&huart2)
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  SR04_Init();
  while (MPU6050_Init(&hi2c1) == 1)
    {
  	  sprintf(message,"Device not found. Retry...\n");
  	  HAL_UART_Transmit(&huart1, message, sizeof(message), HAL_MAX_DELAY);
  	  HAL_Delay(100);
    };
  buzzer();
  HAL_UART_Receive_DMA(&huart1,receiveBuff,sizeof(receiveBuff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  SR04_Start();
	  setVelocity(motors.LeftID, motors.LeftSpeed, 0);
	  HAL_Delay(4);
	  setVelocity(motors.RightID, motors.RightSpeed, 0);
	  receiveFromBuffer();
	  Parse_DMA_All(&wheelsensor);
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  distance_Calculate();
	  SendToHost(&wheelsensor);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
