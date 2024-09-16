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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for comm_controller */
osThreadId_t comm_controllerHandle;
const osThreadAttr_t comm_controller_attributes = {
  .name = "comm_controller",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_controlle */
osThreadId_t motor_controlleHandle;
const osThreadAttr_t motor_controlle_attributes = {
  .name = "motor_controlle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imu_controller */
osThreadId_t imu_controllerHandle;
const osThreadAttr_t imu_controller_attributes = {
  .name = "imu_controller",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for io_controller */
osThreadId_t io_controllerHandle;
const osThreadAttr_t io_controller_attributes = {
  .name = "io_controller",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void comm_control_task(void *argument);
void motor_control_task(void *argument);
void imu_control_task(void *argument);
void io_control_task(void *argument);

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
  /* creation of comm_controller */
  comm_controllerHandle = osThreadNew(comm_control_task, NULL, &comm_controller_attributes);

  /* creation of motor_controlle */
  motor_controlleHandle = osThreadNew(motor_control_task, NULL, &motor_controlle_attributes);

  /* creation of imu_controller */
  imu_controllerHandle = osThreadNew(imu_control_task, NULL, &imu_controller_attributes);

  /* creation of io_controller */
  io_controllerHandle = osThreadNew(io_control_task, NULL, &io_controller_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_comm_control_task */
/**
  * @brief  Function implementing the comm_controller thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_comm_control_task */
void comm_control_task(void *argument)
{
  /* USER CODE BEGIN comm_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END comm_control_task */
}

/* USER CODE BEGIN Header_motor_control_task */
/**
* @brief Function implementing the motor_controlle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_control_task */
void motor_control_task(void *argument)
{
  /* USER CODE BEGIN motor_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_control_task */
}

/* USER CODE BEGIN Header_imu_control_task */
/**
* @brief Function implementing the imu_controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_control_task */
void imu_control_task(void *argument)
{
  /* USER CODE BEGIN imu_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_control_task */
}

/* USER CODE BEGIN Header_io_control_task */
/**
* @brief Function implementing the io_controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_io_control_task */
void io_control_task(void *argument)
{
  /* USER CODE BEGIN io_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END io_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

