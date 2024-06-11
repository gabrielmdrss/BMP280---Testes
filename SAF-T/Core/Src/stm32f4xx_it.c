/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

extern int contador;
extern int teste;
extern const float accelScalingFactor;//fator de escala do acelerômetro
extern const float gyroScalingFactor;//fator de escala do giroscópio

extern float somAccel_x, somAccel_y, somAccel_z, somGyros_x, somGyros_y,
			somGyros_z, temp;

extern const int16_t RAW_ACCEL_X_OFFSET;	//offsets do acelerômetro
extern const int16_t RAW_ACCEL_Y_OFFSET;
extern const int16_t RAW_ACCEL_Z_OFFSET;
extern const int16_t RAW_GYRO_X_OFFSET;	//offsets do giroscópio
extern const int16_t RAW_GYRO_Y_OFFSET;
extern const int16_t RAW_GYRO_Z_OFFSET;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	uint8_t rawData[14];							//valores crus dos sensores
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do acelerômetro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do giroscópio
	int16_t RAW_TEMP;								//valor cru da temperatura

	Read_MData(0x3B, 14, rawData);
	RAW_ACCEL_X = ((int16_t) rawData[0] << 8) + (rawData[1])
			+ RAW_ACCEL_X_OFFSET;
	RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) + (rawData[3])
			+ RAW_ACCEL_Y_OFFSET;
	RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) + (rawData[5])
			+ RAW_ACCEL_Z_OFFSET;
	RAW_TEMP = ((int16_t) rawData[6] << 8) + (rawData[7]);
	RAW_GYRO_X = ((int16_t) rawData[8] << 8) + (rawData[9]) + RAW_GYRO_X_OFFSET;
	RAW_GYRO_Y = ((int16_t) rawData[10] << 8) + (rawData[11])
			+ RAW_GYRO_Y_OFFSET;
	RAW_GYRO_Z = ((int16_t) rawData[12] << 8) + (rawData[13])
			+ RAW_GYRO_Z_OFFSET;

	somAccel_x += RAW_ACCEL_X;
	somAccel_y += RAW_ACCEL_Y;
	somAccel_z += RAW_ACCEL_Z;
	somGyros_x += RAW_GYRO_X;
	somGyros_y += RAW_GYRO_Y;
	somGyros_z += RAW_GYRO_Z;
	temp = RAW_TEMP;
	contador++;

	if (contador == 10) {
		somAccel_x /= 10.f, somAccel_y /= 10.f, somAccel_z /= 10.f, somGyros_x /=
				10.f, somGyros_y /= 10.f, somGyros_z /= 10.f;
		somAccel_x *= accelScalingFactor, somAccel_y *= accelScalingFactor, somAccel_z *=
				accelScalingFactor, somGyros_x *= gyroScalingFactor, somGyros_y *=
				gyroScalingFactor, somGyros_z *= gyroScalingFactor;

		printf("Impressão dos valores escalonados:\n");
		printf("ACCEL_X = %.1f\n", somAccel_x);
		printf("ACCEL_Y = %.1f\n", somAccel_y);
		printf("ACCEL_Z = %.1f\n\n", somAccel_z);
		printf("GYRO_X = %.0f\n", somGyros_x);
		printf("GYRO_Y = %.0f\n", somGyros_y);
		printf("GYRO_Z = %.0f\n\n", somGyros_z);

		printf("TEMP = %.1f°C\n\n\n\n", (float) temp / 333.87 + 21.0);

		contador = 0;
		somAccel_x = somAccel_y = somAccel_z = somGyros_x = somGyros_y =
				somGyros_z = temp = 0;

	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
