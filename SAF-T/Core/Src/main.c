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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "BMP280_SPI.h"
//#include "BMP280_SPI_2.h"
#include "MPU6500_SPI.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "DEFINES_SAFT.h"
#include "Utility.h"

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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int teste = 0;
int teste2 = 0;
int contador = 0;

float somAccel_x, somAccel_y, somAccel_z, somGyros_x, somGyros_y,
			somGyros_z, temp = 0;
						//valor cru da temperatura

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
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
  Utility_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	SPI2_Init();		//inicialização da interface SPI2

	printf("\n--------  Exemplo de aplicação de uso MPU-9250 via SPI  --------\n\n");

	MPU6500_Config();
	MPU6500_Offset_Cancellation();

	  //Configuração do acesso via DMA
	SPI2->CR2 |= SPI_CR2_TXDMAEN;	//habilita solicitações DMA no TX do SPI2
	SPI2->CR2 |= SPI_CR2_RXDMAEN;	//habilita solicitações DMA no RX do SPI2
	DMA1_Config();
	PB9_Int_Config();

	//Iniciando os acumuladores dos valores do sensor
	ACC_RAW_ACCEL_X = 0;
	ACC_RAW_ACCEL_Y = 0;
	ACC_RAW_ACCEL_Z = 0;
	ACC_RAW_GYRO_X = 0;
	ACC_RAW_GYRO_Y = 0;
	ACC_RAW_GYRO_Z = 0;
	ACC_RAW_TEMP = 0;
	//HAL_Delay(1000);

//	SysTick->CTRL = 0;			//desabilita o SysTick
//	SysTick->LOAD = 2.1e5;		//carrega o registrador Reload Value (interrupções a cada 500ms)
//	SysTick->VAL = 0;			//reinicia a contagem do contador
//	SysTick->CTRL = 0b011;		//liga o Systick, habilita a interrupção e seleciona a fonte de clock

//	Who_am_I();


  while (1)
  {
		if (Sensor_Data_Ready) {
			//Separação dos valores do sensor do array de dados
			RAW_ACCEL_X = (((uint16_t) Rx_Data[1] << 8) | (Rx_Data[2]));
			RAW_ACCEL_Y = (((uint16_t) Rx_Data[3] << 8) | (Rx_Data[4]));
			RAW_ACCEL_Z = (((uint16_t) Rx_Data[5] << 8) | (Rx_Data[6]));
			RAW_TEMP = (((int16_t) Rx_Data[7] << 8) | (Rx_Data[8]));
			RAW_GYRO_X = (((int16_t) Rx_Data[9] << 8) | (Rx_Data[10]));
			RAW_GYRO_Y = (((int16_t) Rx_Data[11] << 8) | (Rx_Data[12]));
			RAW_GYRO_Z = (((int16_t) Rx_Data[13] << 8) | (Rx_Data[14]));
			//Atualização dos acumuladores
			ACC_RAW_ACCEL_X += RAW_ACCEL_X;
			ACC_RAW_ACCEL_Y += RAW_ACCEL_Y;
			ACC_RAW_ACCEL_Z += RAW_ACCEL_Z;
			ACC_RAW_GYRO_X += RAW_GYRO_X;
			ACC_RAW_GYRO_Y += RAW_GYRO_Y;
			ACC_RAW_GYRO_Z += RAW_GYRO_Z;
			ACC_RAW_TEMP += RAW_TEMP;

			++sample_counter;
			if (sample_counter == N_SAMPLES) {
				//Impressão dos valores lidos do sensor
				/*printf("Dados do sensor:\n");
				 printf("ACCEL_X = %.1fg\n", ((float)ACC_RAW_ACCEL_X/N_SAMPLES)*accelScalingFactor);
				 printf("ACCEL_Y = %.1fg\n", ((float)ACC_RAW_ACCEL_Y/N_SAMPLES)*accelScalingFactor);
				 printf("ACCEL_Z = %.1fg\n\n", ((float)ACC_RAW_ACCEL_Z/N_SAMPLES)*accelScalingFactor);

				 printf("GYRO_X = %.1f°/s\n", ((float)ACC_RAW_GYRO_X/N_SAMPLES)*gyroScalingFactor);
				 printf("GYRO_Y = %.1f°/s\n", ((float)ACC_RAW_GYRO_Y/N_SAMPLES)*gyroScalingFactor);
				 printf("GYRO_Z = %.1f°/s\n\n", ((float)ACC_RAW_GYRO_Z/N_SAMPLES)*gyroScalingFactor);

				 printf("TEMP = %.1f°C\n", ((float)ACC_RAW_TEMP/N_SAMPLES)/333.87 + 21.0);*/
				ACCEL_X = ((float) ACC_RAW_ACCEL_X / N_SAMPLES)
						* accelScalingFactor;
				ACCEL_Y = ((float) ACC_RAW_ACCEL_Y / N_SAMPLES)
						* accelScalingFactor;
				ACCEL_Z = ((float) ACC_RAW_ACCEL_Z / N_SAMPLES)
						* accelScalingFactor;

				GYRO_X = ((float) ACC_RAW_GYRO_X / N_SAMPLES)
						* gyroScalingFactor;
				GYRO_Y = ((float) ACC_RAW_GYRO_Y / N_SAMPLES)
						* gyroScalingFactor;
				GYRO_Z = ((float) ACC_RAW_GYRO_Z / N_SAMPLES)
						* gyroScalingFactor;
				ACCEL = sqrt(
						ACCEL_X * ACCEL_X + ACCEL_Y * ACCEL_Y
								+ ACCEL_Z * ACCEL_Z);
				temperatura = ((float) ACC_RAW_TEMP / N_SAMPLES)
						/ 333.87 + 21.0;
				//if((fabs(ACCEL - 1) > LIMITE_ACCEL) || (fabs(GYRO_X) > LIMITE_GYRO) || (fabs(GYRO_Y) > LIMITE_GYRO) || (fabs(GYRO_Z) > LIMITE_GYRO)){
				if ((fabs(ACCEL - 1) > LIMITE_ACCEL)) {
					estado_mobilidade = 1;
					printf("Se mexeu\n");
				} else {
					estado_mobilidade = 0;
					printf("Não se mexeu\n");
				}

				//Resetando os acumuladores
				ACC_RAW_ACCEL_X = 0;
				ACC_RAW_ACCEL_Y = 0;
				ACC_RAW_ACCEL_Z = 0;
				ACC_RAW_GYRO_X = 0;
				ACC_RAW_GYRO_Y = 0;
				ACC_RAW_GYRO_Z = 0;
				ACC_RAW_TEMP = 0;

				sample_counter = 0;	//reseta o contador de amostras
			}

			Sensor_Data_Ready = FALSE;
		}
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	__HAL_RCC_USART1_CLK_ENABLE();	//habilita o clock da USART
	UART_HandleTypeDef huart1;
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 1000000;	//baud rate = 1Mbps
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	//Enable RX interrupt
	USART1->CR1 |= USART_CR1_RXNEIE;
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	//Delay_ms(1);
	/* USER CODE END USART1_Init 2 */
}

int __io_putchar(int ch) {
	USART1->DR = (ch & (uint16_t) 0x01FF);
	while (!(USART1->SR & USART_SR_TXE))
		;//espera pelo fim da transmissão do caractere para evitar a segunda transmissão antes da primeira ser concluída
	return ch;
}
int __io_getchar(void) {
	return (uint16_t) (USART1->DR & (uint16_t) 0x01FF);
}
//ISR da USART1. Todas as ISR's estão definidas no arquivo startup_stm32.s
void USART1_IRQHandler(void) {
	__io_putchar(__io_getchar());
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	EXTI->PR |= (1 << 9);	//limpa o flag de pendência em EXTI9

	DMA1_Stream4->M0AR = (uint32_t) Tx_Data;//ponteiro do buffer dos dados a transmitir
	DMA1_Stream3->M0AR = (uint32_t) Rx_Data;//ponteiro do buffer dos dados a receber

	DMA1->LIFCR |= 0b111101 << 22;//limpa os flags de interrupção do stream 3 antes de reabilitá-lo
	DMA1->HIFCR |= 0b111101;//limpa os flags de interrupção do stream 4 antes de reabilitá-lo

	GPIOD->ODR &= ~(1 << 12);//faz o pino CS ir para nível baixo (inicia o comando SPI)
	DMA1_Stream3->CR |= DMA_SxCR_EN;	//reabilita o stream3 do DMA1 (SPI2_RX)
	DMA1_Stream4->CR |= DMA_SxCR_EN;	//reabilita o stream4 do DMA1 (SPI2_TX)

}

void bmp280_altitude_cTemp(){

	float po = 1013.25;
	float T, P, sumTemp, sumPress, Pmed, Tmed = 0;
	sumTemp = 0;
	sumPress = 0;

			for (int i = 0; i < 10; i++) {
				T = 0;							//temperatura
				P = 0;							//pressão
				BMP280_Measures(&T, &P);		//medição da temperatura e pressão
				sumTemp += T;
				sumPress += P;
				HAL_Delay(100);
			}

			Tmed = sumTemp / 10.0f;
			Pmed = sumPress / 10.0f;

			//Imprimindo os valores de temperatura e pressão medidos
			printf("Temperatura Media = %.2f °C\n", Tmed);
			printf("Pressão Media= %.2f hPa\n", Pmed);

			//Estimativa da altitude
			float altitude = (((pow((Pmed) / po, (1.0f / 5.255f)) - 1) * (Tmed + 273.15)) / 0.0065f) * - 1; //equação barométrica
			printf("Altitude(Temp):		%.0f m       ", altitude);
}

void bmp280_altitude_sTemp(){

	float po = 1013.25;
	float T, P, sumTemp, sumPress, Pmed, Tmed = 0;
	sumTemp = 0;
	sumPress = 0;

	for (int i = 0; i < 10; i++) {
		T = 0;							//temperatura
		P = 0;							//pressão
		BMP280_Measures(&T, &P);		//medição da temperatura e pressão
		sumTemp += T;
		sumPress += P;
		HAL_Delay(100);
	}

	Tmed = sumTemp / 10.0f;
	Pmed = sumPress / 10.0f;

	//Imprimindo os valores de temperatura e pressão medidos
	printf("Temperatura Media = %.2f °C\n", Tmed);
	printf("Pressão Media= %.2f hPa\n", Pmed);

	//Estimativa da altitude
	float altitude = 44330.0f * (1.0 - pow((Pmed) / po, (1.0f / 5.255f))); //equação barométrica
	printf("Altitude:		%.0f m\n\n\n\n", altitude);

}

void bmp280_amostragem_dados() {

	float T, P, sumTemp, sumPress, Pmed, Tmed = 0;
	sumTemp = 0;
	sumPress = 0;

	for (int i = 0; i < 50; i++) {
		T = 0;							//temperatura
		P = 0;							//pressão
		BMP280_Measures(&T, &P);		//medição da temperatura e pressão
		sumTemp += T;
		sumPress += P;
		HAL_Delay(100);
	}

	Tmed = sumTemp / 50.0f;
	Pmed = sumPress / 50.0f;

	printf("%.4f, %.4f\n", Pmed, Tmed);

}

void bmp280_comparacao_2sensores() {

	float T, P, T_2, P_2, sumTemp, sumPress, sumTemp_2, sumPress_2, Pmed, Tmed,
			Pmed_2, Tmed_2 = 0;

	sumTemp = sumPress = sumTemp_2 = sumPress_2 = 0;
	for (int i = 0; i < 50; i++) {
		T = P = T_2 = P_2 = 0;
		BMP280_Measures(&T, &P);		//medição da temperatura e pressão
		sumTemp += T;
		sumPress += P;
		BMP280_Measures_2(&T_2, &P_2);		//medição da temperatura e pressão
		sumTemp_2 += T_2;
		sumPress_2 += P_2;
		HAL_Delay(100);
	}

	Pmed = sumPress / 50.0f;
	Tmed = sumTemp / 50.0f;
	Pmed_2 = sumPress_2 / 50.0f;
	Tmed_2 = sumTemp_2 / 50.0f;

//	float altitude =
//			(((pow((Pmed) / Po, (1.0f / 5.255f)) - 1) * (Tmed + 273.15))
//					/ 0.0065f) * -1;
//	float altitude2 = (((pow((Pmed_2) / Po, (1.0f / 5.255f)) - 1)
//			* (Tmed_2 + 273.15)) / 0.0065f) * -1;
//	printf("Temp bmp1 = %.2f     ", Tmed);
//	printf("Temp bmp2 = %.2f\n", Tmed_2);
//	printf("Pres bmp1 =  %.2f     ", Pmed);
//	printf("Pres bmp2 =  %.2f\n", Pmed_2);
//	printf("Alti bmp1=    %.2f     ", altitude);
//	printf("Alti bmp2 =    %.2f\n\n\n\n", altitude2);

	printf("%.4f,%.4f,%.4f,%.4f\n", Pmed, Tmed, Pmed_2, Tmed_2);

}

void mpu_9250_offsets() {

	uint8_t rawData[14];							//valores crus dos sensores
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do acelerômetro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do giroscópio
	int somAccel_x, somAccel_y, somAccel_z, somGyros_x, somGyros_y, somGyros_z = 0;

	//leitura e separação dos valores crus dos sensores

	for(int i=0; i < 10 ;i++){
	Read_MData(0x3B, 14, rawData);
	RAW_ACCEL_X = ((int16_t) rawData[0] << 8) + (rawData[1]); //+ RAW_ACCEL_X_OFFSET;
	RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) + (rawData[3]); //+ RAW_ACCEL_Y_OFFSET;
	RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) + (rawData[5]); //+ RAW_ACCEL_Z_OFFSET;
	RAW_GYRO_X = ((int16_t) rawData[8] << 8) + (rawData[9]); // + RAW_GYRO_X_OFFSET;
	RAW_GYRO_Y = ((int16_t) rawData[10] << 8) + (rawData[11]); //+ RAW_GYRO_Y_OFFSET;
	RAW_GYRO_Z = ((int16_t) rawData[12] << 8) + (rawData[13]); //+ RAW_GYRO_Z_OFFSET;
	somAccel_x += RAW_ACCEL_X;
	somAccel_y += RAW_ACCEL_Y;
	somAccel_z += RAW_ACCEL_Z;
	somGyros_x += RAW_GYRO_X;
	somGyros_y += RAW_GYRO_Y;
	somGyros_z += RAW_GYRO_Z;
	HAL_Delay(10);
	}

	somAccel_x /= 10.f, somAccel_y /= 10.f, somAccel_z /= 10.f, somGyros_x /= 10.f, somGyros_y /= 10.f, somGyros_z /= 10.f;

	//impressão dos valores escalonados
	printf("Impressão dos valores crus:\n");
	printf("ACCEL_X = %d\n", somAccel_x);
	printf("ACCEL_Y = %d\n", somAccel_y);
	printf("ACCEL_Z = %d\n\n", somAccel_z);

	printf("GYRO_X = %d\n", somGyros_x);
	printf("GYRO_Y = %d\n", somGyros_y);
	printf("GYRO_Z = %d\n\n\n", somGyros_z);

	somAccel_x = somAccel_y = somAccel_z = somGyros_x = somGyros_y = somGyros_z = 0;

}
//void mpu_9250_amostras(void){
//	uint8_t rawData[14];							//valores crus dos sensores
//		int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do acelerômetro
//		int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do giroscópio
//		int16_t RAW_TEMP;								//valor cru da temperatura
//
//		Read_MData(0x3B, 14, rawData);
//		RAW_ACCEL_X = ((int16_t) rawData[0] << 8) + (rawData[1])
//				+ RAW_ACCEL_X_OFFSET;
//		RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) + (rawData[3])
//				+ RAW_ACCEL_Y_OFFSET;
//		RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) + (rawData[5])
//				+ RAW_ACCEL_Z_OFFSET;
//		RAW_TEMP = ((int16_t) rawData[6] << 8) + (rawData[7]);
//		RAW_GYRO_X = ((int16_t) rawData[8] << 8) + (rawData[9]) + RAW_GYRO_X_OFFSET;
//		RAW_GYRO_Y = ((int16_t) rawData[10] << 8) + (rawData[11])
//				+ RAW_GYRO_Y_OFFSET;
//		RAW_GYRO_Z = ((int16_t) rawData[12] << 8) + (rawData[13])
//				+ RAW_GYRO_Z_OFFSET;
//
//		somAccel_x += RAW_ACCEL_X;
//		somAccel_y += RAW_ACCEL_Y;
//		somAccel_z += RAW_ACCEL_Z;
//		somGyros_x += RAW_GYRO_X;
//		somGyros_y += RAW_GYRO_Y;
//		somGyros_z += RAW_GYRO_Z;
//		temp = RAW_TEMP;
//		contador += 1;
//
//		if(contador == 10){
//			somAccel_x /= 10.f, somAccel_y /= 10.f, somAccel_z /= 10.f, somGyros_x /=
//					10.f, somGyros_y /= 10.f, somGyros_z /= 10.f;
//			somAccel_x *= accelScalingFactor, somAccel_y *= accelScalingFactor, somAccel_z *=
//					accelScalingFactor, somGyros_x *= gyroScalingFactor, somGyros_y *=
//					gyroScalingFactor, somGyros_z *= gyroScalingFactor;
//}
//		teste=0;
//}
//void mpu_9250_amostras() {
//
//	const float accelScalingFactor = ((float) 4 / 32768); //fator de escala do acelerômetro
//	const float gyroScalingFactor = ((float) 500 / 32768); //fator de escala do giroscópio
//
//	uint8_t rawData[14];							//valores crus dos sensores
//	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do acelerômetro
//	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do giroscópio
//	int16_t RAW_TEMP;								//valor cru da temperatura
//	float somAccel_x, somAccel_y, somAccel_z, somGyros_x, somGyros_y,
//			somGyros_z = 0;
//
//	for (int i = 0; i < 10; i++) {
//		//leitura e separação dos valores crus dos sensores
//		Read_MData(0x3B, 14, rawData);
//		RAW_ACCEL_X = ((int16_t) rawData[0] << 8) + (rawData[1])
//				+ RAW_ACCEL_X_OFFSET;
//		RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) + (rawData[3])
//				+ RAW_ACCEL_Y_OFFSET;
//		RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) + (rawData[5])
//				+ RAW_ACCEL_Z_OFFSET;
//		RAW_TEMP = ((int16_t) rawData[6] << 8) + (rawData[7]);
//		RAW_GYRO_X = ((int16_t) rawData[8] << 8) + (rawData[9])
//				+ RAW_GYRO_X_OFFSET;
//		RAW_GYRO_Y = ((int16_t) rawData[10] << 8) + (rawData[11])
//				+ RAW_GYRO_Y_OFFSET;
//		RAW_GYRO_Z = ((int16_t) rawData[12] << 8) + (rawData[13])
//				+ RAW_GYRO_Z_OFFSET;
//
//		somAccel_x += RAW_ACCEL_X;
//		somAccel_y += RAW_ACCEL_Y;
//		somAccel_z += RAW_ACCEL_Z;
//		somGyros_x += RAW_GYRO_X;
//		somGyros_y += RAW_GYRO_Y;
//		somGyros_z += RAW_GYRO_Z;
//		HAL_Delay(10);
//	}
//	somAccel_x /= 10.f, somAccel_y /= 10.f, somAccel_z /= 10.f, somGyros_x /=
//			10.f, somGyros_y /= 10.f, somGyros_z /= 10.f;
//	somAccel_x *= accelScalingFactor, somAccel_y *= accelScalingFactor, somAccel_z *=
//			accelScalingFactor, somGyros_x *= gyroScalingFactor, somGyros_y *=
//			gyroScalingFactor, somGyros_z *= gyroScalingFactor;
//
//	//impressão dos valores escalonados
//	printf("Impressão dos valores escalonados:\n");
//	printf("ACCEL_X = %.1f\n", somAccel_x);
//	printf("ACCEL_Y = %.1f\n", somAccel_y);
//	printf("ACCEL_Z = %.1f\n\n", somAccel_z);
//	printf("GYRO_X = %.0f\n", somGyros_x);
//	printf("GYRO_Y = %.0f\n", somGyros_y);
//	printf("GYRO_Z = %.0f\n\n", somGyros_z);
//
//	printf("TEMP = %.1f°C\n\n\n\n", (float) RAW_TEMP / 333.87 + 21.0);
//}

void DMA1_Stream3_IRQHandler(void)
{
	DMA1->LIFCR |= 0b111101 << 22;	//limpa as flags de interrupção
	GPIOD->ODR |= (1 << 12);		//faz o pino CS ir para nível alto (finaliza o comando SPI)
	Sensor_Data_Ready = TRUE;		//sinaliza que os dados estão prontos para processamento
}

void DMA1_Config(void)
{
	//Configuração do canal 0 do stream 4 do DMA1 (SPI2_TX)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;			//habilita o clock da interface do DMA1

	DMA1_Stream4->PAR = (uint32_t) &(SPI2->DR);	//ponteiro do periférico para onde os dados devem ser transferidos
	DMA1_Stream4->CR &= ~DMA_SxCR_CHSEL;		//seleciona canal 0
	DMA1_Stream4->NDTR = 15;					//quantidade de dados a serem transferidos para o periférico
	DMA1_Stream4->CR |= DMA_SxCR_DIR_0;			//direção da memória para o periférico
	DMA1_Stream4->CR |= DMA_SxCR_MINC;			//ponteiro da memória pós incrementado automaticamente
	DMA1_Stream4->CR &= ~DMA_SxCR_PINC;			//ponteiro do periférico fixo
	DMA1_Stream4->CR &= ~DMA_SxCR_PSIZE;		//tamanho do dado do periférico (8 bits)
	DMA1_Stream4->CR &= ~DMA_SxCR_MSIZE;		//tamanho do dado na memória (8 bits)
	DMA1_Stream4->FCR &= ~DMA_SxFCR_DMDIS;		//habilita o modo de transferência direta (sem FIFO)

	//Configuração do canal 0 do stream 3 do DMA1 (SPI2_RX)
	DMA1_Stream3->PAR = (uint32_t) &(SPI2->DR);	//ponteiro do periférico de onde os dados devem ser transferidos
	DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL;		//seleciona canal 0
	DMA1_Stream3->NDTR = 15;					//quantidade de dados a serem transferidos para a memória
	DMA1_Stream3->CR &= ~DMA_SxCR_DIR;			//direção do periférico para a memória
	DMA1_Stream3->CR |= DMA_SxCR_MINC;			//ponteiro da memória pós incrementado automaticamente
	DMA1_Stream3->CR &= ~DMA_SxCR_PINC;			//ponteiro do periférico fixo
	DMA1_Stream3->CR &= ~DMA_SxCR_PSIZE;		//tamanho do dado do periférico (8 bits)
	DMA1_Stream3->CR &= ~DMA_SxCR_MSIZE;		//tamanho do dado na memória (8 bits)
	DMA1_Stream3->FCR &= ~DMA_SxFCR_DMDIS;		//habilita o modo de transferência direta (sem FIFO)

	DMA1_Stream3->CR |= DMA_SxCR_TCIE;			//habilita a interrupção de transferência completa do Stream3 do DMA1
	//NVIC_EnableIRQ(DMA1_Stream3_IRQn);			//habilita a interrupção no NVIC
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void PB9_Int_Config(void)
{
	//Configuração do pino PB9 como entrada digital com resistor de pull-down
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	//habilita o clock do GPIOB
	GPIOB->MODER &= ~(0b11 << 18);			//seleciona modo de entrada digital no pino PB9
	GPIOB->PUPDR |= 0b10 << 18;				//habilita o resistor de pull-down no pino PB9

	//Configuração da EXTI9 para receber pulsos externos em PB9
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	//habilita o clock de SYSCFG
	SYSCFG->EXTICR[2] |= 0b0001 << 4;		//seleciona PB9 como gatilho de EXTI9
	EXTI->RTSR |= 1 << 9;			 		//habilita a detecção por borda de subida em PB9
	EXTI->IMR |= 1 << 9;				 	//habilita a interrupção EXTI9 no controlador EXTI
	NVIC_EnableIRQ(EXTI9_5_IRQn);			//habilita a interrupção EXTI9 no NVIC
}



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
