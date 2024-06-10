/*
 * MPU6500_SPI.h
 *
 *  Created on: 31 de jul de 2023
 *      Author: Fagner
 */

#ifndef MPU6500_SPI_H_
#define MPU6500_SPI_H_

//Defini��es dos endere�os dos registradores
//Todos os registradores iniciam em 0x00, exceto: (WHO_I_AM = 0x70) e (PWR_MGMT_1 = 0x01)
#define	SMPLRT_DIV			0x19	//sample rate divider
#define	CONFIG				0x1A	//configura��es gerais
#define	GYRO_CONFIG			0x1B	//configura��es do girosc�pio
#define	ACCEL_CONFIG		0x1C	//configura��es do aceler�metro
#define	ACCEL_CONFIG2		0x1D	//configura��es do aceler�metro
#define INT_PIN_CFG			0x37	//configura��es do pino de interrup��o
#define	INT_ENABLE			0x38	//habilita��o de interrup��es
#define	INT_STATUS			0x3A	//status das interrup��es
#define	SIGNAL_PATH_RESET	0x68	//reset do caminho de dados digitais
#define USER_CTRL			0x6A	//controles do usu�rio
#define	PWR_MGMT_1			0x6B	//gerenciamento de energia
#define	PWR_MGMT_2			0x6C	//gerenciamento de energia
#define	WHO_AM_I			0x75	//ID do MPU 9250 (ID = 0x71)


//defini��es dos bits dos registradores
#define	H_RESET			(1<<7)	//bit 7 do registrador PWR_MGMT_1
#define	I2C_IF_DIS		(1<<4)	//bit 4 do registrador USER_CTRL
#define	SIG_COND_RST	(1<<0)	//bit 0 do registrador USER_CTRL


//Prot�tipos de fun��es de acesso ao sensor MPU-6500
void SPI2_Init(void);												//inicializa��o da interface SPI2
void MPU6500_Config(void);											//configura��o do sensor
void MPU6500_Calibration(void);
uint8_t Read_Data(uint8_t address);									//leitura de um byte
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data);	//leitura de m�ltiplos bytes
void Write_Data(uint8_t address, uint8_t data);						//escrita de um byte


//Declara��o de fun��es de acesso ao sensor MPU9250
//inicializa��o da interface SPI2
void SPI2_Init(void)
{
	//configurando os pinos PB0 como sa�da CS e (PB13[sck], PB14[miso] e PB15[mosi]) da interface SPI2 no modo de fun��o alternativa
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;								//habilita o clock do GPIOB
	GPIOB->MODER |= (0b10 << 30) | (0b10 << 28) | (0b10 << 26) ;		//pinos PB13, PB14 e PB15 como fun��o alternativa
	GPIOB->AFR[1] |= (0b0101 << 28) | (0b0101 << 24) | (0b0101 << 20);	//fun��o alternativa 7 (SPI2)
	GPIOB->ODR |= 1;													//pino PB0 inicialmente em n�vel alto
	GPIOB->MODER |= 0b01;												//pino PB0 como sa�da digital (CS)

	//configurando a interface SPI2
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;			//habilita o clock da interface SPI2
	//pino SS controlado por software, baud rate em 656.25 kHz, configura o modo master e habilita a interface
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (SPI_CR1_BR_2 | SPI_CR1_BR_0) | SPI_CR1_MSTR | SPI_CR1_SPE;
}

//Configura��o inicial do sensor
void MPU6500_Config(void)
{
	HAL_Delay(100);							//aguarda start-up time

	//Configurando o sensor MPU-6500
	Write_Data(PWR_MGMT_1, H_RESET);					//reset do MPU-6500
	HAL_Delay(100);										//aguarda 100ms
	Write_Data(SIGNAL_PATH_RESET, 0b00000111);			//reset dos filtros digitais
	HAL_Delay(100);										//aguarda 100ms
	Write_Data(USER_CTRL, (I2C_IF_DIS|SIG_COND_RST));	//desabilita a interface I2C

	Write_Data(PWR_MGMT_1, 0b00000001);	//seleciona a melhor fonte de clock dispon�vel (PLL do girosc�pio)
	HAL_Delay(100);						//aguarda 100ms ap�s mudar a fonte de clock

	Write_Data(CONFIG, 0b00000010);			//par�metros de amostragem e filtragem (Fs=1kHz, gyro=92Hz, temp=98Hz)
	Write_Data(SMPLRT_DIV, 4);				//sample rate [1kHz/(1+4) = 200 sps]
	Write_Data(GYRO_CONFIG, 0b00001000);	//fundo de escala do girosc�pio (�500�/s)
	Write_Data(ACCEL_CONFIG, 0b00001000);	//fundo de escala do aceler�metro (�4g)
	Write_Data(ACCEL_CONFIG2, 0b00000010);	//par�metros do LPF (Fs=1kHz, accel= 99Hz)
}

//leitura de um byte
uint8_t Read_Data(uint8_t address)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1<<7));			//envia o endere�o com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;							//l� o dummy byte do sensor

	SPI2->DR = 0xFF;						//envia um dummy byte para o sensor
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte de dado ser recebido
	uint8_t RX = SPI2->DR;					//l� o byte de dado do sensor
	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)

	return RX;								//retorna o byte lido
}


void MPU6500_Calibration(void)
{
	uint16_t MAX_SAMPLE = 5000;
	uint8_t rawData[14];
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;	//valores crus do aceler�metro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do girosc�pio
	int32_t acc_RAW_ACCEL_X, acc_RAW_ACCEL_Y, acc_RAW_ACCEL_Z;	//valores acumulados do aceler�metro
	int32_t acc_RAW_GYRO_X, acc_RAW_GYRO_Y, acc_RAW_GYRO_Z;		//valores acumulados do girosc�pio
	acc_RAW_ACCEL_X = 0;
	acc_RAW_ACCEL_Y = 0;
	acc_RAW_ACCEL_Z = 0;
	acc_RAW_GYRO_X = 0;
	acc_RAW_GYRO_Y = 0;
	acc_RAW_GYRO_Z = 0;

	for(uint16_t contador = 0; contador < MAX_SAMPLE; ++contador)
	{
		Read_MData(0x3B, 14, rawData);
		RAW_ACCEL_X = ((int16_t)rawData[0] << 8) + (rawData[1]);
		RAW_ACCEL_Y = ((int16_t)rawData[2] << 8) + (rawData[3]);
		RAW_ACCEL_Z = ((int16_t)rawData[4] << 8) + (rawData[5]);
		RAW_GYRO_X = ((int16_t)rawData[8] << 8) + (rawData[9]);
		RAW_GYRO_Y = ((int16_t)rawData[10] << 8) + (rawData[11]);
		RAW_GYRO_Z = ((int16_t)rawData[12] << 8) + (rawData[13]);

		acc_RAW_ACCEL_X += RAW_ACCEL_X;
		acc_RAW_ACCEL_Y += RAW_ACCEL_Y;
		acc_RAW_ACCEL_Z += RAW_ACCEL_Z;
		acc_RAW_GYRO_X += RAW_GYRO_X;
		acc_RAW_GYRO_Y += RAW_GYRO_Y;
		acc_RAW_GYRO_Z += RAW_GYRO_Z;

		HAL_Delay(5);
	}

	printf("Valores de OFFSET\n\n");
	printf("const int RAW_ACCEL_X_OFFSET = %li;\n", -(acc_RAW_ACCEL_X/MAX_SAMPLE));
	printf("const int RAW_ACCEL_Y_OFFSET = %li;\n", -(acc_RAW_ACCEL_Y/MAX_SAMPLE));
	printf("const int RAW_ACCEL_Z_OFFSET = %li;\n", (8192 - acc_RAW_ACCEL_Z/MAX_SAMPLE));
	printf("const int RAW_GYRO_X_OFFSET = %li;\n", -(acc_RAW_GYRO_X/MAX_SAMPLE));
	printf("const int RAW_GYRO_Y_OFFSET = %li;\n", -(acc_RAW_GYRO_Y/MAX_SAMPLE));
	printf("const int RAW_GYRO_Z_OFFSET = %li;\n", -(acc_RAW_GYRO_Z/MAX_SAMPLE));


	while(1);
}



//leitura de m�ltiplos bytes
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1<<7));			//envia o endere�o com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;							//l� o dummy byte do sensor

	uint8_t contador = 0;
	while(Nbytes)
	{
		SPI2->DR = 0xFF;					//envia um dummy byte para o sensor
		while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o byte de dado ser recebido
		Data[contador] = SPI2->DR;			//l� o byte de dado do sensor
		++contador;
		--Nbytes;
	}

	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
}

//escrita de um byte
void Write_Data(uint8_t address, uint8_t data)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address & ~(1<<7));			//envia o endere�o com o bit 7 resetado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;							//l� o dummy byte do sensor

	SPI2->DR = data;						//envia o byte de dado para o sensor
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte de dado ser recebido
	(void)SPI2->DR;							//l� o dummy byte do sensor
	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
	HAL_Delay(1);							//delay entre escritas
}


#endif /* MPU6500_SPI_H_ */
