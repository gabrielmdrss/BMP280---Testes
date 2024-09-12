/*
 * DEFINES_SAFT.h
 *
 *  Created on: Jul 16, 2024
 *      Author: Esivaldo
 */

#ifndef SRC_DEFINES_SAFT_H_
#define SRC_DEFINES_SAFT_H_


//Definições globais do SAF-T
#define	TRUE		1
#define	FALSE		0
#define N_SAMPLES	20				//número de amostras para cálculo da média das leituras do sensor inercial
#define LIMITE_ACCEL 0.1				//limiar de aceleração usado na detecção de movimento
#define LIMITE_GYRO 40				//limiar de giroscõpio usado na detecção de movimento
#define AD_BUFFER_SIZE 5000		//quantidade de amostras do buffer dos conversores AD
#define obter_offset 0					//flag para captura dos offsets dos conversores AD
#define QUANT_ENVIO 5				//número de amostras para calculo da média das leituras do conversor AD
#define VP_3PH1 388.9087			//tensão de pico das fases do sistema
#define SAMPLE_RATE (100) // replace this with actual sample rate

//Variáveis globais
const uint8_t Tx_Data[15] = {(0x3B | (1<<7)), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	//array para envio dos dados para o sensor
const float b[5] = {0.43284664, -1.73138658, 2.59707987, -1.73138658, 0.43284664};
const float a[5] = {1.0, -2.36951301, 2.31398841, -1.05466541, 0.18737949};
float amostras_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float amostras_y[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_y[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float amostras_z[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_z[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float passa_alta_butterworth(float new_input, float *x, float *y);
float ACCEL_MAG;
float accel_Debug=0;

uint8_t Rx_Data[15];	//array para recebimento dos dados do sensor
uint8_t Sensor_Data_Ready = FALSE;				//flag que indica o estado dos dados do sensor após o DMA
int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;	//valores crus do acelerômetro
int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do giroscópio
int16_t RAW_TEMP;								//valor cru da temperatura
int32_t ACC_RAW_TEMP;										//valores acumulados da temperatura
uint8_t sample_counter = 0;	//contador de amostras acumuladas para cálculo da média de leituras do sensor
float ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z, temperatura;
float ACCEL_FILTERED_X, ACCEL_FILTERED_Y, ACCEL_FILTERED_Z;
float stationary;
//float ULTIMA_ACCEL_FILT;
float VELOC_FILTERED = 0;
float ULTIMA_VELOC_FILT;
float POS_FILTERED;
float ULTIMA_POS_FILT;
float sampleperiod = 1.0/100.0;
float filtro_complementar;
float ultimo_filtro;
uint16_t estado_mobilidade;
uint16_t buffer_1_fase_a[AD_BUFFER_SIZE];
uint16_t buffer_2_fase_a[AD_BUFFER_SIZE];
uint16_t buffer_1_fase_b[AD_BUFFER_SIZE];
uint16_t buffer_2_fase_b[AD_BUFFER_SIZE];
uint16_t buffer_1_fase_c[AD_BUFFER_SIZE];
uint16_t buffer_2_fase_c[AD_BUFFER_SIZE];
uint8_t buffer_cheio=0;
float media_fase_a=0;
float media_fase_b=0;
float media_fase_c=0;
uint8_t controle_envio=0;
double ACCEL=0;
double g;

uint8_t buffer[10];
uint8_t controle_rec =0;
//Variável com o status do transformador

#endif /* SRC_DEFINES_SAFT_H_ */
