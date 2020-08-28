/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#define ENDERECO 0
#define FUNCAO 1
#define ADR_ALTO 2
#define ADR_BAIXO 3
#define NUM_ESTADOS_ALTO 4
#define NUM_ESTADOS_BAIXO 5
#define CRC_ALTO 6
#define CRC_BAIXO 7
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t dado_serial;
uint8_t ESTADO_ATUAL=ENDERECO;
uint8_t funcao_modbus;
uint8_t endereco_periferico;
uint8_t slave;
uint8_t num_estados;
uint8_t crcalto;
uint8_t crcbaixo;
uint8_t erro[30];
uint8_t msg[30];
uint8_t msg1[30];
uint8_t msg2[30];
uint8_t msg3[30];
uint8_t msg4[30];
uint8_t flag_protocol=0;
int i = 0;
unsigned char buffer[30];
unsigned short variavel;
int aux_tensao;
float tensao;
ADC_ChannelConfTypeDef sConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void READ_HOLDING_REGISTERS(void)
{
//	float tensao;
//	int x;
//	ADC_ChannelConfTypeDef hor;
//	
//	HAL_ADC_Start(&hadc);
//	hor.Channel = ADC_CHANNEL_0;
//	HAL_ADC_ConfigChannel(&hadc, &hor);
//	HAL_ADC_Start(&hadc);
//	HAL_ADC_PollForConversion(&hadc,ADC_CHANNEL_0);
//	x = HAL_ADC_GetValue(&hadc); //LEITURA DO CANAL 0
//	tensao = (x/4096)*3.3;
//	
//	sprintf((char*)msg,"TENSÃO: %.2f\n\r",tensao);
//	HAL_UART_Transmit(&huart2, msg, strlen((char*)msg),500);
//	HAL_Delay(100);
}
void READ_COIL_STATUS(void)
{
	char state[5];
	int x;
	int y;
	int z;
	int w;
	x = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
	y = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
	z = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	w = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
	

sprintf((char*)msg1,"CHAVE [1]: %d\n\r",x);
HAL_UART_Transmit(&huart2, msg1, strlen((char*)msg1),500);
HAL_Delay(100);
	
sprintf((char*)msg2,"CHAVE [2]: %d\n\r",y);
HAL_UART_Transmit(&huart2, msg2, strlen((char*)msg2),500);
HAL_Delay(100);
	
sprintf((char*)msg3,"CHAVE [3]: %d\n\r",z);
HAL_UART_Transmit(&huart2, msg3, strlen((char*)msg3),500);
HAL_Delay(100);

sprintf((char*)msg4,"CHAVE [4]: %d\n\r",w);
HAL_UART_Transmit(&huart2, msg4, strlen((char*)msg4),500);
HAL_Delay(100);
	
}
void FORCE_MULTIPLE_COILS(void)
{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
		HAL_Delay(100);
}
	
int16_t le_umidade(void)
{
  uint16_t H0_rH_x2;
	uint16_t H1_rH_x2;
	int16_t H0_T0_OUT;
	int16_t H1_T0_OUT;
	int16_t H_OUT;
	uint8_t temp[2];
	uint8_t x;
	int16_t umidade;
	
	x=0x81;
	HAL_I2C_Mem_Write(&hi2c1,0xbf,0x20,I2C_MEMADD_SIZE_8BIT,&x,1,500);
		
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x30, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x31, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	H0_rH_x2=temp[0]/2;
	H1_rH_x2=temp[1]/2;
	
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x36, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x37, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	H0_T0_OUT = ((uint16_t)(temp[1]<<8) | (uint16_t)temp[0]);
	HAL_Delay(100);
	
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3A, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3B, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	H1_T0_OUT = ((uint16_t)temp[1]<<8) | (uint16_t)temp[0];
	HAL_Delay(100);
	
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x28, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x29, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	H_OUT = ((uint16_t)temp[1]<<8) | (uint16_t)temp[0];
	
	umidade = (((H1_rH_x2 - H0_rH_x2)*(H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT)) + H0_rH_x2;
	
	return umidade;
}
	
int16_t le_temperatura(void)
{
int16_t T_OUT;
	uint8_t T0_degC_x8;
	uint8_t T1_degC_x8;
	uint8_t T0_degC;
	uint8_t T1_degC;
	int16_t T0_OUT;
	int16_t T1_OUT;
	uint8_t temp[2], x, tmp;
	int16_t temperatura;
	
	x=0x81;
	HAL_I2C_Mem_Write(&hi2c1,0xbf,0x20,I2C_MEMADD_SIZE_8BIT,&x,1,500);
		
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x32, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x33, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	T0_degC_x8 = temp[0];
	T1_degC_x8 = temp[1];
	
	T0_degC = T0_degC_x8/8;
	T1_degC = T1_degC_x8/8;
	
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x35, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 500);
	HAL_Delay(100);
	T1_degC = ((uint16_t)tmp<<8)|(uint16_t)T1_degC;
	T0_degC = ((uint16_t)tmp<<8)|(uint16_t)T0_degC;
	
  HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3C, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3D, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
  T0_OUT = ((uint16_t)(temp[1]<<8) | (uint16_t)temp[0]);
	HAL_Delay(100);
		
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3E, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x3F, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	T1_OUT = ((uint16_t)temp[1]<<8) | (uint16_t)temp[0];
	HAL_Delay(100);	
	
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x2A, I2C_MEMADD_SIZE_8BIT, &temp[0], 1, 500);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xbf, 0x2B, I2C_MEMADD_SIZE_8BIT, &temp[1], 1, 500);
	HAL_Delay(100);
	T_OUT = ((uint16_t)temp[1]<<8) | (uint16_t)temp[0];
	
	//temperatura = (((T1_degC_x8 - T0_degC_x8)*(T_OUT - T0_OUT))/(T1_OUT - T0_OUT)) + T0_degC_x8;
	temperatura = (((T1_degC-T0_degC)*(T_OUT - T0_OUT))/(T1_OUT - T0_OUT) + T0_degC)+10;
	return temperatura;
}
static unsigned char auchCRCHi[] = 
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

static char auchCRCLo[] = 
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

unsigned short CRC16 (unsigned char *puchMsg, unsigned short usDataLen) 
{
unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
unsigned uIndex ; /* will index into CRC lookup table */
while (usDataLen--) /* pass through message buffer */
{
uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
uchCRCHi = auchCRCLo[uIndex] ;
}
return (uchCRCHi << 8 | uchCRCLo);
}

void maquina_modbus(uint8_t dado)
{
switch(ESTADO_ATUAL)
{
	case ENDERECO:
		if(dado==0x01)//é o endereço correto
		{
			slave = dado;
			ESTADO_ATUAL = FUNCAO;
			buffer[i] = slave;
			i++;
		}
		else 
		{
			ESTADO_ATUAL = ENDERECO;//reset
			sprintf((char*)erro,"ERRO: SLAVE ADRESS INCORRETO! \n\r");
			HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
			//informa que o endereço do slave está incorreto
		}
	break;	
		
	case FUNCAO:
		if(dado==0x02)//Testa se a função enviada é a padrão
			{
			funcao_modbus = dado;
			ESTADO_ATUAL = ADR_ALTO;
			buffer[i]=funcao_modbus;
			i++;
		}
		else if(dado==0x03)
			{
			funcao_modbus = dado;
			ESTADO_ATUAL = ADR_ALTO;
			buffer[i]=funcao_modbus;
			i++;
			}
		else if(dado==0x0F)
			{
			funcao_modbus = dado;
			ESTADO_ATUAL = ADR_ALTO;
			buffer[i]=funcao_modbus;
			i++;				
			}
		else
		{
			ESTADO_ATUAL = ENDERECO;//reset
			sprintf((char*)erro,"ERRO: FUNCAO INCORRETA! \n\r");
			HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
			//Enviar mensagem de erro (pela porta serial) de comando não especificado
			// resetar a máquina, começar novo frame de comunicação.
		}
		break;	
		
		case ADR_ALTO:
			if(dado == 0x00)
			{
				buffer[i]=dado;
				i++;
				ESTADO_ATUAL = ADR_BAIXO;
			}
		break;	
		
		case ADR_BAIXO:
				if((dado>=0x01)&&(dado<=0x05))
				{
					endereco_periferico=dado;
					buffer[i]=endereco_periferico;
					i++;
					ESTADO_ATUAL = NUM_ESTADOS_ALTO;
				}
				else
				{
					ESTADO_ATUAL = ENDERECO;//reset
					sprintf((char*)erro,"ERRO: ENDERECO(BYTE BAIXO) INCORRETO! \n\r");
					HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
			//Enviar mensagem de erro (pela porta serial) de comando não especificado
			// resetar a máquina, começar novo frame de comunicação.
				}
		break;
		
		case  NUM_ESTADOS_ALTO:
			if(dado==0X00)
			{
				buffer[i]=dado;
				i++;
				ESTADO_ATUAL = NUM_ESTADOS_BAIXO;
			}
			else
			{
				ESTADO_ATUAL = ENDERECO;//reset
				sprintf((char*)erro,"ERRO: NUM_ESTADOS(BYTE ALTO) INCORRETO! \n\r");
				HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
			//Enviar mensagem de erro (pela porta serial) de comando não especificado
			// resetar a máquina, começar novo frame de comunicação.
			}
		break;

		case  NUM_ESTADOS_BAIXO:
			if((dado==0x01)||(dado==0x02)||(dado==0x03)||(dado==0x04))
			{
				num_estados = dado;
				buffer[i] = num_estados;
				i++;
				ESTADO_ATUAL = CRC_ALTO;
			}
			else
			{
				ESTADO_ATUAL = ENDERECO;//reset
				sprintf((char*)erro,"ERRO: NUM_ESTADOS(BYTE BAIXO) INCORRETO! \n\r");
				HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
			//Enviar mensagem de erro (pela porta serial) de comando não especificado
			// resetar a máquina, começar novo frame de comunicação.
			}
		break;
		
		case  CRC_ALTO:
				crcalto = dado;
				ESTADO_ATUAL = CRC_BAIXO;
					
		break;
			
		case  CRC_BAIXO:
				crcbaixo = dado;
				ESTADO_ATUAL = CRC_BAIXO;
				flag_protocol = 1;
		break;
		}
}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	maquina_modbus(dado_serial);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t vetor[30];
	uint8_t vetor_temperatura[30];
	uint8_t vetor_umidade[30];
	uint8_t vetor_tensao[30];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2,&dado_serial,1);
	HAL_ADC_Start(&hadc1);
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(flag_protocol==1)
		{
			variavel = CRC16(buffer,i);
			if((crcalto==(variavel>>8))&&(crcbaixo==(variavel&0xFF)))//testa CRC recebido com o calculado
			{
				if(funcao_modbus==0x03)//READ_HOLDING_REGISTERS()
				{
					if(endereco_periferico==0x03)
					{
						sConfig.Channel = ADC_CHANNEL_0;
						HAL_ADC_ConfigChannel(&hadc1, &sConfig);
						HAL_ADC_Start(&hadc1);
						HAL_ADC_PollForConversion(&hadc1,ADC_CHANNEL_0);
						aux_tensao = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 0
						tensao = ((float)(aux_tensao)*5)/4096;
						
						sprintf((char*)vetor_tensao,"TENSAO = %.2f V \n\r", tensao);
						HAL_UART_Transmit(&huart2,vetor_tensao,strlen((char*)vetor_tensao),500);
						HAL_Delay(500);
					}
					else if(endereco_periferico==0x04)
					{
						sprintf((char*)vetor_temperatura,"TEMPERATURA = %d °C \n\r", le_temperatura());
						HAL_UART_Transmit(&huart2,vetor_temperatura,strlen((char*)vetor_temperatura),500);
						HAL_Delay(500);
					}
					else if(endereco_periferico==0x05)
					{
						sprintf((char*)vetor_umidade,"UMIDADE = %d %%ur \n\r", le_umidade());
						HAL_UART_Transmit(&huart2,vetor_umidade,strlen((char*)vetor_umidade),500);
						HAL_Delay(500);	
					}
					else
					{
						sprintf((char*)erro,"ERRO 0x02: Endereço de registro não válido! \n\r");
						HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
						HAL_Delay(500);
					}
				}
				else if(funcao_modbus==0x0F)//FORCE_MULTIPLE_COILS()
				{
					if(endereco_periferico==0x01)
					{
						if(num_estados==0x01)
						{
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
						}
						else if(num_estados==0x02)
						{
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
						}
						else if(num_estados==0x03)
						{
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
						}
						else if(num_estados==0x04)
						{
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
							HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
							HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
						}
						else
						{
							sprintf((char*)erro,"ERRO: Número de endereços não válido! \n\r");
							HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
							HAL_Delay(500);
						}
					}
					else
					{
						sprintf((char*)erro,"ERRO 0x02: Endereço de registro não válido! \n\r");
						HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
						HAL_Delay(500);
					}
				}
				else if(funcao_modbus==0x02)//READ_INPUT_STATUS()
				{
						if(endereco_periferico==0x02)
						{
							READ_COIL_STATUS();
						}
						else
						{
							sprintf((char*)erro,"ERRO 0x02: Endereço de registro não válido! \n\r");
							HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
							HAL_Delay(500);							
						}
				}
				else
				{
						sprintf((char*)erro,"ERRO 0x01: Código de função não válido! \n\r");
						HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
						HAL_Delay(500);
				}
			
			}
			else
			{
				sprintf((char*)erro,"ERRO 0x04: ERRO DE CRC!\n\r");
				HAL_UART_Transmit(&huart2, erro, strlen((char*)erro),500);
				HAL_Delay(500);
				//Enviar mensagem de erro de CRC
			}
			flag_protocol=0;
//			funcao_modbus=0;
//			endereco_periferico=0;
//			slave=0;
//			num_estados=0;
//			crcalto=0;
//			crcbaixo=0;
		}
  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
