/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "rtc.h"
#include "gpio.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void lcd_comando (int comando)//feita pelo Benfica. fazer as demais.
{
	HAL_GPIO_WritePin(GPIOA,LCD_RS_Pin,0); //ou 1<<9 o RS_Pin já é um define criado pelo Cube.
	
	if ((comando & 0x80)== 0x80)
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,0);
	
	if ((comando & 0x40)==0x40)
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,0);
	
	if ((comando & 0x20)==0x20)
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,0);
	
	if ((comando & 0x10)==0x10)
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,0); // concluída primeira metade do comando. agora o enable:
	
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,1);
	HAL_Delay(1); // 1ms
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,0);
	HAL_Delay(1); // 1ms
	
	
		if ((comando & 0x08)== 0x08)
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,0);
	
	if ((comando & 0x04)==0x04)
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,0);
	
	if ((comando & 0x02)==0x02)
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,0);
	
	if ((comando & 0x01)==0x01)
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,0); 
	
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,1);
	HAL_Delay(1); // 1ms
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,0);
	HAL_Delay(1); // 1ms
}
void lcd_dado (int dado)
{
		HAL_GPIO_WritePin(GPIOA,LCD_RS_Pin,1); //ou 1<<9 o RS_Pin já é um define criado pelo Cube.
	
	if ((dado & 0x80)== 0x80)
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,0);
	
	if ((dado & 0x40)==0x40)
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,0);
	
	if ((dado & 0x20)==0x20)
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,0);
	
	if ((dado & 0x10)==0x10)
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,0);
	
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,1);
	HAL_Delay(1); // 1ms
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,0);
	HAL_Delay(1); // 1ms
	
	
		if ((dado & 0x08)== 0x08)
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D7_Pin,0);
	
	if ((dado & 0x04)==0x04)
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D6_Pin,0);
	
	if ((dado & 0x02)==0x02)
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D5_Pin,0);
	
	if ((dado & 0x01)==0x01)
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,1);
	else
		HAL_GPIO_WritePin(GPIOA,LCD_D4_Pin,0);
	
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,1);
	HAL_Delay(1); // 1ms
	HAL_GPIO_WritePin(GPIOB,LCD_EN_Pin,0);
	HAL_Delay(1); // 1ms
}
void lcd_init (void)
{
	lcd_comando(0x33);
	lcd_comando(0x32);
	lcd_comando(0x28);
	lcd_comando(0x0e);
	lcd_comando(0x06);
	lcd_comando(0x01);
	HAL_Delay(100);
	
}  
void lcd_goto (int linha, int coluna)
{
		if (linha ==0) lcd_comando(0x80+coluna);
		if (linha ==1) lcd_comando(0xC0+coluna);
}

//void lcd_string (char vetor[])//Formato Prof. Benfica
//{

//	int i=0, x=0;
//	x = srtlen(vetor);
//	for (i=0;i<x;i++)
//	{
//		lcd_dado(vetor[i]);
//	}
//	
//}

void lcd_string (char *str)// Formato Prof. Julio
{
while (*str) lcd_dado(*str ++);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

//	uint8_t vetor_hora[30];
//	uint8_t vetor_data[30];
//	
//	RTC_TimeTypeDef sTime; // estrutura que receberá a hora
//	RTC_DateTypeDef sDate; // estrutura que receberá a data
//	
//	sTime.Hours = 10; //valor da hora
//	sTime.Minutes = 54; //valor dos minutos
//	sTime.Seconds = 28; //valor dos segundos
//	
//	HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);
//	
//	sDate.Date = 20; //(Dia do mês de 1 a 31)
//	sDate.Month = RTC_MONTH_SEPTEMBER; //(Mês de 1 a 12)
//	sDate.WeekDay = RTC_WEEKDAY_TUESDAY; //(Dia da semana de 1 a 7)
//	sDate.Year = 16; //(Ano de 0 a 99)
//	
//	HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);
//	
//	while(1)
//	{
//			HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
//			HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
//			sprintf(vetor_hora,"%02d:%02d:%02d\n\r",sTime.Hours,sTime.Minutes,sTime.Seconds);
//			sprintf(vetor_data,"%02d/%02d/20%02d\n\r", sDate.Date, sDate.Month, sDate.Month);
//	}
	
	  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
		
		lcd_init();
		lcd_string("d/m/a HH:MM");
		lcd_goto (1,0);
	
while (1)
		{	

		int i;
					
					
			if (HAL_GPIO_ReadPin(GPIOC,1<<9)==0)
			i=i+1;
			if(i==4) i=0;
		
		
		if (i==0)
			lcd_string("MODO -> B1");
		if (i==1)
			lcd_string("AJUSTE RELOGIO");
		if (i==2)
			lcd_string("MANUAL");
		if (i==3)
			lcd_string("AUTOMATICO");
		
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
