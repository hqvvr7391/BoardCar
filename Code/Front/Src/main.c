/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "maxim-max11270.h"
#include "stdio.h"
#include "MPU9250.h"
#include "Filters.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

////																		//MPU_SelectTypeDef hmpu1;


//uint8_t c=0x55;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

																											/*static void MPU1_Init(void); ///////////////////////////////////////MPU

																											int fputc(int ch, FILE *f)
																											{
																												uint8_t temp[1] = {ch};
																												HAL_UART_Transmit(&huart3, temp, 1,2);
																												return(ch);
																											}
																											*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
		uint8_t R_buffer_1[3],R_buffer_2[3],R_buffer_3[1]={0},Conf=0x00;
	  uint32_t Reciver_1=0, Reciver_2=0;
																															/* uint32_t tickstart = 0;
																																uint16_t i=0;
																																float response[6] = {0,0,0,0,0,0};
																																uint8_t _response[6];
																																float deg;
																																float dt=0.001f;
																																float a = 0.96f;*/
																																
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	RSTB_HIGH_1;
	SYNC_LOW_1;
	
	RSTB_HIGH_2;
	SYNC_LOW_2;
	
	Conf=CONTSC|SCYCLE|FORMAT|U;
	Max11270_writeReg8_1(CTRL1,Conf);
	Conf=PGAEN|MAX11270_GAIN1;
	Max11270_writeReg8_1(CTRL2,Conf);
	
	Conf=CONTSC|SCYCLE|FORMAT|U;
	Max11270_writeReg8_2(CTRL1,Conf);
	Conf=PGAEN|MAX11270_GAIN4;
	Max11270_writeReg8_2(CTRL2,Conf);
  /* Initialize all user configured peripherals */
																																					//  MPU1_Init();
																																						
																																						
																																						/* Initialize all user configured interrupts */
																																						//HAL_TIM_Base_Start_IT(&htim2);
																																					//	HAL_UART_Receive_IT(&huart3, &c, 1);
																																						//HAL_SPI_TransmitReceive_IT(&hspi1, IT_master_tx, IT_master_rx, 1);
																																						
																																						
																																						/* user code begin */
																																						
																																						//tickstart = HAL_GetTick();


																																							
																																					//	printf("\nHello \r\n");
																																						//MPU9250_deselect(&hmpu1);

																																						
																																						//MPU9250_init(&hmpu1);
																																						
																																						
																																						

																																						
																																					 //MPU9250_read_allReg(&hmpu1);
																																						
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	  
		
		// Max11270_readReg8_2(CTRL1,R_buffer_2);		//-----register status read
		// Max11270_readReg8_2(CTRL2,R_buffer_2+1);
		
		//HAL_UART_Receive(&huart1,R_buffer_3,1,1);
		
	//	if(R_buffer_3[0]==0x01)  //waiting until rear cycle
	//{	  
		
		Max11270_CovCmd_1(MAX11270_RATE1000); //conversion start
		Max11270_DataRead_1(R_buffer_1);
	
		Max11270_CovCmd_2(MAX11270_RATE1000);
		Max11270_DataRead_2(R_buffer_2);
		
		HAL_UART_Transmit(&huart1,R_buffer_2,3,10); //transmit gauge data to rear
		R_buffer_3[0]=0x00;
		
	//}
																																			
		
																																			/*MPU9250_calcangle(&hmpu1);
																																			hmpu1.angle[0] = Complemetary(C_Alpha, hmpu1.unfiltered_data[0], hmpu1.unfiltered_data[3]);
																																			hmpu1.angle[1] = Complemetary(C_Alpha, hmpu1.unfiltered_data[1], hmpu1.unfiltered_data[4]);
																																			hmpu1.angle[2] = Complemetary(C_Alpha, hmpu1.unfiltered_data[2], hmpu1.unfiltered_data[5]);
																																			
																																			for(i=0; i<3; i++) {
																																				printf("%lf \t", hmpu1.unfiltered_data[i]);
																																			}
																																			printf("\r\n");
																																			
																																			
																																		MPU9250_read_all(&hmpu1);

																																		MPU9250_read_acc(&hmpu1);
																																		MPU9250_read_gyro(&hmpu1);
																																		MPU9250_read_mag(&hmpu1);


																																		printf("GX : %4.4f \t", hmpu1.gyro_data[0]);
																																		printf("GY : %4.4f \t", hmpu1.gyro_data[1]);
																																		printf("GZ : %4.4f \t\r\n", hmpu1.gyro_data[2]);


																																		printf("AX : %4.4f \t", hmpu1.accel_data[0]);
																																		printf("AY : %4.4f \t", hmpu1.accel_data[1]);
																																		printf("AZ : %4.4f \t\r\n", hmpu1.accel_data[2]);

																																		printf("MX : %4.4f \t", hmpu1.mag_data[0]);
																																		printf("MY : %4.4f \t", hmpu1.mag_data[1]);
																																		printf("MZ : %4.4f \t\r\n", hmpu1.mag_data[2]);

																																		printf("\r\n \r\n");	
																																		

																																		HAL_Delay(100);*/
	  
	  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC1   ------> ETH_MDC
     PA7   ------> FMC_SDNWE
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

																																			/*static void MPU1_Init(void)
																																			{
																																				hmpu1.hspi = hspi1;
																																				hmpu1.GPIOx = SPI1_NSS_GPIO_Port;
																																				hmpu1.GPIO_PIN = SPI1_NSS_Pin;
																																				hmpu1.my_low_pass_filter = BITS_DLPF_CFG_188HZ;
																																				hmpu1.my_low_pass_filter_acc = BITS_DLPF_CFG_188HZ;
																																				
																																				hmpu1.dt = 0.01f;
																																				hmpu1.angle[0] = 0;
																																				hmpu1.angle[1] = 0;
																																				hmpu1.angle[2] = 0;
																																			}


																																			void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
																																			{
																																				if(huart->Instance == huart3.Instance){
																																					HAL_UART_Receive_IT(&huart3, &c, 1);
																																					
																																					HAL_SPI_Transmit(&hspi1, IT_master_tx, 2, 1);
																																					HAL_SPI_Receive_IT(&hspi1, IT_master_rx,2);

																																				}
																																			}

																																			void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
																																			{
																																				if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
																																				{
																																					
																																				}
																																				
																																			}

																																			void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
																																			{
																																				HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
																																			}

																																				*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
