/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#include "i2c-lcd.h"
	#include "stdio.h"
	#define ADE7753_VOLTAGE_RMS 0x17
	#define ADE7753_CURRENT_RMS 0x16
	#define ADE7753_ACTIVE_POWER 0x04
	#define ADE7753_APPARENT_POWER 0x07
	#define ADE7753_VOLTAGE_RMS0S 0X18
	#define MODE 0x09
	uint32_t Vbat_temp=0,i,button_temp,Vtai_temp,Itai_temp,Ptai_temp,Vin_temp,Iin_temp,Vbat_tb, Vtai_tb, Itai_tb;
	float Vbat_f=0,Vtai_f=0, Itai_f=0,Ptai_f=0,Vin_f=0,Iin_f=0;
	uint16_t LCD_Menu=0;
	char Vbat[12]={0}, Vtai[12]={0},Itai[12]={0},Ptai[12]={0},Vin[12]={0}, Iin[12]={0};
	int tx_uart_temp;
	char tx_uart_buffer[9];

	uint8_t rx_buffer[4];
	int k=0;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void doc_vacquy()
{
	uint32_t vacquy_sum100=0;
	for (i=0;i<100;i++)
		{
			vacquy_sum100 = vacquy_sum100+Vbat_temp;
			HAL_Delay(1);
		}
	Vbat_tb=vacquy_sum100/100;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	if (GPIO_Pin == button_mode_Pin)
		{
			button_temp=1;
			LCD_Menu++;
			if (LCD_Menu>2)
				LCD_Menu=0;
			for (int x=500000; x>0; x--);
			__HAL_GPIO_EXTI_CLEAR_IT(button_mode_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		}
}
void LCD_display()
{
	switch(LCD_Menu)
		{
			case 0:
				sprintf(Vbat,"%2.2f",Vbat_f);
				lcd_put_cur(0,1);
				lcd_send_string("DIEN AP ACQUY");
				lcd_put_cur(1,1);
				lcd_send_string("U=");
				lcd_send_string(Vbat);
				lcd_send_string("V");
				if (button_temp==1)
				{
						lcd_send_cmd(0x01);
						HAL_Delay(1);
						button_temp=0;
				}

				break;
			case 1:		
				sprintf(Vtai,"%3.1f",Vtai_f);
				sprintf(Itai,"%2.2f",Itai_f);
				sprintf(Ptai,"%3.1f",Ptai_f);
				lcd_put_cur(0,0);	
				lcd_send_string("OUTPUT");
				lcd_put_cur(0,8);
				lcd_send_string("U=");
				lcd_send_string(Vtai);
				lcd_send_string("V");
				
				lcd_put_cur(1,0);
				lcd_send_string("I=");
				lcd_send_string(Itai);
				lcd_send_string("A");
			
				lcd_put_cur(1,8);
				lcd_send_string("P=");
				lcd_send_string(Ptai);
				lcd_send_string("W");
				
				if (button_temp==1)
				{
						lcd_send_cmd(0x01);
						HAL_Delay(1);
					button_temp=0;
				}
				break;
			case 2:
				lcd_put_cur(0,1);
				lcd_send_string("DAU RA GIO");
				if (button_temp==1)
				{
						lcd_send_cmd(0x01);
						HAL_Delay(1);
					button_temp=0;
				}
				break;
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//xu ly ngat gui uart, khong xu ly
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
	for (int i=0;i<4;i++)
		{
			if (rx_buffer[i]=='a')
				{
					for (int j=0;j<4;j++)
					{
						if (rx_buffer[j]=='1')
						HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,0);
						else if (rx_buffer[j]=='0')
						HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
					}
				}
			if (rx_buffer[i]=='b')
				{
					for (int j=0;j<4;j++)
					{
						if (rx_buffer[j]=='1')
						HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,0);
						else if (rx_buffer[j]=='0')
						HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
					}
				}
			if (rx_buffer[i]=='c')
				{
					for (int j=0;j<4;j++)
					{
						if (rx_buffer[j]=='1')
						HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,0);
						else if (rx_buffer[j]=='0')
						HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
					}
				}
			if (rx_buffer[i]=='d')
				{
					for (int j=0;j<4;j++)
					{
						if (rx_buffer[j]=='1')
						HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,0);
						else if (rx_buffer[j]=='0')
						HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,1);
					}
				}
		}

		HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 4);
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
		{
			switch (k)
				{
					case 0:
						sprintf(tx_uart_buffer,"VBAT%05d",(int)(Vbat_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k++;
						break;
					case 1:
						sprintf(tx_uart_buffer,"VTAI%05d",(int)(Vtai_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k++;
						break;	
					case 2:
						sprintf(tx_uart_buffer,"ITAI%05d",(int)(Itai_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k++;
						break;	
					case 3:
						sprintf(tx_uart_buffer,"PTAI%05d",(int)(Ptai_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k++;
						break;

					case 4:
						sprintf(tx_uart_buffer,"VIN0%05d",(int)(Vin_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k++;
						break;
					case 5:
						sprintf(tx_uart_buffer,"IIN0%05d",(int)(Iin_f*100));
						HAL_UART_Transmit_IT(&huart1,(uint8_t*)tx_uart_buffer,9);
						k=0;
						break;
				}
		}
}
		//READ_ADE
uint32_t Read_ADE24(uint8_t add)
	{
		uint8_t m[3]={0x00, 0x00, 0x00};
		uint8_t n[3]={0x00, 0x00, 0x00};
		HAL_GPIO_WritePin(GPIOA ,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1,&add,1,10);
		HAL_SPI_TransmitReceive(&hspi1,m,n,3,10);
		HAL_GPIO_WritePin(GPIOA ,GPIO_PIN_4,GPIO_PIN_SET);
		return (n[0]<<16 | n[1]<<8 | n[2]);	
	}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	HAL_ADC_Start_DMA(&hadc1,&Vbat_temp,1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) rx_buffer,4);
	HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			uint32_t Vtai_sum=0, Itai_sum=0;
				for (int k=0;k<20;k++)
		{
			
			Vtai_temp= Read_ADE24(ADE7753_VOLTAGE_RMS); 
			Vtai_sum=Vtai_sum+Vtai_temp;
			}
			Vtai_tb= Vtai_sum/20;
		for (int k=0;k<20;k++)
			{
			Itai_temp= Read_ADE24(ADE7753_CURRENT_RMS); 
			Itai_sum=Itai_sum+Itai_temp;
			}
			Itai_tb=Itai_sum/20;
			 // Xu ly du lieu
	 Vtai_f= (Vtai_tb-50000) * 114 / 264208;
	 Itai_f=(Itai_tb-1480) * 0.000030221625;
			doc_vacquy();	
			
			Vbat_f=((float)Vbat_temp/4070)*3.32*(9800+970)/970;

			if (Vbat_f>14.10)
				{
					HAL_Delay(100);
					if (Vbat_f>14.10)
						{
							HAL_GPIO_WritePin(GPIOB,stop_charging_Pin,1);
							HAL_GPIO_WritePin(GPIOB,stop_discharging_Pin,1);
						}
				}
			if (Vbat_f<=14.1 && Vbat_f>=6.0)
				{
					HAL_Delay(100);
					if (Vbat_f<=14.1 && Vbat_f>=6.0)
						{
							HAL_GPIO_WritePin(GPIOB,stop_charging_Pin,1);
							HAL_GPIO_WritePin(GPIOB,stop_discharging_Pin,1);
						}
				}
			if (Vbat_f<6.0)
				{
					HAL_Delay(100);
					if (Vbat_f<6.0)
						{
							HAL_GPIO_WritePin(GPIOB,stop_charging_Pin,1);
							HAL_GPIO_WritePin(GPIOB,stop_discharging_Pin,1);
						}
				}


			LCD_display();
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|ADE2_SS_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, stop_discharging_Pin|stop_charging_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT1_Pin|OUT2_Pin|OUT3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 ADE2_SS_Pin CS_Pin
                           OUT1_Pin OUT2_Pin OUT3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|ADE2_SS_Pin|CS_Pin
                          |OUT1_Pin|OUT2_Pin|OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : stop_discharging_Pin stop_charging_Pin OUT4_Pin */
  GPIO_InitStruct.Pin = stop_discharging_Pin|stop_charging_Pin|OUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : button_mode_Pin */
  GPIO_InitStruct.Pin = button_mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(button_mode_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
