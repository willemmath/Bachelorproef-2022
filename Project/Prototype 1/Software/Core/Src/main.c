/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

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
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint16_t rxBuf[8];
uint16_t rxBuf2[8];
uint8_t txBuf[16];

uint8_t mode = 1;
uint16_t counter = 0;

int U1Sample = 0;
int U2Sample = 0;
int U3Sample = 0;
int U4Sample = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPI1_Init(void);
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
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SAI1_Init();
  MX_SPI1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2S_Receive_DMA(&hi2s2, rxBuf, 4);
  HAL_I2S_Receive_DMA(&hi2s3, rxBuf2, 4);
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, txBuf, 4);




  HAL_Delay(100);


  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,(mode == 1));
  HAL_GPIO_WritePin(DAC_RESET_GPIO_Port,DAC_RESET_Pin,1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);

/*

     for(int i = 0; i<16; i++){
  		  if(i == mode-1){
  			  txBuf[i] = 0xff;
  		  }
  		  else{
  			  txBuf[i] = 0b00000000;
  		  }
  	  }

/*



	  for(int i = 0; i<16; i++){
		  if(i == mode-1){
			  txBuf[i] = rxBuf[0] & 0xff;
		  }
		  if(i == mode){
			  txBuf[i] = rxBuf[0] >>8;
		  }
		  else{
			  txBuf[i] = 0b00000000;
		  }

	  }

		*/


	  if(HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin)==1){
		  if(mode == 3){
			  mode = 1;
		  }
		  else{
			  mode++;
		  }

		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,(mode == 1));
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,(mode == 2));
		  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,(mode == 3));

		  while(HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin)==1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 78;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2S
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL;
  PeriphClkInit.I2sClockSelection = RCC_I2SCLKSOURCE_PLL;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_MSBJUSTIFIED, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_RESET_GPIO_Port, DAC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_RESET_Pin */
  GPIO_InitStruct.Pin = DAC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button1_Pin */
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){


/*

	int lSample = (int) (rxBuf2[0]<<16 | rxBuf2[1]);




*/
	if(mode == 1){

		txBuf[3]= (rxBuf2[0]>>8) & 0xFF;
		txBuf[2]=  rxBuf2[0] & 0xFF;
		txBuf[1]= (rxBuf2[1]>>8) & 0xFF;
		txBuf[0]=  rxBuf2[1] & 0xFF;

	}


	if(mode == 2){

		U3Sample = (int) (rxBuf2[0]<<16 | rxBuf2[1]);
		U4Sample = (int) (rxBuf2[2]<<16 | rxBuf2[3]);

		int TXSample =  (U3Sample>>1) + (U4Sample>>1);

		txBuf[3] = (TXSample >> 24)  & 0xFF ;
		txBuf[2] = (TXSample >> 16)  & 0xFF ;
		txBuf[1] = (TXSample >> 8 )  & 0xFF ;
		txBuf[0] = (TXSample )       & 0xFF ;

	}

	if(mode == 3){

		U1Sample = (int) ( rxBuf[0]<<16 | rxBuf[1]);
		U2Sample = (int) ( rxBuf[2]<<16 | rxBuf[3]);
		U3Sample = (int) (rxBuf2[0]<<16 | rxBuf2[1]);
		U4Sample = (int) (rxBuf2[2]<<16 | rxBuf2[3]);

		int TXSample = (U1Sample>>2) + (U2Sample>>2) + (U3Sample>>2) + (U4Sample>>2);

		txBuf[3] = (TXSample >> 24)  & 0xFF ;
		txBuf[2] = (TXSample >> 16)  & 0xFF ;
		txBuf[1] = (TXSample >> 8 )  & 0xFF ;
		txBuf[0] = (TXSample )       & 0xFF ;

	}



	/* deze wel nodig!!
	txBuf[7]= (rxBuf2[2]>>8) & 0xFF;
	txBuf[6]=  rxBuf2[2] & 0xFF;
	txBuf[5]= (rxBuf2[3]>>8) & 0xFF;
	txBuf[4]=  rxBuf2[3] & 0xFF;

	*/

	/*dit is voor de U1 MEMS

	if(mode == 3){

		txBuf[3]= (rxBuf[0]>>8) & 0xFF;
		txBuf[2]=  rxBuf[0] & 0xFF;
		txBuf[1]= (rxBuf[1]>>8) & 0xFF;
		txBuf[0]=  rxBuf[1] & 0xFF;

	}

 */

	  UNUSED(hi2s);
}





void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){



	if(mode == 1){
		txBuf[11]= (rxBuf2[4]>>8) & 0xFF;
		txBuf[10]=  rxBuf2[4] & 0xFF;
		txBuf[9]= (rxBuf2[5]>>8) & 0xFF;
		txBuf[8]=  rxBuf2[5] & 0xFF;
	}

	if (mode == 2){

		U3Sample = (int) (rxBuf2[4]<<16 | rxBuf2[5]);
		U4Sample = (int) (rxBuf2[6]<<16 | rxBuf2[7]);

		int TXSample =  (U3Sample>>1) + (U4Sample>>1);

		txBuf[11] = (TXSample >> 24)  & 0xFF ;
		txBuf[10] = (TXSample >> 16)  & 0xFF ;
		txBuf[9]  = (TXSample >> 8 )  & 0xFF ;
		txBuf[8]  = (TXSample )       & 0xFF ;

	}

	if(mode == 3){

		U1Sample = (int) ( rxBuf[4]<<16 | rxBuf[5]);
		U2Sample = (int) ( rxBuf[6]<<16 | rxBuf[7]);
		U3Sample = (int) (rxBuf2[4]<<16 | rxBuf2[5]);
		U4Sample = (int) (rxBuf2[6]<<16 | rxBuf2[7]);

		int TXSample = (U1Sample>>2) + (U2Sample>>2) + (U3Sample>>2) + (U4Sample>>2);

		txBuf[11] = (TXSample >> 24)  & 0xFF ;
		txBuf[10] = (TXSample >> 16)  & 0xFF ;
		txBuf[9]  = (TXSample >> 8 )  & 0xFF ;
		txBuf[8]  = (TXSample )       & 0xFF ;

	}

	/* deze wel nodig!!
	txBuf[15]= (rxBuf2[6]>>8) & 0xFF;
	txBuf[14]=  rxBuf2[6] & 0xFF;
	txBuf[13]= (rxBuf2[7]>>8) & 0xFF;
	txBuf[12]=  rxBuf2[7] & 0xFF;
    */

	/* dit is voor de U1 MEMS
	if(mode == 3){
		txBuf[11]= (rxBuf[4]>>8) & 0xFF;
		txBuf[10]=  rxBuf[4] & 0xFF;
		txBuf[9]= (rxBuf[5]>>8) & 0xFF;
		txBuf[8]=  rxBuf[5] & 0xFF;
	}
	*/

	UNUSED(hi2s);


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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
