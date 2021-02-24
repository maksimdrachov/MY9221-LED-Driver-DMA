/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <MY9221.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define period_data 90
#define period_clk 90

#define hspd 	0
/* HSPD: Iout Tr/Tf select
 * 0: Iout slow mode
 * 1: Iout fast mode
*/
#define bs 		0
/* BS: Grayscale resolution select
 * 0: 8-bit grayscale application
 * 1: 12-bit grayscale application
 * 2: 14-bit grayscale application
 * 3: 16-bit grayscale application
 */
#define gck		0
/* GCK: Internal oscillator frequency select
 * 0: Original frequency (8.6MHz)
 * 1: Original frequency/2
 * 2: Original frequency/4
 * 3: Original frequency/8
 * 4: Original frequency/16
 * 5: Original frequency/64
 * 6: Original frequency/128
 * 7: Original frequency/256
*/
#define sep		0
/* SEP: Output waveform select
 * 0: PY-PWM output waveform (similar to traditional waveform)
 * 1: APDM output waveform
*/
#define osc		0
/* OSC: Grayscale clock source select
 * 0: Internal oscillator (8.6MHZ) (internal GCK source)
 * 1: External clock from GCKI pin (external GCK source)
*/
#define pol		0
/* POL: Output polarity select
 * 0: work as LED driver
 * 1: work as MY-PWM/APDM generator
*/
#define cntset	0
/* CNTSET: Counter reset select
 * 0: free running mode
 * 1: counter reset mode (only usable when osc = '1')
*/
#define onest	0
/* ONEST: One-shot select
 * 0: frame cycle repeat mode
 * 1: frame cycle one-shot mode (only usable when cntset = '1')
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* USER CODE BEGIN PV */
uint32_t w[] = {
		  0xffffffff, 0x00000000, 0xffffffff, 0x00000000,
		  0xffffffff, 0x00000000, 0xffffffff, 0x00000000};
//int InputCounter = {0};

uint32_t w1 = 0x0000ffff; //Sets each port
uint32_t w2 = 0xffff0000; //Resets each port

uint32_t SetAll = 0x0000ffff; //Sets each port
uint32_t ResetAll = 0xffff0000; //Resets each port

bool CMDArray[16];
bool GrayscaleArray[192];
bool ColorArray[8] = {true,false,false,false,false,false,false,true}; //Make sure length matches selected grayscale!
//int DataArray[208];
uint32_t InputArray[208];
short int InputCounter = 0;
short int SwitchCounter = 1000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void uS_Delay(int);

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
  CMDArray_Init(CMDArray, hspd, bs, gck, sep, osc, pol, cntset, onest);
  Grayscale_Init(GrayscaleArray, bs, ColorArray);
  InputArray_Init(InputArray, CMDArray, GrayscaleArray);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  htim3.Instance->CCR1 = (int)(period_clk/2);
  htim8.Instance->CCR4 = (int)(period_data/2);

  // Set GPIOA6 to TIM output
  GPIOA->MODER |= (1<<13);
  GPIOA->AFR[0] |= (1<<25);
  //GPIOA->OSPEEDR |= (0b10 << 12);

  //Set GPIOC9 to TIM output
  //GPIOC->MODER |= (1<<19);
  //GPIOC->AFR[1] |= (1<<4);

  //Set GPIOC9 to GPIO output
  GPIOC->MODER &= ~(1<<19);
  GPIOC->MODER |= (1<<18);

  //Set DMA
  /*
  DMA2_Stream1->CR &= ~(0b1);		 // Disable stream before config
  DMA2_Stream1->NDTR = 0x1;			 // 1 data item to be transmitted
  DMA2_Stream1->CR |= (0b111 << 25); // Channel 7 selected
  DMA2_Stream1->CR |= (0b10 << 13);	 // word (32-bit) selected (memory)
  DMA2_Stream1->CR |= (0b10 << 11);  // word (32-bit) selected (peripheral)
  DMA2_Stream1->CR |= (0b10 << 6);	 // Data transfer direction

  DMA2_Stream1->PAR = (uint32_t)&w2;	 // Set peripheral address to start w2 array
  DMA2_Stream1->M0AR = 0x40021018;			 // Set memory address to GPIOE->BSRR

  DMA2_Stream1->FCR |= (0b1 << 2);		// Disable direct mode
  DMA2_Stream1->CR |= (0b1);		 // Enable stream before config
  //DMA_Cmd(DMA2_Channel7, ENABLE); */

  DMA2_Stream1->CR |= (0b111 << 25); // Channel 7 selected
  DMA2_Stream1->NDTR = 0x1;			 // 1 data item to be transmitted
  DMA2_Stream1->FCR |= (0b1 << 2);	 // Disable direct mode

  //HAL_DMA_Start_IT(DMA2_Stream1, (uint32_t)&w2, 0x40021018, 32);
  //DMA2_Stream1->State = HAL_DMA_STATE_BUSY;
  //DMA2_Stream1->ErrorCode = HAL_DMA_ERROR_NONE;
  //DMA_SetConfig(DMA2_Stream1, (uint32_t)&w2, 0x40021018, 32);
  DMA2_Stream1->PAR = (uint32_t)&w1;	 // Set peripheral address to start w2 array
  DMA2_Stream1->M0AR = 0x40021018;			 // Set memory address to GPIOE->BSRR
  DMA2_Stream1->NDTR = 1;
  //regs->IFCR = 0x3FU << DMA2_Stream1->StreamIndex;
  //DMA2_Stream1->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
  //__HAL_DMA_ENABLE(DMA2_Stream1);
  //DMA2_Stream1->CR |= (0b1 << 4);
  TIM8->CR2 |= (0b1 << 3);
  DMA2_Stream1->CR |= (0b1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim8);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (InputCounter == 208)
	{
		__disable_irq();
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

		GPIOA->MODER &= ~(1<<13);
		GPIOA->MODER |= (1<<12);
		GPIOA->BSRR |= (1<<22);

		GPIOE->BSRR = ResetAll;
		//HAL_Delay(2);
		uS_Delay(40000);
		//Toggle Data lines 4 times;
		GPIOE->BSRR = SetAll;
		uS_Delay(100);
		GPIOE->BSRR = ResetAll;
		uS_Delay(100);
		GPIOE->BSRR = SetAll;
		uS_Delay(100);
		GPIOE->BSRR = ResetAll;
		uS_Delay(100);
		GPIOE->BSRR = SetAll;
		uS_Delay(100);
		GPIOE->BSRR = ResetAll;
		uS_Delay(100);
		GPIOE->BSRR = SetAll;
		uS_Delay(100);
		GPIOE->BSRR = ResetAll;
		uS_Delay(20000);

		SwitchCounter--;
		if (SwitchCounter == 0)
		{
			if (ColorArray[0] == true)
			{
				ColorArray[0] = false;
				ColorArray[7] = true;
			}
			else
			{
				ColorArray[0] = true;
				ColorArray[7] = false;
			}
			SwitchCounter = 1000;
			CMDArray_Init(CMDArray, hspd, bs, gck, sep, osc, pol, cntset, onest);
			Grayscale_Init(GrayscaleArray, bs, ColorArray);
			InputArray_Init(InputArray, CMDArray, GrayscaleArray);
		}

		InputCounter = 0;

		GPIOA->MODER |= (1<<13);	// Set back to AF (TIM)
		GPIOA->MODER &= ~(1<<12);

		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		__enable_irq();
	}

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void uS_Delay(int delay)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	while ((DWT->CYCCNT) < delay)
	{
		;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
