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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_BUFF_SIZE 20
struct high
{
		uint32_t now;
		uint32_t set;
};

struct motion
{
		struct high high;
		uint16_t speed;
		uint8_t dir;
};

struct motion motion = {{0,0},0,0};

struct frame
{
		uint8_t enable;
		uint8_t data;
		uint8_t buff[UART_BUFF_SIZE];
};

struct frame frame = {1};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void delay_ns(uint32_t times)
{
	while(times--);
}

void delay_us(uint32_t times)
{
	int i;
	uint32_t count;
	for(i=0; i<10; i++)
	{
		count = times;
		while(count--);
	}
}

void delay_ms(uint32_t times)
{
	uint32_t count;
	count = times;
	while(count--)
		delay_us(1000);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE3_GPIO_Port, OUTPUT_573LE3_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_AUTORELOAD(&htim1, 200);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart1, &(frame.data), 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int i,j;
  while (1)
  {
  /* USER CODE END WHILE */
#if 0
	if(frame.enable)
	{
		frame.enable = 0;
		for(j=0; j<1; j++)
		{
			HAL_Delay(100);
			HAL_GPIO_WritePin(OUTPUT_DIR1_GPIO_Port, OUTPUT_DIR1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			for(i=1000; i>0; i--)
			{
				HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_RESET);
				//delay_ns(10);
				delay_us(i/5+1);
				HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_SET);
				delay_us(i/5+1);
				//HAL_Delay(1);
			}
			HAL_Delay(100);
			HAL_GPIO_WritePin(OUTPUT_DIR1_GPIO_Port, OUTPUT_DIR1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_Delay(5000);
			for(i=1000; i>0; i--)
			{
				HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_RESET);
				//delay_ns(10);
				delay_us(i/5+1);
				HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_SET);
				delay_us(i/5+1);
				//HAL_Delay(1);
			}
			
		}
	}
#endif
  /* USER CODE BEGIN 3 */
		
		HAL_GPIO_WritePin(OUTPUT_SEATLED4_GPIO_Port, OUTPUT_SEATLED4_Pin, GPIO_PIN_SET);
		//HAL_Delay(1000);
		delay_ms(1000);
		HAL_GPIO_WritePin(OUTPUT_SEATLED4_GPIO_Port, OUTPUT_SEATLED4_Pin, 0);
		//HAL_Delay(1000);
		delay_ms(1000);
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : OUTPUT_SEATLED4_Pin OUTPUT_SEATLED3_Pin OUTPUT_573LE1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SEATLED4_Pin|OUTPUT_SEATLED3_Pin|OUTPUT_573LE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_SEATLED2_Pin OUTPUT_SEATLED1_Pin OUTPUT_CLR1_Pin OUTPUT_DIR3_Pin 
                           OUTPUT_DIR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SEATLED2_Pin|OUTPUT_SEATLED1_Pin|OUTPUT_CLR1_Pin|OUTPUT_DIR3_Pin 
                          |OUTPUT_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_UPLIMIT1_Pin EXTI_DOWNLIMIT2_Pin EXTI_UPLIMIT2_Pin EXTI_DOWNLIMIT2C3_Pin 
                           EXTI_UPLIMIT3_Pin EXTI_DOWNLIMIT3_Pin */
  GPIO_InitStruct.Pin = EXTI_UPLIMIT1_Pin|EXTI_DOWNLIMIT2_Pin|EXTI_UPLIMIT2_Pin|EXTI_DOWNLIMIT2C3_Pin 
                          |EXTI_UPLIMIT3_Pin|EXTI_DOWNLIMIT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL1_Pin OUTPUT_485RW_Pin OUTPUT_573LE2_Pin OUTPUT_CLR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL1_Pin|OUTPUT_485RW_Pin|OUTPUT_573LE2_Pin|OUTPUT_CLR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL2_Pin OUTPUT_LED0_Pin OUTPUT_LED1_Pin OUTPUT_SP8_Pin 
                           OUTPUT_SP7_Pin OUTPUT_SP6_Pin OUTPUT_SP5_Pin OUTPUT_SP4_Pin 
                           OUTPUT_PUL3_Pin OUTPUT_SP3_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL2_Pin|OUTPUT_LED0_Pin|OUTPUT_LED1_Pin|OUTPUT_SP8_Pin 
                          |OUTPUT_SP7_Pin|OUTPUT_SP6_Pin|OUTPUT_SP5_Pin|OUTPUT_SP4_Pin 
                          |OUTPUT_PUL3_Pin|OUTPUT_SP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_SW_Pin */
  GPIO_InitStruct.Pin = INPUT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_573LE3_Pin OUTPUT_DIR1_Pin OUTPUT_CLR3_Pin OUTPUT_NUP3_Pin 
                           OUTPUT_NDOWN3_Pin OUTPUT_NUP2_Pin OUTPUT_NDOWN2_Pin OUTPUT_NUP1_Pin 
                           OUTPUT_NDOWN1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_573LE3_Pin|OUTPUT_DIR1_Pin|OUTPUT_CLR3_Pin|OUTPUT_NUP3_Pin 
                          |OUTPUT_NDOWN3_Pin|OUTPUT_NUP2_Pin|OUTPUT_NDOWN2_Pin|OUTPUT_NUP1_Pin 
                          |OUTPUT_NDOWN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_1_Pin INPUT_BCD2_1_Pin INPUT_BCD4_1_Pin INPUT_BCD8_1_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_1_Pin|INPUT_BCD2_1_Pin|INPUT_BCD4_1_Pin|INPUT_BCD8_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_2_Pin INPUT_BCD2_2_Pin INPUT_BCD4_2_Pin INPUT_BCD8_2_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_2_Pin|INPUT_BCD2_2_Pin|INPUT_BCD4_2_Pin|INPUT_BCD8_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

int output_pul1(GPIO_PinState sign)
{
	static uint8_t status = 0;
	static GPIO_PinState dir = GPIO_PIN_RESET;
	switch(status)
	{
		case 0:
			HAL_GPIO_WritePin(OUTPUT_DIR1_GPIO_Port, OUTPUT_DIR1_Pin, sign);
			if(dir != sign)
			{
				dir = sign;
				return 0;
			}
			HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_RESET);
			status++;
			return 0;
		case 1:
			HAL_GPIO_WritePin(OUTPUT_PUL1_GPIO_Port, OUTPUT_PUL1_Pin, GPIO_PIN_SET);
			status = 0;
			return (dir?-1:1);
		default:
			status = 0;
			return 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint32_t speed = 0;
	if(htim->Instance == TIM1)
	{
		if(motion.high.now == motion.high.set)
			return;
		if(motion.high.now < motion.high.set)
			speed = 0xffff/(motion.high.set-motion.high.now);
		else
			speed = 0xffff/(motion.high.now-motion.high.set);
		speed = (speed<27)?27:speed;
			__HAL_TIM_SET_AUTORELOAD(&htim1, speed);
		motion.high.now += output_pul1((motion.high.now < motion.high.set)?GPIO_PIN_RESET:GPIO_PIN_SET);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		__set_PRIMASK(1);
		frame.enable = 1;
		motion.high.set = frame.data<<6;
		__set_PRIMASK(0);
		HAL_UART_Receive_IT(&huart1, &(frame.data), 1);
	HAL_GPIO_TogglePin(OUTPUT_SEATLED3_GPIO_Port, OUTPUT_SEATLED3_Pin);
}

/* USER CODE END 4 */

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
