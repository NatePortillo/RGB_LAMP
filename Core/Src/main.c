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
#include "sk6812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NEOPIXEL_ZERO	6
#define NEOPIXEL_ONE	12
#define NUM_PIXELS		24
#define SK_BYTES		24
#define DMA_BUFF_SIZE 	(NUM_PIXELS * SK_BYTES) + 1

#define LED_MODE_COUNT	12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch3;

/* USER CODE BEGIN PV */
PixelRGB_t pixel[NUM_PIXELS] = {0};
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
uint32_t *pBuff;

volatile uint8_t colorCase = 0;
volatile uint8_t sleepFlag = 0;  // Flag to signal button press/release event
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    if (sleepFlag == 1) {
	        // Turn off LEDs in sleep mode
	        LEDS_OFF(pixel, NUM_PIXELS);
	        SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        HAL_SuspendTick();
	        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	        HAL_ResumeTick();
	        continue;
	    }

	    switch (colorCase)
	    {
	    case 0:
	        RGB_COLORS(pixel, NUM_PIXELS, 200, 0, 0);  // RED
	        SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 1:
	        RGB_COLORS(pixel, NUM_PIXELS, 0, 200, 0);  // Blue
	        SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 2:
	        RGB_COLORS(pixel, NUM_PIXELS, 200, 0, 60);  // Orange
	    	SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 3:
	        RGB_COLORS(pixel, NUM_PIXELS, 255, 255, 255);  // White
	        SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 4:
	    	RGB_COLORS(pixel, NUM_PIXELS, 150, 200, 0);  // Purple
	    	SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 5:
	        RGB_COLORS(pixel, NUM_PIXELS, 0, 0, 200);  // Green
	        SEND_DATA(pixel, dmaBuffer, NUM_PIXELS, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 6:
	    	TheaterChaseRGB(pixel, dmaBuffer, NUM_PIXELS,
	    	                160, 10, 0,  // Orange (Red 255, Green 69, Blue 0)
	    	                3, 20,       // Spacing of 3, 10 iterations
	    	                80,          // Delay of 50 ms
	    	                NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 7:
	        CometTail(pixel, dmaBuffer, NUM_PIXELS, 0xFF0000, 16, 100, NEOPIXEL_ONE, NEOPIXEL_ZERO);  // Red comet
	        break;
	    case 8:
	        Twinkle(pixel, dmaBuffer, NUM_PIXELS, 0xFFFFFF, 400, 100, NEOPIXEL_ONE, NEOPIXEL_ZERO);  // White twinkle
	        break;
	    case 9:
	    	ColorWipe(pixel, dmaBuffer, NUM_PIXELS, 50, NEOPIXEL_ONE, NEOPIXEL_ZERO, 1000);
	    	break;
	    case 10:
	        RainbowFade(pixel, dmaBuffer, NUM_PIXELS, 30, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    case 11:
	        DodgersBlueWhiteWave(pixel, dmaBuffer, NUM_PIXELS, 150, NEOPIXEL_ONE, NEOPIXEL_ZERO);
	        break;
	    default:
	        colorCase = 0; // Reset to first case
	        break;
	    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Period = 18;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_tick = 0;  // Stores the last tick when the button was pressed
    uint32_t current_tick = HAL_GetTick();  // Get the current system tick

    // Debounce: Ignore interrupts within 500ms of the last trigger
    if (GPIO_Pin == GPIO_PIN_3) {  // Ensure it's triggered by the correct pin
        if ((current_tick - last_tick) < 500) {  // 200 ms debounce time
            return;
        }

        last_tick = current_tick;  // Update the last tick

        if (sleepFlag == 1) {
            // Wake up from sleep mode
            sleepFlag = 0;
            colorCase = 0;
        } else {
            // Cycle through color cases, including sleep mode
            colorCase = (colorCase + 1) % (LED_MODE_COUNT + 1); // Add one for sleep case
            if (colorCase == LED_MODE_COUNT) {
                sleepFlag = 1; // Enter sleep mode
            }
        }
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
