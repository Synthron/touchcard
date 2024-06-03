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
#include "adc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

uint16_t values[10][7], raw[10][7], cal[10][7];
char out_buf[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void timer_init(void);
void delay_us(uint16_t us);
void ADC_Select(uint8_t ch);
void init_calibration(void);
void Scan_Pad(void);
void Scan_ADC(uint8_t col);

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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  timer_init();
  while (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
    ;

  /**
   * Timer Overview:
   *
   * Timer 1:
   *  - Channel 1
   *  - Channel 2
   *  - Channel 3
   * Timer 2:
   *  - Channel 1
   *  - Channel 2
   *  - Channel 3
   *  - Channel 4
   * Timer 3:
   *  - Channel 1
   *  - Channel 2
   *  - Channel 3
   * totals 10 channels
   *
   * ADC Channels:
   *  0, 1, 4, 5, 6, 7, 9
   * totals 7 channels
   *
   */

  init_calibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    ADC_Select(0);
    uint32_t temp;
    // calculate median of 5 to reduce noise
    for (uint8_t i = 0; i < 5; i++)
    {
      HAL_ADC_Start(&hadc);
      while(HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY));
      temp += HAL_ADC_GetValue(&hadc);
      delay_us(5);
    }
    raw[0][0] = temp / 5;
    temp = 0;
    sprintf(out_buf, "%d\n", raw[0][0]);
    CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
    HAL_Delay(250);*/
    
    Scan_Pad();

    int32_t temp;
    for (uint8_t i = 0; i < 10; i++)
    {
      for (uint8_t j = 0; j < 7; j++)
      {
        temp = (int16_t)raw[i][j] - (int16_t)cal[i][j];
        if (temp < 0)
          temp = 0;
        values[i][j] = /*raw[i][j];*/ (uint16_t)temp;
      }
    }

    for (uint8_t i = 0; i < 7; i++)
    {
      sprintf(out_buf, "%3d:%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d\n", i,
              values[0][i],
              values[1][i],
              values[2][i],
              values[3][i],
              values[4][i],
              values[5][i],
              values[6][i],
              values[7][i],
              values[8][i],
              values[9][i]);
      CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
    }

    HAL_Delay(1000);


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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
   */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
   */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */

void init_calibration(void)
{
  Scan_Pad();

  for (uint8_t i = 0; i < 10; i++)
  {
    for (uint8_t j = 0; j < 7; j++)
    {
      cal[i][j] = raw[i][j];
    }
  }
}

void timer_init(void)
{
  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);

  // presets for PWM ratio if 25%
  TIM1->CCR1 = 2;
  TIM1->CCR2 = 2;
  TIM1->CCR3 = 2;
  TIM2->CCR1 = 2;
  TIM2->CCR2 = 2;
  TIM2->CCR3 = 2;
  TIM2->CCR4 = 2;
  TIM3->CCR1 = 2;
  TIM3->CCR2 = 2;
  TIM3->CCR3 = 2;
}

void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim14, 0); // set counter to zero
  while (__HAL_TIM_GET_COUNTER(&htim14) < us)
    ; // wait for counter to reach us input
}

void Scan_ADC(uint8_t col)
{
  uint32_t temp;
  // iterate through al ADC channels
  for (uint8_t j = 0; j < 7; j++)
  {
    ADC_Select(j);
    // calculate median of 5 to reduce noise
    for (uint8_t i = 0; i < 5; i++)
    {
      HAL_ADC_Start(&hadc);
      while(HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY));
      temp += HAL_ADC_GetValue(&hadc);
      delay_us(5);
    }
    raw[col][j] = temp / 5;
    temp = 0;
  }
}

void Scan_Pad(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  delay_us(20);
  Scan_ADC(0);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  delay_us(20);
  Scan_ADC(1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  delay_us(20);
  Scan_ADC(2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  delay_us(20);
  Scan_ADC(3);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  delay_us(20);
  Scan_ADC(4);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  delay_us(20);
  Scan_ADC(5);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  delay_us(20);
  Scan_ADC(6);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  delay_us(20);
  Scan_ADC(7);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  delay_us(20);
  Scan_ADC(8);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  delay_us(20);
  Scan_ADC(9);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

void ADC_Select(uint8_t ch)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  switch (ch)
  {
  case 0:
    sConfig.Channel = ADC_CHANNEL_0;
    break;
  case 1:
    sConfig.Channel = ADC_CHANNEL_1;
    break;
  case 2:
    sConfig.Channel = ADC_CHANNEL_4;
    break;
  case 3:
    sConfig.Channel = ADC_CHANNEL_5;
    break;
  case 4:
    sConfig.Channel = ADC_CHANNEL_6;
    break;
  case 5:
    sConfig.Channel = ADC_CHANNEL_7;
    break;
  case 6:
    sConfig.Channel = ADC_CHANNEL_9;
    break;

  default:
    Error_Handler();
    break;
  }

  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
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

#ifdef USE_FULL_ASSERT
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
