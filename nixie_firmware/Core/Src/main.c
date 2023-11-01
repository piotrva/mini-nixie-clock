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
typedef enum {
  REMOTE_IDLE,
  REMOTE_INC_H,
  REMOTE_DEC_H,
  REMOTE_INC_M,
  REMOTE_DEC_M,
  REMOTE_MENU,
  REMOTE_EXIT
} remoteAction_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef time;
volatile uint8_t signs[4] = {1, 2, 3, 4};
volatile bool signDot = true;
volatile remoteAction_t remoteAction = REMOTE_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
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
  uint32_t lastTickDot = 0;
  uint32_t lastTickRefresh = 0;
  uint32_t lastTickMenu = 0;
  bool inMenu = false;
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
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* refresh every 25ms */
    if (HAL_GetTick() - lastTickRefresh > 25)
    {
      lastTickRefresh = HAL_GetTick();

      /* if more than 15s without remote activity - turn of menu */
      if (HAL_GetTick() - lastTickMenu > 15000)
      {
        inMenu = false;
      }

      /* toggle dot every 500ms (100ms when in menu) */
      if (HAL_GetTick() - lastTickDot > (inMenu?100:500))
      {
        lastTickDot = HAL_GetTick();
        __disable_irq();
        signDot = !signDot;
        __enable_irq();
      }

      HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);

      __disable_irq();
      signs[0] = time.Hours/10;
      signs[1] = time.Hours%10;
      signs[2] = time.Minutes/10;
      signs[3] = time.Minutes%10;

      if (remoteAction != REMOTE_IDLE)
      {
        lastTickMenu = HAL_GetTick();
        switch(remoteAction)
        {
          case REMOTE_DEC_H:
            if (time.Hours > 0)
            {
              time.Hours--;
            }
            else
            {
              time.Hours = 23;
            }
            time.Seconds = 0;
            break;
          case REMOTE_INC_H:
            if (time.Hours < 23)
            {
              time.Hours++;
            }
            else
            {
              time.Hours = 0;
            }
            time.Seconds = 0;
            break;
          case REMOTE_DEC_M:
            if (time.Minutes > 0)
            {
              time.Minutes--;
            }
            else
            {
              time.Minutes = 59;
            }
            time.Seconds = 0;
            break;
          case REMOTE_INC_M:
            if (time.Minutes < 59)
            {
              time.Minutes++;
            }
            else
            {
              time.Minutes = 0;
            }
            time.Seconds = 0;
            break;
          case REMOTE_EXIT:
            inMenu = false;
            break;
          default:
          case REMOTE_IDLE:
            break;
        }
        if (inMenu) HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
        if (!inMenu && remoteAction == REMOTE_MENU)
        {
          inMenu = true;
        }
        remoteAction = REMOTE_IDLE;
      }

      __enable_irq();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 23;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SIGN_0_Pin|SIGN_9_Pin|SIGN_8_Pin|SIGN_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIGN_6_Pin|SIGN_5_Pin|SIGN_4_Pin|SIGN_3_Pin
                          |SIGN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_REC_Pin */
  GPIO_InitStruct.Pin = IR_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_REC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_1_Pin DIGIT_2_Pin DIGIT_3_Pin DIGIT_4_Pin */
  GPIO_InitStruct.Pin = DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SIGN_0_Pin SIGN_9_Pin SIGN_8_Pin SIGN_7_Pin */
  GPIO_InitStruct.Pin = SIGN_0_Pin|SIGN_9_Pin|SIGN_8_Pin|SIGN_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SIGN_6_Pin SIGN_5_Pin SIGN_4_Pin SIGN_3_Pin
                           SIGN_2_Pin */
  GPIO_InitStruct.Pin = SIGN_6_Pin|SIGN_5_Pin|SIGN_4_Pin|SIGN_3_Pin
                          |SIGN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define PULSE_US_START  13500
#define PULSE_US_REPEAT 11250
#define PULSE_US_0       1120
#define PULSE_US_1       2250
#define PULSE_US_TOL      500

typedef enum {
  NEC_START,
  NEC_REPEAT,
  NEC_0,
  NEC_1,
  NEC_ERROR,  
} necPulse_t;

necPulse_t necPulseDecode(uint16_t pulse_us)
{
  if (pulse_us > PULSE_US_0 - PULSE_US_TOL && pulse_us < PULSE_US_0 + PULSE_US_TOL)
  {
    return NEC_0;
  }
  else if (pulse_us > PULSE_US_1 - PULSE_US_TOL && pulse_us < PULSE_US_1 + PULSE_US_TOL)
  {
    return NEC_1;
  }
  else if (pulse_us > PULSE_US_REPEAT - PULSE_US_TOL && pulse_us < PULSE_US_REPEAT + PULSE_US_TOL)
  {
    return NEC_REPEAT;
  }
  else if (pulse_us > PULSE_US_START - PULSE_US_TOL && pulse_us < PULSE_US_START + PULSE_US_TOL)
  {
    return NEC_START;
  }
  else 
  {
    return NEC_ERROR;
  }
}

#define NEC_BITS 32
#define NEC_DEBOUNCE_MS 250

void necCallback(uint8_t address, uint8_t command)
{
  static uint32_t debounce = 0;
  if (address == 0x77 && (HAL_GetTick() - debounce > NEC_DEBOUNCE_MS))
  {
    debounce = HAL_GetTick();
    switch (command)
    {
      case 0x8D: /* DOWN */
        remoteAction = REMOTE_INC_H;
        break;
      case 0x8E: /* UP */
        remoteAction = REMOTE_DEC_H;
        break;
      case 0x90: /* VOL+ */
        remoteAction = REMOTE_INC_M;
        break;
      case 0x8F: /* VOL- */
        remoteAction = REMOTE_DEC_M;
        break;
      case 0x95: /* MENU */
        remoteAction = REMOTE_MENU;
        break;
      case 0x96: /* EXIT */
        remoteAction = REMOTE_EXIT;
        break;
      default:
        remoteAction = REMOTE_IDLE;
        break;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint16_t last = 0;
  static uint32_t data;
  static uint8_t counter = 0;
  necPulse_t pulse;
  if (GPIO_Pin == IR_REC_Pin)
  {
    pulse = necPulseDecode(__HAL_TIM_GET_COUNTER(&htim1) - last);
    last = __HAL_TIM_GET_COUNTER(&htim1);
    if (counter == 0)
    {
      if (pulse == NEC_START)
      {
        data = 0;
        counter = NEC_BITS;
      }
      else if (pulse == NEC_REPEAT)
      {
        /* repeat not supported now */
      }
    }
    else
    {
      if (pulse == NEC_ERROR || pulse == NEC_REPEAT)
      {
        /* error - exit decoding immedetely (also we do not except REPEAT here...) */
        counter = 0;
        return;
      }
      data >>=1;
      data |= (pulse==NEC_1)?(1UL<<31):0;
      counter--;
      if (counter == 0)
      {
        /* all bits shall be collected by now - verify negated parts */
        uint8_t address, address_neg, command, command_neg;
        command = ~(data>>24);
        command_neg = data>>16;
        address = ~(data>>8);
        address_neg = data;
        if (command == command_neg && address == address_neg)
        {
          necCallback(address, command);
        }
      }
    }
  }
}

typedef enum {
  NO_DOT,
  DOT_0,
  DOT_1
} dot_t;

#define SIGN_DISABLE 10
#define SIGN_BLANK   99

void setSign(uint8_t sign, dot_t dot)
{
  /* for DOT we use SIGN_0 line of the 10's hour digit and the digit does not display 0 */
  if (dot != NO_DOT && sign == 0)
  {
    sign = SIGN_DISABLE;
  }
  switch(sign)
  {
    case 0:
      HAL_GPIO_WritePin(SIGN_0_GPIO_Port, SIGN_0_Pin, GPIO_PIN_SET);
      break;
    case 1:
      //HAL_GPIO_WritePin(SIGN_1_GPIO_Port, SIGN_1_Pin, GPIO_PIN_SET); /* TODO: SIGN_1 line colides with SWD interface */
      break;
    case 2:
      HAL_GPIO_WritePin(SIGN_2_GPIO_Port, SIGN_2_Pin, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(SIGN_3_GPIO_Port, SIGN_3_Pin, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(SIGN_4_GPIO_Port, SIGN_4_Pin, GPIO_PIN_SET);
      break;
    case 5:
      HAL_GPIO_WritePin(SIGN_5_GPIO_Port, SIGN_5_Pin, GPIO_PIN_SET);
      break;
    case 6:
      HAL_GPIO_WritePin(SIGN_6_GPIO_Port, SIGN_6_Pin, GPIO_PIN_SET);
      break;
    case 7:
      HAL_GPIO_WritePin(SIGN_7_GPIO_Port, SIGN_7_Pin, GPIO_PIN_SET);
      break;
    case 8:
      HAL_GPIO_WritePin(SIGN_8_GPIO_Port, SIGN_8_Pin, GPIO_PIN_SET);
      break;
    case 9:
      HAL_GPIO_WritePin(SIGN_9_GPIO_Port, SIGN_9_Pin, GPIO_PIN_SET);
      break;
    case SIGN_DISABLE:
      break;
    case SIGN_BLANK:
    default:
      HAL_GPIO_WritePin(SIGN_0_GPIO_Port, SIGN_0_Pin, GPIO_PIN_RESET);
      //HAL_GPIO_WritePin(SIGN_1_GPIO_Port, SIGN_1_Pin, GPIO_PIN_RESET); /* TODO: SIGN_1 line colides with SWD interface */
      HAL_GPIO_WritePin(SIGN_2_GPIO_Port, SIGN_2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_3_GPIO_Port, SIGN_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_4_GPIO_Port, SIGN_4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_5_GPIO_Port, SIGN_5_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_6_GPIO_Port, SIGN_6_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_7_GPIO_Port, SIGN_7_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_8_GPIO_Port, SIGN_8_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SIGN_9_GPIO_Port, SIGN_9_Pin, GPIO_PIN_RESET);
      break;
  }
  /* if DOT is to be activated */
  if (dot == DOT_1)
  {
    HAL_GPIO_WritePin(SIGN_0_GPIO_Port, SIGN_0_Pin, GPIO_PIN_SET);
  }
}

#define DIGIT_BLANK 99

void setDigit(uint8_t digit)
{
  switch(digit)
  {
    case 0:
      HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(DIGIT_4_GPIO_Port, DIGIT_4_Pin, GPIO_PIN_RESET);
      break;
    case DIGIT_BLANK:
    default:
      HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIGIT_4_GPIO_Port, DIGIT_4_Pin, GPIO_PIN_SET);
      break;
  }
}

#define BLANK_MS  1
#define ACTIVE_MS 2

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t digit = 0;
  static bool isBlanking = true;
  static uint16_t counter = 0;
  if (htim == &htim2)
  {
    if (isBlanking)
    {
      if (counter == 0)
      {
        setDigit(digit);
        setSign(signs[digit], (digit==0)?(signDot?DOT_1:DOT_0):NO_DOT);
        digit = (digit + 1) % 4;
        isBlanking = false;
        counter = ACTIVE_MS - 1;
      }
      else
      {
        counter--;
      }
    }
    else /* not blanking */
    {
      if (counter == 0)
      {
        setSign(SIGN_BLANK, NO_DOT);
        setDigit(DIGIT_BLANK);
        isBlanking = true;
        counter = BLANK_MS - 1;
      }
      else
      {
        counter--;
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
