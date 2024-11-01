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
#include "serial.h"
#include "cli.h"
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
#define BLINK_IDLE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

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
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void commandPASS(char *args)
{
  UART_WriteString(&huart2, "  Password could not be changed correctly!\r\n");
}
CLI_CommandItem item_PASS = { .callback = commandPASS,
                              .commandName = "PASS",
                              .description = "new     * Set new password"};

void commandHELP(char *args)
{
  CLI_PrintAllCommands();
}
CLI_CommandItem item_HELP = {   .callback = commandHELP,
                .commandName = "?",
                .description = "             Display this help"};
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  CLI_AddCommand(&item_PASS);
  CLI_AddCommand(&item_HELP);
  UART_Init(&huart2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    CLI_Proc();
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
        #if BLINK_IDLE == 1
          signDot = !signDot;
        #else
          if (inMenu)
          {
            signDot = !signDot;
          }
          else
          {
            signDot = true;
          }
        #endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
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
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_OE_GPIO_Port, SPI_OE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_LATCH_GPIO_Port, SPI_LATCH_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI_OE_Pin */
  GPIO_InitStruct.Pin = SPI_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_OE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_LATCH_Pin */
  GPIO_InitStruct.Pin = SPI_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_LATCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_REC_Pin */
  GPIO_InitStruct.Pin = IR_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_REC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

#define U7 0
#define U6 1
#define U5 2
#define U4 3

#define QA 0
#define QB 1
#define QC 2
#define QD 3
#define QE 4
#define QF 5
#define QG 6
#define QH 7

#define DG0 0
#define DG1 (1UL<<((8 * U4) + QA))
#define DG2 (1UL<<((8 * U4) + QB))

#define JG1 (1UL<<((8 * U4) + QC))
#define JG2 (1UL<<((8 * U4) + QD))
#define JG3 (1UL<<((8 * U4) + QE))
#define JG4 (1UL<<((8 * U4) + QF))
#define JG5 (1UL<<((8 * U4) + QG))
#define JG6 (1UL<<((8 * U4) + QH))
#define JG7 (1UL<<((8 * U5) + QA))
#define JG8 (1UL<<((8 * U5) + QB))
#define JG9 (1UL<<((8 * U5) + QC))
#define JG0 (1UL<<((8 * U5) + QD))

#define DOT (1UL<<((8 * U5) + QE))

#define DM1 (1UL<<((8 * U5) + QF))
#define DM2 (1UL<<((8 * U5) + QG))
#define DM3 (1UL<<((8 * U5) + QH))
#define DM4 (1UL<<((8 * U6) + QA))
#define DM5 (1UL<<((8 * U6) + QB))
#define DM0 (1UL<<((8 * U6) + QC))

#define JM1 (1UL<<((8 * U6) + QD))
#define JM2 (1UL<<((8 * U6) + QE))
#define JM3 (1UL<<((8 * U6) + QF))
#define JM4 (1UL<<((8 * U6) + QG))
#define JM5 (1UL<<((8 * U6) + QH))
#define JM6 (1UL<<((8 * U7) + QA))
#define JM7 (1UL<<((8 * U7) + QB))
#define JM8 (1UL<<((8 * U7) + QC))
#define JM9 (1UL<<((8 * U7) + QD))
#define JM0 (1UL<<((8 * U7) + QE))

const uint32_t DG_LUT[] = {DG0, DG1, DG2};
const uint32_t JG_LUT[] = {JG0, JG1, JG2, JG3, JG4, JG5, JG6, JG7, JG8, JG9};
const uint32_t DM_LUT[] = {DM0, DM1, DM2, DM3, DM4, DM5};
const uint32_t JM_LUT[] = {JM0, JM1, JM2, JM3, JM4, JM5, JM6, JM7, JM8, JM9};


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    /* Here transfer sign configuration over SPI */
    uint32_t dataSPI = 0x00000000;

    dataSPI |= DG_LUT[signs[0]];
    dataSPI |= JG_LUT[signs[1]];
    dataSPI |= DM_LUT[signs[2]];
    dataSPI |= JM_LUT[signs[3]];
    dataSPI |= signDot?DOT:0UL;

    HAL_GPIO_WritePin(SPI_LATCH_GPIO_Port, SPI_LATCH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&dataSPI, 4, 10000UL);
    HAL_GPIO_WritePin(SPI_LATCH_GPIO_Port, SPI_LATCH_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI_OE_GPIO_Port, SPI_OE_Pin, GPIO_PIN_RESET);
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
