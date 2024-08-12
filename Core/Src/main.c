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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "stm32l0xx.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
//----------------------------------------------------------

//----------------------------------------------------------
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void pwmFP7103();
void setTimeNow();
int Clock();
void writeCHARSEG(char CHAR, int seg);
char* setActualMenu(int v, int h);
int getMenuIndexByID(int id);
int getNearMenuIndexByID(int parentid, int id, int side);
void testMelody(){}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LEDa_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define LEDa_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define LEDb_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1)
#define LEDb_OFF() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1)
#define LEDc_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define LEDc_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define LEDd_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define LEDd_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define LEDe_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LEDe_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LEDf_ON() 		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LEDf_OFF() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LEDg_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define LEDg_OFF() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define LEDdp_ON() 		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3)
#define LEDdp_OFF() 	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3)

#define LEDD1_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3)
#define LEDD1_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3)
#define LEDD2_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define LEDD2_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define LEDD3_ON() 		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define LEDD3_OFF() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define LEDD4_ON() 		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4)
#define LEDD4_OFF() 	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4)

#define LEDl1l2_ON() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10)
#define LEDl1l2_OFF() 	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10)
#define LEDalarm_ON() 	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8)
#define LEDalarm_OFF() 	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8)

#define pinEN_ON() 		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15)
#define pinEN_OFF() 	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool flagDecrementButton;      // Було натискання кнопки
bool flagEnterButton;      // Було натискання кнопки
bool flagIncrementButton;      // Було натискання кнопки
bool flagDecrementButtonLong;  // Було довге утримання кнопки
bool flagEnterButtonLong;  // Було довге утримання кнопки
bool flagIncrementButtonLong;  // Було довге утримання кнопки

int menuArraySize = 18;        	// Встановлюємо розмір масиву
int actualIndex = 0;
bool isParamEditMode = false;	// Прапорець режиму редагування параметра
int tmpVValue = 0;       		// Тимчасова змінна для зберігання змінюваного параметра

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};

struct strMenu {
			int id;         // Унікальний ідентифікаційний індекс ID
			int parentid;   // ID батька
			bool isParam;   // Чи є пункт змінним параметром
			char _name[4];  // Назва
			int value;      // Поточне значення
			int _min;       // Мінімально можливе значення параметра
			int _max;       // Максимально можливе значення параметра
};
/* PPPP
 * P__0			Time_Now
 * 		P_0.0	Hour_Now
 * 		P_0.1	Minute_Now
 * 		P_0.2	Set
 * P__1			Time_Rise
 * 		P_1.0	Hour_Rise
 * 		P_1.1	Minute_Rise
 * P__2			Rising_Parametrs
 * 		P_2.0	Period_Rising
 * P__3			Alarm_Parametrs
 * 		P_3.0	Alarm_Status
 * 		P_3.1	Alarm_Melody
 * 		P_3.2	Alarm_Melody_test
 * P__4			Menu_Parametrs
 * 		P_4.0	Numbers_Change_Style
 * 		P_4.1	Menu_Night_Mode
 * P__5			Clock
 */
struct strMenu menu[] = {                         // Встановлюємо пункти меню
	  {0, -1,    false,	"PPPP",		0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {1, 0,     false,	"P__0",		0, 0, 	0},
	  {2, 1,     true,	"P_00",	0, 2, 	0},
	  {3, 2,     true,	"P_01", 	0, 59, 	0},
	  {4, 1,     false, "P_02", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {5, 0,     false, "P__1", 	0, 0, 	0},
	  {6, 5,     true,	"P_10", 	0, 23, 	0},
	  {7, 5,     true,	"P_11", 	0, 59, 	0},
	  //-----------------------------------------------------------------------
	  {8, 0,   	 false, "P__2", 	0, 0, 	0},
	  {9, 8,     true,	"P_20", 	0, 255, 64},
	  //-----------------------------------------------------------------------
	  {10, 0,    false, "P__3", 	0, 0, 	0},
	  {11, 10,   true,	"P_30", 	0, 1, 	1},
	  {12, 10,   true,	"P_31", 	0, 8, 	0},
	  {13, 10,   false,	"P_32", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {14, 0,    false, "P__4", 	0, 0, 	1},
	  {15, 14,   true,	"P_40", 	0, 3, 	0},
	  {16, 14,   true,	"P_41", 	0, 1, 	1},
	  {17, 0,    false, "P__5", 	0, 0, 	0}
	  //-----------------------------------------------------------------------
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int vmenu = 0; // Змінна, що зберігає дію по вертикалі 1 - вхід в меню, -1 - вихід з меню
	  int hmenu = 0; // Змінна, що зберігає дію по горизонталі 1 - вправо, -1 - вліво
	  char* tmpValue;

	 if (flagDecrementButton){
	  hmenu = 1;// Якщо при спаді лінії A на лінії B лог. одиниця, то обертання в один бік
	  flagDecrementButton = false;       // Действие обработано - сбрасываем флаг
	 }
	  					  // Прапорець обертання за годинниковою стрілкою
	 else if (flagIncrementButton){
	  hmenu = -1;	// Якщо при спаді лінії A на лінії B лог. нуль, то обертання в інший бік
	  flagIncrementButton = false;       // Действие обработано - сбрасываем флаг
	 }
	  						// Прапорець обертання проти годинникової стрілки
	 if (flagEnterButton) {           // Кнопка нажата
	  vmenu = 1;                // По нажатию кнопки - переходим на уровень вниз
	  flagEnterButton = false;       // Действие обработано - сбрасываем флаг
	 }
	 else if (flagEnterButtonLong){
	  vmenu = -1;
	  flagEnterButtonLong = false;               // Действие обработано - сбрасываем флаг
	 }
	 if (vmenu != 0 || hmenu != 0) tmpValue = setActualMenu(vmenu, hmenu); // Если было действие - реагируем на него
	 for(int i = 0; i<4;i++){
		 writeCHARSEG(tmpValue[i], i);
		 LL_mDelay(50);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
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
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(EN_GPIO_Port, EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D1_GPIO_Port, D1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D2_GPIO_Port, D2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D3_GPIO_Port, D3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(D4_GPIO_Port, D4_Pin);

  /**/
  LL_GPIO_SetOutputPin(d_GPIO_Port, d_Pin);

  /**/
  LL_GPIO_SetOutputPin(c_GPIO_Port, c_Pin);

  /**/
  LL_GPIO_SetOutputPin(a_GPIO_Port, a_Pin);

  /**/
  LL_GPIO_SetOutputPin(g_GPIO_Port, g_Pin);

  /**/
  LL_GPIO_SetOutputPin(b_GPIO_Port, b_Pin);

  /**/
  LL_GPIO_SetOutputPin(alarm_GPIO_Port, alarm_Pin);

  /**/
  LL_GPIO_SetOutputPin(f_GPIO_Port, f_Pin);

  /**/
  LL_GPIO_SetOutputPin(l1l2_GPIO_Port, l1l2_Pin);

  /**/
  LL_GPIO_SetOutputPin(e_GPIO_Port, e_Pin);

  /**/
  LL_GPIO_SetOutputPin(dp_GPIO_Port, dp_Pin);

  /**/
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(D2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = d_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(d_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = c_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(c_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = a_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(a_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = g_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(g_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = b_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(b_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = alarm_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(alarm_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = f_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(f_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = l1l2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(l1l2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = e_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(e_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = dp_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(dp_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = D4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE2);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE9);

  /**/
  LL_GPIO_SetPinPull(decrement_GPIO_Port, decrement_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(enter_GPIO_Port, enter_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(increment_GPIO_Port, increment_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(pwr_GPIO_Port, pwr_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(decrement_GPIO_Port, decrement_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(enter_GPIO_Port, enter_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(increment_GPIO_Port, increment_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(pwr_GPIO_Port, pwr_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_1_IRQn, 0);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI2_3_IRQn, 0);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void pwmFP7103() {
	if (menu[11].value) {
		int timeWakeUp 	= menu[6].value 	* 3600
					+ menu[7].value	* 60;
		int timeNow 	= sTime.Hours 		* 3600
					+ sTime.Minutes		* 60
					+ sTime.Seconds;
		if(menu[9].value * 60 >= timeWakeUp - timeNow){
			pinEN_ON();
			TIM_Cmd(TIM21, ENABLE);
			TIM21->CCR1 = (int16_t) (65535 * pow((1 - timeNow / timeWakeUp), 2.24));
		}
	} else {
			pinEN_OFF();
			TIM_Cmd(TIM21, DISABLE);
			TIM21->CCR1 = 0;
		}
}

int Clock(){
	char tmpClock[4]={};
	int j = 0;
	tmpClock[0] = sTime.Hours/10;
	if (sTime.Hours/10 == 0){j=1;}
	tmpClock[1] = sTime.Hours - tmpClock[0];
	tmpClock[2] = sTime.Minutes/10;
	tmpClock[3] = sTime.Minutes - tmpClock[0];
	if(sTime.Hours > 5 && sTime.Hours < 22 || flagDecrementButton || flagEnterButton || flagIncrementButton || flagDecrementButtonLong || flagEnterButtonLong || flagIncrementButtonLong)
	{
		for(int i = 0 + j; i<4;i++){
		 writeCHARSEG(tmpClock[i], i);
		 LL_mDelay(50);
	 }
	}
	return flagDecrementButtonLong&&flagIncrementButtonLong?0:1;
}

void setTimeNow(){
		sTime.Hours 	= menu[2].value;
		sTime.Minutes 	= menu[3].value;
		sTime.Seconds 	= 00;
	}

void writeCHARSEG(char CHAR, int seg){
	switch (seg) {
		case 0:
			LEDD1_ON();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		case 1:
			LEDD1_OFF();
			LEDD2_ON();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		case 2:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_ON();
			LEDD4_OFF();
			break;
		case 3:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_ON();
			break;
		default:
			LEDD1_OFF();
			LEDD2_OFF();
			LEDD3_OFF();
			LEDD4_OFF();
			break;
		}
	switch (CHAR) {
		case 'A':
			LEDa_ON();
			LEDb_ON();
			LEDc_OFF();
			LEDd_ON();
			LEDe_ON();
			LEDf_ON();
			LEDg_ON();
			LEDdp_OFF();
			break;
		case 'P':
			LEDa_ON();
			LEDb_ON();
			LEDc_OFF();
			LEDd_OFF();
			LEDe_ON();
			LEDf_ON();
			LEDg_OFF();
			LEDdp_OFF();
			break;
		case '_':
			LEDa_OFF();
			LEDb_OFF();
			LEDc_ON();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_OFF();
			break;
		case '.':
			LEDa_OFF();
			LEDb_OFF();
			LEDc_OFF();
			LEDd_OFF();
			LEDe_OFF();
			LEDf_OFF();
			LEDg_OFF();
			LEDdp_ON();
			break;


		case '0':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '1':
				LEDa_OFF();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '2':
				LEDa_ON();
				LEDb_ON();
				LEDc_OFF();
				LEDd_ON();
				LEDe_ON();
				LEDf_OFF();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '3':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '4':
				LEDa_OFF();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '5':
				LEDa_ON();
				LEDb_OFF();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '6':
				LEDa_ON();
				LEDb_OFF();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '7':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
			case '8':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_ON();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			case '9':
				LEDa_ON();
				LEDb_ON();
				LEDc_ON();
				LEDd_ON();
				LEDe_OFF();
				LEDf_ON();
				LEDg_ON();
				LEDdp_OFF();
				break;
			default:
				LEDa_OFF();
				LEDb_OFF();
				LEDc_OFF();
				LEDd_OFF();
				LEDe_OFF();
				LEDf_OFF();
				LEDg_OFF();
				LEDdp_OFF();
				break;
	}
}

char* setActualMenu(int v, int h) {
	if (v != 0) {               // Рухаємося по вертикалі
		if (v == -1) {            // Команда ВГОРУ (скасування)
			if (isParamEditMode) { // Якщо параметр у режимі редагування, то скасовуємо зміни
				isParamEditMode = false;
			} else { // Якщо пункт меню не у режимі редагування, переміщаємося до батька
				if (menu[actualIndex].parentid > 0) { // Якщо є куди переміщатися вгору (ParentID>0)
					actualIndex = getMenuIndexByID(menu[actualIndex].parentid);
				}
			}
		} else {                        // Якщо команда ВН�?З - входу/редагування
			if (menu[actualIndex].isParam && !isParamEditMode) { // Якщо не в режимі редагування, то ...
				isParamEditMode = true; // Переходимо в режим редагування параметра
				tmpVValue = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
			} else if (menu[actualIndex].isParam && isParamEditMode) { // Якщо в режимі редагування
				menu[actualIndex].value = tmpVValue; // Зберігаємо задане значення
				isParamEditMode = false;      // І виходимо з режиму редагування

			} else {
				bool nochild = true;  // Прапорець, чи є дочірні елементи
				for (int i = 0; i < menuArraySize; i++) {
					if (menu[i].parentid == menu[actualIndex].id) {
						actualIndex = i; // Якщо є, робимо перший попавшийся актуальним елементом
						nochild = false;  // Потомки є
						break;            // Виходимо з for
					}
				}
				if (nochild) { // Якщо ж потомків немає, воспринимаємо як команду
					switch (menu[actualIndex].id) { // Serial.println("Executing command...");         // І тут обробляємо за власним баченням
					case 4:						// Зберігаємо налаштування з комірки памті
						setTimeNow();
						break;
					case 13:						// Завантажуємо налаштування з комірки памті
						testMelody();
						break;
					case 17:
						while (Clock()){Clock();}
						break;
					default:
						break;
					}
				}
			}
		}
	}

	if (h != 0) {             // Якщо горизонтальна навігація
		if (isParamEditMode) {  // У режимі редагування параметра
			tmpVValue += h;        // Змінюємо його значення і ...
			// ... контролюємо, щоб воно залишилося в заданому діапазоні
			if (tmpVValue > menu[actualIndex]._max)
				tmpVValue = menu[actualIndex]._min;
			if (tmpVValue < menu[actualIndex]._min)
				tmpVValue = menu[actualIndex]._max;
		} else { // Якщо режим редагування не активний, навігація серед дочірніх одного батька
			actualIndex = getNearMenuIndexByID(menu[actualIndex].parentid,
					menu[actualIndex].id, h);
		}
	}
	// Отображаем информацию
	if (isParamEditMode) {
		int tmpV[4] = {};
		tmpV[0]=	tmpVValue/1000;
		tmpV[1]=	tmpVValue/100 	- 	tmpV[0]*10;
		tmpV[2]=	tmpVValue/10 	- 	tmpV[0]*100 	- tmpV[1]*10;
		tmpV[3]=	tmpVValue 		- 	tmpV[0]*1000 	- tmpV[1]*100 	- tmpV[2]*10;
		return tmpV;
	} else {
		return menu[actualIndex]._name;
	}
}

int getMenuIndexByID(int id) { // Функція отримання індексу пункту меню за його ID
	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].id == id)
			return i;
	}
	return -1;
}

int getNearMenuIndexByID(int parentid, int id, int side) { // Функція отримання індексу пункту меню наступного або попереднього від актуального
	int prevID = -1;      // Змінна для зберігання індексу попереднього елемента
	int nextID = -1;        // Змінна для зберігання індексу наступного елемента
	int actualID = -1;     // Змінна для зберігання індексу актуального елемента

	int firstID = -1;  // Змінна для зберігання індексу першого елемента
	int lastID = -1;   // Змінна для зберігання індексу останнього елемента

	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].parentid == parentid) { // Перебираємо всі елементи з одним батьківським ID
			if (firstID == -1)
				firstID = i;     // Запам'ятовуємо перший елемент списку

			if (menu[i].id == id) {
				actualID = i;  // Запам'ятовуємо актуальний елемент списку
			} else {
				if (actualID == -1) { // Якщо зустрівся елемент до актуального, робимо його попереднім
					prevID = i;
				} else if (actualID != -1 && nextID == -1) { // Якщо зустрівся елемент після актуального, робимо його наступним
					nextID = i;
				}
			}
			lastID = i;  // Кожний наступний елемент - останній
		}
	}

	if (nextID == -1)
		nextID = firstID; // Якщо наступного елемента немає - по колу видаємо перший
	if (prevID == -1)
		prevID = lastID; // Якщо попереднього елемента немає - по колу видаємо останній
	if (side == -1)
		return prevID; // В залежності від напрямку обертання, видаємо потрібний індекс
	else
		return nextID;
	return -1;
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
