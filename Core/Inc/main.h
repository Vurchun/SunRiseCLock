/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_Pin LL_GPIO_PIN_15
#define EN_GPIO_Port GPIOC
#define decrement_Pin LL_GPIO_PIN_0
#define decrement_GPIO_Port GPIOA
#define decrement_EXTI_IRQn EXTI0_1_IRQn
#define enter_Pin LL_GPIO_PIN_1
#define enter_GPIO_Port GPIOA
#define enter_EXTI_IRQn EXTI0_1_IRQn
#define increment_Pin LL_GPIO_PIN_2
#define increment_GPIO_Port GPIOA
#define increment_EXTI_IRQn EXTI2_3_IRQn
#define D1_Pin LL_GPIO_PIN_3
#define D1_GPIO_Port GPIOA
#define D2_Pin LL_GPIO_PIN_4
#define D2_GPIO_Port GPIOA
#define d_Pin LL_GPIO_PIN_5
#define d_GPIO_Port GPIOA
#define c_Pin LL_GPIO_PIN_6
#define c_GPIO_Port GPIOA
#define a_Pin LL_GPIO_PIN_7
#define a_GPIO_Port GPIOA
#define g_Pin LL_GPIO_PIN_0
#define g_GPIO_Port GPIOB
#define b_Pin LL_GPIO_PIN_1
#define b_GPIO_Port GPIOB
#define alarm_Pin LL_GPIO_PIN_8
#define alarm_GPIO_Port GPIOA
#define f_Pin LL_GPIO_PIN_9
#define f_GPIO_Port GPIOA
#define l1l2_Pin LL_GPIO_PIN_10
#define l1l2_GPIO_Port GPIOA
#define e_Pin LL_GPIO_PIN_11
#define e_GPIO_Port GPIOA
#define D3_Pin LL_GPIO_PIN_12
#define D3_GPIO_Port GPIOA
#define buzzer_Pin LL_GPIO_PIN_15
#define buzzer_GPIO_Port GPIOA
#define dp_Pin LL_GPIO_PIN_3
#define dp_GPIO_Port GPIOB
#define D4_Pin LL_GPIO_PIN_4
#define D4_GPIO_Port GPIOB
#define mainLED_Pin LL_GPIO_PIN_5
#define mainLED_GPIO_Port GPIOB
#define pwr_Pin LL_GPIO_PIN_9
#define pwr_GPIO_Port GPIOB
#define pwr_EXTI_IRQn EXTI4_15_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
