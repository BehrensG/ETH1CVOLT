/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"

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
#define MCU_DEF_Pin LL_GPIO_PIN_2
#define MCU_DEF_GPIO_Port GPIOE
#define MCU_ADC_nCS_Pin LL_GPIO_PIN_4
#define MCU_ADC_nCS_GPIO_Port GPIOA
#define EEPROM_nWR_Pin LL_GPIO_PIN_9
#define EEPROM_nWR_GPIO_Port GPIOA
#define MCU_DAC_nCS_Pin LL_GPIO_PIN_15
#define MCU_DAC_nCS_GPIO_Port GPIOA
#define MCU_DAC_SCLK_Pin LL_GPIO_PIN_10
#define MCU_DAC_SCLK_GPIO_Port GPIOC
#define MCU_DAC_DIN_Pin LL_GPIO_PIN_12
#define MCU_DAC_DIN_GPIO_Port GPIOC
#define MCU_DAC_nLDAC_Pin LL_GPIO_PIN_0
#define MCU_DAC_nLDAC_GPIO_Port GPIOD
#define MCU_ADC_SCLK_Pin LL_GPIO_PIN_3
#define MCU_ADC_SCLK_GPIO_Port GPIOB
#define MCU_ADC_SDO_Pin LL_GPIO_PIN_4
#define MCU_ADC_SDO_GPIO_Port GPIOB
#define MCU_ADC_SW1_Pin LL_GPIO_PIN_7
#define MCU_ADC_SW1_GPIO_Port GPIOB
#define MCU_ADC_SW2_Pin LL_GPIO_PIN_8
#define MCU_ADC_SW2_GPIO_Port GPIOB
#define MCU_ADC_SW3_Pin LL_GPIO_PIN_9
#define MCU_ADC_SW3_GPIO_Port GPIOB
#define MCU_REL_RESET_Pin LL_GPIO_PIN_0
#define MCU_REL_RESET_GPIO_Port GPIOE
#define MCU_REL_SET_Pin LL_GPIO_PIN_1
#define MCU_REL_SET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
