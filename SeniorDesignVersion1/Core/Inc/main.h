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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

// Define GPIO pins to enable power supplies
// ... is the placeholder for actual GPIO port
#define POWER_SUPPLY_15V_ENABLE_PIN ...
#define POWER_SUPPLY_3V3_ENABLE_PIN ...
#define POWER_SUPPLY_500V_ENABLE_PIN ...
#define H_BRIDGE_ENABLE_PIN ...

// ADC Channel definitions
#define ADC_CHANNEL_15V  ADC_CHANNEL_0 // Replace with the actual channel number
#define ADC_CHANNEL_3V3  ADC_CHANNEL_1 // Replace with the actual channel number
#define ADC_CHANNEL_500V ADC_CHANNEL_2 // Replace with the actual channel number
#define ADC_CHANNEL_1KV2KV ADC_CHANNEL_3 // Replace with the actual channel number

// Define voltage thresholds
// minimum acceptable voltage should be (Vin/Vref)*(2^(ADC bit resolution)  - 1)
#define VOLTAGE_15V_THRESHOLD  ... // The minimum acceptable voltage reading for 15V
#define VOLTAGE_3V3_THRESHOLD  ... // The minimum acceptable voltage reading for 3.3V
#define VOLTAGE_500V_THRESHOLD ... // The minimum acceptable voltage reading for 500V
#define VOLTAGE_1KV2KV_THRESHOLD ... // The minimum acceptable voltage reading for 1-2kVrms

// Constants for adjustments
#define K_1 ... // Define your constant for frequency adjustment
#define K_2 ... // Define your constant for dead time adjustment
#define TIM_PERIOD_MAX ... // Define the maximum value for TIM1's period

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
