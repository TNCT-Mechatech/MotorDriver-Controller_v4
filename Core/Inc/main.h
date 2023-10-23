/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define CURRENT_SENSOR_1_Pin GPIO_PIN_0
#define CURRENT_SENSOR_1_GPIO_Port GPIOC
#define CURRENT_SENSOR_2_Pin GPIO_PIN_1
#define CURRENT_SENSOR_2_GPIO_Port GPIOC
#define CURRENT_SENSOR_3_Pin GPIO_PIN_2
#define CURRENT_SENSOR_3_GPIO_Port GPIOC
#define CURRENT_SENSOR_4_Pin GPIO_PIN_3
#define CURRENT_SENSOR_4_GPIO_Port GPIOC
#define PWM_2_Pin GPIO_PIN_1
#define PWM_2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define EMERGENCT_STOP_Pin GPIO_PIN_4
#define EMERGENCT_STOP_GPIO_Port GPIOA
#define THERMISTOR_Pin GPIO_PIN_5
#define THERMISTOR_GPIO_Port GPIOA
#define ENCODER2_CH1_Pin GPIO_PIN_6
#define ENCODER2_CH1_GPIO_Port GPIOA
#define ENCODER2_CH2_Pin GPIO_PIN_7
#define ENCODER2_CH2_GPIO_Port GPIOA
#define DIR_1_Pin GPIO_PIN_4
#define DIR_1_GPIO_Port GPIOC
#define DIR_2_Pin GPIO_PIN_5
#define DIR_2_GPIO_Port GPIOC
#define DIR_3_Pin GPIO_PIN_0
#define DIR_3_GPIO_Port GPIOB
#define DIR_4_Pin GPIO_PIN_1
#define DIR_4_GPIO_Port GPIOB
#define PWM_4_Pin GPIO_PIN_2
#define PWM_4_GPIO_Port GPIOB
#define PWM_3_Pin GPIO_PIN_10
#define PWM_3_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define ENCODER4_CH1_Pin GPIO_PIN_6
#define ENCODER4_CH1_GPIO_Port GPIOC
#define ENCODER4_CH2_Pin GPIO_PIN_7
#define ENCODER4_CH2_GPIO_Port GPIOC
#define ENCODER1_CH1_Pin GPIO_PIN_8
#define ENCODER1_CH1_GPIO_Port GPIOA
#define ENCODER1_CH2_Pin GPIO_PIN_9
#define ENCODER1_CH2_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_10
#define DEBUG_LED_GPIO_Port GPIOA
#define COM_LED_Pin GPIO_PIN_11
#define COM_LED_GPIO_Port GPIOA
#define ALIVE_LED_Pin GPIO_PIN_12
#define ALIVE_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_15
#define PWM_1_GPIO_Port GPIOA
#define DIP_SW_4_Pin GPIO_PIN_10
#define DIP_SW_4_GPIO_Port GPIOC
#define DIP_SW_3_Pin GPIO_PIN_11
#define DIP_SW_3_GPIO_Port GPIOC
#define DIP_SW_2_Pin GPIO_PIN_12
#define DIP_SW_2_GPIO_Port GPIOC
#define DIP_SW_1_Pin GPIO_PIN_2
#define DIP_SW_1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DEBUG_BUTTON_Pin GPIO_PIN_4
#define DEBUG_BUTTON_GPIO_Port GPIOB
#define DEBUG_SWITCH_Pin GPIO_PIN_5
#define DEBUG_SWITCH_GPIO_Port GPIOB
#define ENCODER3_CH1_Pin GPIO_PIN_6
#define ENCODER3_CH1_GPIO_Port GPIOB
#define ENCODER3_CH2_Pin GPIO_PIN_7
#define ENCODER3_CH2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
