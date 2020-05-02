/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TOF_GPIO_FR_Pin GPIO_PIN_0
#define TOF_GPIO_FR_GPIO_Port GPIOC
#define TOF_GPIO_FL_Pin GPIO_PIN_1
#define TOF_GPIO_FL_GPIO_Port GPIOC
#define TOF_GPIO_R_Pin GPIO_PIN_2
#define TOF_GPIO_R_GPIO_Port GPIOC
#define TOF_GPIO_L_Pin GPIO_PIN_3
#define TOF_GPIO_L_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_4
#define BUTTON_GPIO_Port GPIOA
#define TOF_GPIO_F_Pin GPIO_PIN_4
#define TOF_GPIO_F_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_13
#define RED_LED_GPIO_Port GPIOB
#define PWML_Pin GPIO_PIN_6
#define PWML_GPIO_Port GPIOC
#define PWMR_Pin GPIO_PIN_8
#define PWMR_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ENC_L_A_Pin GPIO_PIN_15
#define ENC_L_A_GPIO_Port GPIOA
#define DIR_L_Pin GPIO_PIN_10
#define DIR_L_GPIO_Port GPIOC
#define DIR_R_Pin GPIO_PIN_12
#define DIR_R_GPIO_Port GPIOC
#define ENC_L_B_Pin GPIO_PIN_3
#define ENC_L_B_GPIO_Port GPIOB
#define ENC_R_A_Pin GPIO_PIN_6
#define ENC_R_A_GPIO_Port GPIOB
#define ENC_R_B_Pin GPIO_PIN_7
#define ENC_R_B_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
