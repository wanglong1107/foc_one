/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#define HW_VERSION_MAJOR 3
#define HW_VERSION_MINOR 4
#define HW_VERSION_VOLTAGE 48

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#include "prev_board_ver/main_V3_2.h"
#else
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
#define TIM_1_8_CLOCK_HZ 168000000
#define TIM_1_8_PERIOD_CLOCKS 10192
#define TIM_1_8_DEADTIME_CLOCKS 20
#define TIM_APB1_CLOCK_HZ 84000000
#define TIM_APB1_PERIOD_CLOCKS 4096
#define TIM_APB1_DEADTIME_CLOCKS 40
#define M0_nCS_Pin GPIO_PIN_13
#define M0_nCS_GPIO_Port GPIOC
#define M1_nCS_Pin GPIO_PIN_14
#define M1_nCS_GPIO_Port GPIOC
#define M1_DC_CAL_Pin GPIO_PIN_15
#define M1_DC_CAL_GPIO_Port GPIOC
#define M0_IB_Pin GPIO_PIN_0
#define M0_IB_GPIO_Port GPIOC
#define M0_IC_Pin GPIO_PIN_1
#define M0_IC_GPIO_Port GPIOC
#define GPIO_1_Pin GPIO_PIN_0
#define GPIO_1_GPIO_Port GPIOA
#define GPIO_2_Pin GPIO_PIN_1
#define GPIO_2_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_2
#define GPIO_3_GPIO_Port GPIOA
#define GPIO_3_EXTI_IRQn EXTI2_IRQn
#define PWM_IN_Pin GPIO_PIN_3
#define PWM_IN_GPIO_Port GPIOA
#define M1_TEMP_Pin GPIO_PIN_4
#define M1_TEMP_GPIO_Port GPIOA
#define AUX_I_Pin GPIO_PIN_5
#define AUX_I_GPIO_Port GPIOA
#define VBUS_S_Pin GPIO_PIN_6
#define VBUS_S_GPIO_Port GPIOA
#define M1_AL_Pin GPIO_PIN_7
#define M1_AL_GPIO_Port GPIOA
#define AUX_TEMP_Pin GPIO_PIN_4
#define AUX_TEMP_GPIO_Port GPIOC
#define M0_TEMP_Pin GPIO_PIN_5
#define M0_TEMP_GPIO_Port GPIOC
#define M1_BL_Pin GPIO_PIN_0
#define M1_BL_GPIO_Port GPIOB
#define M1_CL_Pin GPIO_PIN_1
#define M1_CL_GPIO_Port GPIOB
#define GPIO_5_Pin GPIO_PIN_2
#define GPIO_5_GPIO_Port GPIOB
#define AS_nCS_Pin GPIO_PIN_11
#define AS_nCS_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define M0_AL_Pin GPIO_PIN_13
#define M0_AL_GPIO_Port GPIOB
#define M0_BL_Pin GPIO_PIN_14
#define M0_BL_GPIO_Port GPIOB
#define M0_CL_Pin GPIO_PIN_15
#define M0_CL_GPIO_Port GPIOB
#define M1_AH_Pin GPIO_PIN_6
#define M1_AH_GPIO_Port GPIOC
#define M1_BH_Pin GPIO_PIN_7
#define M1_BH_GPIO_Port GPIOC
#define M1_CH_Pin GPIO_PIN_8
#define M1_CH_GPIO_Port GPIOC
#define M0_DC_CAL_Pin GPIO_PIN_9
#define M0_DC_CAL_GPIO_Port GPIOC
#define M0_AH_Pin GPIO_PIN_8
#define M0_AH_GPIO_Port GPIOA
#define M0_BH_Pin GPIO_PIN_9
#define M0_BH_GPIO_Port GPIOA
#define M0_CH_Pin GPIO_PIN_10
#define M0_CH_GPIO_Port GPIOA
#define M0_ENC_Z_Pin GPIO_PIN_15
#define M0_ENC_Z_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_2
#define nFAULT_GPIO_Port GPIOD
#define M1_ENC_Z_Pin GPIO_PIN_3
#define M1_ENC_Z_GPIO_Port GPIOB
#define M0_ENC_A_Pin GPIO_PIN_4
#define M0_ENC_A_GPIO_Port GPIOB
#define M0_ENC_B_Pin GPIO_PIN_5
#define M0_ENC_B_GPIO_Port GPIOB
#define M1_ENC_A_Pin GPIO_PIN_6
#define M1_ENC_A_GPIO_Port GPIOB
#define M1_ENC_B_Pin GPIO_PIN_7
#define M1_ENC_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#endif

//TODO: make this come automatically out of CubeMX somehow
#define TIM_TIME_BASE TIM14
#define CURRENT_MEAS_PERIOD ((float)(2*TIM_1_8_PERIOD_CLOCKS)/(float)TIM_1_8_CLOCK_HZ)
#define CURRENT_MEAS_HZ (TIM_1_8_CLOCK_HZ/(2*TIM_1_8_PERIOD_CLOCKS))

#if HW_VERSION_VOLTAGE == 48
#define VBUS_S_DIVIDER_RATIO 19.0f  
#elif HW_VERSION_VOLTAGE == 24
#define VBUS_S_DIVIDER_RATIO 11.0f  //11.0  20.0
#else
#error "unknown board voltage"
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
