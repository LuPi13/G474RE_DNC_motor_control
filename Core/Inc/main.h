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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
 * @brief 정상 injected 완료 IRQ를 수집하고 fast loop까지 경량 경로로 실행한다.
 * @param[in] hadc IRQ가 발생한 ADC handle.
 * @return 정상 completion ADC JEOC를 처리했으면 true, HAL fallback이 필요하면 false.
 * @note true이면 현재 완료 flag 정리, App fast loop와 IRQ cycle 기록까지 완료한 상태다.
 */
bool app_adc_injected_irq_handle_fast(ADC_HandleTypeDef *hadc);

/**
 * @brief HAL fallback callback이 남긴 pending fast-loop를 실행한다.
 * @note 정상 JEOC 경로는 app_adc_injected_irq_handle_fast()에서 직접 완료한다.
 */
void app_adc_irq_epilogue(void);

/**
 * @brief 1 kHz SysTick에서 speed PI scheduler를 실행한다.
 * @note SysTick_Handler()의 USER CODE 영역에서만 호출한다.
 */
void app_speed_scheduler_tick(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBUS_sense_Pin GPIO_PIN_9
#define VBUS_sense_GPIO_Port GPIOA
#define DP_pullup_Pin GPIO_PIN_10
#define DP_pullup_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
