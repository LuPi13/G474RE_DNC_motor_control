/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_driver.h"
#include "app.h"
#include "cordic_driver.h"
#include "hall_driver.h"
#include "hall_estimator.h"
#include "pwm_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define APP_FAST_LOOP_FREQUENCY_HZ  40000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan2;

HRTIM_HandleTypeDef hhrtim1;

TIM_HandleTypeDef htim2;

USART_HandleTypeDef husart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
static pwm_driver_t pwm_driver;
static adc_driver_t adc_driver;
static hall_driver_t hall_driver;
static hall_estimator_t hall_estimator;
static app_t app;

/* HRTIM Timer C reset과 동기화된 현재 40 kHz fast-loop 주기 [s]. */
static const float hall_estimator_test_period_s =
    1.0f / (float)APP_FAST_LOOP_FREQUENCY_HZ;

static abc_t duty_abc = {
    .a = 0.50f,
    .b = 0.50f,
    .c = 0.50f,
};

/* 디버거에서 함수 실행 결과 확인용 */
static volatile pwm_driver_status_t pwm_test_status;

/* ADC Live Expressions 확인용. 오류는 이후 수집에 성공해도 지우지 않는다. */
static volatile adc_driver_status_t adc_test_last_error;
static volatile uint32_t adc_test_sample_count;
static volatile uint32_t adc_test_not_ready_count;
static volatile adc_driver_raw_sample_t adc_test_raw;
static volatile abc_t adc_test_i_abc;
static volatile float adc_test_v_dc;

/* Open-loop App Live Expressions 확인용. last_error는 정상 주기에도 유지한다. */
static volatile app_status_t app_test_status;
static volatile app_status_t app_test_last_error;

/* ADC IRQ 후처리와 fast-loop 실행시간 확인용 */
static volatile bool adc_fast_loop_pending;
static volatile uint32_t adc_fast_loop_budget_cycles;
static volatile bool adc_fast_loop_cycle_measurement_active;
static volatile uint32_t adc_fast_loop_cycle_start;
static volatile uint32_t adc_fast_loop_cycles_last;
static volatile uint32_t adc_fast_loop_cycles_max;
static volatile uint32_t adc_fast_loop_deadline_miss_count;
static volatile uint32_t adc_fast_loop_body_cycles_last;
static volatile uint32_t adc_fast_loop_body_cycles_max;

/* Hall sensor Live Expressions 확인용 */
static volatile hall_driver_status_t hall_test_init_status;
static volatile hall_driver_status_t hall_test_start_status;
static volatile hall_driver_status_t hall_test_capture_status;
static volatile hall_driver_status_t hall_test_timeout_status;
static volatile hall_driver_status_t hall_test_feedback_status;
static volatile hall_driver_feedback_t hall_test_feedback;

/* Hall 연속 전기각 추정 Live Expressions 확인용 */
static volatile hall_estimator_status_t hall_estimator_test_init_status;
static volatile hall_estimator_status_t hall_estimator_test_update_status;
static volatile hall_driver_status_t hall_estimator_test_feedback_status;
static volatile hall_estimator_output_t hall_estimator_test_output;
static volatile uint32_t hall_estimator_test_update_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_CORDIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void hall_test_refresh_feedback(void)
{
    hall_driver_feedback_t feedback;

    hall_test_feedback_status = hall_driver_get_feedback(
        &hall_driver,
        &feedback
    );

    if (hall_test_feedback_status == HALL_DRIVER_STATUS_OK) {
        hall_test_feedback = feedback;
    }
}

/* Cortex-M4 DWT cycle counter를 켜고 현재 CPU clock 기준 fast-loop budget을 계산한다. */
static void adc_fast_loop_timing_test_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    adc_fast_loop_budget_cycles =
        SystemCoreClock / APP_FAST_LOOP_FREQUENCY_HZ;
}

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

  adc_fast_loop_timing_test_init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();
  MX_USART3_Init();
  MX_USB_PCD_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  if (cordic_driver_init() != CORDIC_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  const hall_driver_config_t hall_config = {
      .timer = &htim2,

      /* Hall state bit 순서: A/B/C = bit 2/1/0 */
      .hall_a = {
          .port = GPIOA,
          .pin = GPIO_PIN_0,
      },
      .hall_b = {
          .port = GPIOA,
          .pin = GPIO_PIN_1,
      },
      .hall_c = {
          .port = GPIOA,
          .pin = GPIO_PIN_2,
      },

      /* 사용자 정의 정방향: 101 -> 100 -> 110 -> 010 -> 011 -> 001 */
      .hall_state_by_sector = {
          0x5U,
          0x4U,
          0x6U,
          0x2U,
          0x3U,
          0x1U,
      },

      /* 현재 CubeMX 설정의 APB1 timer kernel clock [Hz] */
      .timer_clock_hz = 170000000U,

      /* Bring-up용 임시 기준. 실제 FOC 적용 전 rotor flux 기준으로 보정한다. */
      .electrical_offset_rad = 0.0f,
  };

  hall_test_init_status = hall_driver_init(&hall_driver, &hall_config);
  if (hall_test_init_status != HALL_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  hall_test_start_status = hall_driver_start(&hall_driver);
  if (hall_test_start_status != HALL_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  hall_test_refresh_feedback();

  hall_estimator_test_init_status = hall_estimator_init(&hall_estimator);
  if (hall_estimator_test_init_status != HALL_ESTIMATOR_STATUS_OK) {
      Error_Handler();
  }

  /* PWM counter가 trigger를 발생시키기 전에 모든 ADC의 보정과 시작을 완료한다. */
  const adc_driver_config_t adc_config = {
      .phase_a = {
          .adc = &hadc2,
          .channel = ADC_CHANNEL_12,
          .injected_rank = ADC_INJECTED_RANK_1,
          .offset_counts = 2048.0f,
          .gain_a_per_count = 1.0f / 163.8f,
      },
      .phase_b = {
          .adc = &hadc3,
          .channel = ADC_CHANNEL_1,
          .injected_rank = ADC_INJECTED_RANK_1,
          .offset_counts = 2048.0f,
          .gain_a_per_count = 1.0f / 163.8f,
      },
      .phase_c = {
          .adc = &hadc1,
          .channel = ADC_CHANNEL_15,
          .injected_rank = ADC_INJECTED_RANK_1,
          .offset_counts = 2048.0f,
          .gain_a_per_count = 1.0f / 163.8f,
      },
      .dc_link = {
          .adc = &hadc1,
          .channel = ADC_CHANNEL_6,
          .offset_counts = 2048.0f,
          .gain_v_per_count = 0.06448461162677f,
      },
  };

  adc_test_last_error = adc_driver_init(&adc_driver, &adc_config);
  if (adc_test_last_error != ADC_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  adc_test_last_error = adc_driver_start(&adc_driver);
  if (adc_test_last_error != ADC_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  /* PWM counter가 시작되기 전에 App의 ADC/PWM 연결과 안전한 0 V command를 준비한다. */
  const app_config_t app_config = {
      .adc_driver = &adc_driver,
      .pwm_driver = &pwm_driver,
      .sampling_period_s = 1.0f / (float)APP_FAST_LOOP_FREQUENCY_HZ,
      .initial_voltage_angle_rad = 0.0f,
  };

  app_test_status = app_init(&app, &app_config);
  if (app_test_status != APP_STATUS_OK) {
      app_test_last_error = app_test_status;
      Error_Handler();
  }

  /* PWM 초기화 */
  const pwm_driver_config_t pwm_config = {
        .hrtim = &hhrtim1,

        .phase_a = {
            .timer_index = HRTIM_TIMERINDEX_TIMER_F,
            .compare_unit = HRTIM_COMPAREUNIT_1,
        },
        .phase_b = {
            .timer_index = HRTIM_TIMERINDEX_TIMER_D,
            .compare_unit = HRTIM_COMPAREUNIT_1,
        },
        .phase_c = {
            .timer_index = HRTIM_TIMERINDEX_TIMER_C,
            .compare_unit = HRTIM_COMPAREUNIT_1,
        },
  };
  pwm_test_status = pwm_driver_init(&pwm_driver, &pwm_config);
    if (pwm_test_status != PWM_DRIVER_STATUS_OK) {
        Error_Handler();
    }

    /* Output을 켜기 전에 초기 duty를 먼저 기록한다. */
    pwm_test_status = pwm_driver_set_duty(&pwm_driver, &duty_abc);
    if (pwm_test_status != PWM_DRIVER_STATUS_OK) {
        Error_Handler();
    }

    /* 현재 command는 0 V, 0 rad/s다. 실제 구동값은 App API로 별도 설정한다. */
    app_test_status = app_start_open_loop(&app);
    if (app_test_status != APP_STATUS_OK) {
        app_test_last_error = app_test_status;
        Error_Handler();
    }

    pwm_test_status = pwm_driver_enable(&pwm_driver);
    if (pwm_test_status != PWM_DRIVER_STATUS_OK) {
        Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_C;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERC_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERC_RESET;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 34000;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 17000;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL4;
  pDeadTimeCfg.RisingValue = 500;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
  pDeadTimeCfg.FallingValue = 500;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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

  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart3.SlaveMode = USART_SLAVEMODE_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart3, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart3, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_DisableFifoMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DP_pullup_GPIO_Port, DP_pullup_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VBUS_sense_Pin */
  GPIO_InitStruct.Pin = VBUS_sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_sense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DP_pullup_Pin */
  GPIO_InitStruct.Pin = DP_pullup_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DP_pullup_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void hall_estimator_test_update(void)
{
    hall_driver_rotor_feedback_t hall_feedback;
    hall_estimator_output_t estimator_output;

    /* Fast-loop에서는 diagnostic field를 제외한 rotor feedback만 snapshot으로 읽는다. */
    hall_estimator_test_feedback_status = hall_driver_get_rotor_feedback(
        &hall_driver,
        &hall_feedback
    );

    if (hall_estimator_test_feedback_status != HALL_DRIVER_STATUS_OK) {
        return;
    }

    const hall_estimator_observation_t observation = {
        .theta_e_rad = hall_feedback.theta_e_rad,
        .omega_e_rad_s = hall_feedback.omega_e_rad_s,
        .transition_count = hall_feedback.transition_count,
        .sector = hall_feedback.sector,
        .has_valid_state = hall_feedback.has_valid_state,
        .has_valid_direction = hall_feedback.has_valid_direction,
        .has_valid_angle = hall_feedback.has_valid_angle,
        .has_valid_speed = hall_feedback.has_valid_speed,
        .is_angle_from_edge = hall_feedback.is_angle_from_edge,
        .is_timed_out = hall_feedback.is_timed_out,
    };

    hall_estimator_test_update_status = hall_estimator_update(
        &hall_estimator,
        &observation,
        hall_estimator_test_period_s,
        &estimator_output
    );

    if (hall_estimator_test_update_status == HALL_ESTIMATOR_STATUS_OK) {
        hall_estimator_test_output = estimator_output;
        ++hall_estimator_test_update_count;
    }
}

/* App fast loop 결과와 Hall 연속각을 debugger에서 함께 확인한다. */
static void app_fast_loop_test_update(void)
{
    app_fast_loop_output_t output;

    app_test_status = app_motor_fast_loop(&app, &output);
    if (app_test_status == APP_STATUS_ADC_NOT_READY) {
        /* 시작 직후 전압이 아직 변환되지 않은 경우 등을 관찰한다. */
        ++adc_test_not_ready_count;
        return;
    }

    if (app_test_status != APP_STATUS_OK) {
        app_test_last_error = app_test_status;
        if (app.last_adc_status != ADC_DRIVER_STATUS_OK) {
            adc_test_last_error = app.last_adc_status;
        }
        return;
    }

    /* App이 실제 사용한 측정 묶음만 기존 debugger 관찰 변수에 복사한다. */
    adc_test_raw = output.raw;
    adc_test_i_abc = output.i_abc;
    adc_test_v_dc = output.v_dc;
    ++adc_test_sample_count;

    hall_estimator_test_update();
}

void app_adc_irq_prologue(void)
{
    /* 측정 자체가 IRQ deadline에 주는 영향을 최소화한다. */
    if (!adc_fast_loop_cycle_measurement_active) {
        adc_fast_loop_cycle_start = DWT->CYCCNT;
        adc_fast_loop_cycle_measurement_active = true;
    }
}

void app_adc_irq_epilogue(void)
{
    if (!adc_fast_loop_pending) {
        /* 처리한 ADC event가 없는 shared IRQ 재진입은 측정 묶음에서 제외한다. */
        if ((adc_driver.complete_mask == 0U) && !adc_driver.is_sample_ready) {
            adc_fast_loop_cycle_measurement_active = false;
        }
        return;
    }

    /* Pending을 먼저 소비해 같은 ADC 묶음을 두 IRQ 후단에서 중복 실행하지 않는다. */
    adc_fast_loop_pending = false;

    const uint32_t body_start_cycles = DWT->CYCCNT;
    app_fast_loop_test_update();
    const uint32_t end_cycles = DWT->CYCCNT;
    const uint32_t body_elapsed_cycles = end_cycles - body_start_cycles;

    adc_fast_loop_body_cycles_last = body_elapsed_cycles;
    if (body_elapsed_cycles > adc_fast_loop_body_cycles_max) {
        adc_fast_loop_body_cycles_max = body_elapsed_cycles;
    }

    if (!adc_fast_loop_cycle_measurement_active) {
        return;
    }

    /* 첫 ADC IRQ 진입부터 HAL 처리와 fast-loop 종료까지를 측정한다. */
    const uint32_t elapsed_cycles = end_cycles - adc_fast_loop_cycle_start;
    adc_fast_loop_cycle_measurement_active = false;

    adc_fast_loop_cycles_last = elapsed_cycles;
    if (elapsed_cycles > adc_fast_loop_cycles_max) {
        adc_fast_loop_cycles_max = elapsed_cycles;
    }

    if (elapsed_cycles >= adc_fast_loop_budget_cycles) {
        ++adc_fast_loop_deadline_miss_count;
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    bool is_complete = false;
    adc_driver_status_t status = adc_driver_handle_injected_complete(
        &adc_driver,
        hadc,
        &is_complete
    );

    if (status != ADC_DRIVER_STATUS_OK) {
        adc_test_last_error = status;
        app_test_status = app_handle_adc_error(&app, status);
        app_test_last_error = app_test_status;
        adc_fast_loop_pending = false;
        adc_fast_loop_cycle_measurement_active = false;
        return;
    }

    if (is_complete) {
        /* 실제 계산은 HAL이 현재 ADC의 JEOC/JEOS를 지운 뒤 IRQ 후단에서 실행한다. */
        adc_fast_loop_pending = true;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != &hadc1) && (hadc != &hadc2) && (hadc != &hadc3)) {
        return;
    }

    adc_test_last_error = ADC_DRIVER_STATUS_HAL_ERROR;
    app_test_status = app_handle_adc_error(
        &app,
        ADC_DRIVER_STATUS_HAL_ERROR
    );
    app_test_last_error = app_test_status;
    adc_fast_loop_pending = false;
    adc_fast_loop_cycle_measurement_active = false;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) {
        return;
    }

    hall_test_capture_status = hall_driver_handle_capture(
        &hall_driver,
        htim
    );

    hall_test_refresh_feedback();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) {
        return;
    }

    hall_test_timeout_status = hall_driver_handle_timeout(
        &hall_driver,
        htim
    );

    hall_test_refresh_feedback();
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
#ifdef USE_FULL_ASSERT
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
