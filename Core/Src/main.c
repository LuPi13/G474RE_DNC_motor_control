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
#include "canopen_service.h"
#include "cordic_driver.h"
#include "current_sensor.h"
#include "drive_debug_command_source.h"
#include "drive_command.h"
#include "fault_manager.h"
#include "fdcan_driver.h"
#include "hall_decoder.h"
#include "hall_driver.h"
#include "hall_estimator.h"
#include "motor_config.h"
#include "motor_control.h"
#include "pwm_driver.h"
#include "voltage_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define APP_FAST_LOOP_FREQUENCY_HZ  40000U
#define APP_PHASE_CURRENT_TRIP_ABS_A  (3.0f)
#define APP_PHASE_CURRENT_CLEAR_ABS_A (1.0f)
#define APP_DC_LINK_OVERVOLTAGE_TRIP_V  (79.2f)
#define APP_DC_LINK_OVERVOLTAGE_CLEAR_V (75.0f)
#define CURRENT_SENSOR_OFFSET_SETTLING_SAMPLE_COUNT  128U
#define CURRENT_SENSOR_OFFSET_AVERAGING_SAMPLE_COUNT 2048U
#define CURRENT_SENSOR_OFFSET_TIMEOUT_MARGIN_MS      50U
#define CURRENT_SENSOR_OFFSET_REQUIRED_SAMPLE_COUNT \
    (CURRENT_SENSOR_OFFSET_SETTLING_SAMPLE_COUNT + \
     CURRENT_SENSOR_OFFSET_AVERAGING_SAMPLE_COUNT)
#define CURRENT_SENSOR_OFFSET_EXPECTED_DURATION_MS \
    (((CURRENT_SENSOR_OFFSET_REQUIRED_SAMPLE_COUNT * 1000U) + \
      APP_FAST_LOOP_FREQUENCY_HZ - 1U) / APP_FAST_LOOP_FREQUENCY_HZ)
#define CURRENT_SENSOR_OFFSET_TIMEOUT_MS \
    (CURRENT_SENSOR_OFFSET_EXPECTED_DURATION_MS + \
     CURRENT_SENSOR_OFFSET_TIMEOUT_MARGIN_MS)
#define ACS725_10AB_GAIN_A_PER_COUNT  ((3.3f / 4096.0f) / 0.132f)
#define APP_SPEED_STOP_OMEGA_M_THRESHOLD_RAD_S       (52.3598785f)
#define APP_SPEED_STOP_DWELL_MS                       (10U)

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
static current_sensor_t current_sensor;
static voltage_sensor_t voltage_sensor;
static hall_driver_t hall_driver;
static hall_decoder_t hall_decoder;
static hall_estimator_t hall_estimator;
static motor_control_t motor_control;
static fault_manager_t fault_manager;
static fdcan_driver_t fdcan_driver;
static app_t app;
static canopen_service_t canopen_service;
static drive_command_router_t canopen_drive_command_router;

static uint32_t canopen_service_last_tick_ms;

/* ADC3 IRQ에서 완성한 세 ADC 묶음을 한 번만 fast loop에 전달한다. */
static bool adc_fast_loop_pending;

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

  const fdcan_driver_config_t fdcan_config = {
      .hfdcan = &hfdcan2,
      .receive_callback = NULL,
      .receive_context = NULL,
  };
  if (fdcan_driver_init(&fdcan_driver, &fdcan_config) !=
      FDCAN_DRIVER_STATUS_OK) {
      Error_Handler();
  }
  if (fdcan_driver_start(&fdcan_driver) != FDCAN_DRIVER_STATUS_OK) {
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

      /* 현재 CubeMX 설정의 APB1 timer kernel clock [Hz] */
      .timer_clock_hz = 170000000U,
  };

  if (hall_driver_init(&hall_driver, &hall_config) != HALL_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  if (hall_decoder_init(
      &hall_decoder,
      &motor_config_hall_profile
  ) != HALL_DECODER_STATUS_OK) {
      Error_Handler();
  }

  if (hall_driver_start(&hall_driver) != HALL_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  if (hall_estimator_init(&hall_estimator) != HALL_ESTIMATOR_STATUS_OK) {
      Error_Handler();
  }

  const float fast_loop_sampling_period_s =
      1.0f / (float)APP_FAST_LOOP_FREQUENCY_HZ;
  const motor_control_config_t motor_control_config = {
      .current_reference_min = {.d = -2.0f, .q = -2.0f},
      .current_reference_max = {.d = 2.0f, .q = 2.0f},
      .current_reference_rise_rate_per_s = {.d = 100.0f, .q = 100.0f},
      .current_reference_fall_rate_per_s = {.d = 100.0f, .q = 100.0f},
      .current_reference_magnitude_limit = 2.0f,
      .sampling_period_s = fast_loop_sampling_period_s,
      .foc = {
          .d_axis_pi = {
              .kp = 1.71530959f,
              .ki = 2623.22987f,
              .anti_windup_gain_per_s = 1529.30403f,
              .sampling_period_s = fast_loop_sampling_period_s,
              .output_min = -100.0f,
              .output_max = 100.0f,
          },
          .q_axis_pi = {
              .kp = 1.85982285f,
              .ki = 2623.22987f,
              .anti_windup_gain_per_s = 1410.47297f,
              .sampling_period_s = fast_loop_sampling_period_s,
              .output_min = -100.0f,
              .output_max = 100.0f,
          },
          .current_filter = {
              .cutoff_frequency_hz = 5000.0f,
              .sampling_period_s = fast_loop_sampling_period_s,
          },
          .voltage_utilization = 0.9f,
          .d_axis_inductance_h = 546.0e-6f,
          .q_axis_inductance_h = 592.0e-6f,
          .permanent_magnet_flux_linkage_wb = 6.74e-3f,
          .is_decoupling_enabled = false,
      },
      .speed_controller = motor_config_speed_controller,
      .speed_reference_min_rad_s = -314.159265f,
      .speed_reference_max_rad_s = 314.159265f,
      .speed_reference_rise_rate_rad_s2 = 31.415927f,
      .speed_reference_fall_rate_rad_s2 = 31.415927f,
      .pole_pairs = 5U,
  };

  if (motor_control_init(
      &motor_control,
      &motor_control_config
  ) != MOTOR_CONTROL_STATUS_OK) {
      Error_Handler();
  }

  /* PWM counter가 trigger를 발생시키기 전에 모든 ADC의 보정과 시작을 완료한다. */
  const adc_driver_config_t adc_config = {
      .phase_a = {
          .adc = &hadc2,
          .channel = ADC_CHANNEL_12,
          .injected_rank = ADC_INJECTED_RANK_1,
      },
      .phase_b = {
          .adc = &hadc3,
          .channel = ADC_CHANNEL_1,
          .injected_rank = ADC_INJECTED_RANK_1,
      },
      .phase_c = {
          .adc = &hadc1,
          .channel = ADC_CHANNEL_15,
          .injected_rank = ADC_INJECTED_RANK_1,
      },
      .injected_completion_adc = &hadc3,
      .dc_link = {
          .adc = &hadc1,
          .channel = ADC_CHANNEL_6,
      },
  };

  if (adc_driver_init(&adc_driver, &adc_config) != ADC_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  if (adc_driver_start(&adc_driver) != ADC_DRIVER_STATUS_OK) {
      Error_Handler();
  }

  const current_sensor_config_t current_sensor_config = {
      /* ACS725LLCTR-10AB-T typ. 132 mV/A와 nominal ADC Vref 3.3 V 기준. */
      .gain_a_per_count = {
          .a = ACS725_10AB_GAIN_A_PER_COUNT,
          .b = ACS725_10AB_GAIN_A_PER_COUNT,
          .c = ACS725_10AB_GAIN_A_PER_COUNT,
      },
      .settling_sample_count =
          CURRENT_SENSOR_OFFSET_SETTLING_SAMPLE_COUNT,
      .averaging_sample_count =
          CURRENT_SENSOR_OFFSET_AVERAGING_SAMPLE_COUNT,

      /* Bring-up 중 rail/단선 수준의 비정상만 거르는 넓은 허용 범위. */
      .minimum_offset_counts = 1536.0f,
      .maximum_offset_counts = 2560.0f,
  };

  if (current_sensor_init(
      &current_sensor,
      &current_sensor_config
  ) != CURRENT_SENSOR_STATUS_OK) {
      Error_Handler();
  }

  const voltage_sensor_config_t voltage_sensor_config = {
      .offset_counts = 2048.0f,
      .gain_v_per_count = 0.06448461162677f,
  };

  if (voltage_sensor_init(
      &voltage_sensor,
      &voltage_sensor_config
  ) != VOLTAGE_SENSOR_STATUS_OK) {
      Error_Handler();
  }

  const fault_manager_config_t fault_config = {
      .phase_current_trip_abs_a = APP_PHASE_CURRENT_TRIP_ABS_A,
      .phase_current_clear_abs_a = APP_PHASE_CURRENT_CLEAR_ABS_A,
      .dc_link_overvoltage_trip_v = APP_DC_LINK_OVERVOLTAGE_TRIP_V,
      .dc_link_overvoltage_clear_v = APP_DC_LINK_OVERVOLTAGE_CLEAR_V,
  };

  if (fault_manager_init(
      &fault_manager,
      &fault_config
  ) != FAULT_MANAGER_STATUS_OK) {
      Error_Handler();
  }

  /* PWM counter가 시작되기 전에 App의 ADC/PWM 연결과 안전한 0 V command를 준비한다. */
  const app_config_t app_config = {
      .adc_driver = &adc_driver,
      .current_sensor = &current_sensor,
      .voltage_sensor = &voltage_sensor,
      .pwm_driver = &pwm_driver,
      .fault_manager = &fault_manager,
      .hall_driver = &hall_driver,
      .hall_decoder = &hall_decoder,
      .hall_estimator = &hall_estimator,
      .motor_control = &motor_control,
      .fast_loop_profile = NULL,
      .motor_control_profile = NULL,
      .cycle_counter_reader = NULL,
      .sampling_period_s = fast_loop_sampling_period_s,
      .speed_loop_period_s = 0.001f,
      .current_offset_calibration_timeout_ms =
          CURRENT_SENSOR_OFFSET_TIMEOUT_MS,
      .speed_stop_omega_m_threshold_rad_s =
          APP_SPEED_STOP_OMEGA_M_THRESHOLD_RAD_S,
      .speed_stop_dwell_ms = APP_SPEED_STOP_DWELL_MS,
      .initial_voltage_angle_rad = 0.0f,
  };

  if (app_init(&app, &app_config) != APP_STATUS_OK) {
      Error_Handler();
  }

  drive_debug_command_source_init();

  /* PWM output은 끈 채 ADC fast loop가 무전류 offset을 수집하게 한다. */
  if (app_drive_start(&app) != APP_STATUS_OK) {
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
  if (pwm_driver_init(&pwm_driver, &pwm_config) != PWM_DRIVER_STATUS_OK) {
        Error_Handler();
    }

  drive_command_router_init(&canopen_drive_command_router);
  const canopen_service_motor_profile_t canopen_motor_profile = {
      .pole_pairs = motor_control_config.pole_pairs,
      .permanent_magnet_flux_linkage_wb =
          motor_control_config.foc.permanent_magnet_flux_linkage_wb,
      .torque_reference_current_peak_a =
          motor_config_canopen_torque_reference_current_peak_a,
      .maximum_mechanical_speed_rad_s =
          motor_control_config.speed_reference_max_rad_s,
  };
  const canopen_service_config_t canopen_config = {
      .fdcan_driver = &fdcan_driver,
      .app = &app,
      .drive_command_router = &canopen_drive_command_router,
      .fault_manager = &fault_manager,
      .node_id = 1U,
      .bit_rate_kbit_s = 500U,
  };
  if (canopen_service_init(
      &canopen_service,
      &canopen_config,
      &canopen_motor_profile
  ) != CANOPEN_SERVICE_STATUS_OK) {
      Error_Handler();
  }
  canopen_service_last_tick_ms = HAL_GetTick();

    /* calibration 완료와 PWM enable/disable은 이후 main-context 상태기계가 처리한다. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      (void)app_drive_update(&app);

      drive_debug_command_source_update(&app);

      const uint32_t canopen_tick_ms = HAL_GetTick();
      const uint32_t canopen_elapsed_ms =
          canopen_tick_ms - canopen_service_last_tick_ms;
      if (canopen_elapsed_ms != 0U) {
          app_speed_feedback_t speed_feedback = {0};

          canopen_service_last_tick_ms = canopen_tick_ms;
          (void)app_get_speed_feedback_snapshot(&app, &speed_feedback);
          (void)canopen_service_process(
              &canopen_service,
              canopen_elapsed_ms * 1000U,
              0.0f,
              false,
              speed_feedback.omega_e_rad_s,
              speed_feedback.has_valid_speed
          );
      }

      HAL_Delay(1U);
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
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 20;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 3;
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
  htim2.Init.Period = 999999;
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
/**
 * @brief ADC complete IRQ 후단에서 App fast loop를 실행한다.
 *
 * @note ADC3 IRQ context에서만 호출한다. App과 하위 module이 runtime/fault 상태를 소유하며
 *       main.c는 측정 결과의 별도 복사본을 만들지 않는다.
 */
static void app_fast_loop_update(void)
{
    if ((app.mode == APP_MODE_CURRENT) || (app.mode == APP_MODE_SPEED)) {
        (void)app_motor_current_fast_loop_drive_fast(&app);
    } else {
        app_fast_loop_output_t output;

        (void)app_motor_fast_loop_fast(&app, &output);
    }
}

void app_speed_scheduler_tick(void)
{
    if (!app.is_initialized) {
        return;
    }

    app_drive_scheduler_tick(&app);
}

void app_adc_irq_epilogue(void)
{
    if (!adc_fast_loop_pending) {
        return;
    }

    /* Pending을 먼저 소비해 같은 ADC 묶음을 두 IRQ 후단에서 중복 실행하지 않는다. */
    adc_fast_loop_pending = false;

    app_fast_loop_update();
}

static void app_adc_process_injected_complete(ADC_HandleTypeDef *hadc)
{
    bool is_complete = false;
    adc_driver_status_t status = adc_driver_handle_injected_complete(
        &adc_driver,
        hadc,
        &is_complete
    );

    if (status != ADC_DRIVER_STATUS_OK) {
        (void)app_handle_adc_error(&app, status);
        adc_fast_loop_pending = false;
        return;
    }

    if (is_complete) {
        /* 실제 계산은 HAL이 현재 ADC의 JEOC/JEOS를 지운 뒤 IRQ 후단에서 실행한다. */
        adc_fast_loop_pending = true;
    }
}

bool app_adc_injected_irq_try_handle_fast(ADC_HandleTypeDef *hadc)
{
    if ((hadc == NULL) ||
        (hadc != adc_driver.config.injected_completion_adc) ||
        (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC) == RESET) ||
        (__HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_JEOC) == RESET)) {
        return false;
    }

    app_adc_process_injected_complete(hadc);
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    return true;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    app_adc_process_injected_complete(hadc);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != &hadc1) && (hadc != &hadc2) && (hadc != &hadc3)) {
        return;
    }

    (void)app_handle_adc_error(
        &app,
        ADC_DRIVER_STATUS_HAL_ERROR
    );
    adc_fast_loop_pending = false;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) {
        return;
    }

    (void)hall_driver_handle_capture(
        &hall_driver,
        htim
    );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) {
        return;
    }

    (void)hall_driver_handle_timeout(
        &hall_driver,
        htim
    );
}

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t interrupt_flags
)
{
    fdcan_driver_handle_rx_fifo0(
        &fdcan_driver,
        hfdcan,
        interrupt_flags
    );
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    fdcan_driver_handle_error(&fdcan_driver, hfdcan);
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
