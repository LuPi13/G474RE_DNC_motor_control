/**
 * @file hall_driver.h
 * @brief TIM Hall Sensor Interface 기반 raw Hall signal 수집 API.
 * @ingroup platform_hall_driver
 */

#ifndef PLATFORM_HALL_DRIVER_H
#define PLATFORM_HALL_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/**
 * @defgroup platform_hall_driver Hall driver
 * @brief 3개 Hall GPIO의 raw state와 TIM edge interval을 일관된 snapshot으로 제공한다.
 *
 * CubeMX는 TIM Hall Sensor Interface, GPIO alternate function, input filter, prescaler,
 * auto-reload와 NVIC를 설정한다. Driver는 GPIO/TIM mapping을 검증하고 capture/timeout을
 * 수집하지만 raw state의 유효성, motor sector, 방향 또는 electrical angle은 판단하지 않는다.
 * 이 motor-specific 의미는 Control의 hall_decoder와 hall_decoder_profile_t가 소유한다.
 *
 * Hall TIM writer보다 높은 priority의 ADC fast-loop reader가 writer를 선점해도 부분적으로
 * 갱신된 field가 노출되지 않도록 두 feedback buffer와 원자적인 active index publish를 사용한다.
 * @{
 */

/** @brief Hall driver 함수의 실행 결과. */
typedef enum {
    HALL_DRIVER_STATUS_OK = 0, /**< 요청한 처리를 정상적으로 완료함. */
    HALL_DRIVER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 다른 TIM handle. */
    HALL_DRIVER_STATUS_INVALID_CONFIG, /**< GPIO/clock/CubeMX TIM 설정 오류. */
    HALL_DRIVER_STATUS_INVALID_STATE, /**< 실행 상태가 요청과 맞지 않음. */
    HALL_DRIVER_STATUS_INVALID_CAPTURE, /**< CH1이 아니거나 capture tick이 0임. */
    HALL_DRIVER_STATUS_HAL_ERROR /**< HAL start/stop 실패. */
} hall_driver_status_t;

/** @brief 논리 Hall 입력 하나의 GPIO mapping. */
typedef struct {
    GPIO_TypeDef *port; /**< Hall pin이 연결된 GPIO port. */
    uint16_t pin;       /**< 정확히 하나의 GPIO_PIN_* bit. */
} hall_driver_input_config_t;

/** @brief Board별 Hall peripheral mapping과 timer clock 설정. */
typedef struct {
    TIM_HandleTypeDef *timer; /**< CubeMX가 Hall Sensor Interface로 초기화한 TIM handle. */
    hall_driver_input_config_t hall_a; /**< Raw state bit 2. */
    hall_driver_input_config_t hall_b; /**< Raw state bit 1. */
    hall_driver_input_config_t hall_c; /**< Raw state bit 0. */
    uint32_t timer_clock_hz; /**< Prescaler 적용 전 TIM kernel clock [Hz]. */
} hall_driver_config_t;

/**
 * @brief 최신 raw Hall signal과 hardware diagnostic snapshot.
 *
 * edge_interval_s는 직전 capture부터 현재 capture까지의 실제 시간이며
 * has_valid_interval이 true일 때만 decoder의 speed 계산에 사용한다. Start, timeout,
 * capture tick 0 또는 overflow와 capture가 겹친 경우에는 interval을 무효화한다.
 */
typedef struct {
    uint8_t hall_state;       /**< A/B/C = bit 2/1/0인 raw state, 범위 [0, 7]. */
    uint32_t capture_ticks;   /**< 최신 TIM CH1 capture 값 [counter tick]. */
    float edge_interval_s;    /**< 직전 edge부터 현재 edge까지의 시간 [s]. */
    uint32_t capture_count;   /**< 수락한 CH1 Hall capture 누적 횟수. */
    uint32_t invalid_capture_count; /**< 0 tick 등 interval을 사용할 수 없던 capture 횟수. */
    uint32_t timeout_count;   /**< Hall edge timeout 진입 횟수. */
    bool has_state_sample;    /**< hall_state가 실제 GPIO에서 읽힌 값임. */
    bool has_valid_interval;  /**< edge_interval_s를 사용할 수 있음. */
    bool is_timed_out;        /**< Hall edge 없이 timer overflow가 발생함. */
} hall_driver_feedback_t;

/** @brief Fast loop가 motor-independent raw signal만 읽기 위한 경량 snapshot. */
typedef struct {
    uint8_t hall_state; /**< A/B/C = bit 2/1/0인 raw state. */
    uint32_t capture_count; /**< Hall edge마다 증가하는 sequence. */
    float edge_interval_s; /**< 유효할 때 직전 edge와의 시간 [s]. */
    bool has_state_sample; /**< hall_state가 실제 GPIO sample임. */
    bool has_valid_interval; /**< edge_interval_s를 사용할 수 있음. */
    bool is_timed_out; /**< Hall edge 없이 timer overflow가 발생함. */
} hall_driver_signal_feedback_t;

/** @brief Hall peripheral 설정과 ISR-to-fast-loop raw snapshot state. */
typedef struct {
    hall_driver_config_t config; /**< 초기화 시 복사한 peripheral mapping. */
    float counter_frequency_hz; /**< Prescaler 적용 후 counter 주파수 [Hz]. */
    float timeout_s; /**< Auto-reload가 나타내는 timeout [s]. */
    volatile hall_driver_feedback_t feedback_buffer[2]; /**< ISR publish용 완성 snapshot 두 벌. */
    volatile uint32_t active_feedback_index; /**< Reader에 공개된 buffer index. */
    volatile bool has_valid_interval_reference; /**< 다음 capture interval의 시작 edge가 있음. */
    volatile bool capture_with_pending_timeout; /**< Capture와 overflow가 같은 IRQ에 pending이었음. */
    volatile bool is_initialized; /**< Mapping/TIM 검증 완료 여부. */
    volatile bool is_running; /**< Capture/timeout interrupt 실행 여부. */
} hall_driver_t;

/**
 * @brief Hall GPIO/TIM mapping을 검증하고 raw signal driver를 초기화한다.
 * @param[in,out] self 초기화할 driver instance.
 * @param[in] config Board별 TIM/GPIO mapping과 timer kernel clock.
 * @pre CubeMX GPIO와 HAL_TIMEx_HallSensor_Init()가 완료되고 TIM은 정지 상태여야 한다.
 * @retval HALL_DRIVER_STATUS_OK 초기화 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL handle 또는 instance.
 * @retval HALL_DRIVER_STATUS_INVALID_CONFIG GPIO/clock/TIM 설정 오류.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE 실행 중인 instance 또는 TIM.
 */
hall_driver_status_t hall_driver_init(
    hall_driver_t *self,
    const hall_driver_config_t *config
);

/**
 * @brief Hall capture와 counter-overflow timeout interrupt를 시작한다.
 * @param[in,out] self 초기화된 driver instance.
 * @post 현재 raw GPIO state를 publish하고 capture/update interrupt를 시작한다.
 * @retval HALL_DRIVER_STATUS_OK 시작 완료 또는 이미 실행 중임.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL 또는 초기화되지 않은 instance.
 * @retval HALL_DRIVER_STATUS_HAL_ERROR HAL Hall Sensor Interface 시작 실패.
 */
hall_driver_status_t hall_driver_start(hall_driver_t *self);

/**
 * @brief Hall capture와 timeout interrupt를 정지하고 interval validity를 해제한다.
 * @param[in,out] self 초기화된 driver instance.
 * @retval HALL_DRIVER_STATUS_OK 정지 완료 또는 이미 정지 상태임.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL 또는 초기화되지 않은 instance.
 * @retval HALL_DRIVER_STATUS_HAL_ERROR HAL Hall Sensor Interface 정지 실패.
 */
hall_driver_status_t hall_driver_stop(hall_driver_t *self);

/**
 * @brief TIM CH1 Hall capture에서 raw state와 edge interval을 수집한다.
 * @param[in,out] self 실행 중인 driver instance.
 * @param[in] htim HAL_TIM_IC_CaptureCallback()에서 받은 TIM handle.
 * @note Raw state 000/111을 포함한 모든 3-bit 조합을 그대로 publish한다.
 * @retval HALL_DRIVER_STATUS_OK Raw state와 capture interval publish 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL, 초기화되지 않은 instance 또는 다른 TIM.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 * @retval HALL_DRIVER_STATUS_INVALID_CAPTURE CH1 callback이 아니거나 capture tick이 0임.
 */
hall_driver_status_t hall_driver_handle_capture(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
);

/**
 * @brief Hall edge 없이 발생한 timer overflow를 timeout으로 publish한다.
 * @param[in,out] self 실행 중인 driver instance.
 * @param[in] htim HAL_TIM_PeriodElapsedCallback()에서 받은 TIM handle.
 * @note Capture와 overflow가 같은 IRQ에 pending이면 capture 기준은 보존하고 interval만 무효화한다.
 * @retval HALL_DRIVER_STATUS_OK Timeout 상태 publish 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL, 초기화되지 않은 instance 또는 다른 TIM.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 */
hall_driver_status_t hall_driver_handle_timeout(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
);

/**
 * @brief 최신 raw Hall signal과 hardware diagnostic 전체를 복사한다.
 * @param[in] self 초기화된 driver instance.
 * @param[out] feedback 최신 완성 snapshot.
 * @retval HALL_DRIVER_STATUS_OK Snapshot 복사 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL 또는 초기화되지 않은 instance.
 */
hall_driver_status_t hall_driver_get_feedback(
    const hall_driver_t *self,
    hall_driver_feedback_t *feedback
);

/**
 * @brief Fast loop에 필요한 raw Hall signal field만 복사한다.
 * @param[in] self 초기화된 driver instance.
 * @param[out] feedback 최신 경량 signal snapshot.
 * @details Active index를 한 번 읽은 뒤 같은 buffer의 field만 복사하므로 ISR writer의
 *          부분 갱신은 노출되지 않는다.
 * @retval HALL_DRIVER_STATUS_OK Snapshot 복사 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL 또는 초기화되지 않은 instance.
 */
hall_driver_status_t hall_driver_get_signal_feedback(
    const hall_driver_t *self,
    hall_driver_signal_feedback_t *feedback
);

/** @} */

#endif /* PLATFORM_HALL_DRIVER_H */
