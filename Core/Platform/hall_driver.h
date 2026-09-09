/**
 * @file hall_driver.h
 * @brief TIM Hall Sensor Interface 기반 rotor 상태 수집과 기본 전기각 추정 API.
 * @ingroup platform_hall_driver
 * @see @ref platform_hall_driver "Hall driver 사용 안내"
 */

#ifndef PLATFORM_HALL_DRIVER_H
#define PLATFORM_HALL_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/**
 * @defgroup platform_hall_driver Hall driver
 * @brief 3개 Hall 입력으로 sector, 방향, edge 전기각 및 전기각속도를 추정하는 hardware driver.
 *
 * @par 책임과 설정 위치
 * CubeMX는 TIM Hall Sensor Interface, Hall GPIO alternate function, input filter,
 * prescaler, auto-reload 및 NVIC를 설정한다. 호출자는 hall_driver_config_t 로
 * TIM/GPIO mapping, 정방향 Hall sequence, TIM kernel clock 및 전기각 offset을 지정한다.
 * Driver는 capture/timeout 처리와 이상적인 60 electrical degree Hall의 기본 추정을 맡는다.
 * FOC/SVPWM 실행과 motor-control scheduling은 App/Control 계층의 책임이다.
 *
 * @par Hall state와 방향
 * Hall state의 bit 순서는 A/B/C = bit 2/1/0이다. 000과 111은 유효하지 않으며,
 * 정방향은 hall_driver_config_t::hall_state_by_sector 에 지정한 sector 증가 방향이다.
 * 실제 축의 시계/반시계 방향 중 어느 쪽을 정방향으로 부를지는 application이 정한다.
 *
 * @par 각도와 속도 모델
 * Sensor가 정확히 60 electrical degree 간격으로 배치되었다고 가정한다. 유효한 인접
 * transition에서 sector 중심을 기준으로 정방향은 -pi/6, 역방향은 +pi/6인 edge 각도를
 * 사용한다. Edge 사이의 연속 angle extrapolation, filtering, hysteresis 및 sensor별
 * 위치 보정은 수행하지 않는다. Hall만으로 절대 기계 위치를 제공하지 않는다.
 *
 * @par ISR과 feedback 전달
 * HAL callback은 main.c의 USER CODE 영역에 두고 hall_driver_handle_capture() 또는
 * hall_driver_handle_timeout()만 호출한다. TIM ISR은 ADC fast-loop보다 낮은 preemption
 * priority로 운용할 수 있다. 이때 ADC가 feedback 작성 도중 TIM ISR을 선점해도 부분적으로
 * 갱신된 구조체를 읽지 않도록 driver instance 내부에서 double buffer를 사용한다.
 *
 * @par 호출 순서
 *
 * 1. CubeMX TIM/GPIO 초기화를 완료한다.
 * 2. hall_driver_init()으로 mapping과 peripheral 설정을 검증한다.
 * 3. hall_driver_start()로 capture와 overflow timeout interrupt를 시작한다.
 * 4. HAL TIM callback에서 대응하는 handler를 호출한다.
 * 5. Fast-loop는 hall_driver_get_rotor_feedback()으로 경량 snapshot을 읽고,
 *    진단 경로는 hall_driver_get_feedback()으로 전체 snapshot을 읽는다.
 *
 * @{
 */

/** @brief Hall state lookup table의 전체 항목 수. */
#define HALL_DRIVER_STATE_COUNT       8U

/** @brief 한 electrical revolution에 존재하는 이상적 Hall sector 수. */
#define HALL_DRIVER_SECTOR_COUNT      6U

/** @brief 000, 111 또는 아직 결정되지 않은 sector를 나타내는 값. */
#define HALL_DRIVER_INVALID_SECTOR    UINT8_MAX

/**
 * @brief Hall driver 함수의 실행 결과.
 *
 * INVALID_HALL_STATE와 INVALID_TRANSITION은 ISR에서 검출한 sensor 신호 오류이다.
 * INVALID_CAPTURE는 유효한 Hall 전이는 확인했지만 capture 시간을 속도 계산에 사용할 수
 * 없음을 뜻한다.
 */
typedef enum {
    HALL_DRIVER_STATUS_OK = 0,             /**< 요청한 처리를 정상적으로 완료함. */
    HALL_DRIVER_STATUS_INVALID_ARGUMENT,   /**< NULL, 초기화되지 않은 instance 또는 다른 TIM handle. */
    HALL_DRIVER_STATUS_INVALID_CONFIG,     /**< GPIO/sequence/clock 또는 CubeMX TIM 설정이 유효하지 않음. */
    HALL_DRIVER_STATUS_INVALID_STATE,      /**< 실행/정지 상태가 요청한 동작에 적합하지 않음. */
    HALL_DRIVER_STATUS_INVALID_HALL_STATE, /**< Hall state가 000 또는 111임. */
    HALL_DRIVER_STATUS_INVALID_TRANSITION, /**< 이전 sector와 인접하지 않은 Hall transition. */
    HALL_DRIVER_STATUS_INVALID_CAPTURE,    /**< CH1이 아니거나 capture tick이 0이어서 시간을 사용할 수 없음. */
    HALL_DRIVER_STATUS_HAL_ERROR            /**< Hall Sensor Interface HAL start/stop 실패. */
} hall_driver_status_t;

/**
 * @brief Hall sector transition으로 판별한 회전 방향.
 * @note FORWARD는 config의 sector 증가 방향이며 물리적인 시계 방향을 고정하여 의미하지 않는다.
 */
typedef enum {
    HALL_DRIVER_DIRECTION_UNKNOWN = 0, /**< 시작, resync 또는 비정상 transition으로 방향을 모름. */
    HALL_DRIVER_DIRECTION_FORWARD = 1, /**< sector가 정방향 sequence의 다음 항목으로 이동함. */
    HALL_DRIVER_DIRECTION_REVERSE = -1 /**< sector가 정방향 sequence의 이전 항목으로 이동함. */
} hall_driver_direction_t;

/**
 * @brief 논리적 Hall 입력 하나의 GPIO mapping.
 * @note GPIO mode, alternate function, pull 및 speed는 CubeMX에서 설정한다.
 */
typedef struct {
    GPIO_TypeDef *port; /**< Hall pin이 연결된 GPIO port. */
    uint16_t pin;       /**< 정확히 하나의 GPIO_PIN_* bit. */
} hall_driver_input_config_t;

/**
 * @brief Board/motor별 Hall hardware mapping과 전기각 기준 설정.
 *
 * hall_state_by_sector에는 정방향 회전 시 나타나는 Hall state를 sector 0부터 순서대로
 * 기록한다. Hall state는 A/B/C = bit 2/1/0 순서이며, 001부터 110까지의 여섯 상태를
 * 중복 없이 한 번씩 포함해야 한다.
 *
 * 현재 확인된 motor의 정방향 sequence는 다음과 같다.
 *
 * @code{.c}
 * .hall_state_by_sector = {0x5U, 0x4U, 0x6U, 0x2U, 0x3U, 0x1U}
 * // 101 -> 100 -> 110 -> 010 -> 011 -> 001
 * @endcode
 *
 * @note timer_clock_hz는 prescaler 적용 전 TIM kernel clock [Hz]이다. APB prescaler에
 *       따라 TIM clock이 PCLK의 두 배가 될 수 있으므로 SystemCoreClock을 무조건 넣지 않는다.
 * @note electrical_offset_rad는 sector 0의 정방향 진입 edge에 해당하는 전기각 [rad]이다.
 *       기계적인 절대 0도가 아니며 실제 FOC 전에 rotor flux/phase 기준으로 보정해야 한다.
 */
typedef struct {
    TIM_HandleTypeDef *timer; /**< CubeMX가 Hall Sensor Interface로 초기화한 TIM handle. */

    hall_driver_input_config_t hall_a; /**< Hall A: state bit 2의 GPIO mapping. */
    hall_driver_input_config_t hall_b; /**< Hall B: state bit 1의 GPIO mapping. */
    hall_driver_input_config_t hall_c; /**< Hall C: state bit 0의 GPIO mapping. */

    uint8_t hall_state_by_sector[HALL_DRIVER_SECTOR_COUNT]; /**< Sector 0부터 5까지의 정방향 Hall sequence. */
    uint32_t timer_clock_hz;        /**< Prescaler 적용 전 TIM kernel clock [Hz]. */
    float electrical_offset_rad;   /**< Sector 0 정방향 진입 edge의 전기각 offset [rad]. */
} hall_driver_config_t;

/**
 * @brief 가장 최근에 완성된 Hall rotor feedback snapshot.
 *
 * theta_e_rad는 [0, 2*pi), omega_e_rad_s는 방향 부호가 있는 전기각속도 [rad/s]이다.
 * 속도는 한 Hall edge 간격으로 계산하므로 저속에서는 계단 형태이고 측정 jitter가 나타날
 * 수 있다. 기계각속도는 상위 계층에서 motor pole-pair 수로 나누어 계산한다.
 *
 * @note Start/resync 시에는 현재 sector의 중심각을 임시로 제공하며 is_angle_from_edge가
 *       false이다. 유효한 인접 transition 이후에는 방향에 따른 실제 sector 경계각을 제공한다.
 * @note 유효 Hall state에서 timeout이 확정되면 omega_e_rad_s는 0이고 has_valid_speed와
 *       is_timed_out가 true이다. Hall state도 유효하지 않으면 0 값을 신뢰할 수 없으므로
 *       has_valid_speed는 false이다. Timeout 뒤 첫 edge에서는 완전한 edge 간격이 아니므로
 *       has_valid_speed가 false이고, 다음 유효 edge부터 다시 속도를 계산한다.
 */
typedef struct {
    uint8_t hall_state;                  /**< A/B/C = bit 2/1/0인 최신 3-bit Hall state. */
    uint8_t sector;                      /**< 0부터 5까지의 sector 또는 HALL_DRIVER_INVALID_SECTOR. */
    hall_driver_direction_t direction;   /**< 최신 유효 인접 transition의 방향. */

    uint32_t capture_ticks; /**< 최신 TIM CH1 capture 값 [counter tick]. */
    float theta_e_rad;      /**< 최신 sector 중심 또는 edge 전기각 [rad], 범위 [0, 2*pi). */
    float omega_e_rad_s;    /**< 방향 부호가 있는 전기각속도 [rad/s]. */

    bool has_valid_state;       /**< hall_state와 sector가 유효함. */
    bool has_valid_direction;   /**< direction이 유효한 인접 transition으로 결정됨. */
    bool has_valid_angle;       /**< theta_e_rad를 사용할 수 있음. */
    bool has_valid_speed;       /**< omega_e_rad_s가 측정값 또는 확정된 timeout 0 값임. */
    bool is_angle_from_edge;    /**< theta_e_rad가 임시 sector 중심이 아니라 Hall edge 기준임. */
    bool is_timed_out;          /**< Hall 변화 없이 auto-reload 시간이 지나 정지로 판정됨. */

    uint32_t transition_count;          /**< init 이후 수락한 유효 인접 transition 누적 횟수. */
    uint32_t invalid_state_count;       /**< init 이후 검출한 000/111 및 시작 시 오류 상태 누적 횟수. */
    uint32_t invalid_transition_count;  /**< init 이후 검출한 비인접 transition 누적 횟수. */
    uint32_t timeout_count;             /**< init 이후 검출한 timeout 누적 횟수. 정지 중 반복 overflow는 한 번으로 묶음. */
} hall_driver_feedback_t;

/**
 * @brief Fast-loop rotor estimator에 필요한 Hall feedback의 경량 snapshot.
 *
 * hall_driver_feedback_t의 hardware diagnostic field를 매 fast-loop 주기마다
 * 복사하지 않도록 rotor angle/speed 관측에 필요한 값만 제공한다.
 * 이 구조체는 별도 runtime state의 owner가 아니라 active feedback buffer의
 * 읽기 전용 snapshot이다.
 */
typedef struct {
    float theta_e_rad;   /**< Hall sector 중심 또는 edge 전기각 [rad]. */
    float omega_e_rad_s; /**< 방향 부호가 있는 전기각속도 [rad/s]. */
    uint32_t transition_count; /**< 수락한 유효 Hall transition 누적 횟수. */
    uint8_t sector;             /**< 0부터 5까지의 최신 sector 또는 HALL_DRIVER_INVALID_SECTOR. */
    bool has_valid_state;      /**< Hall state와 sector가 유효함. */
    bool has_valid_direction;  /**< 최신 transition 방향이 유효함. */
    bool has_valid_angle;      /**< theta_e_rad를 사용할 수 있음. */
    bool has_valid_speed;      /**< omega_e_rad_s를 사용할 수 있음. */
    bool is_angle_from_edge;   /**< 각도가 임시 sector 중심이 아니라 Hall edge 기준임. */
    bool is_timed_out;         /**< Hall 변화 timeout으로 정지 상태가 확정됨. */
} hall_driver_rotor_feedback_t;

/**
 * @brief Hall driver 설정, lookup table 및 runtime 상태를 소유하는 instance.
 *
 * 최초 사용 전 0으로 초기화하고 init 이후 내부 필드를 application에서 직접 변경하지 않는다.
 * 같은 TIM, GPIO 및 instance를 다른 driver나 ISR에서 동시에 조작하지 않는다.
 *
 * @par Feedback double buffer
 * TIM2처럼 Hall writer IRQ의 priority가 ADC fast-loop reader보다 낮으면, ADC가 writer의
 * 여러 feedback field 갱신 중간에 선점할 수 있다. 단일 구조체를 사용하면 새 sector와 이전
 * angle/speed가 섞인 snapshot이 노출될 수 있으며 volatile만으로는 이를 막을 수 없다.
 *
 * Driver는 inactive feedback_buffer를 완성한 뒤 memory barrier를 수행하고, 정렬된
 * active_feedback_index 한 개만 전환한다. 따라서 writer보다 높은 priority의 reader는 writer를
 * 선점하더라도 이전 완성본 또는 새 완성본 중 하나만 읽는다. 동적 메모리는 사용하지 않으며
 * 두 buffer는 hall_driver_t instance 내부에 고정 크기로 포함된다.
 *
 * @warning Double buffer는 하나의 TIM writer와 그보다 높은 priority의 reader를 전제로 한다.
 *          Main context처럼 writer가 reader를 반복 선점할 수 있는 곳에서 강한 snapshot 보장이
 *          필요하면 호출자가 TIM IRQ를 막아야 한다.
 */
typedef struct {
    hall_driver_config_t config; /**< 초기화 시 복사하고 offset을 [0, 2*pi)로 정규화한 설정. */
    uint8_t sector_by_state[HALL_DRIVER_STATE_COUNT]; /**< Hall state에서 sector로 변환하는 lookup table. */

    float counter_frequency_hz; /**< 실제 TIM counter frequency [Hz]. */
    float timeout_s;            /**< Auto-reload overflow로 판정하는 timeout [s]. */

    volatile hall_driver_feedback_t feedback_buffer[2]; /**< 번갈아 publish하는 완성 feedback 두 벌. */
    volatile uint32_t active_feedback_index;             /**< 현재 reader에 공개된 buffer index, 0 또는 1. */

    volatile bool has_valid_interval_reference; /**< 최신 capture가 다음 속도 계산의 시간 기준인지 여부. */
    volatile bool capture_with_pending_timeout; /**< 같은 HAL IRQ에 capture와 실제 overflow가 함께 있었음. */
    volatile bool is_initialized;               /**< Mapping과 TIM 설정 검증 완료 여부. */
    volatile bool is_running;                   /**< Hall capture와 timeout interrupt 실행 상태. */
} hall_driver_t;

/**
 * @brief Hall mapping과 CubeMX TIM 설정을 검증하고 driver instance를 초기화한다.
 *
 * @param[in,out] self 최초에는 0으로 초기화된 instance. 재초기화 시 완전 정지 상태여야 함.
 * @param[in] config TIM/GPIO mapping, 정방향 sequence, clock 및 전기각 offset.
 *
 * @pre CubeMX의 GPIO와 HAL_TIMEx_HallSensor_Init()가 먼저 완료되어야 한다.
 * @pre TIM은 정지 상태이며 CC1/update interrupt가 비활성화되어 있어야 한다.
 * @post 성공 시 두 feedback buffer가 동일한 초기값을 가지며 is_initialized는 true이다.
 * @note TIM은 up-counter, TI1S XOR, TI1 edge detector trigger, reset slave mode 및
 *       CH1 TRC input으로 설정되어 있어야 한다. 이 함수는 TIM을 시작하지 않는다.
 *
 * @retval HALL_DRIVER_STATUS_OK 초기화 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT self, config, TIM handle 또는 TIM instance가 NULL임.
 * @retval HALL_DRIVER_STATUS_INVALID_CONFIG GPIO/sequence/clock/offset 또는 TIM 설정이 유효하지 않음.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE 실행 중인 instance이거나 TIM/interrupt가 이미 동작 중임.
 * @see hall_driver_start()
 */
hall_driver_status_t hall_driver_init(
    hall_driver_t *self,
    const hall_driver_config_t *config
);

/**
 * @brief Hall capture와 counter-overflow timeout interrupt를 시작한다.
 *
 * @param[in,out] self hall_driver_init()이 성공한 instance.
 * @pre TIM/GPIO mapping과 IRQ callback 연결이 준비되어 있어야 한다.
 * @post 성공 시 is_running은 true이고 TIM CH1 capture와 update interrupt가 동작한다.
 *
 * @details 현재 Hall state가 유효하면 sector 중심각을 초기 추정치로 publish한다. 시작 시점은
 *          Hall edge가 아니므로 첫 capture에서는 방향/edge 각도만 갱신하고 속도는 계산하지 않는다.
 *          TIM URS를 counter overflow/underflow로 제한하여 Hall slave reset이 timeout callback을
 *          발생시키지 않게 한다. Hall edge의 counter reset 동작 자체는 유지된다.
 * @note 이미 실행 중이면 hardware를 다시 조작하지 않고 성공한다.
 *
 * @retval HALL_DRIVER_STATUS_OK 시작 완료 또는 이미 실행 중임.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval HALL_DRIVER_STATUS_HAL_ERROR HAL_TIMEx_HallSensor_Start_IT() 실패.
 * @see hall_driver_stop()
 */
hall_driver_status_t hall_driver_start(hall_driver_t *self);

/**
 * @brief Hall capture와 timeout interrupt를 정지하고 속도 feedback을 무효화한다.
 *
 * @param[in,out] self 초기화된 Hall driver instance.
 * @post is_running은 false이며 omega_e_rad_s는 0, has_valid_speed와 is_timed_out는 false이다.
 *       마지막 Hall state, sector, angle 및 diagnostic counter는 관찰을 위해 보존한다.
 * @note 이미 정지 상태이면 hardware와 feedback을 다시 조작하지 않고 성공한다.
 *
 * @retval HALL_DRIVER_STATUS_OK 정지 완료 또는 이미 정지 상태임.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval HALL_DRIVER_STATUS_HAL_ERROR HAL_TIMEx_HallSensor_Stop_IT() 실패.
 * @see hall_driver_start()
 */
hall_driver_status_t hall_driver_stop(hall_driver_t *self);

/**
 * @brief TIM CH1 Hall capture를 처리하고 완성된 새 feedback snapshot을 publish한다.
 *
 * @param[in,out] self 실행 중인 Hall driver instance.
 * @param[in] htim HAL_TIM_IC_CaptureCallback()에서 받은 TIM handle.
 *
 * @pre htim은 config의 TIM이고 callback active channel은 HAL_TIM_ACTIVE_CHANNEL_1이어야 한다.
 * @pre 같은 instance에는 하나의 Hall TIM writer만 접근해야 한다.
 * @post 유효 transition이면 Hall state, sector, direction 및 edge angle을 갱신한다.
 *       완전한 이전 edge 시간 기준이 있으면 signed electrical speed도 갱신한다.
 * @note 시작, timeout 또는 오류 resync 직후의 첫 capture는 다음 edge의 시간 기준으로만 사용한다.
 * @note HAL callback에서는 반환 상태를 App 오류/diagnostic 경로에 전달하고 가능한 한 빨리 종료한다.
 *
 * @retval HALL_DRIVER_STATUS_OK capture를 처리함. has_valid_speed로 속도 준비 여부를 확인함.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL, 초기화되지 않은 instance 또는 다른 TIM handle.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 * @retval HALL_DRIVER_STATUS_INVALID_HALL_STATE GPIO에서 읽은 Hall state가 000 또는 111임.
 * @retval HALL_DRIVER_STATUS_INVALID_TRANSITION 이전 sector와 인접하지 않은 transition임.
 * @retval HALL_DRIVER_STATUS_INVALID_CAPTURE CH1 callback이 아니거나 capture tick이 0임.
 * @see hall_driver_get_feedback()
 * @see hall_driver_get_rotor_feedback()
 */
hall_driver_status_t hall_driver_handle_capture(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
);

/**
 * @brief Hall 변화 없이 발생한 TIM counter overflow를 정지 상태로 처리한다.
 *
 * @param[in,out] self 실행 중인 Hall driver instance.
 * @param[in] htim HAL_TIM_PeriodElapsedCallback()에서 받은 TIM handle.
 *
 * @pre htim은 config의 TIM이어야 하며 TIM URS가 counter-only로 설정되어 있어야 한다.
 * @post 일반 timeout이면 omega_e_rad_s는 0이고 is_timed_out는 true이며, 다음 capture의
 *       속도 계산에 사용할 시간 기준은 무효화된다. Hall state가 유효한 경우에만
 *       has_valid_speed도 true로 설정되어 0 rad/s를 신뢰할 수 있음을 나타낸다.
 * @note Capture와 실제 overflow가 같은 HAL IRQ에 pending이면 capture에서 얻은 새 edge를 다음
 *       시간 기준으로 보존하되 이번 속도는 무효로 publish한다.
 *
 * @retval HALL_DRIVER_STATUS_OK Timeout 처리 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT NULL, 초기화되지 않은 instance 또는 다른 TIM handle.
 * @retval HALL_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 * @see hall_driver_handle_capture()
 */
hall_driver_status_t hall_driver_handle_timeout(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
);

/**
 * @brief Reader에 공개된 최신 완성 Hall feedback snapshot을 복사한다.
 *
 * @param[in] self 초기화된 Hall driver instance.
 * @param[out] feedback 성공 시 최신 active buffer의 복사본. 호출해도 driver 상태를 소비하지 않음.
 *
 * @details TIM writer는 inactive buffer를 완성한 뒤 active index를 전환한다. TIM ISR보다 높은
 *          priority의 ADC reader가 writer를 선점한 경우 이전 완성본을 읽으며, publish 이후에
 *          호출된 reader는 새 완성본을 읽는다. 부분 갱신된 buffer는 공개되지 않는다.
 * @warning Main context 등 Hall writer가 reader를 반복 선점할 수 있는 문맥에서 강한 snapshot
 *          보장이 필요하면 호출자가 해당 TIM IRQ의 실행을 막아야 한다.
 *
 * @retval HALL_DRIVER_STATUS_OK Feedback 복사 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT self/feedback이 NULL이거나 instance가 초기화되지 않음.
 */
hall_driver_status_t hall_driver_get_feedback(
    const hall_driver_t *self,
    hall_driver_feedback_t *feedback
);

/**
 * @brief Active Hall buffer에서 rotor estimator용 필수 field만 snapshot으로 읽는다.
 *
 * @param[in] self 초기화된 Hall driver instance.
 * @param[out] feedback 성공 시 sector, 각도, 속도, transition과 validity field를 받는다.
 *
 * @details Active index를 한 번만 읽은 뒤 같은 double buffer에서 필요한 field만
 *          복사한다. hall_driver_get_feedback()과 같은 snapshot 동시성 계약을 따른다.
 * @note Fast-loop의 rotor angle/speed 경로에서 사용한다. 전체 Hall diagnostic이
 *       필요하면 hall_driver_get_feedback()을 사용한다.
 *
 * @retval HALL_DRIVER_STATUS_OK Rotor feedback 복사 완료.
 * @retval HALL_DRIVER_STATUS_INVALID_ARGUMENT self/feedback이 NULL이거나 instance가 초기화되지 않음.
 */
hall_driver_status_t hall_driver_get_rotor_feedback(
    const hall_driver_t *self,
    hall_driver_rotor_feedback_t *feedback
);

/** @} */

#endif /* PLATFORM_HALL_DRIVER_H */
