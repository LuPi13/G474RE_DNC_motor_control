/**
 * @file pwm_driver.h
 * @brief HRTIM 기반 3상 PWM driver의 public interface를 정의한다.
 *
 * 논리적 a, b, c상을 실제 HRTIM sub-timer와 compare unit에 매핑하고,
 * 정규화된 3상 duty command를 HRTIM compare 값으로 변환하여 적용한다.
 *
 * HRTIM의 period, dead time, output set/reset source, polarity, preload/update
 * 설정 등은 CubeMX에서 생성된 HRTIM 초기화가 완료되어 있음을 전제로 한다.
 */

#ifndef PLATFORM_PWM_DRIVER_H
#define PLATFORM_PWM_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "vector_types.h"

/**
 * @brief PWM driver 함수의 실행 결과.
 */
typedef enum {
    PWM_DRIVER_STATUS_OK = 0,              /**< 요청한 동작을 정상적으로 완료함. */
    PWM_DRIVER_STATUS_INVALID_ARGUMENT,    /**< NULL 인자 또는 초기화되지 않은 instance. */
    PWM_DRIVER_STATUS_INVALID_TIMER,       /**< 지원하지 않거나 중복된 HRTIM timer mapping. */
    PWM_DRIVER_STATUS_INVALID_COMPARE_UNIT, /**< 지원하지 않는 HRTIM compare unit. */
    PWM_DRIVER_STATUS_HAL_ERROR            /**< HRTIM HAL 호출 실패. */
} pwm_driver_status_t;

/**
 * @brief 한 전기적 상에 대응하는 HRTIM 설정.
 */
typedef struct {
    uint32_t timer_index;   /**< HRTIM_TIMERINDEX_TIMER_A부터 TIMER_F 중 하나. */
    uint32_t compare_unit;  /**< duty 갱신에 사용할 HRTIM_COMPAREUNIT_1부터 4 중 하나. */
} pwm_driver_phase_config_t;

/**
 * @brief 3상 PWM driver의 hardware mapping 설정.
 *
 * 논리적 a, b, c상을 각각 어느 HRTIM sub-timer와 compare unit에
 * 연결할지를 정의한다.
 */
typedef struct {
    HRTIM_HandleTypeDef *hrtim;  /**< CubeMX에서 초기화된 HRTIM handle. */

    pwm_driver_phase_config_t phase_a;  /**< 논리적 a상의 hardware mapping. */
    pwm_driver_phase_config_t phase_b;  /**< 논리적 b상의 hardware mapping. */
    pwm_driver_phase_config_t phase_c;  /**< 논리적 c상의 hardware mapping. */
} pwm_driver_config_t;

/**
 * @brief PWM driver instance.
 *
 * @note timer_mask와 output_mask는 pwm_driver_init()에서 config를 기준으로
 *       계산되며 이후 enable/disable 시 재사용된다.
 */
typedef struct {
    pwm_driver_config_t config;  /**< 초기화 시 복사된 hardware mapping. */

    uint32_t timer_mask;   /**< 선택된 counter의 HRTIM_TIMERID_* bit mask. */
    uint32_t output_mask;  /**< 선택된 상의 두 output을 포함하는 bit mask. */

    bool is_initialized;  /**< pwm_driver_init() 정상 완료 여부. */
    bool is_enabled;      /**< PWM output 활성화 여부. */
} pwm_driver_t;

/**
 * @brief PWM driver를 초기화하고 선택된 HRTIM counter를 동기화하여 시작한다.
 *
 * @param self PWM driver instance.
 * @param config HRTIM handle과 3상 hardware mapping 설정.
 *
 * @pre CubeMX에서 생성된 HRTIM 초기화 함수가 먼저 호출되어 있어야 한다.
 * @pre 선택된 세 sub-timer는 동기 운전에 적합한 동일 counter 설정을 사용해야 한다.
 * @post 정상 완료 시 선택된 counter는 하나의 software reset event로 정렬된 뒤
 *       동작하며, PWM output은 비활성 상태이다.
 *
 * @retval PWM_DRIVER_STATUS_OK 초기화 완료.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self, @p config 또는 HRTIM handle이 NULL임.
 * @retval PWM_DRIVER_STATUS_INVALID_TIMER timer mapping이 유효하지 않거나 중복됨.
 * @retval PWM_DRIVER_STATUS_INVALID_COMPARE_UNIT compare unit이 지원 범위를 벗어남.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 정지, counter 시작 또는 reset 실패.
 */
pwm_driver_status_t pwm_driver_init(
    pwm_driver_t *self,
    const pwm_driver_config_t *config
);

/**
 * @brief 3상 PWM output을 활성화한다.
 *
 * @param self PWM driver instance.
 *
 * @note HRTIM counter는 pwm_driver_init() 이후 계속 동작하며,
 *       이 함수는 실제 output만 활성화한다.
 * @note 이미 활성화된 상태에서 호출하면 hardware를 다시 조작하지 않고 성공한다.
 *
 * @retval PWM_DRIVER_STATUS_OK output 활성화 완료 또는 이미 활성화됨.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self가 NULL이거나 초기화되지 않음.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 활성화 실패.
 */
pwm_driver_status_t pwm_driver_enable(
    pwm_driver_t *self
);

/**
 * @brief 3상 PWM output을 비활성화한다.
 *
 * @param self PWM driver instance.
 *
 * @note HRTIM counter는 정지하지 않는다.
 * @note 이미 비활성화된 상태에서 호출하면 hardware를 다시 조작하지 않고 성공한다.
 *
 * @retval PWM_DRIVER_STATUS_OK output 비활성화 완료 또는 이미 비활성화됨.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self가 NULL이거나 초기화되지 않음.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 비활성화 실패.
 */
pwm_driver_status_t pwm_driver_disable(
    pwm_driver_t *self
);

/**
 * @brief 다음 PWM update에 사용할 3상 duty command를 적용한다.
 *
 * @param self PWM driver instance.
 * @param duty a, b, c상의 정규화된 duty command.
 *
 * @note 입력 duty는 각 상별로 [0.0, 1.0] 범위로 제한한다.
 * @note 이 함수는 선택된 세 HRTIM compare register를 갱신한다. 실제 output에
 *       반영되는 시점은 CubeMX의 preload/update 설정을 따른다.
 * @note 정확한 0%/100% 출력 동작은 CubeMX에서 설정한 HRTIM output의
 *       set/reset source와 update 방식에 영향을 받을 수 있다.
 *
 * @retval PWM_DRIVER_STATUS_OK compare register 갱신 완료.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self 또는 @p duty가 NULL이거나
 *         driver가 초기화되지 않음.
 */
pwm_driver_status_t pwm_driver_set_duty(
    pwm_driver_t *self,
    const abc_t *duty
);

#endif /* PLATFORM_PWM_DRIVER_H */
