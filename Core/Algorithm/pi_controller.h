/**
 * @file pi_controller.h
 * @brief Scalar saturation과 외부 tracking을 지원하는 back-calculation PI API.
 * @ingroup algorithm_pi_controller
 * @see @ref algorithm_pi_controller "PI controller 사용 안내"
 */

#ifndef ALGORITHM_PI_CONTROLLER_H
#define ALGORITHM_PI_CONTROLLER_H

#include <stdbool.h>

/**
 * @defgroup algorithm_pi_controller PI controller
 * @brief 고정 sampling period에서 실행되는 float 기반 PI controller.
 *
 * @par 이산 PI와 back-calculation
 * 한 update의 출력과 다음 적분 상태는 다음 식으로 계산한다.
 *
 * @code
 * output_unsaturated = kp * error + integral
 * output = clamp(output_unsaturated, output_min, output_max)
 * integral += ki * Ts * error
 *           + Kaw * Ts * (output - output_unsaturated)
 * @endcode
 *
 * `output - output_unsaturated`은 scalar saturation으로 실제 적용하지 못한 출력이다.
 * 이 차이를 적분기에 되먹여 saturation이 지속될 때 integral windup을 줄인다.
 *
 * @par 외부 saturation tracking
 * d/q 전압의 원형 제한처럼 여러 PI 출력을 함께 제한하는 정책은 이 scalar module이
 * 결정할 수 없다. pi_controller_update() 이후 같은 제어 주기에서 최종 적용 가능한
 * 출력이 더 제한되었다면 pi_controller_apply_tracking()으로 해당 PI 성분의 실제 적용값을
 * 전달한다. 내부와 외부 보정을 합하면 다음 tracking term이 된다.
 *
 * @code
 * Kaw * Ts * (externally_applied_output - output_unsaturated)
 * @endcode
 *
 * Current d/q PI에서 external vector limiter를 사용할 때는 내부 scalar limit이 vector
 * 방향을 먼저 왜곡하지 않도록 충분히 넓게 설정하고, vector 제한 뒤 각 축의 실제 적용값을
 * tracking한다. Feedforward/decoupling을 더한 뒤 제한한다면 PI에 전달할 applied output은
 * 최종 축 전압에서 해당 feedforward 성분을 뺀 PI 기여분이어야 한다.
 *
 * @par Gain 단위
 * error와 output의 물리 단위는 사용하는 loop가 정한다. kp의 단위는 [output/error],
 * ki는 [output/(error*s)], anti_windup_gain_per_s는 [1/s]이다. 흔히
 * `anti_windup_gain_per_s = ki / kp`를 초기값으로 사용할 수 있지만 이 module은 gain을
 * 자동으로 결정하지 않는다.
 *
 * @par 실행 조건
 * sampling_period_s와 실제 호출 주기는 일치해야 한다. Multi-rate scheduling은 App 또는
 * motor_control이 소유하며 controller 내부에 prescaler를 두지 않는다. 같은 instance는
 * 하나의 실행 문맥에서 순차적으로 갱신하고 update/apply_tracking 중에 다른 문맥에서
 * reset하지 않는다.
 * @{
 */

/**
 * @brief PI controller 함수의 실행 결과.
 */
typedef enum {
    PI_CONTROLLER_STATUS_OK = 0,           /**< 요청한 처리를 정상적으로 완료함. */
    PI_CONTROLLER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 값. */
    PI_CONTROLLER_STATUS_INVALID_CONFIG,   /**< Gain, sampling period 또는 출력 범위가 유효하지 않음. */
    PI_CONTROLLER_STATUS_INVALID_STATE,    /**< 초기화되지 않은 instance를 사용함. */
    PI_CONTROLLER_STATUS_NUMERIC_ERROR     /**< 계산 결과가 float의 유한 범위를 벗어남. */
} pi_controller_status_t;

/**
 * @brief PI gain, 실행 주기 및 내부 scalar 출력 범위 설정.
 */
typedef struct {
    float kp;                      /**< 비례 gain [output/error], 0 이상. */
    float ki;                      /**< 적분 gain [output/(error*s)], 0 이상. */
    float anti_windup_gain_per_s;  /**< Back-calculation tracking gain [1/s], 0 이상. */
    float sampling_period_s;       /**< pi_controller_update()의 고정 호출 주기 [s], 0보다 큼. */
    float output_min;              /**< 내부 scalar saturation 하한 [output unit]. */
    float output_max;              /**< 내부 scalar saturation 상한 [output unit]. */
} pi_controller_config_t;

/**
 * @brief PI 설정, 사전 계산 gain 및 runtime 상태를 보관하는 instance.
 *
 * 최초 사용 전 pi_controller_init()을 호출한다. Field를 직접 수정하면 config와
 * precomputed gain이 불일치할 수 있으므로 공개 API로만 상태를 변경한다.
 */
typedef struct {
    pi_controller_config_t config; /**< 현재 적용 중인 PI configuration. */
    float ki_step;                 /**< 한 update용 `ki * sampling_period_s`. */
    float anti_windup_step;        /**< 한 update용 `Kaw * sampling_period_s`. */
    float integral;                /**< 다음 update에 사용할 적분 상태 [output unit]. */
    float unsaturated_output;      /**< 가장 최근 saturation 이전 PI 출력 [output unit]. */
    float output;                  /**< 가장 최근 내부 제한 또는 external tracking 출력 [output unit]. */
    bool is_initialized;           /**< pi_controller_init() 완료 여부. */
} pi_controller_t;

/**
 * @brief PI controller를 설정하고 runtime 상태를 초기화한다.
 *
 * @param[out] self 초기화할 PI controller instance.
 * @param[in] config Gain, sampling period와 scalar 출력 범위.
 *
 * @post 성공하면 ki_step과 anti_windup_step을 미리 계산하고 integral을 0으로 만든다.
 * @post 초기 unsaturated_output은 0이고 output은 0을 설정 범위로 제한한 값이다.
 * @note 출력 범위에는 비대칭 범위를 사용할 수 있으며 `output_min < output_max`여야 한다.
 *
 * @retval PI_CONTROLLER_STATUS_OK 초기화 완료.
 * @retval PI_CONTROLLER_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval PI_CONTROLLER_STATUS_INVALID_CONFIG 음수/비유한 gain, 0 이하/비유한 sampling period,
 *                                             잘못된 출력 범위 또는 float로 표현할 수 없는 step.
 */
pi_controller_status_t pi_controller_init(
    pi_controller_t *self,
    const pi_controller_config_t *config
);

/**
 * @brief Configuration을 유지하면서 적분과 출력 상태를 초기화한다.
 *
 * @param[in,out] self 초기화된 PI controller instance.
 *
 * @post integral과 unsaturated_output은 0이 되고 output은 0을 설정 범위로 제한한 값이 된다.
 * @warning 실행 중 reset은 controller 출력을 즉시 변경할 수 있다. Loop 비활성화 또는
 *          mode 전환처럼 호출 순서가 통제된 문맥에서 사용한다.
 *
 * @retval PI_CONTROLLER_STATUS_OK Runtime state reset 완료.
 * @retval PI_CONTROLLER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval PI_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
pi_controller_status_t pi_controller_reset(pi_controller_t *self);

/**
 * @brief PI 출력과 다음 적분 상태를 한 sampling period만큼 갱신한다.
 *
 * @param[in,out] self 초기화된 PI controller instance.
 * @param[in] error `reference - feedback`으로 계산한 제어 오차 [error unit].
 * @param[out] output 성공 시 내부 scalar saturation이 적용된 PI 출력 [output unit].
 *
 * @pre 실제 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @pre @p output 은 @p self 내부 field와 겹치지 않는 별도 저장 위치여야 한다.
 * @post Scalar saturation 차이를 사용한 back-calculation과 error 적분이 integral에 반영된다.
 * @note External coupled saturation이 없다면 반환된 output을 그대로 사용한다.
 * @note External saturation이 있다면 같은 주기에 pi_controller_apply_tracking()을 이어서 호출한다.
 * @note 오류 반환 시 self와 output은 변경하지 않는다.
 *
 * @retval PI_CONTROLLER_STATUS_OK PI update 완료.
 * @retval PI_CONTROLLER_STATUS_INVALID_ARGUMENT NULL 인자 또는 유한하지 않은 error.
 * @retval PI_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval PI_CONTROLLER_STATUS_NUMERIC_ERROR 비례항, 출력 또는 적분 계산이 유한 범위를 벗어남.
 */
pi_controller_status_t pi_controller_update(
    pi_controller_t *self,
    float error,
    float *output
);

/**
 * @brief 외부 제한 뒤 실제 적용 가능한 출력을 추가 back-calculation에 반영한다.
 *
 * @param[in,out] self 현재 제어 주기에 pi_controller_update()가 성공한 PI instance.
 * @param[in] applied_output 외부 제한과 보상을 모두 반영한 실제 PI 기여분 [output unit].
 *
 * @pre 같은 제어 주기의 pi_controller_update() 직후 호출해야 한다.
 * @post `Kaw * Ts * (applied_output - output)`이 integral에 추가되고 output은
 *       @p applied_output 으로 갱신된다.
 * @note External limiter가 update 결과를 변경하지 않았다면 호출하지 않아도 된다.
 * @note anti_windup_gain_per_s가 0이면 integral은 바꾸지 않고 output만 실제 값으로 갱신한다.
 * @note 오류 반환 시 self는 변경하지 않는다.
 *
 * @retval PI_CONTROLLER_STATUS_OK External tracking 반영 완료.
 * @retval PI_CONTROLLER_STATUS_INVALID_ARGUMENT self가 NULL이거나 applied_output이 유한하지 않음.
 * @retval PI_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval PI_CONTROLLER_STATUS_NUMERIC_ERROR Tracking correction 또는 적분 계산이 유한 범위를 벗어남.
 */
pi_controller_status_t pi_controller_apply_tracking(
    pi_controller_t *self,
    float applied_output
);

/** @} */

#endif /* ALGORITHM_PI_CONTROLLER_H */
