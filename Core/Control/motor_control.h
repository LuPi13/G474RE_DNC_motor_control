/**
 * @file motor_control.h
 * @brief Motor-control mode routing과 d/q 전류 지령 조정 public interface.
 * @ingroup control_motor_control
 * @see @ref control_motor_control "Motor control 사용 안내"
 */

#ifndef CONTROL_MOTOR_CONTROL_H
#define CONTROL_MOTOR_CONTROL_H

#include <stdbool.h>

#include "rate_limiter.h"
#include "vector_types.h"

/**
 * @defgroup control_motor_control Motor control
 * @brief 상위 command와 FOC 사이의 d/q 전류 지령을 제한하고 변화율을 조정한다.
 *
 * @par 현재 구현 범위
 * 현재 vertical slice는 Current mode에 필요한 reference conditioning만 제공한다.
 * 향후 position/speed/FOC module은 이 coordinator가 호출하며 controller끼리 서로
 * 직접 include하거나 호출하지 않는다.
 *
 * @par 전류 지령 처리 순서
 *
 * @code
 * requested i_dq
 *   -> d/q axis clamp
 *   -> current magnitude saturation
 *   -> d/q scalar rate limiter
 *   -> final current magnitude saturation
 *   -> applied i_dq_ref
 * @endcode
 *
 * 첫 saturation은 도달할 수 없는 target을 향해 limiter state가 누적되는 것을 막는다.
 * d/q scalar limiter의 중간 경로가 원형 전류 제한을 벗어날 수 있으므로 마지막에 직전
 * 적용 지령과 중간 지령 사이의 이동량을 줄인다. 따라서 magnitude 제한과 축별 rate를
 * 함께 만족하며, 제한이 개입하면 limiter state도 실제 적용 지령으로 동기화한다.
 *
 * @par 실행 주기와 안전 정지
 * motor_control_update_current_reference()는 통신 수신 시점이 아니라 config의
 * sampling_period_s와 같은 고정 scheduler 주기에서 호출한다. 정상 stop은 0 A target을
 * rate limit할 수 있지만 fault/emergency shutdown은 이 module을 거치지 않고 PWM을 즉시
 * 차단해야 한다.
 * @{
 */

/**
 * @brief Motor control 함수의 실행 결과.
 */
typedef enum {
    MOTOR_CONTROL_STATUS_OK = 0,          /**< 요청한 처리를 완료함. */
    MOTOR_CONTROL_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 입력. */
    MOTOR_CONTROL_STATUS_INVALID_CONFIG,  /**< 전류 제한, 변화율 또는 실행 주기가 유효하지 않음. */
    MOTOR_CONTROL_STATUS_INVALID_STATE,   /**< 초기화되지 않은 instance를 사용함. */
    MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR /**< 내부 rate limiter 호출 실패. */
} motor_control_status_t;

/**
 * @brief d/q 전류 지령의 범위와 변화율 설정.
 *
 * d/q axis 범위는 닫힌 구간이며 0 A를 포함해야 한다. magnitude limit은 axis 제한 뒤
 * 적용하는 원형 제한이다. Rise/fall은 전류값의 수치 증가/감소 방향을 뜻한다.
 */
typedef struct {
    dq_t current_reference_min; /**< d/q축 전류 지령 하한 [A]. */
    dq_t current_reference_max; /**< d/q축 전류 지령 상한 [A]. */
    dq_t current_reference_rise_rate_per_s; /**< d/q축 수치 증가 변화율 [A/s]. */
    dq_t current_reference_fall_rate_per_s; /**< d/q축 수치 감소 변화율 [A/s]. */
    float current_reference_magnitude_limit; /**< d/q 전류 vector 크기 상한 [A]. */
    float sampling_period_s; /**< Reference limiter의 고정 실행 주기 [s]. */
} motor_control_config_t;

/**
 * @brief Motor control 설정과 current-reference runtime state.
 *
 * i_dq_ref는 FOC에 전달할 최종 전류 지령의 유일한 owner다. 외부 target은 호출 입력이며
 * instance에 복제하지 않는다. FOC, speed 및 position controller state는 해당 기능을
 * 구현할 때 책임에 맞게 추가한다.
 */
typedef struct {
    motor_control_config_t config; /**< 초기화 시 복사한 전류 지령 설정. */
    rate_limiter_t i_d_rate_limiter; /**< d축 전류 지령 rate limiter. */
    rate_limiter_t i_q_rate_limiter; /**< q축 전류 지령 rate limiter. */
    dq_t i_dq_ref; /**< 마지막으로 적용한 d/q 전류 지령 [A]. */
    bool is_current_reference_rate_limited; /**< 마지막 update에서 slew 제한이 개입했음. */
    bool is_current_reference_saturated; /**< 마지막 update/reset에서 axis/vector 제한이 개입했음. */
    bool is_initialized; /**< motor_control_init() 완료 여부. */
} motor_control_t;

/**
 * @brief d/q 전류 지령 제한 설정과 0 A 초기 상태를 준비한다.
 *
 * @param[out] self 초기화할 motor control instance.
 * @param[in] config Axis/magnitude 전류 제한, 변화율과 고정 실행 주기.
 *
 * @post 성공 시 i_dq_ref는 0 A이고 두 rate limiter도 0 A에서 시작한다.
 * @retval MOTOR_CONTROL_STATUS_OK 초기화 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval MOTOR_CONTROL_STATUS_INVALID_CONFIG 비유한 값, 음수 rate, 0 이하 magnitude/주기,
 *         잘못된 axis 범위 또는 0 A를 포함하지 않는 axis 범위.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter 초기화 실패.
 */
motor_control_status_t motor_control_init(
    motor_control_t *self,
    const motor_control_config_t *config
);

/**
 * @brief Current mode 진입점의 d/q 전류 지령 상태를 지정한 값에 동기화한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] initial_i_dq_ref 새 reference 시작점 [A].
 *
 * @details 입력에 axis/magnitude 제한을 즉시 적용하고 두 rate limiter를 같은 값으로
 *          reset한다. 정상 운전 중 ramp 용도로 호출하지 않는다.
 * @post 성공 시 i_dq_ref와 rate limiter state가 같은 제한값을 가진다.
 * @retval MOTOR_CONTROL_STATUS_OK Reset 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 또는 비유한 전류 입력.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter reset 실패.
 */
motor_control_status_t motor_control_reset_current_reference(
    motor_control_t *self,
    const dq_t *initial_i_dq_ref
);

/**
 * @brief 요청한 d/q 전류를 제한하고 한 고정 주기만큼 reference를 갱신한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] requested_i_dq_ref 외부 Current command 또는 상위 controller의 요구 전류 [A].
 * @param[out] applied_i_dq_ref FOC에 전달할 최종 d/q 전류 지령 [A]. 오류 시 변경하지 않음.
 *
 * @pre 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @post 성공 시 axis 범위와 current_reference_magnitude_limit을 만족한다.
 * @post 직전 i_dq_ref 대비 각 축의 변화량은 방향별 rate와 sampling_period_s의 곱을
 *       넘지 않는다.
 * @note 지령 제한은 software/hardware 과전류 보호를 대체하지 않는다.
 * @note 오류 시 self와 applied_i_dq_ref를 변경하지 않는다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Reference 갱신 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 또는 비유한 전류 입력.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter update/reset 실패.
 */
motor_control_status_t motor_control_update_current_reference(
    motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    dq_t *applied_i_dq_ref
);

/** @} */

#endif /* CONTROL_MOTOR_CONTROL_H */
