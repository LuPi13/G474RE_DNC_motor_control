/**
 * @file speed_controller.h
 * @brief 기계속도 feedback filter와 q축 전류 PI 제어기의 public interface.
 * @ingroup control_speed_controller
 */

#ifndef CONTROL_SPEED_CONTROLLER_H
#define CONTROL_SPEED_CONTROLLER_H

#include <stdbool.h>

#include "filter.h"
#include "pi_controller.h"

/**
 * @defgroup control_speed_controller Speed controller
 * @brief 기계속도 오차를 제한된 q축 전류 지령으로 변환하는 speed-loop subsystem.
 *
 * @par 처리 순서
 *
 * @code
 * omega_m_feedback
 *   -> low-pass filter
 *   -> omega_m_ref - omega_m_feedback_filtered
 *   -> PI with scalar saturation
 *   -> i_q_ref
 * @endcode
 *
 * 이 module은 기계속도 reference의 범위/변화율 제한, electrical-to-mechanical speed 변환,
 * scheduler, FOC 또는 PWM hardware를 소유하지 않는다. 상위 motor_control은 고정 1 kHz 같은
 * 명시적인 주기로 이 module을 호출하고 출력 `i_q_ref`를 current-reference 경로에 전달한다.
 *
 * @par External current limitation
 * Motor-control의 current vector/rate 제한으로 실제 적용 가능한 q축 전류가 PI 출력보다
 * 작아지면 같은 speed-loop 주기에 speed_controller_apply_tracking()으로 실제 적용값을
 * 전달한다. Fault나 emergency shutdown에서는 tracking으로 정지를 지연하지 않고 PWM을
 * 즉시 차단한 뒤 controller를 reset한다.
 *
 * @par Feedback filter 초기화
 * init/reset 직후 첫 유효 speed feedback으로 filter output을 초기화한다. 따라서 enable 순간에
 * 0 rad/s에서 실제 회전속도로 filter가 천천히 이동하는 인위적인 과도 응답을 만들지 않는다.
 *
 * @{
 */

/**
 * @brief Speed controller 함수의 실행 결과.
 */
typedef enum {
    SPEED_CONTROLLER_STATUS_OK = 0, /**< 요청한 처리를 정상적으로 완료함. */
    SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 입력. */
    SPEED_CONTROLLER_STATUS_INVALID_CONFIG, /**< PI/filter 설정 또는 실행 주기가 유효하지 않음. */
    SPEED_CONTROLLER_STATUS_INVALID_STATE, /**< 초기화되지 않은 instance를 사용함. */
    SPEED_CONTROLLER_STATUS_FILTER_ERROR, /**< 내부 speed-feedback filter 호출 실패. */
    SPEED_CONTROLLER_STATUS_PI_ERROR, /**< 내부 PI update/reset/tracking 호출 실패. */
    SPEED_CONTROLLER_STATUS_NUMERIC_ERROR /**< 속도 오차를 float 유한 범위로 표현할 수 없음. */
} speed_controller_status_t;

/**
 * @brief Speed PI와 mechanical-speed feedback filter 설정.
 *
 * PI error 단위는 [rad/s], output 단위는 [A]다. PI와 filter의 sampling period는 정확히
 * 같아야 하며 PI 출력 범위는 0 A를 포함해야 한다.
 */
typedef struct {
    pi_controller_config_t pi; /**< 기계속도 오차를 q축 전류로 변환하는 PI 설정. */
    filter_low_pass_config_t feedback_filter; /**< 기계속도 feedback 저역통과 filter 설정. */
} speed_controller_config_t;

/**
 * @brief Speed-loop 한 주기에 필요한 reference와 feedback.
 */
typedef struct {
    float omega_m_ref_rad_s; /**< 상위에서 범위/변화율 제한을 완료한 기계각속도 지령 [rad/s]. */
    float omega_m_feedback_rad_s; /**< Rotor feedback의 signed 기계각속도 [rad/s]. */
} speed_controller_input_t;

/**
 * @brief Speed-loop 한 주기의 계산 결과 snapshot.
 */
typedef struct {
    float omega_m_feedback_filtered_rad_s; /**< PI에 사용한 filtered 기계각속도 [rad/s]. */
    float omega_m_error_rad_s; /**< `omega_m_ref_rad_s - omega_m_feedback_filtered_rad_s` [rad/s]. */
    float i_q_ref; /**< PI scalar 제한 뒤 q축 전류 지령 [A]. */
    bool is_i_q_ref_saturated; /**< PI의 q축 전류 제한이 이번 update에 개입했음. */
} speed_controller_output_t;

/**
 * @brief Speed PI와 feedback filter runtime state를 소유하는 instance.
 */
typedef struct {
    pi_controller_t pi; /**< Speed PI runtime state. */
    filter_low_pass_t feedback_filter; /**< Mechanical-speed feedback filter state. */
    bool is_feedback_initialized; /**< 첫 유효 feedback으로 filter를 초기화했는지 여부. */
    bool is_initialized; /**< speed_controller_init() 완료 여부. */
} speed_controller_t;

/**
 * @brief Speed controller의 PI와 feedback filter를 초기화한다.
 *
 * @param[out] self 초기화할 speed controller instance.
 * @param[in] config PI, q축 전류 범위 및 speed-feedback filter 설정.
 *
 * @pre PI와 filter의 sampling period가 정확히 같아야 한다.
 * @post PI state는 0 A이고 filter는 첫 update의 유효 feedback을 기다린다.
 * @note 오류 반환 시 @p self 는 변경하지 않는다.
 *
 * @retval SPEED_CONTROLLER_STATUS_OK 초기화 완료.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_CONFIG PI/filter 설정이 유효하지 않거나 PI 범위가 0 A를 포함하지 않음.
 */
speed_controller_status_t speed_controller_init(
    speed_controller_t *self,
    const speed_controller_config_t *config
);

/**
 * @brief PI와 feedback-filter runtime state를 비활성 초기 상태로 되돌린다.
 *
 * @param[in,out] self 초기화된 speed controller instance.
 *
 * @pre Speed-loop update가 중지된 mode 전환 또는 disable 문맥에서 호출한다.
 * @post PI 출력은 0 A이고 다음 update에서 filter를 새 feedback으로 초기화한다.
 * @note 오류 반환 시 @p self 는 변경하지 않는다.
 *
 * @retval SPEED_CONTROLLER_STATUS_OK Reset 완료.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval SPEED_CONTROLLER_STATUS_FILTER_ERROR 내부 filter reset 실패.
 * @retval SPEED_CONTROLLER_STATUS_PI_ERROR 내부 PI reset 실패.
 */
speed_controller_status_t speed_controller_reset(speed_controller_t *self);

/**
 * @brief 기계속도 feedback을 filtering하고 q축 전류 지령을 한 주기 갱신한다.
 *
 * @param[in,out] self 초기화된 speed controller instance.
 * @param[in] input 기계속도 reference와 feedback [rad/s].
 * @param[out] output 성공 시 filtered feedback, 오차와 q축 전류 지령.
 *
 * @pre 실제 호출 간격은 config의 sampling period와 일치해야 한다.
 * @pre @p output 은 @p self 또는 @p input 과 겹치지 않는 별도 저장 위치여야 한다.
 * @post 성공 시에만 filter와 PI runtime state를 함께 갱신한다.
 * @note 오류 반환 시 @p self 와 @p output 은 변경하지 않는다.
 *
 * @retval SPEED_CONTROLLER_STATUS_OK Update 완료.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT NULL 또는 유한하지 않은 speed 입력.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval SPEED_CONTROLLER_STATUS_FILTER_ERROR 내부 filter update/reset 실패.
 * @retval SPEED_CONTROLLER_STATUS_PI_ERROR 내부 PI update 실패.
 * @retval SPEED_CONTROLLER_STATUS_NUMERIC_ERROR 속도 오차를 float 유한 범위로 표현할 수 없음.
 */
speed_controller_status_t speed_controller_update(
    speed_controller_t *self,
    const speed_controller_input_t *input,
    speed_controller_output_t *output
);

/**
 * @brief Downstream 제한 뒤 실제 적용 가능한 q축 전류를 PI anti-windup에 반영한다.
 *
 * @param[in,out] self 같은 주기에 update를 완료한 speed controller instance.
 * @param[in] applied_i_q_ref 실제 current-reference 경로가 적용한 q축 전류 지령 [A].
 *
 * @pre 같은 speed-loop 주기의 speed_controller_update() 직후 호출해야 한다.
 * @pre 적용값은 PI configuration의 output 범위 안에 있어야 한다.
 * @note Downstream 제한이 speed-controller 출력과 같다면 호출하지 않아도 된다.
 * @note 오류 반환 시 @p self 는 변경하지 않는다.
 *
 * @retval SPEED_CONTROLLER_STATUS_OK Tracking 반영 완료.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT self가 NULL이거나 적용값이 유효 범위 밖임.
 * @retval SPEED_CONTROLLER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval SPEED_CONTROLLER_STATUS_PI_ERROR 내부 PI tracking 실패.
 */
speed_controller_status_t speed_controller_apply_tracking(
    speed_controller_t *self,
    float applied_i_q_ref
);

/** @} */

#endif /* CONTROL_SPEED_CONTROLLER_H */
