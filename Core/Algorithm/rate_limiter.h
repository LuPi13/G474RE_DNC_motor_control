/**
 * @file rate_limiter.h
 * @brief 주기적으로 갱신되는 scalar reference의 상승·하강 변화율 제한 API.
 * @ingroup algorithm_rate_limiter
 * @see @ref algorithm_rate_limiter "Rate limiter 사용 안내"
 */

#ifndef ALGORITHM_RATE_LIMITER_H
#define ALGORITHM_RATE_LIMITER_H

#include <stdbool.h>

/**
 * @defgroup algorithm_rate_limiter Rate limiter
 * @brief 외부 command와 controller reference 사이의 변화율을 제한하는 상태형 Algorithm.
 *
 * @par 단위
 * target, initial_output 및 output은 모두 호출자가 정한 같은 물리 단위를 사용한다.
 * rise_rate_per_s와 fall_rate_per_s는 그 단위의 초당 변화량을 나타낸다. 예를 들어
 * output이 [rad/s]이면 rate의 단위는 [rad/s^2]이다.
 *
 * @par 실행 주기
 * 통신 packet을 수신할 때만 호출하면 실제 변화율이 통신 주기에 의존한다. 통신 경로는
 * target만 갱신하고 rate_limiter_update()는 config에 지정한 sampling_period_s와 같은
 * 고정 주기의 scheduler에서 호출한다. 실행 rate를 만들기 위한 prescaler는 이 module에
 * 포함하지 않는다.
 *
 * @par 상승과 하강의 의미
 * rise는 숫자 값이 증가하는 방향이고 fall은 숫자 값이 감소하는 방향이다. Signed speed처럼
 * 0을 기준으로 양방향인 값에서도 속력의 증가/감소가 아니라 수치의 증가/감소를 뜻한다.
 * 두 방향에 같은 제한을 적용하려면 rise_rate_per_s와 fall_rate_per_s를 같게 설정한다.
 *
 * @par Runtime rate 변경
 * rate_limiter_set_rates()는 현재 output을 유지하고 다음 update부터 새 변화율을 적용한다.
 * 출력은 연속이지만 변화율은 즉시 바뀌므로 기울기가 꺾일 수 있다. Rate 자체의 변화도
 * 제한해야 하는 경우에는 jerk-limited trajectory generator가 별도로 필요하다.
 *
 * @par Safety와 동시 접근
 * 정상적인 reference ramp에 사용하며 fault/emergency shutdown을 지연시키는 용도로
 * 사용하지 않는다. 같은 instance의 init/reset/set_rates/update는 하나의 실행 문맥에서
 * 순서대로 호출해야 한다. 통신과 ISR이 instance를 공유한다면 통신에서는 pending 값을
 * 기록하고 limiter를 실행하는 scheduler 문맥에서 rate 변경을 적용한다.
 * @{
 */

/**
 * @brief Rate limiter 함수의 실행 결과.
 */
typedef enum {
    RATE_LIMITER_STATUS_OK = 0,           /**< 요청한 처리를 정상적으로 완료함. */
    RATE_LIMITER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 값. */
    RATE_LIMITER_STATUS_INVALID_CONFIG,   /**< Rate, sampling period 또는 계산된 step이 유효하지 않음. */
    RATE_LIMITER_STATUS_INVALID_STATE     /**< 초기화되지 않은 instance를 사용함. */
} rate_limiter_status_t;

/**
 * @brief Rate limiter의 변화율과 고정 실행 주기 설정.
 */
typedef struct {
    float rise_rate_per_s;    /**< 숫자 증가 방향의 최대 변화율 [output unit/s], 0 이상. */
    float fall_rate_per_s;    /**< 숫자 감소 방향의 최대 변화율 크기 [output unit/s], 0 이상. */
    float sampling_period_s;  /**< rate_limiter_update()의 고정 호출 주기 [s], 0보다 큼. */
} rate_limiter_config_t;

/**
 * @brief Rate limiter의 설정과 현재 출력 상태를 보관하는 instance.
 *
 * 최초 사용 전 rate_limiter_init()을 호출한다. Runtime field를 직접 변경하지 않고
 * reset/set_rates/update API를 사용해야 precomputed step과 설정이 일치한다.
 */
typedef struct {
    rate_limiter_config_t config; /**< 현재 적용 중인 변화율과 실행 주기. */
    float rise_step;              /**< 한 update에서 증가할 수 있는 최대값 [output unit]. */
    float fall_step;              /**< 한 update에서 감소할 수 있는 최대값의 크기 [output unit]. */
    float output;                 /**< 가장 최근 limited output [output unit]. */
    bool is_initialized;          /**< rate_limiter_init() 완료 여부. */
} rate_limiter_t;

/**
 * @brief Rate limiter를 설정하고 현재 출력을 지정한 초기값으로 만든다.
 *
 * @param[out] self 초기화할 rate limiter instance.
 * @param[in] config 상승·하강 rate와 고정 sampling period 설정.
 * @param[in] initial_output 초기 limited output. 유한한 값이어야 한다.
 *
 * @post 성공하면 rate와 sampling period의 곱을 rise_step/fall_step으로 미리 계산한다.
 * @note 정상적인 closed-loop enable이나 mode 전환에서는 불필요한 reference step을 피하도록
 *       현재 feedback 또는 직전 활성 reference를 @p initial_output 으로 사용한다.
 * @note 출력의 절대 상한·하한은 이 module이 소유하지 않는다. 필요한 경우 호출자가
 *       target에 limiter_clamp()를 적용한 뒤 update한다.
 *
 * @retval RATE_LIMITER_STATUS_OK 초기화 완료.
 * @retval RATE_LIMITER_STATUS_INVALID_ARGUMENT NULL 인자 또는 유한하지 않은 initial_output.
 * @retval RATE_LIMITER_STATUS_INVALID_CONFIG 음수/비유한 rate, 0 이하/비유한 sampling period,
 *                                             또는 float로 표현할 수 없는 step.
 */
rate_limiter_status_t rate_limiter_init(
    rate_limiter_t *self,
    const rate_limiter_config_t *config,
    float initial_output
);

/**
 * @brief 설정은 유지하면서 limited output을 지정한 값으로 즉시 재설정한다.
 *
 * @param[in,out] self 초기화된 rate limiter instance.
 * @param[in] output 새 limited output. 유한한 값이어야 한다.
 *
 * @post 다음 update는 @p output 에서 target을 향해 시작한다.
 * @warning 실행 중 reset은 rate 제한을 거치지 않는 출력 점프를 만든다. Startup, mode 전환
 *          또는 명시적으로 즉시 동기화해야 하는 경우에만 호출한다.
 *
 * @retval RATE_LIMITER_STATUS_OK Output reset 완료.
 * @retval RATE_LIMITER_STATUS_INVALID_ARGUMENT self가 NULL이거나 output이 유한하지 않음.
 * @retval RATE_LIMITER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
rate_limiter_status_t rate_limiter_reset(rate_limiter_t *self, float output);

/**
 * @brief 현재 출력을 유지하면서 상승·하강 변화율을 변경한다.
 *
 * @param[in,out] self 초기화된 rate limiter instance.
 * @param[in] rise_rate_per_s 숫자 증가 방향의 새 최대 변화율 [output unit/s], 0 이상.
 * @param[in] fall_rate_per_s 숫자 감소 방향의 새 최대 변화율 크기 [output unit/s], 0 이상.
 *
 * @post 성공하면 현재 output은 바뀌지 않고 rise_step/fall_step이 즉시 갱신된다.
 * @note 0으로 설정한 방향은 target이 그 방향에 있어도 출력이 움직이지 않는다.
 * @warning rate_limiter_update()와 동시에 호출하지 않는다. 여러 field를 함께 바꾸므로
 *          단일 32-bit float write의 atomicity만으로 configuration 일관성이 보장되지 않는다.
 *
 * @retval RATE_LIMITER_STATUS_OK Rate 변경 완료.
 * @retval RATE_LIMITER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval RATE_LIMITER_STATUS_INVALID_CONFIG 음수/비유한 rate 또는 float로 표현할 수 없는 step.
 * @retval RATE_LIMITER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
rate_limiter_status_t rate_limiter_set_rates(
    rate_limiter_t *self,
    float rise_rate_per_s,
    float fall_rate_per_s
);

/**
 * @brief Target을 향해 한 sampling period만큼 limited output을 갱신한다.
 *
 * @param[in,out] self 초기화된 rate limiter instance.
 * @param[in] target 도달하려는 reference. 유한한 값이어야 한다.
 * @param[out] output 성공 시 갱신된 limited output.
 *
 * @pre 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @post 증가 방향에서는 rise_step 이하, 감소 방향에서는 fall_step 이하로 변한다.
 * @post Target까지 남은 차이가 허용 step 이하면 target을 그대로 출력하여 overshoot하지 않는다.
 * @note 이 함수는 target의 절대 범위를 제한하지 않는다.
 * @note 오류 반환 시 self와 output은 변경하지 않는다.
 *
 * @retval RATE_LIMITER_STATUS_OK Output 갱신 완료.
 * @retval RATE_LIMITER_STATUS_INVALID_ARGUMENT NULL 인자 또는 유한하지 않은 target.
 * @retval RATE_LIMITER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
rate_limiter_status_t rate_limiter_update(
    rate_limiter_t *self,
    float target,
    float *output
);

/** @} */

#endif /* ALGORITHM_RATE_LIMITER_H */
