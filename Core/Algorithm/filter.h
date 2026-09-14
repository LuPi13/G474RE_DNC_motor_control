/**
 * @file filter.h
 * @brief 고정 주기로 실행하는 scalar 1차 저역통과 IIR filter API.
 * @ingroup algorithm_filter
 */

#ifndef ALGORITHM_FILTER_H
#define ALGORITHM_FILTER_H

#include <stdbool.h>

/**
 * @defgroup algorithm_filter Filter
 * @brief 하드웨어와 무관한 scalar 1차 저역통과 IIR filter.
 *
 * 연속 시간 1차 저역통과 filter의 pole을 matched-pole 방식으로 이산화한다.
 *
 * @f[
 * \alpha = 1 - e^{-2\pi f_c T_s}
 * @f]
 *
 * @f[
 * y[k] = y[k-1] + \alpha (x[k] - y[k-1])
 * @f]
 *
 * `cutoff_frequency_hz`는 연속 시간 pole에 대응하는 주파수이며 Nyquist frequency보다
 * 작아야 한다. 실행 중에는 미리 계산한 coefficient로 곱셈과 덧셈만 수행한다.
 *
 * 하나의 instance는 하나의 scalar 신호만 소유한다. FOC에서 `i_d`, `i_q`를 filtering할
 * 때에는 두 instance를 사용하며, Clarke/Park 변환 뒤의 feedback 경로에 배치한다.
 * 상전류 과전류 보호 경로에는 지연이 생기는 이 filter 출력을 사용하지 않는다.
 *
 * 호출자는 filter_low_pass_update()를 `sampling_period_s`와 같은 고정 주기로 실행해야 한다.
 * Module 내부에는 scheduler나 prescaler가 없다.
 * @{
 */

/**
 * @brief 1차 저역통과 filter 함수의 실행 결과.
 */
typedef enum {
    FILTER_STATUS_OK = 0,           /**< 요청을 정상적으로 처리함. */
    FILTER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 값. */
    FILTER_STATUS_INVALID_CONFIG,   /**< Cutoff, sampling period 또는 coefficient가 유효하지 않음. */
    FILTER_STATUS_INVALID_STATE,    /**< 초기화되지 않은 instance를 사용함. */
    FILTER_STATUS_NUMERIC_ERROR     /**< Update 계산 결과를 float로 표현할 수 없음. */
} filter_status_t;

/**
 * @brief 1차 저역통과 filter의 고정 설정.
 */
typedef struct {
    float cutoff_frequency_hz; /**< 연속 시간 pole에 대응하는 cutoff frequency [Hz], 0보다 큼. */
    float sampling_period_s;   /**< filter_low_pass_update()의 고정 호출 주기 [s], 0보다 큼. */
} filter_low_pass_config_t;

/**
 * @brief 1차 저역통과 filter의 설정과 runtime 상태를 보관하는 instance.
 *
 * 최초 사용 전에 filter_low_pass_init()을 호출한다. Runtime field를 직접 변경하지 않고
 * init/reset/update API를 사용해야 설정, coefficient와 출력 상태의 일관성이 유지된다.
 */
typedef struct {
    filter_low_pass_config_t config; /**< 현재 적용 중인 cutoff와 sampling period. */
    float coefficient;               /**< 미리 계산한 IIR coefficient alpha, 0보다 크고 1보다 작음. */
    float output;                    /**< 가장 최근의 filtered output. */
    bool is_initialized;             /**< filter_low_pass_init() 완료 여부. */
} filter_low_pass_t;

/**
 * @brief 1차 저역통과 filter를 설정하고 출력을 지정한 값으로 초기화한다.
 *
 * @param[out] self 초기화할 filter instance.
 * @param[in] config Cutoff frequency와 고정 sampling period 설정.
 * @param[in] initial_output 초기 filtered output. 유한한 값이어야 한다.
 *
 * @pre `cutoff_frequency_hz < 1 / (2 * sampling_period_s)`이어야 한다.
 * @post 성공하면 coefficient를 한 번 계산하고, 이후 update hot path에서는 expf()를 호출하지 않는다.
 * @note Closed-loop enable 시 불필요한 과도 응답을 피하려면 첫 유효 feedback을
 *       @p initial_output 으로 사용한다.
 * @note 오류를 반환하면 @p self 는 변경되지 않는다.
 *
 * @retval FILTER_STATUS_OK 초기화 완료.
 * @retval FILTER_STATUS_INVALID_ARGUMENT NULL 인자 또는 유한하지 않은 initial_output.
 * @retval FILTER_STATUS_INVALID_CONFIG 유효하지 않은 cutoff/sampling period 또는 float로
 *                                      표현할 수 없는 coefficient.
 */
filter_status_t filter_low_pass_init(
    filter_low_pass_t *self,
    const filter_low_pass_config_t *config,
    float initial_output
);

/**
 * @brief 설정과 coefficient는 유지하면서 filtered output을 즉시 재설정한다.
 *
 * @param[in,out] self 초기화된 filter instance.
 * @param[in] output 새 filtered output. 유한한 값이어야 한다.
 *
 * @note 오류를 반환하면 @p self 는 변경되지 않는다.
 *
 * @retval FILTER_STATUS_OK Output 재설정 완료.
 * @retval FILTER_STATUS_INVALID_ARGUMENT self가 NULL이거나 output이 유한하지 않음.
 * @retval FILTER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
filter_status_t filter_low_pass_reset(filter_low_pass_t *self, float output);

/**
 * @brief 검증된 값으로 fast-loop filter 출력을 즉시 재설정한다.
 *
 * @param[in,out] self 초기화된 filter instance.
 * @param[in] output 상위 경계에서 유한성이 보장된 새 출력.
 * @pre self는 NULL이 아니고 초기화되어 있으며 output은 유한해야 한다.
 * @warning 인자와 state를 검사하지 않는다. 일반 경로에서는 filter_low_pass_reset()을 사용한다.
 */
void filter_low_pass_reset_fast(filter_low_pass_t *self, float output);

/**
 * @brief 입력 한 sample로 1차 저역통과 filter 출력을 갱신한다.
 *
 * @param[in,out] self 초기화된 filter instance.
 * @param[in] input 현재 입력 sample. 유한한 값이어야 한다.
 * @param[out] output 성공 시 갱신된 filtered output.
 *
 * @pre 호출 간격은 config의 `sampling_period_s`와 일치해야 한다.
 * @note 오류를 반환하면 @p self 와 @p output 은 변경되지 않는다.
 *
 * @retval FILTER_STATUS_OK Output 갱신 완료.
 * @retval FILTER_STATUS_INVALID_ARGUMENT NULL 인자 또는 유한하지 않은 input.
 * @retval FILTER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval FILTER_STATUS_NUMERIC_ERROR 중간값 또는 결과를 float로 표현할 수 없음.
 */
filter_status_t filter_low_pass_update(
    filter_low_pass_t *self,
    float input,
    float *output
);

/**
 * @brief 검증이 끝난 fast-loop 입력으로 filter를 최소 연산만 수행해 갱신한다.
 *
 * @param[in,out] self 초기화된 filter instance.
 * @param[in] input 상위 fast path에서 유한성이 보장된 입력 sample.
 * @return 갱신된 filtered output.
 *
 * @pre self는 NULL이 아니고 초기화되어 있어야 한다.
 * @pre input과 기존 output 및 coefficient로 계산한 결과가 유한해야 한다.
 * @pre 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @warning 인자와 계산 결과를 검사하지 않는다. 일반 경로에서는 filter_low_pass_update()를 사용한다.
 */
float filter_low_pass_update_fast(filter_low_pass_t *self, float input);

/** @} */

#endif /* ALGORITHM_FILTER_H */
