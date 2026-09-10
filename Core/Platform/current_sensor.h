/**
 * @file current_sensor.h
 * @brief 3상 전류 센서의 ADC 영점 보정과 전류 환산 API.
 * @ingroup platform_current_sensor
 */

#ifndef PLATFORM_CURRENT_SENSOR_H
#define PLATFORM_CURRENT_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_driver.h"
#include "vector_types.h"

/**
 * @defgroup platform_current_sensor Current sensor
 * @brief ADC raw code를 상별로 보정하여 3상 전류 [A]로 변환한다.
 *
 * ADC peripheral 접근은 adc_driver가 담당한다. 이 module은 센서 gain, 무전류
 * offset, 보정 누적 상태의 유일한 owner이며 HAL이나 peripheral register에 접근하지 않는다.
 * 현재 보드의 ACS725LLCTR-10AB-T 세 개를 동일한 interface로 처리한다.
 * @{
 */

/**
 * @brief Current sensor 함수의 실행 결과.
 */
typedef enum {
    CURRENT_SENSOR_STATUS_OK = 0,          /**< 요청한 처리를 완료함. */
    CURRENT_SENSOR_STATUS_INVALID_ARGUMENT, /**< NULL 또는 범위 밖 raw code. */
    CURRENT_SENSOR_STATUS_INVALID_CONFIG,  /**< Gain, sample 수 또는 offset 범위가 유효하지 않음. */
    CURRENT_SENSOR_STATUS_INVALID_STATE,   /**< 초기화/보정 상태가 요청한 동작에 맞지 않음. */
    CURRENT_SENSOR_STATUS_OFFSET_OUT_OF_RANGE /**< 측정한 하나 이상의 영점이 허용 범위를 벗어남. */
} current_sensor_status_t;

/**
 * @brief 무전류 offset 보정의 진행 상태.
 */
typedef enum {
    CURRENT_SENSOR_OFFSET_CALIBRATION_IDLE = 0, /**< 아직 보정을 시작하지 않음. */
    CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING,  /**< Settling/평균 sample을 수집 중임. */
    CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE, /**< Offset을 계산했고 전류 환산이 가능함. */
    CURRENT_SENSOR_OFFSET_CALIBRATION_FAILED    /**< 보정이 중단되었거나 결과가 유효하지 않음. */
} current_sensor_offset_calibration_state_t;

/**
 * @brief 3상 전류 센서의 환산 및 영점 보정 설정.
 */
typedef struct {
    abc_t gain_a_per_count; /**< 상별 `(raw - offset) * gain`의 gain [A/count]. */
    uint32_t settling_sample_count; /**< 평균 전에 버릴 ADC sample 수. 0 허용. */
    uint32_t averaging_sample_count; /**< Offset 평균에 포함할 sample 수. 1 이상. */
    float minimum_offset_counts; /**< 정상으로 허용할 offset ADC code 하한 [count]. */
    float maximum_offset_counts; /**< 정상으로 허용할 offset ADC code 상한 [count]. */
} current_sensor_config_t;

/**
 * @brief 전류 센서 설정, offset 보정 상태와 진단값.
 *
 * Raw ADC count와 offset 누적 상태는 이 Platform instance만 소유한다. App과 Control은
 * public API를 통해 보정 상태 또는 환산된 abc_t 전류 [A]만 사용한다.
 */
typedef struct {
    current_sensor_config_t config; /**< 초기화 시 복사한 센서/보정 설정. */
    uint64_t phase_a_sum_counts; /**< a상 offset 평균용 누적 합 [count]. */
    uint64_t phase_b_sum_counts; /**< b상 offset 평균용 누적 합 [count]. */
    uint64_t phase_c_sum_counts; /**< c상 offset 평균용 누적 합 [count]. */
    abc_t offset_counts; /**< 마지막 완료된 상별 offset [count]. Platform 진단값. */
    uint32_t settled_sample_count; /**< 현재 요청에서 폐기한 sample 수. */
    uint32_t averaged_sample_count; /**< 현재 요청에서 누적한 sample 수. */
    volatile current_sensor_offset_calibration_state_t calibration_state; /**< Main/ISR 공유 상태. */
    bool is_initialized; /**< current_sensor_init() 완료 여부. */
} current_sensor_t;

/**
 * @brief 전류 센서 설정과 보정 초기 상태를 준비한다.
 *
 * @param[out] self 초기화할 current sensor instance.
 * @param[in] config 상별 gain과 offset 측정 조건.
 *
 * @post 성공 시 calibration_state는 IDLE이며 current_sensor_convert()는 아직 허용되지 않는다.
 * @retval CURRENT_SENSOR_STATUS_OK 초기화 완료.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval CURRENT_SENSOR_STATUS_INVALID_CONFIG 유효하지 않은 gain, sample 수 또는 offset 범위.
 */
current_sensor_status_t current_sensor_init(
    current_sensor_t *self,
    const current_sensor_config_t *config
);

/**
 * @brief 무전류 offset 보정 누적 상태를 시작한다.
 *
 * @param[in,out] self 초기화된 current sensor instance.
 * @pre 호출자가 PWM output 비활성, 실제 상전류 0 A와 rotor 정지를 보장한다.
 * @note 이전 완료 offset은 새 보정이 완료될 때까지 보존하지만, RUNNING 중 전류 환산은 거부한다.
 *
 * @retval CURRENT_SENSOR_STATUS_OK 새 보정 시작.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval CURRENT_SENSOR_STATUS_INVALID_STATE 초기화되지 않았거나 이미 보정 중임.
 */
current_sensor_status_t current_sensor_start_offset_calibration(
    current_sensor_t *self
);

/**
 * @brief Offset 보정에 ADC raw sample 하나를 반영한다.
 *
 * @param[in,out] self RUNNING 상태의 current sensor instance.
 * @param[in] raw ADC driver가 반환한 12-bit raw sample.
 * @post 지정한 settling sample을 버린 뒤 평균 수를 채우면 COMPLETE가 된다.
 * @note ADC IRQ fast-loop 한 곳에서만 호출하여 누적 상태 변경을 직렬화한다.
 *
 * @retval CURRENT_SENSOR_STATUS_OK Sample 폐기/누적 또는 보정 완료.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT NULL 또는 [0, 4095] 밖의 전류 raw code.
 * @retval CURRENT_SENSOR_STATUS_INVALID_STATE 초기화되지 않았거나 RUNNING 상태가 아님.
 * @retval CURRENT_SENSOR_STATUS_OFFSET_OUT_OF_RANGE 계산한 offset이 허용 범위를 벗어남.
 */
current_sensor_status_t current_sensor_process_offset_sample(
    current_sensor_t *self,
    const adc_driver_raw_sample_t *raw
);

/**
 * @brief 외부 timeout 등으로 진행 중인 offset 보정을 실패 상태로 종료한다.
 *
 * @param[in,out] self RUNNING 상태의 current sensor instance.
 * @note Main에서 호출하는 동안 ADC ISR이 보정을 완료하는 경쟁은 atomic state 전이로 보호한다.
 * @retval CURRENT_SENSOR_STATUS_OK FAILED 상태로 전환함.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval CURRENT_SENSOR_STATUS_INVALID_STATE 초기화되지 않았거나 RUNNING 상태가 아님.
 */
current_sensor_status_t current_sensor_abort_offset_calibration(
    current_sensor_t *self
);

/**
 * @brief Offset 보정 상태를 반환한다.
 *
 * @param[in] self 초기화된 current sensor instance.
 * @param[out] state 현재 offset 보정 상태.
 * @note Offset과 sample count는 current_sensor_t가 소유하는 Platform 진단값이다.
 *       Control/App feedback으로 복제하지 않는다.
 * @retval CURRENT_SENSOR_STATUS_OK 상태 반환 완료.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval CURRENT_SENSOR_STATUS_INVALID_STATE 초기화되지 않음.
 */
current_sensor_status_t current_sensor_get_offset_calibration_state(
    const current_sensor_t *self,
    current_sensor_offset_calibration_state_t *state
);

/**
 * @brief 보정된 ADC raw code를 3상 전류 [A]로 환산한다.
 *
 * @param[in] self offset 보정을 완료한 current sensor instance.
 * @param[in] raw ADC driver가 반환한 12-bit raw sample.
 * @param[out] i_abc 보정된 a/b/c상 전류 [A]. 오류 시 변경하지 않음.
 * @details 각 상에 `(float(raw) - offset_counts) * gain_a_per_count`를 적용한다.
 *          Filtering과 clamp는 수행하지 않는다.
 *
 * @retval CURRENT_SENSOR_STATUS_OK 전류 환산 완료.
 * @retval CURRENT_SENSOR_STATUS_INVALID_ARGUMENT NULL 또는 [0, 4095] 밖의 전류 raw code.
 * @retval CURRENT_SENSOR_STATUS_INVALID_STATE 초기화되지 않았거나 보정이 완료되지 않음.
 */
current_sensor_status_t current_sensor_convert(
    const current_sensor_t *self,
    const adc_driver_raw_sample_t *raw,
    abc_t *i_abc
);

/** @} */

#endif /* PLATFORM_CURRENT_SENSOR_H */
