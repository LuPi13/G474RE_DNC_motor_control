/**
 * @file voltage_sensor.h
 * @brief DC-link 전압 센서의 ADC raw code 환산 API.
 * @ingroup platform_voltage_sensor
 */

#ifndef PLATFORM_VOLTAGE_SENSOR_H
#define PLATFORM_VOLTAGE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup platform_voltage_sensor Voltage sensor
 * @brief ADC raw code를 DC-link 전압 [V]로 변환한다.
 * @{
 */

/**
 * @brief Voltage sensor 함수의 실행 결과.
 */
typedef enum {
    VOLTAGE_SENSOR_STATUS_OK = 0,          /**< 요청한 처리를 완료함. */
    VOLTAGE_SENSOR_STATUS_INVALID_ARGUMENT, /**< NULL 또는 범위 밖 raw code. */
    VOLTAGE_SENSOR_STATUS_INVALID_CONFIG,  /**< Offset 또는 gain이 유효하지 않음. */
    VOLTAGE_SENSOR_STATUS_INVALID_STATE    /**< 초기화되지 않은 instance. */
} voltage_sensor_status_t;

/**
 * @brief DC-link 전압 센서 환산 설정.
 */
typedef struct {
    float offset_counts;    /**< 0 V의 ADC code [count], 유한한 [0, 4095]. */
    float gain_v_per_count; /**< `(raw - offset) * gain`의 gain [V/count], 유한한 0 이외 값. */
} voltage_sensor_config_t;

/**
 * @brief DC-link 전압 센서 설정과 초기화 상태.
 */
typedef struct {
    voltage_sensor_config_t config; /**< 초기화 시 복사한 환산 설정. */
    bool is_initialized;            /**< voltage_sensor_init() 완료 여부. */
} voltage_sensor_t;

/**
 * @brief DC-link 전압 센서 환산 설정을 초기화한다.
 *
 * @param[out] self 초기화할 voltage sensor instance.
 * @param[in] config ADC offset과 전압 gain.
 * @retval VOLTAGE_SENSOR_STATUS_OK 초기화 완료.
 * @retval VOLTAGE_SENSOR_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval VOLTAGE_SENSOR_STATUS_INVALID_CONFIG Offset 또는 gain 범위 오류.
 */
voltage_sensor_status_t voltage_sensor_init(
    voltage_sensor_t *self,
    const voltage_sensor_config_t *config
);

/**
 * @brief ADC raw code를 DC-link 전압 [V]로 환산한다.
 *
 * @param[in] self 초기화된 voltage sensor instance.
 * @param[in] raw_counts 12-bit ADC raw code [count].
 * @param[out] v_dc 환산된 DC-link 전압 [V]. 오류 시 변경하지 않음.
 * @retval VOLTAGE_SENSOR_STATUS_OK 전압 환산 완료.
 * @retval VOLTAGE_SENSOR_STATUS_INVALID_ARGUMENT NULL 또는 [0, 4095] 밖의 raw code.
 * @retval VOLTAGE_SENSOR_STATUS_INVALID_STATE 초기화되지 않음.
 */
voltage_sensor_status_t voltage_sensor_convert(
    const voltage_sensor_t *self,
    uint16_t raw_counts,
    float *v_dc
);

/** @} */

#endif /* PLATFORM_VOLTAGE_SENSOR_H */
