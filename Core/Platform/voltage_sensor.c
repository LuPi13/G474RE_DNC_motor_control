/**
 * @file voltage_sensor.c
 * @brief DC-link 전압 센서의 ADC offset 보정과 SI 단위 환산을 구현한다.
 */

#include "voltage_sensor.h"

#include <math.h>
#include <stddef.h>

#define VOLTAGE_SENSOR_MAX_ADC_CODE  (4095U)

voltage_sensor_status_t voltage_sensor_init(
    voltage_sensor_t *self,
    const voltage_sensor_config_t *config
)
{
    if ((self == NULL) || (config == NULL)) {
        return VOLTAGE_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->offset_counts) ||
        (config->offset_counts < 0.0f) ||
        (config->offset_counts > (float)VOLTAGE_SENSOR_MAX_ADC_CODE) ||
        !isfinite(config->gain_v_per_count) ||
        (config->gain_v_per_count == 0.0f)) {
        return VOLTAGE_SENSOR_STATUS_INVALID_CONFIG;
    }

    *self = (voltage_sensor_t){
        .config = *config,
        .is_initialized = true,
    };
    return VOLTAGE_SENSOR_STATUS_OK;
}

voltage_sensor_status_t voltage_sensor_convert(
    const voltage_sensor_t *self,
    uint16_t raw_counts,
    float *v_dc
)
{
    if ((self == NULL) || (v_dc == NULL) ||
        (raw_counts > VOLTAGE_SENSOR_MAX_ADC_CODE)) {
        return VOLTAGE_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return VOLTAGE_SENSOR_STATUS_INVALID_STATE;
    }

    *v_dc = ((float)raw_counts - self->config.offset_counts) *
        self->config.gain_v_per_count;
    return VOLTAGE_SENSOR_STATUS_OK;
}
