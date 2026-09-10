/**
 * @file current_sensor.c
 * @brief 3상 전류 센서의 무전류 offset 보정과 SI 단위 환산을 구현한다.
 */

#include "current_sensor.h"

#include <math.h>
#include <stddef.h>

#define CURRENT_SENSOR_MAX_ADC_CODE  (4095U)

static bool current_sensor_is_valid_raw(
    const adc_driver_raw_sample_t *raw
)
{
    return (raw != NULL) &&
        (raw->phase_a <= CURRENT_SENSOR_MAX_ADC_CODE) &&
        (raw->phase_b <= CURRENT_SENSOR_MAX_ADC_CODE) &&
        (raw->phase_c <= CURRENT_SENSOR_MAX_ADC_CODE);
}

static bool current_sensor_is_valid_gain(float gain_a_per_count)
{
    return isfinite(gain_a_per_count) && (gain_a_per_count != 0.0f);
}

static bool current_sensor_offset_is_in_range(
    const current_sensor_t *self,
    float offset_counts
)
{
    return (offset_counts >= self->config.minimum_offset_counts) &&
        (offset_counts <= self->config.maximum_offset_counts);
}

current_sensor_status_t current_sensor_init(
    current_sensor_t *self,
    const current_sensor_config_t *config
)
{
    if ((self == NULL) || (config == NULL)) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }

    if (!current_sensor_is_valid_gain(config->gain_a_per_count.a) ||
        !current_sensor_is_valid_gain(config->gain_a_per_count.b) ||
        !current_sensor_is_valid_gain(config->gain_a_per_count.c) ||
        (config->averaging_sample_count == 0U) ||
        !isfinite(config->minimum_offset_counts) ||
        !isfinite(config->maximum_offset_counts) ||
        (config->minimum_offset_counts < 0.0f) ||
        (config->maximum_offset_counts > (float)CURRENT_SENSOR_MAX_ADC_CODE) ||
        (config->minimum_offset_counts >= config->maximum_offset_counts)) {
        return CURRENT_SENSOR_STATUS_INVALID_CONFIG;
    }

    *self = (current_sensor_t){
        .config = *config,
        .calibration_state = CURRENT_SENSOR_OFFSET_CALIBRATION_IDLE,
        .is_initialized = true,
    };
    return CURRENT_SENSOR_STATUS_OK;
}

current_sensor_status_t current_sensor_start_offset_calibration(
    current_sensor_t *self
)
{
    if (self == NULL) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized ||
        (self->calibration_state ==
            CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING)) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }

    self->phase_a_sum_counts = 0U;
    self->phase_b_sum_counts = 0U;
    self->phase_c_sum_counts = 0U;
    self->settled_sample_count = 0U;
    self->averaged_sample_count = 0U;

    /* ADC ISR가 RUNNING을 보기 전에 누적 상태 초기화를 완료한다. */
    __DMB();
    self->calibration_state = CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING;
    return CURRENT_SENSOR_STATUS_OK;
}

current_sensor_status_t current_sensor_process_offset_sample(
    current_sensor_t *self,
    const adc_driver_raw_sample_t *raw
)
{
    if ((self == NULL) || !current_sensor_is_valid_raw(raw)) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized ||
        (self->calibration_state !=
            CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING)) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }

    if (self->settled_sample_count < self->config.settling_sample_count) {
        ++self->settled_sample_count;
        return CURRENT_SENSOR_STATUS_OK;
    }

    self->phase_a_sum_counts += raw->phase_a;
    self->phase_b_sum_counts += raw->phase_b;
    self->phase_c_sum_counts += raw->phase_c;
    ++self->averaged_sample_count;

    if (self->averaged_sample_count < self->config.averaging_sample_count) {
        return CURRENT_SENSOR_STATUS_OK;
    }

    const float sample_count = (float)self->config.averaging_sample_count;
    const float phase_a_offset_counts =
        (float)self->phase_a_sum_counts / sample_count;
    const float phase_b_offset_counts =
        (float)self->phase_b_sum_counts / sample_count;
    const float phase_c_offset_counts =
        (float)self->phase_c_sum_counts / sample_count;

    if (!current_sensor_offset_is_in_range(self, phase_a_offset_counts) ||
        !current_sensor_offset_is_in_range(self, phase_b_offset_counts) ||
        !current_sensor_offset_is_in_range(self, phase_c_offset_counts)) {
        self->calibration_state = CURRENT_SENSOR_OFFSET_CALIBRATION_FAILED;
        return CURRENT_SENSOR_STATUS_OFFSET_OUT_OF_RANGE;
    }

    self->offset_counts = (abc_t){
        .a = phase_a_offset_counts,
        .b = phase_b_offset_counts,
        .c = phase_c_offset_counts,
    };
    __DMB();
    self->calibration_state = CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE;
    return CURRENT_SENSOR_STATUS_OK;
}

current_sensor_status_t current_sensor_abort_offset_calibration(
    current_sensor_t *self
)
{
    current_sensor_offset_calibration_state_t expected_state =
        CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING;

    if (self == NULL) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }

    if (!__atomic_compare_exchange_n(
            &self->calibration_state,
            &expected_state,
            CURRENT_SENSOR_OFFSET_CALIBRATION_FAILED,
            false,
            __ATOMIC_ACQ_REL,
            __ATOMIC_ACQUIRE)) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }
    __DMB();
    return CURRENT_SENSOR_STATUS_OK;
}

current_sensor_status_t current_sensor_get_offset_calibration_state(
    const current_sensor_t *self,
    current_sensor_offset_calibration_state_t *state
)
{
    if ((self == NULL) || (state == NULL)) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }

    *state = self->calibration_state;
    __DMB();
    return CURRENT_SENSOR_STATUS_OK;
}

current_sensor_status_t current_sensor_convert(
    const current_sensor_t *self,
    const adc_driver_raw_sample_t *raw,
    abc_t *i_abc
)
{
    if ((self == NULL) || (i_abc == NULL) ||
        !current_sensor_is_valid_raw(raw)) {
        return CURRENT_SENSOR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized ||
        (self->calibration_state !=
            CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE)) {
        return CURRENT_SENSOR_STATUS_INVALID_STATE;
    }
    __DMB();

    *i_abc = (abc_t){
        .a = ((float)raw->phase_a - self->offset_counts.a) *
            self->config.gain_a_per_count.a,
        .b = ((float)raw->phase_b - self->offset_counts.b) *
            self->config.gain_a_per_count.b,
        .c = ((float)raw->phase_c - self->offset_counts.c) *
            self->config.gain_a_per_count.c,
    };
    return CURRENT_SENSOR_STATUS_OK;
}
