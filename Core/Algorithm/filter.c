/**
 * @file filter.c
 * @brief Matched-pole 방식의 scalar 1차 저역통과 IIR filter를 구현한다.
 */

#include "filter.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define FILTER_TWO_PI_F (6.28318530717958647692f)

static bool filter_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool filter_calculate_low_pass_coefficient(
    const filter_low_pass_config_t *config,
    float *coefficient
)
{
    float cutoff_times_period;
    float calculated_coefficient;

    if ((!filter_float_is_finite(config->cutoff_frequency_hz)) ||
        (!filter_float_is_finite(config->sampling_period_s)) ||
        (config->cutoff_frequency_hz <= 0.0f) ||
        (config->sampling_period_s <= 0.0f)) {
        return false;
    }

    cutoff_times_period =
        config->cutoff_frequency_hz * config->sampling_period_s;

    /* fc * Ts < 0.5는 cutoff가 Nyquist frequency보다 작다는 조건이다. */
    if ((!filter_float_is_finite(cutoff_times_period)) ||
        (cutoff_times_period <= 0.0f) ||
        (cutoff_times_period >= 0.5f)) {
        return false;
    }

    calculated_coefficient =
        1.0f - expf(-FILTER_TWO_PI_F * cutoff_times_period);

    if ((!filter_float_is_finite(calculated_coefficient)) ||
        (calculated_coefficient <= 0.0f) ||
        (calculated_coefficient >= 1.0f)) {
        return false;
    }

    *coefficient = calculated_coefficient;

    return true;
}

filter_status_t filter_low_pass_init(
    filter_low_pass_t *self,
    const filter_low_pass_config_t *config,
    float initial_output
)
{
    filter_low_pass_t initialized_filter;

    if ((self == NULL) || (config == NULL) ||
        (!filter_float_is_finite(initial_output))) {
        return FILTER_STATUS_INVALID_ARGUMENT;
    }

    if (!filter_calculate_low_pass_coefficient(
            config,
            &initialized_filter.coefficient)) {
        return FILTER_STATUS_INVALID_CONFIG;
    }

    initialized_filter.config = *config;
    initialized_filter.output = initial_output;
    initialized_filter.is_initialized = true;

    *self = initialized_filter;

    return FILTER_STATUS_OK;
}

filter_status_t filter_low_pass_reset(filter_low_pass_t *self, float output)
{
    if ((self == NULL) || (!filter_float_is_finite(output))) {
        return FILTER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FILTER_STATUS_INVALID_STATE;
    }

    self->output = output;

    return FILTER_STATUS_OK;
}

filter_status_t filter_low_pass_update(
    filter_low_pass_t *self,
    float input,
    float *output
)
{
    float delta;
    float correction;
    float next_output;

    if ((self == NULL) || (output == NULL) ||
        (!filter_float_is_finite(input))) {
        return FILTER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FILTER_STATUS_INVALID_STATE;
    }

    delta = input - self->output;
    correction = self->coefficient * delta;
    next_output = self->output + correction;

    if ((!filter_float_is_finite(delta)) ||
        (!filter_float_is_finite(correction)) ||
        (!filter_float_is_finite(next_output))) {
        return FILTER_STATUS_NUMERIC_ERROR;
    }

    self->output = next_output;
    *output = next_output;

    return FILTER_STATUS_OK;
}

