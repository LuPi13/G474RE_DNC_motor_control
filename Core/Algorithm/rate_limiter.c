/**
 * @file rate_limiter.c
 * @brief 고정 sampling period 기반의 비대칭 scalar rate limiter를 구현한다.
 */

#include "rate_limiter.h"

#include <float.h>
#include <stddef.h>

static bool rate_limiter_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool rate_limiter_calculate_steps(
    float rise_rate_per_s,
    float fall_rate_per_s,
    float sampling_period_s,
    float *rise_step,
    float *fall_step
)
{
    float calculated_rise_step;
    float calculated_fall_step;

    if ((!rate_limiter_float_is_finite(rise_rate_per_s)) ||
        (!rate_limiter_float_is_finite(fall_rate_per_s)) ||
        (!rate_limiter_float_is_finite(sampling_period_s)) ||
        (rise_rate_per_s < 0.0f) ||
        (fall_rate_per_s < 0.0f) ||
        (sampling_period_s <= 0.0f)) {
        return false;
    }

    calculated_rise_step = rise_rate_per_s * sampling_period_s;
    calculated_fall_step = fall_rate_per_s * sampling_period_s;

    if ((!rate_limiter_float_is_finite(calculated_rise_step)) ||
        (!rate_limiter_float_is_finite(calculated_fall_step))) {
        return false;
    }

    *rise_step = calculated_rise_step;
    *fall_step = calculated_fall_step;

    return true;
}

rate_limiter_status_t rate_limiter_init(
    rate_limiter_t *self,
    const rate_limiter_config_t *config,
    float initial_output
)
{
    rate_limiter_t initialized_limiter;

    if ((self == NULL) || (config == NULL) ||
        (!rate_limiter_float_is_finite(initial_output))) {
        return RATE_LIMITER_STATUS_INVALID_ARGUMENT;
    }

    if (!rate_limiter_calculate_steps(
            config->rise_rate_per_s,
            config->fall_rate_per_s,
            config->sampling_period_s,
            &initialized_limiter.rise_step,
            &initialized_limiter.fall_step)) {
        return RATE_LIMITER_STATUS_INVALID_CONFIG;
    }

    initialized_limiter.config = *config;
    initialized_limiter.output = initial_output;
    initialized_limiter.is_initialized = true;

    *self = initialized_limiter;

    return RATE_LIMITER_STATUS_OK;
}

rate_limiter_status_t rate_limiter_reset(rate_limiter_t *self, float output)
{
    if ((self == NULL) || (!rate_limiter_float_is_finite(output))) {
        return RATE_LIMITER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return RATE_LIMITER_STATUS_INVALID_STATE;
    }

    self->output = output;

    return RATE_LIMITER_STATUS_OK;
}

rate_limiter_status_t rate_limiter_set_rates(
    rate_limiter_t *self,
    float rise_rate_per_s,
    float fall_rate_per_s
)
{
    float rise_step;
    float fall_step;

    if (self == NULL) {
        return RATE_LIMITER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return RATE_LIMITER_STATUS_INVALID_STATE;
    }

    if (!rate_limiter_calculate_steps(
            rise_rate_per_s,
            fall_rate_per_s,
            self->config.sampling_period_s,
            &rise_step,
            &fall_step)) {
        return RATE_LIMITER_STATUS_INVALID_CONFIG;
    }

    /* 모든 검증을 마친 뒤 함께 갱신해 실패 시 기존 configuration을 보존한다. */
    self->config.rise_rate_per_s = rise_rate_per_s;
    self->config.fall_rate_per_s = fall_rate_per_s;
    self->rise_step = rise_step;
    self->fall_step = fall_step;

    return RATE_LIMITER_STATUS_OK;
}

rate_limiter_status_t rate_limiter_update(
    rate_limiter_t *self,
    float target,
    float *output
)
{
    float next_output;
    float remaining;

    if ((self == NULL) || (output == NULL) ||
        (!rate_limiter_float_is_finite(target))) {
        return RATE_LIMITER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return RATE_LIMITER_STATUS_INVALID_STATE;
    }

    next_output = self->output;

    if (target > self->output) {
        remaining = target - self->output;
        if (remaining > self->rise_step) {
            next_output += self->rise_step;
        } else {
            next_output = target;
        }
    } else if (target < self->output) {
        remaining = self->output - target;
        if (remaining > self->fall_step) {
            next_output -= self->fall_step;
        } else {
            next_output = target;
        }
    }

    self->output = next_output;
    *output = next_output;

    return RATE_LIMITER_STATUS_OK;
}
