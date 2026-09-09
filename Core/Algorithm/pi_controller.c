/**
 * @file pi_controller.c
 * @brief Scalar 및 external saturation용 back-calculation PI를 구현한다.
 */

#include "pi_controller.h"

#include <float.h>
#include <stddef.h>

#include "limiter.h"

static bool pi_controller_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool pi_controller_prepare_config(
    const pi_controller_config_t *config,
    float *ki_step,
    float *anti_windup_step
)
{
    float calculated_ki_step;
    float calculated_anti_windup_step;

    if ((!pi_controller_float_is_finite(config->kp)) ||
        (!pi_controller_float_is_finite(config->ki)) ||
        (!pi_controller_float_is_finite(config->anti_windup_gain_per_s)) ||
        (!pi_controller_float_is_finite(config->sampling_period_s)) ||
        (!pi_controller_float_is_finite(config->output_min)) ||
        (!pi_controller_float_is_finite(config->output_max)) ||
        (config->kp < 0.0f) ||
        (config->ki < 0.0f) ||
        (config->anti_windup_gain_per_s < 0.0f) ||
        (config->sampling_period_s <= 0.0f) ||
        (config->output_min >= config->output_max)) {
        return false;
    }

    calculated_ki_step = config->ki * config->sampling_period_s;
    calculated_anti_windup_step =
        config->anti_windup_gain_per_s * config->sampling_period_s;

    if ((!pi_controller_float_is_finite(calculated_ki_step)) ||
        (!pi_controller_float_is_finite(calculated_anti_windup_step))) {
        return false;
    }

    *ki_step = calculated_ki_step;
    *anti_windup_step = calculated_anti_windup_step;

    return true;
}

pi_controller_status_t pi_controller_init(
    pi_controller_t *self,
    const pi_controller_config_t *config
)
{
    pi_controller_t initialized_controller;

    if ((self == NULL) || (config == NULL)) {
        return PI_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!pi_controller_prepare_config(
            config,
            &initialized_controller.ki_step,
            &initialized_controller.anti_windup_step)) {
        return PI_CONTROLLER_STATUS_INVALID_CONFIG;
    }

    initialized_controller.config = *config;
    initialized_controller.integral = 0.0f;
    initialized_controller.unsaturated_output = 0.0f;
    initialized_controller.output = limiter_clamp(
        0.0f,
        config->output_min,
        config->output_max
    );
    initialized_controller.is_initialized = true;

    *self = initialized_controller;

    return PI_CONTROLLER_STATUS_OK;
}

pi_controller_status_t pi_controller_reset(pi_controller_t *self)
{
    if (self == NULL) {
        return PI_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return PI_CONTROLLER_STATUS_INVALID_STATE;
    }

    self->integral = 0.0f;
    self->unsaturated_output = 0.0f;
    self->output = limiter_clamp(
        0.0f,
        self->config.output_min,
        self->config.output_max
    );

    return PI_CONTROLLER_STATUS_OK;
}

pi_controller_status_t pi_controller_update(
    pi_controller_t *self,
    float error,
    float *output
)
{
    float proportional;
    float unsaturated_output;
    float limited_output;
    float integral_delta;
    float next_integral;

    if ((self == NULL) || (output == NULL) ||
        (!pi_controller_float_is_finite(error))) {
        return PI_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return PI_CONTROLLER_STATUS_INVALID_STATE;
    }

    proportional = self->config.kp * error;
    unsaturated_output = proportional + self->integral;

    if ((!pi_controller_float_is_finite(proportional)) ||
        (!pi_controller_float_is_finite(unsaturated_output))) {
        return PI_CONTROLLER_STATUS_NUMERIC_ERROR;
    }

    limited_output = limiter_clamp(
        unsaturated_output,
        self->config.output_min,
        self->config.output_max
    );

    integral_delta = (self->ki_step * error) +
        (self->anti_windup_step * (limited_output - unsaturated_output));
    next_integral = self->integral + integral_delta;

    if ((!pi_controller_float_is_finite(integral_delta)) ||
        (!pi_controller_float_is_finite(next_integral))) {
        return PI_CONTROLLER_STATUS_NUMERIC_ERROR;
    }

    self->integral = next_integral;
    self->unsaturated_output = unsaturated_output;
    self->output = limited_output;
    *output = limited_output;

    return PI_CONTROLLER_STATUS_OK;
}

pi_controller_status_t pi_controller_apply_tracking(
    pi_controller_t *self,
    float applied_output
)
{
    float tracking_delta;
    float next_integral;

    if ((self == NULL) || (!pi_controller_float_is_finite(applied_output))) {
        return PI_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return PI_CONTROLLER_STATUS_INVALID_STATE;
    }

    if (self->anti_windup_step == 0.0f) {
        self->output = applied_output;
        return PI_CONTROLLER_STATUS_OK;
    }

    tracking_delta =
        self->anti_windup_step * (applied_output - self->output);
    next_integral = self->integral + tracking_delta;

    if ((!pi_controller_float_is_finite(tracking_delta)) ||
        (!pi_controller_float_is_finite(next_integral))) {
        return PI_CONTROLLER_STATUS_NUMERIC_ERROR;
    }

    self->integral = next_integral;
    self->output = applied_output;

    return PI_CONTROLLER_STATUS_OK;
}
