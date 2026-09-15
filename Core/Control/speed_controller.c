/**
 * @file speed_controller.c
 * @brief Filtered mechanical-speed PI와 q축 전류 제한을 구현한다.
 */

#include "speed_controller.h"

#include <float.h>
#include <stddef.h>

static bool speed_controller_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool speed_controller_config_is_valid(
    const speed_controller_config_t *config
)
{
    if (config == NULL) {
        return false;
    }

    return (config->pi.output_min <= 0.0f) &&
        (config->pi.output_max >= 0.0f) &&
        (config->pi.sampling_period_s ==
            config->feedback_filter.sampling_period_s);
}

speed_controller_status_t speed_controller_init(
    speed_controller_t *self,
    const speed_controller_config_t *config
)
{
    speed_controller_t initialized_controller = {0};

    if ((self == NULL) || (config == NULL)) {
        return SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if ((!speed_controller_config_is_valid(config)) ||
        (pi_controller_init(
            &initialized_controller.pi,
            &config->pi) != PI_CONTROLLER_STATUS_OK) ||
        (filter_low_pass_init(
            &initialized_controller.feedback_filter,
            &config->feedback_filter,
            0.0f) != FILTER_STATUS_OK)) {
        return SPEED_CONTROLLER_STATUS_INVALID_CONFIG;
    }

    initialized_controller.is_feedback_initialized = false;
    initialized_controller.is_initialized = true;
    *self = initialized_controller;

    return SPEED_CONTROLLER_STATUS_OK;
}

speed_controller_status_t speed_controller_reset(speed_controller_t *self)
{
    if (self == NULL) {
        return SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return SPEED_CONTROLLER_STATUS_INVALID_STATE;
    }

    if (!self->pi.is_initialized) {
        return SPEED_CONTROLLER_STATUS_PI_ERROR;
    }
    if (!self->feedback_filter.is_initialized) {
        return SPEED_CONTROLLER_STATUS_FILTER_ERROR;
    }

    (void)pi_controller_reset(&self->pi);
    filter_low_pass_reset_fast(&self->feedback_filter, 0.0f);
    self->is_feedback_initialized = false;

    return SPEED_CONTROLLER_STATUS_OK;
}

speed_controller_status_t speed_controller_update(
    speed_controller_t *self,
    const speed_controller_input_t *input,
    speed_controller_output_t *output
)
{
    speed_controller_output_t next_output;
    float previous_filter_output;
    bool was_feedback_initialized;

    if ((self == NULL) || (input == NULL) || (output == NULL) ||
        (!speed_controller_float_is_finite(input->omega_m_ref_rad_s)) ||
        (!speed_controller_float_is_finite(
            input->omega_m_feedback_rad_s))) {
        return SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return SPEED_CONTROLLER_STATUS_INVALID_STATE;
    }

    if (!self->pi.is_initialized) {
        return SPEED_CONTROLLER_STATUS_PI_ERROR;
    }
    if (!self->feedback_filter.is_initialized) {
        return SPEED_CONTROLLER_STATUS_FILTER_ERROR;
    }

    previous_filter_output = self->feedback_filter.output;
    was_feedback_initialized = self->is_feedback_initialized;

    if (!self->is_feedback_initialized) {
        filter_low_pass_reset_fast(
            &self->feedback_filter,
            input->omega_m_feedback_rad_s
        );
        self->is_feedback_initialized = true;
    }

    if (filter_low_pass_update(
            &self->feedback_filter,
            input->omega_m_feedback_rad_s,
            &next_output.omega_m_feedback_filtered_rad_s) !=
        FILTER_STATUS_OK) {
        self->feedback_filter.output = previous_filter_output;
        self->is_feedback_initialized = was_feedback_initialized;
        return SPEED_CONTROLLER_STATUS_FILTER_ERROR;
    }

    next_output.omega_m_error_rad_s = input->omega_m_ref_rad_s -
        next_output.omega_m_feedback_filtered_rad_s;
    if (!speed_controller_float_is_finite(
            next_output.omega_m_error_rad_s)) {
        self->feedback_filter.output = previous_filter_output;
        self->is_feedback_initialized = was_feedback_initialized;
        return SPEED_CONTROLLER_STATUS_NUMERIC_ERROR;
    }

    if (pi_controller_update(
            &self->pi,
            next_output.omega_m_error_rad_s,
            &next_output.i_q_ref) != PI_CONTROLLER_STATUS_OK) {
        self->feedback_filter.output = previous_filter_output;
        self->is_feedback_initialized = was_feedback_initialized;
        return SPEED_CONTROLLER_STATUS_PI_ERROR;
    }

    next_output.is_i_q_ref_saturated =
        self->pi.unsaturated_output != next_output.i_q_ref;

    *output = next_output;

    return SPEED_CONTROLLER_STATUS_OK;
}

speed_controller_status_t speed_controller_apply_tracking(
    speed_controller_t *self,
    float applied_i_q_ref
)
{
    if ((self == NULL) ||
        (!speed_controller_float_is_finite(applied_i_q_ref))) {
        return SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return SPEED_CONTROLLER_STATUS_INVALID_STATE;
    }

    if ((applied_i_q_ref < self->pi.config.output_min) ||
        (applied_i_q_ref > self->pi.config.output_max)) {
        return SPEED_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (pi_controller_apply_tracking(&self->pi, applied_i_q_ref) !=
        PI_CONTROLLER_STATUS_OK) {
        return SPEED_CONTROLLER_STATUS_PI_ERROR;
    }

    return SPEED_CONTROLLER_STATUS_OK;
}
