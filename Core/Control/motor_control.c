/**
 * @file motor_control.c
 * @brief d/q 전류 지령 제한과 FOC current-control 실행을 구현한다.
 * @ingroup control_motor_control
 */

#include "motor_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "limiter.h"

static inline uint32_t motor_control_profile_begin(
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    if (profile == NULL) {
        return 0U;
    }

    profile->is_last_sample_complete = false;
    return cycle_counter_reader();
}

static inline uint32_t motor_control_profile_end_segment(
    motor_control_profile_segment_t *segment,
    uint32_t start_cycles,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    const uint32_t end_cycles = cycle_counter_reader();
    const uint32_t elapsed_cycles = end_cycles - start_cycles;

    segment->last_cycles = elapsed_cycles;
    if (elapsed_cycles > segment->max_cycles) {
        segment->max_cycles = elapsed_cycles;
    }

    return end_cycles;
}

static bool motor_control_is_finite_dq(const dq_t *value)
{
    return (value != NULL) && isfinite(value->d) && isfinite(value->q);
}

static bool motor_control_is_finite(float value)
{
    return isfinite(value);
}

static float motor_control_minimum(float first, float second)
{
    return (first < second) ? first : second;
}

static float motor_control_maximum(float first, float second)
{
    return (first > second) ? first : second;
}

static bool motor_control_is_valid_config(
    const motor_control_config_t *config
)
{
    if ((config == NULL) ||
        !motor_control_is_finite_dq(&config->current_reference_min) ||
        !motor_control_is_finite_dq(&config->current_reference_max) ||
        !motor_control_is_finite_dq(
            &config->current_reference_rise_rate_per_s
        ) ||
        !motor_control_is_finite_dq(
            &config->current_reference_fall_rate_per_s
        ) ||
        !isfinite(config->current_reference_magnitude_limit) ||
        !isfinite(config->sampling_period_s) ||
        !motor_control_is_finite(config->speed_reference_min_rad_s) ||
        !motor_control_is_finite(config->speed_reference_max_rad_s) ||
        !motor_control_is_finite(config->speed_reference_rise_rate_rad_s2) ||
        !motor_control_is_finite(config->speed_reference_fall_rate_rad_s2)) {
        return false;
    }

    if ((config->current_reference_min.d > 0.0f) ||
        (config->current_reference_min.q > 0.0f) ||
        (config->current_reference_max.d < 0.0f) ||
        (config->current_reference_max.q < 0.0f) ||
        (config->current_reference_min.d >
            config->current_reference_max.d) ||
        (config->current_reference_min.q >
            config->current_reference_max.q) ||
        (config->current_reference_rise_rate_per_s.d < 0.0f) ||
        (config->current_reference_rise_rate_per_s.q < 0.0f) ||
        (config->current_reference_fall_rate_per_s.d < 0.0f) ||
        (config->current_reference_fall_rate_per_s.q < 0.0f) ||
        (config->current_reference_magnitude_limit <= 0.0f) ||
        (config->sampling_period_s <= 0.0f) ||
        (config->sampling_period_s !=
            config->foc.d_axis_pi.sampling_period_s) ||
        (config->sampling_period_s !=
            config->foc.q_axis_pi.sampling_period_s) ||
        (config->sampling_period_s !=
            config->foc.current_filter.sampling_period_s)) {
        return false;
    }

    if ((config->pole_pairs == 0U) ||
        (config->speed_reference_min_rad_s > 0.0f) ||
        (config->speed_reference_max_rad_s < 0.0f) ||
        (config->speed_reference_min_rad_s >
            config->speed_reference_max_rad_s) ||
        (config->speed_reference_rise_rate_rad_s2 < 0.0f) ||
        (config->speed_reference_fall_rate_rad_s2 < 0.0f) ||
        (config->speed_controller.pi.sampling_period_s !=
            config->speed_controller.feedback_filter.sampling_period_s)) {
        return false;
    }

    return true;
}

/**
 * @brief Axis 범위와 d/q vector magnitude 제한을 순서대로 적용한다.
 */
static bool motor_control_limit_current_reference(
    const motor_control_config_t *config,
    const dq_t *input,
    dq_t *output
)
{
    const float magnitude_limit = config->current_reference_magnitude_limit;
    const float minimum_d = motor_control_maximum(
        config->current_reference_min.d,
        -magnitude_limit
    );
    const float maximum_d = motor_control_minimum(
        config->current_reference_max.d,
        magnitude_limit
    );
    const float minimum_q = motor_control_maximum(
        config->current_reference_min.q,
        -magnitude_limit
    );
    const float maximum_q = motor_control_minimum(
        config->current_reference_max.q,
        magnitude_limit
    );
    float magnitude_squared;

    output->d = limiter_clamp(input->d, minimum_d, maximum_d);
    output->q = limiter_clamp(input->q, minimum_q, maximum_q);
    magnitude_squared = (output->d * output->d) +
        (output->q * output->q);

    if (magnitude_squared > (magnitude_limit * magnitude_limit)) {
        float scale = magnitude_limit / sqrtf(magnitude_squared);

        /* 반올림으로 원 경계를 넘지 않도록 아주 작은 여유를 둔다. */
        scale *= 1.0f - (4.0f * FLT_EPSILON);
        output->d *= scale;
        output->q *= scale;
    }

    return (output->d != input->d) || (output->q != input->q);
}

/**
 * @brief Rate-limited 이동 선분을 따라 current magnitude 제한을 적용한다.
 */
static bool motor_control_limit_current_reference_step(
    const motor_control_config_t *config,
    const dq_t *current,
    const dq_t *candidate,
    dq_t *output
)
{
    const float magnitude_limit = config->current_reference_magnitude_limit;
    const float candidate_magnitude_squared =
        (candidate->d * candidate->d) + (candidate->q * candidate->q);

    if (candidate_magnitude_squared <= (magnitude_limit * magnitude_limit)) {
        output->d = candidate->d;
        output->q = candidate->q;
        return false;
    }

    const float candidate_d = candidate->d / magnitude_limit;
    const float candidate_q = candidate->q / magnitude_limit;
    const float current_d = current->d / magnitude_limit;
    const float current_q = current->q / magnitude_limit;
    const float delta_d = candidate_d - current_d;
    const float delta_q = candidate_q - current_q;
    const float quadratic_a = (delta_d * delta_d) + (delta_q * delta_q);

    if (quadratic_a <= FLT_MIN) {
        output->d = current->d;
        output->q = current->q;
        return true;
    }

    const float quadratic_b =
        2.0f * ((current_d * delta_d) + (current_q * delta_q));
    const float quadratic_c =
        (current_d * current_d) + (current_q * current_q) - 1.0f;
    const float raw_discriminant =
        (quadratic_b * quadratic_b) -
        (4.0f * quadratic_a * quadratic_c);
    const float discriminant = motor_control_maximum(
        0.0f,
        raw_discriminant
    );
    float step_scale =
        (-quadratic_b + sqrtf(discriminant)) / (2.0f * quadratic_a);

    step_scale = limiter_clamp(step_scale, 0.0f, 1.0f);
    step_scale *= 1.0f - (4.0f * FLT_EPSILON);
    output->d = current->d +
        (step_scale * (candidate->d - current->d));
    output->q = current->q +
        (step_scale * (candidate->q - current->q));

    return true;
}

/**
 * @brief Hot path에서 libc 구조체 복사를 만들지 않고 결과를 전달한다.
 */
static void motor_control_copy_output(
    motor_control_output_t *destination,
    const motor_control_output_t *source
)
{
    destination->i_dq_ref.d = source->i_dq_ref.d;
    destination->i_dq_ref.q = source->i_dq_ref.q;
    destination->foc.i_dq_unfiltered.d = source->foc.i_dq_unfiltered.d;
    destination->foc.i_dq_unfiltered.q = source->foc.i_dq_unfiltered.q;
    destination->foc.i_dq_feedback.d = source->foc.i_dq_feedback.d;
    destination->foc.i_dq_feedback.q = source->foc.i_dq_feedback.q;
    destination->foc.i_dq_error.d = source->foc.i_dq_error.d;
    destination->foc.i_dq_error.q = source->foc.i_dq_error.q;
    destination->foc.v_dq_pi.d = source->foc.v_dq_pi.d;
    destination->foc.v_dq_pi.q = source->foc.v_dq_pi.q;
    destination->foc.v_dq_feedforward.d = source->foc.v_dq_feedforward.d;
    destination->foc.v_dq_feedforward.q = source->foc.v_dq_feedforward.q;
    destination->foc.v_dq_applied.d = source->foc.v_dq_applied.d;
    destination->foc.v_dq_applied.q = source->foc.v_dq_applied.q;
    destination->foc.v_alpha_beta_ref.alpha =
        source->foc.v_alpha_beta_ref.alpha;
    destination->foc.v_alpha_beta_ref.beta =
        source->foc.v_alpha_beta_ref.beta;
    destination->foc.is_voltage_saturated =
        source->foc.is_voltage_saturated;
    destination->is_current_reference_rate_limited =
        source->is_current_reference_rate_limited;
    destination->is_current_reference_saturated =
        source->is_current_reference_saturated;
}

motor_control_status_t motor_control_init(
    motor_control_t *self,
    const motor_control_config_t *config
)
{
    motor_control_t initialized;
    rate_limiter_config_t rate_limiter_config;

    if ((self == NULL) || (config == NULL)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!motor_control_is_valid_config(config)) {
        return MOTOR_CONTROL_STATUS_INVALID_CONFIG;
    }

    initialized = (motor_control_t){
        .config = *config,
        .i_dq_ref = {0.0f, 0.0f},
        .is_current_reference_rate_limited = false,
        .is_current_reference_saturated = false,
        .is_initialized = false,
    };

    rate_limiter_config = (rate_limiter_config_t){
        .rise_rate_per_s = config->current_reference_rise_rate_per_s.d,
        .fall_rate_per_s = config->current_reference_fall_rate_per_s.d,
        .sampling_period_s = config->sampling_period_s,
    };
    if (rate_limiter_init(
            &initialized.i_d_rate_limiter,
            &rate_limiter_config,
            0.0f) != RATE_LIMITER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    rate_limiter_config = (rate_limiter_config_t){
        .rise_rate_per_s = config->current_reference_rise_rate_per_s.q,
        .fall_rate_per_s = config->current_reference_fall_rate_per_s.q,
        .sampling_period_s = config->sampling_period_s,
    };
    if (rate_limiter_init(
            &initialized.i_q_rate_limiter,
            &rate_limiter_config,
            0.0f) != RATE_LIMITER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    if (foc_init(&initialized.foc, &config->foc) != FOC_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_INVALID_CONFIG;
    }

    rate_limiter_config = (rate_limiter_config_t){
        .rise_rate_per_s = config->speed_reference_rise_rate_rad_s2,
        .fall_rate_per_s = config->speed_reference_fall_rate_rad_s2,
        .sampling_period_s = config->speed_controller.pi.sampling_period_s,
    };
    if (rate_limiter_init(
            &initialized.speed_reference_rate_limiter,
            &rate_limiter_config,
            0.0f) != RATE_LIMITER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }
    if (speed_controller_init(
            &initialized.speed_controller,
            &config->speed_controller) != SPEED_CONTROLLER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }

    initialized.is_initialized = true;
    *self = initialized;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_reset_current_reference(
    motor_control_t *self,
    const dq_t *initial_i_dq_ref
)
{
    rate_limiter_t i_d_rate_limiter;
    rate_limiter_t i_q_rate_limiter;
    dq_t limited_reference;
    bool is_saturated;

    if ((self == NULL) || !motor_control_is_finite_dq(initial_i_dq_ref)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    is_saturated = motor_control_limit_current_reference(
        &self->config,
        initial_i_dq_ref,
        &limited_reference
    );
    i_d_rate_limiter = self->i_d_rate_limiter;
    i_q_rate_limiter = self->i_q_rate_limiter;

    if ((rate_limiter_reset(
            &i_d_rate_limiter,
            limited_reference.d) != RATE_LIMITER_STATUS_OK) ||
        (rate_limiter_reset(
            &i_q_rate_limiter,
            limited_reference.q) != RATE_LIMITER_STATUS_OK)) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    self->i_d_rate_limiter = i_d_rate_limiter;
    self->i_q_rate_limiter = i_q_rate_limiter;
    self->i_dq_ref = limited_reference;
    self->is_current_reference_rate_limited = false;
    self->is_current_reference_saturated = is_saturated;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_prepare_current_reference_target(
    const motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    motor_control_current_reference_target_t *target
)
{
    motor_control_current_reference_target_t prepared_target;

    if ((self == NULL) || (target == NULL) ||
        !motor_control_is_finite_dq(requested_i_dq_ref)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    prepared_target.was_saturated = motor_control_limit_current_reference(
        &self->config,
        requested_i_dq_ref,
        &prepared_target.i_dq_ref
    );
    *target = prepared_target;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_reset(motor_control_t *self)
{
    if (self == NULL) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }
    if ((!self->i_d_rate_limiter.is_initialized) ||
        (!self->i_q_rate_limiter.is_initialized)) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }
    if (!self->foc.is_initialized) {
        return MOTOR_CONTROL_STATUS_FOC_ERROR;
    }
    if ((!self->speed_reference_rate_limiter.is_initialized) ||
        (!self->speed_controller.is_initialized)) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }

    if (foc_reset(&self->foc) != FOC_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_FOC_ERROR;
    }

    self->i_d_rate_limiter.output = 0.0f;
    self->i_q_rate_limiter.output = 0.0f;
    self->speed_reference_rate_limiter.output = 0.0f;
    if (speed_controller_reset(&self->speed_controller) !=
        SPEED_CONTROLLER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }
    self->i_dq_ref.d = 0.0f;
    self->i_dq_ref.q = 0.0f;
    self->is_current_reference_rate_limited = false;
    self->is_current_reference_saturated = false;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_update_speed(
    motor_control_t *self,
    const motor_control_speed_input_t *input,
    motor_control_speed_output_t *output
)
{
    motor_control_speed_output_t calculated_output;
    speed_controller_input_t speed_input;
    dq_t requested_i_dq_ref;

    if ((self == NULL) || (input == NULL) || (output == NULL) ||
        !motor_control_is_finite(input->omega_m_requested_rad_s) ||
        !motor_control_is_finite(input->omega_e_feedback_rad_s)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    speed_input.omega_m_ref_rad_s = rate_limiter_update_fast(
        &self->speed_reference_rate_limiter,
        limiter_clamp(
            input->omega_m_requested_rad_s,
            self->config.speed_reference_min_rad_s,
            self->config.speed_reference_max_rad_s
        )
    );
    speed_input.omega_m_feedback_rad_s = input->omega_e_feedback_rad_s /
        (float)self->config.pole_pairs;
    if (speed_controller_update(
            &self->speed_controller,
            &speed_input,
            &calculated_output.speed_controller) !=
        SPEED_CONTROLLER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }

    requested_i_dq_ref = (dq_t){
        .d = 0.0f,
        .q = calculated_output.speed_controller.i_q_ref,
    };
    if (motor_control_prepare_current_reference_target(
            self,
            &requested_i_dq_ref,
            &calculated_output.current_reference_target) !=
        MOTOR_CONTROL_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }
    if (speed_controller_apply_tracking(
            &self->speed_controller,
            calculated_output.current_reference_target.i_dq_ref.q) !=
        SPEED_CONTROLLER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR;
    }

    calculated_output.omega_m_ref_limited_rad_s =
        speed_input.omega_m_ref_rad_s;
    calculated_output.omega_m_feedback_rad_s =
        speed_input.omega_m_feedback_rad_s;
    *output = calculated_output;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_update_current_reference(
    motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    dq_t *applied_i_dq_ref
)
{
    dq_t limited_target;
    dq_t rate_limited_reference;
    dq_t final_reference;
    float previous_i_d_output;
    float previous_i_q_output;
    bool is_target_saturated;
    bool is_rate_limited;
    bool is_final_saturated;

    if ((self == NULL) || (applied_i_dq_ref == NULL) ||
        !motor_control_is_finite_dq(requested_i_dq_ref)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    is_target_saturated = motor_control_limit_current_reference(
        &self->config,
        requested_i_dq_ref,
        &limited_target
    );

    previous_i_d_output = self->i_d_rate_limiter.output;
    previous_i_q_output = self->i_q_rate_limiter.output;
    if (rate_limiter_update(
            &self->i_d_rate_limiter,
            limited_target.d,
            &rate_limited_reference.d) != RATE_LIMITER_STATUS_OK) {
        self->i_d_rate_limiter.output = previous_i_d_output;
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }
    if (rate_limiter_update(
            &self->i_q_rate_limiter,
            limited_target.q,
            &rate_limited_reference.q) != RATE_LIMITER_STATUS_OK) {
        self->i_d_rate_limiter.output = previous_i_d_output;
        self->i_q_rate_limiter.output = previous_i_q_output;
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    is_rate_limited =
        (rate_limited_reference.d != limited_target.d) ||
        (rate_limited_reference.q != limited_target.q);
    is_final_saturated = motor_control_limit_current_reference_step(
        &self->config,
        &self->i_dq_ref,
        &rate_limited_reference,
        &final_reference
    );

    if (is_final_saturated &&
        ((rate_limiter_reset(
            &self->i_d_rate_limiter,
            final_reference.d) != RATE_LIMITER_STATUS_OK) ||
         (rate_limiter_reset(
            &self->i_q_rate_limiter,
            final_reference.q) != RATE_LIMITER_STATUS_OK))) {
        self->i_d_rate_limiter.output = previous_i_d_output;
        self->i_q_rate_limiter.output = previous_i_q_output;
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    self->i_dq_ref.d = final_reference.d;
    self->i_dq_ref.q = final_reference.q;
    self->is_current_reference_rate_limited = is_rate_limited;
    self->is_current_reference_saturated =
        is_target_saturated || is_final_saturated;
    applied_i_dq_ref->d = final_reference.d;
    applied_i_dq_ref->q = final_reference.q;
    return MOTOR_CONTROL_STATUS_OK;
}

static void motor_control_update_current_reference_fast(
    motor_control_t *self,
    const motor_control_current_reference_target_t *target,
    dq_t *applied_i_dq_ref
)
{
    dq_t rate_limited_reference;
    dq_t final_reference;
    bool is_rate_limited;
    bool is_final_saturated;

    rate_limited_reference.d = rate_limiter_update_fast(
        &self->i_d_rate_limiter,
        target->i_dq_ref.d
    );
    rate_limited_reference.q = rate_limiter_update_fast(
        &self->i_q_rate_limiter,
        target->i_dq_ref.q
    );

    is_rate_limited =
        (rate_limited_reference.d != target->i_dq_ref.d) ||
        (rate_limited_reference.q != target->i_dq_ref.q);
    is_final_saturated = motor_control_limit_current_reference_step(
        &self->config,
        &self->i_dq_ref,
        &rate_limited_reference,
        &final_reference
    );

    if (is_final_saturated) {
        rate_limiter_reset_fast(
            &self->i_d_rate_limiter,
            final_reference.d
        );
        rate_limiter_reset_fast(
            &self->i_q_rate_limiter,
            final_reference.q
        );
    }

    self->i_dq_ref = final_reference;
    self->is_current_reference_rate_limited = is_rate_limited;
    self->is_current_reference_saturated =
        target->was_saturated || is_final_saturated;
    *applied_i_dq_ref = final_reference;
}

static motor_control_status_t motor_control_update_fast_internal(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    const foc_input_t foc_input = {
        .i_abc = input->i_abc,
        .i_dq_ref = {0.0f, 0.0f},
        .sin_theta = input->sin_theta,
        .cos_theta = input->cos_theta,
        .omega_e_rad_s = input->omega_e_rad_s,
        .v_dc = input->v_dc,
    };
    foc_input_t prepared_foc_input = foc_input;
    uint32_t profile_segment_start_cycles = motor_control_profile_begin(
        profile,
        cycle_counter_reader
    );

    motor_control_update_current_reference_fast(
        self,
        &input->current_reference_target,
        &output->i_dq_ref
    );

    if (profile != NULL) {
        profile_segment_start_cycles = motor_control_profile_end_segment(
            &profile->reference,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    prepared_foc_input.i_dq_ref = output->i_dq_ref;
    const foc_status_t foc_status = (profile != NULL) ?
        foc_update_fast_profiled(
            &self->foc,
            &prepared_foc_input,
            &output->foc,
            &profile->foc_detail,
            cycle_counter_reader) :
        foc_update_fast(
            &self->foc,
            &prepared_foc_input,
            &output->foc);
    if (foc_status != FOC_STATUS_OK) {
        if (profile != NULL) {
            (void)motor_control_profile_end_segment(
                &profile->foc,
                profile_segment_start_cycles,
                cycle_counter_reader
            );
        }
        return MOTOR_CONTROL_STATUS_FOC_ERROR;
    }

    if (profile != NULL) {
        profile_segment_start_cycles = motor_control_profile_end_segment(
            &profile->foc,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    output->is_current_reference_rate_limited =
        self->is_current_reference_rate_limited;
    output->is_current_reference_saturated =
        self->is_current_reference_saturated;

    if (profile != NULL) {
        (void)motor_control_profile_end_segment(
            &profile->output,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
        ++profile->complete_sample_count;
        profile->is_last_sample_complete = true;
    }
    return MOTOR_CONTROL_STATUS_OK;
}

static motor_control_status_t motor_control_update_internal(
    motor_control_t *self,
    const motor_control_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    motor_control_output_t calculated_output;
    foc_input_t foc_input;
    dq_t previous_i_dq_ref;
    float previous_i_d_output;
    float previous_i_q_output;
    bool previous_is_rate_limited;
    bool previous_is_saturated;
    uint32_t profile_segment_start_cycles;

    if ((self == NULL) || (input == NULL) || (output == NULL)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    profile_segment_start_cycles = motor_control_profile_begin(
        profile,
        cycle_counter_reader
    );

    previous_i_d_output = self->i_d_rate_limiter.output;
    previous_i_q_output = self->i_q_rate_limiter.output;
    previous_i_dq_ref = self->i_dq_ref;
    previous_is_rate_limited = self->is_current_reference_rate_limited;
    previous_is_saturated = self->is_current_reference_saturated;

    if (motor_control_update_current_reference(
            self,
            &input->requested_i_dq_ref,
            &calculated_output.i_dq_ref) != MOTOR_CONTROL_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    if (profile != NULL) {
        profile_segment_start_cycles = motor_control_profile_end_segment(
            &profile->reference,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    foc_input = (foc_input_t){
        .i_abc = input->i_abc,
        .i_dq_ref = calculated_output.i_dq_ref,
        .sin_theta = input->sin_theta,
        .cos_theta = input->cos_theta,
        .omega_e_rad_s = input->omega_e_rad_s,
        .v_dc = input->v_dc,
    };
    const foc_status_t foc_status = (profile != NULL) ?
        foc_update_profiled(
            &self->foc,
            &foc_input,
            &calculated_output.foc,
            &profile->foc_detail,
            cycle_counter_reader) :
        foc_update(
            &self->foc,
            &foc_input,
            &calculated_output.foc);
    if (foc_status != FOC_STATUS_OK) {
        if (profile != NULL) {
            (void)motor_control_profile_end_segment(
                &profile->foc,
                profile_segment_start_cycles,
                cycle_counter_reader
            );
        }
        self->i_d_rate_limiter.output = previous_i_d_output;
        self->i_q_rate_limiter.output = previous_i_q_output;
        self->i_dq_ref = previous_i_dq_ref;
        self->is_current_reference_rate_limited =
            previous_is_rate_limited;
        self->is_current_reference_saturated = previous_is_saturated;
        return MOTOR_CONTROL_STATUS_FOC_ERROR;
    }

    if (profile != NULL) {
        profile_segment_start_cycles = motor_control_profile_end_segment(
            &profile->foc,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    calculated_output.is_current_reference_rate_limited =
        self->is_current_reference_rate_limited;
    calculated_output.is_current_reference_saturated =
        self->is_current_reference_saturated;

    motor_control_copy_output(output, &calculated_output);
    if (profile != NULL) {
        (void)motor_control_profile_end_segment(
            &profile->output,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
        ++profile->complete_sample_count;
        profile->is_last_sample_complete = true;
    }
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_update(
    motor_control_t *self,
    const motor_control_input_t *input,
    motor_control_output_t *output
)
{
    return motor_control_update_internal(self, input, output, NULL, NULL);
}

motor_control_status_t motor_control_update_profiled(
    motor_control_t *self,
    const motor_control_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    if ((profile == NULL) || (cycle_counter_reader == NULL)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }

    return motor_control_update_internal(
        self,
        input,
        output,
        profile,
        cycle_counter_reader
    );
}

motor_control_status_t motor_control_update_fast(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    motor_control_output_t *output
)
{
    return motor_control_update_fast_internal(
        self,
        input,
        output,
        NULL,
        NULL
    );
}

motor_control_status_t motor_control_update_fast_voltage(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    alpha_beta_t *v_alpha_beta_ref
)
{
    dq_t i_dq_ref;
    const foc_input_t foc_input_base = {
        .i_abc = input->i_abc,
        .i_dq_ref = {0.0f, 0.0f},
        .sin_theta = input->sin_theta,
        .cos_theta = input->cos_theta,
        .omega_e_rad_s = input->omega_e_rad_s,
        .v_dc = input->v_dc,
    };
    foc_input_t foc_input = foc_input_base;

    motor_control_update_current_reference_fast(
        self, &input->current_reference_target, &i_dq_ref);
    foc_input.i_dq_ref = i_dq_ref;

    return (foc_update_fast_voltage(&self->foc, &foc_input,
                                    v_alpha_beta_ref) == FOC_STATUS_OK) ?
        MOTOR_CONTROL_STATUS_OK : MOTOR_CONTROL_STATUS_FOC_ERROR;
}

motor_control_status_t motor_control_update_fast_profiled(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
)
{
    if ((profile == NULL) || (cycle_counter_reader == NULL)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }

    return motor_control_update_fast_internal(
        self,
        input,
        output,
        profile,
        cycle_counter_reader
    );
}
