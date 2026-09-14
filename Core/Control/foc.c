/**
 * @file foc.c
 * @brief Filtered d/q current PI와 voltage limitation을 포함한 FOC subsystem을 구현한다.
 */

#include "foc.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "transform.h"

#define FOC_ONE_OVER_SQRT_THREE  (0.57735026918962576451f)
#define FOC_SIN_COS_NORM_MIN     (0.999f)
#define FOC_SIN_COS_NORM_MAX     (1.001f)

static inline uint32_t foc_profile_begin(
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader
)
{
    if (profile == NULL) {
        return 0U;
    }

    profile->is_last_sample_complete = false;
    return cycle_counter_reader();
}

static inline uint32_t foc_profile_end_segment(
    foc_profile_segment_t *segment,
    uint32_t start_cycles,
    foc_cycle_counter_reader_t cycle_counter_reader
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

static bool foc_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool foc_dq_is_finite(const dq_t *value)
{
    return (value != NULL) &&
        foc_float_is_finite(value->d) &&
        foc_float_is_finite(value->q);
}

static bool foc_abc_is_finite(const abc_t *value)
{
    return (value != NULL) &&
        foc_float_is_finite(value->a) &&
        foc_float_is_finite(value->b) &&
        foc_float_is_finite(value->c);
}

typedef struct {
    float d_axis_pi_integral;
    float d_axis_pi_unsaturated_output;
    float d_axis_pi_output;
    float q_axis_pi_integral;
    float q_axis_pi_unsaturated_output;
    float q_axis_pi_output;
    float d_axis_filter_output;
    float q_axis_filter_output;
    bool is_feedback_initialized;
} foc_runtime_state_t;

static void foc_capture_runtime_state(
    const foc_t *self,
    foc_runtime_state_t *state
)
{
    state->d_axis_pi_integral = self->d_axis_pi.integral;
    state->d_axis_pi_unsaturated_output =
        self->d_axis_pi.unsaturated_output;
    state->d_axis_pi_output = self->d_axis_pi.output;
    state->q_axis_pi_integral = self->q_axis_pi.integral;
    state->q_axis_pi_unsaturated_output =
        self->q_axis_pi.unsaturated_output;
    state->q_axis_pi_output = self->q_axis_pi.output;
    state->d_axis_filter_output =
        self->d_axis_current_filter.output;
    state->q_axis_filter_output =
        self->q_axis_current_filter.output;
    state->is_feedback_initialized = self->is_feedback_initialized;
}

static void foc_restore_runtime_state(
    foc_t *self,
    const foc_runtime_state_t *state
)
{
    self->d_axis_pi.integral = state->d_axis_pi_integral;
    self->d_axis_pi.unsaturated_output =
        state->d_axis_pi_unsaturated_output;
    self->d_axis_pi.output = state->d_axis_pi_output;
    self->q_axis_pi.integral = state->q_axis_pi_integral;
    self->q_axis_pi.unsaturated_output =
        state->q_axis_pi_unsaturated_output;
    self->q_axis_pi.output = state->q_axis_pi_output;
    self->d_axis_current_filter.output =
        state->d_axis_filter_output;
    self->q_axis_current_filter.output =
        state->q_axis_filter_output;
    self->is_feedback_initialized = state->is_feedback_initialized;
}

static void foc_copy_output(
    foc_output_t *destination,
    const foc_output_t *source
)
{
    destination->i_dq_unfiltered.d = source->i_dq_unfiltered.d;
    destination->i_dq_unfiltered.q = source->i_dq_unfiltered.q;
    destination->i_dq_feedback.d = source->i_dq_feedback.d;
    destination->i_dq_feedback.q = source->i_dq_feedback.q;
    destination->i_dq_error.d = source->i_dq_error.d;
    destination->i_dq_error.q = source->i_dq_error.q;
    destination->v_dq_pi.d = source->v_dq_pi.d;
    destination->v_dq_pi.q = source->v_dq_pi.q;
    destination->v_dq_feedforward.d = source->v_dq_feedforward.d;
    destination->v_dq_feedforward.q = source->v_dq_feedforward.q;
    destination->v_dq_applied.d = source->v_dq_applied.d;
    destination->v_dq_applied.q = source->v_dq_applied.q;
    destination->v_alpha_beta_ref.alpha =
        source->v_alpha_beta_ref.alpha;
    destination->v_alpha_beta_ref.beta =
        source->v_alpha_beta_ref.beta;
    destination->is_voltage_saturated =
        source->is_voltage_saturated;
}

/**
 * @brief FOC가 묶어서 사용하는 하위 Algorithm 설정과 motor-model 조건을 검증한다.
 * @param config 검사할 FOC 설정.
 * @return 모든 설정 계약을 만족하면 true, 아니면 false.
 */
static bool foc_config_is_valid(const foc_config_t *config)
{
    if ((!foc_float_is_finite(config->voltage_utilization)) ||
        (!foc_float_is_finite(config->d_axis_inductance_h)) ||
        (!foc_float_is_finite(config->q_axis_inductance_h)) ||
        (!foc_float_is_finite(
            config->permanent_magnet_flux_linkage_wb)) ||
        (config->voltage_utilization <= 0.0f) ||
        (config->voltage_utilization > 1.0f) ||
        (config->d_axis_inductance_h < 0.0f) ||
        (config->q_axis_inductance_h < 0.0f) ||
        (config->permanent_magnet_flux_linkage_wb < 0.0f)) {
        return false;
    }

    if ((config->d_axis_pi.sampling_period_s !=
            config->q_axis_pi.sampling_period_s) ||
        (config->d_axis_pi.sampling_period_s !=
            config->current_filter.sampling_period_s) ||
        (config->d_axis_pi.output_min >= 0.0f) ||
        (config->d_axis_pi.output_max <= 0.0f) ||
        (config->q_axis_pi.output_min >= 0.0f) ||
        (config->q_axis_pi.output_max <= 0.0f)) {
        return false;
    }

    if (config->is_decoupling_enabled &&
        ((config->d_axis_inductance_h <= 0.0f) ||
         (config->q_axis_inductance_h <= 0.0f))) {
        return false;
    }

    return true;
}

/**
 * @brief Runtime 입력의 유한성과 sine/cosine 정규화 조건을 검증한다.
 * @param input 검사할 한 주기 입력.
 * @return FOC 계산에 사용할 수 있으면 true, 아니면 false.
 */
static bool foc_input_is_valid(const foc_input_t *input)
{
    float sin_cos_norm_squared;

    if ((input == NULL) ||
        (!foc_abc_is_finite(&input->i_abc)) ||
        (!foc_dq_is_finite(&input->i_dq_ref)) ||
        (!foc_float_is_finite(input->sin_theta)) ||
        (!foc_float_is_finite(input->cos_theta)) ||
        (!foc_float_is_finite(input->omega_e_rad_s)) ||
        (!foc_float_is_finite(input->v_dc)) ||
        (input->v_dc <= 0.0f)) {
        return false;
    }

    sin_cos_norm_squared =
        (input->sin_theta * input->sin_theta) +
        (input->cos_theta * input->cos_theta);

    return foc_float_is_finite(sin_cos_norm_squared) &&
        (sin_cos_norm_squared >= FOC_SIN_COS_NORM_MIN) &&
        (sin_cos_norm_squared <= FOC_SIN_COS_NORM_MAX);
}

/**
 * @brief 선택적으로 cross-coupling과 permanent-magnet back-EMF 보상 전압을 계산한다.
 * @param self 검증된 motor-model 설정을 가진 FOC instance.
 * @param i_dq_feedback 이번 주기의 filtered d/q 전류 [A].
 * @param omega_e_rad_s Signed electrical angular velocity [rad/s].
 * @param[out] v_dq_feedforward 계산한 d/q 보상 전압 [V].
 * @return 결과가 유한하면 true, 아니면 false.
 */
static bool foc_calculate_feedforward(
    const foc_t *self,
    const dq_t *i_dq_feedback,
    float omega_e_rad_s,
    dq_t *v_dq_feedforward
)
{
    float q_axis_flux_linkage_wb;

    if (!self->is_decoupling_enabled) {
        v_dq_feedforward->d = 0.0f;
        v_dq_feedforward->q = 0.0f;
        return true;
    }

    v_dq_feedforward->d =
        -omega_e_rad_s * self->q_axis_inductance_h * i_dq_feedback->q;

    q_axis_flux_linkage_wb =
        (self->d_axis_inductance_h * i_dq_feedback->d) +
        self->permanent_magnet_flux_linkage_wb;
    v_dq_feedforward->q = omega_e_rad_s * q_axis_flux_linkage_wb;

    return foc_dq_is_finite(v_dq_feedforward) &&
        foc_float_is_finite(q_axis_flux_linkage_wb);
}

static void foc_calculate_feedforward_fast(
    const foc_t *self,
    const dq_t *i_dq_feedback,
    float omega_e_rad_s,
    dq_t *v_dq_feedforward
)
{
    if (!self->is_decoupling_enabled) {
        v_dq_feedforward->d = 0.0f;
        v_dq_feedforward->q = 0.0f;
        return;
    }

    v_dq_feedforward->d =
        -omega_e_rad_s * self->q_axis_inductance_h * i_dq_feedback->q;
    v_dq_feedforward->q = omega_e_rad_s *
        ((self->d_axis_inductance_h * i_dq_feedback->d) +
         self->permanent_magnet_flux_linkage_wb);
}

/**
 * @brief 선형 SVPWM의 inscribed circle 안으로 d/q 전압 vector를 제한한다.
 * @param requested_voltage PI와 feedforward를 합한 d/q 전압 [V].
 * @param v_dc DC-link 전압 [V].
 * @param voltage_utilization `v_dc / sqrt(3)` 범위의 사용률.
 * @param[out] applied_voltage 실제 적용 가능한 d/q 전압 [V].
 * @param[out] is_saturated 원형 제한 개입 여부.
 * @return 계산 결과가 유한하고 유효하면 true, 아니면 false.
 */
static bool foc_limit_voltage(
    const dq_t *requested_voltage,
    float v_dc,
    float voltage_utilization,
    dq_t *applied_voltage,
    bool *is_saturated
)
{
    float voltage_limit;
    float voltage_limit_squared;
    float requested_magnitude_squared;

    voltage_limit =
        v_dc * voltage_utilization * FOC_ONE_OVER_SQRT_THREE;
    voltage_limit_squared = voltage_limit * voltage_limit;
    requested_magnitude_squared =
        (requested_voltage->d * requested_voltage->d) +
        (requested_voltage->q * requested_voltage->q);

    if ((!foc_float_is_finite(voltage_limit)) ||
        (!foc_float_is_finite(voltage_limit_squared)) ||
        (!foc_float_is_finite(requested_magnitude_squared)) ||
        (voltage_limit <= 0.0f)) {
        return false;
    }

    if (requested_magnitude_squared > voltage_limit_squared) {
        const float requested_magnitude =
            sqrtf(requested_magnitude_squared);
        const float scale = voltage_limit / requested_magnitude;

        if ((!foc_float_is_finite(requested_magnitude)) ||
            (!foc_float_is_finite(scale)) ||
            (requested_magnitude <= 0.0f)) {
            return false;
        }

        applied_voltage->d = requested_voltage->d * scale;
        applied_voltage->q = requested_voltage->q * scale;
        *is_saturated = true;
    } else {
        *applied_voltage = *requested_voltage;
        *is_saturated = false;
    }

    return foc_dq_is_finite(applied_voltage);
}

static void foc_limit_voltage_fast(
    const dq_t *requested_voltage,
    float v_dc,
    float voltage_utilization,
    dq_t *applied_voltage,
    bool *is_saturated
)
{
    const float voltage_limit =
        v_dc * voltage_utilization * FOC_ONE_OVER_SQRT_THREE;
    const float requested_magnitude_squared =
        (requested_voltage->d * requested_voltage->d) +
        (requested_voltage->q * requested_voltage->q);

    if (requested_magnitude_squared > (voltage_limit * voltage_limit)) {
        const float scale =
            voltage_limit / sqrtf(requested_magnitude_squared);

        applied_voltage->d = requested_voltage->d * scale;
        applied_voltage->q = requested_voltage->q * scale;
        *is_saturated = true;
    } else {
        *applied_voltage = *requested_voltage;
        *is_saturated = false;
    }
}

foc_status_t foc_init(foc_t *self, const foc_config_t *config)
{
    foc_t initialized_foc = {0};

    if ((self == NULL) || (config == NULL)) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if (!foc_config_is_valid(config)) {
        return FOC_STATUS_INVALID_CONFIG;
    }

    if ((pi_controller_init(
            &initialized_foc.d_axis_pi,
            &config->d_axis_pi) != PI_CONTROLLER_STATUS_OK) ||
        (pi_controller_init(
            &initialized_foc.q_axis_pi,
            &config->q_axis_pi) != PI_CONTROLLER_STATUS_OK) ||
        (filter_low_pass_init(
            &initialized_foc.d_axis_current_filter,
            &config->current_filter,
            0.0f) != FILTER_STATUS_OK) ||
        (filter_low_pass_init(
            &initialized_foc.q_axis_current_filter,
            &config->current_filter,
            0.0f) != FILTER_STATUS_OK)) {
        return FOC_STATUS_INVALID_CONFIG;
    }

    initialized_foc.voltage_utilization = config->voltage_utilization;
    initialized_foc.d_axis_inductance_h = config->d_axis_inductance_h;
    initialized_foc.q_axis_inductance_h = config->q_axis_inductance_h;
    initialized_foc.permanent_magnet_flux_linkage_wb =
        config->permanent_magnet_flux_linkage_wb;
    initialized_foc.is_decoupling_enabled =
        config->is_decoupling_enabled;
    initialized_foc.is_feedback_initialized = false;
    initialized_foc.is_initialized = true;

    *self = initialized_foc;

    return FOC_STATUS_OK;
}

foc_status_t foc_reset(foc_t *self)
{
    if (self == NULL) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FOC_STATUS_INVALID_STATE;
    }
    if ((!self->d_axis_pi.is_initialized) ||
        (!self->q_axis_pi.is_initialized)) {
        return FOC_STATUS_PI_ERROR;
    }
    if ((!self->d_axis_current_filter.is_initialized) ||
        (!self->q_axis_current_filter.is_initialized)) {
        return FOC_STATUS_FILTER_ERROR;
    }

    if ((pi_controller_reset(&self->d_axis_pi) !=
            PI_CONTROLLER_STATUS_OK) ||
        (pi_controller_reset(&self->q_axis_pi) !=
            PI_CONTROLLER_STATUS_OK)) {
        return FOC_STATUS_PI_ERROR;
    }

    if ((filter_low_pass_reset(
            &self->d_axis_current_filter,
            0.0f) != FILTER_STATUS_OK) ||
        (filter_low_pass_reset(
            &self->q_axis_current_filter,
            0.0f) != FILTER_STATUS_OK)) {
        return FOC_STATUS_FILTER_ERROR;
    }

    self->is_feedback_initialized = false;

    return FOC_STATUS_OK;
}

static foc_status_t foc_update_internal(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output,
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader,
    bool is_fast_path
)
{
    foc_runtime_state_t previous_state = {0};
    foc_output_t calculated_output;
    alpha_beta_t i_alpha_beta;
    dq_t requested_voltage;
    uint32_t profile_segment_start_cycles;

    profile_segment_start_cycles = foc_profile_begin(
        profile,
        cycle_counter_reader
    );

    if (!is_fast_path) {
        if ((self == NULL) || (output == NULL) ||
            (!foc_input_is_valid(input))) {
            return FOC_STATUS_INVALID_ARGUMENT;
        }

        if (!self->is_initialized) {
            return FOC_STATUS_INVALID_STATE;
        }
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->validation,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    transform_clarke(&input->i_abc, &i_alpha_beta);
    transform_park(
        &i_alpha_beta,
        input->sin_theta,
        input->cos_theta,
        &calculated_output.i_dq_unfiltered
    );

    if ((!is_fast_path) &&
        (!foc_dq_is_finite(&calculated_output.i_dq_unfiltered))) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->transform,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    if (!is_fast_path) {
        foc_capture_runtime_state(self, &previous_state);
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->rollback_snapshot,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    if (!self->is_feedback_initialized) {
        if (is_fast_path) {
            filter_low_pass_reset_fast(
                &self->d_axis_current_filter,
                calculated_output.i_dq_unfiltered.d
            );
            filter_low_pass_reset_fast(
                &self->q_axis_current_filter,
                calculated_output.i_dq_unfiltered.q
            );
        } else {
            if ((filter_low_pass_reset(
                    &self->d_axis_current_filter,
                    calculated_output.i_dq_unfiltered.d) !=
                        FILTER_STATUS_OK) ||
                (filter_low_pass_reset(
                    &self->q_axis_current_filter,
                    calculated_output.i_dq_unfiltered.q) !=
                        FILTER_STATUS_OK)) {
                foc_restore_runtime_state(self, &previous_state);
                return FOC_STATUS_FILTER_ERROR;
            }
        }

        calculated_output.i_dq_feedback =
            calculated_output.i_dq_unfiltered;
        self->is_feedback_initialized = true;
    } else {
        if (is_fast_path) {
            calculated_output.i_dq_feedback.d =
                filter_low_pass_update_fast(
                    &self->d_axis_current_filter,
                    calculated_output.i_dq_unfiltered.d
                );
            calculated_output.i_dq_feedback.q =
                filter_low_pass_update_fast(
                    &self->q_axis_current_filter,
                    calculated_output.i_dq_unfiltered.q
                );
        } else {
            if ((filter_low_pass_update(
                    &self->d_axis_current_filter,
                    calculated_output.i_dq_unfiltered.d,
                    &calculated_output.i_dq_feedback.d) !=
                        FILTER_STATUS_OK) ||
                (filter_low_pass_update(
                    &self->q_axis_current_filter,
                    calculated_output.i_dq_unfiltered.q,
                    &calculated_output.i_dq_feedback.q) !=
                        FILTER_STATUS_OK)) {
                foc_restore_runtime_state(self, &previous_state);
                return FOC_STATUS_FILTER_ERROR;
            }
        }
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->filter,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    calculated_output.i_dq_error.d =
        input->i_dq_ref.d - calculated_output.i_dq_feedback.d;
    calculated_output.i_dq_error.q =
        input->i_dq_ref.q - calculated_output.i_dq_feedback.q;

    if ((!is_fast_path) &&
        (!foc_dq_is_finite(&calculated_output.i_dq_error))) {
        foc_restore_runtime_state(self, &previous_state);
        return FOC_STATUS_NUMERIC_ERROR;
    }

    if (is_fast_path) {
        calculated_output.v_dq_pi.d = pi_controller_update_fast(
            &self->d_axis_pi,
            calculated_output.i_dq_error.d
        );
        calculated_output.v_dq_pi.q = pi_controller_update_fast(
            &self->q_axis_pi,
            calculated_output.i_dq_error.q
        );
    } else {
        if ((pi_controller_update(
                &self->d_axis_pi,
                calculated_output.i_dq_error.d,
                &calculated_output.v_dq_pi.d) !=
                    PI_CONTROLLER_STATUS_OK) ||
            (pi_controller_update(
                &self->q_axis_pi,
                calculated_output.i_dq_error.q,
                &calculated_output.v_dq_pi.q) !=
                    PI_CONTROLLER_STATUS_OK)) {
            foc_restore_runtime_state(self, &previous_state);
            return FOC_STATUS_PI_ERROR;
        }
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->pi,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    if (is_fast_path) {
        foc_calculate_feedforward_fast(
            self,
            &calculated_output.i_dq_feedback,
            input->omega_e_rad_s,
            &calculated_output.v_dq_feedforward
        );
    } else {
        if (!foc_calculate_feedforward(
                self,
                &calculated_output.i_dq_feedback,
                input->omega_e_rad_s,
                &calculated_output.v_dq_feedforward)) {
            foc_restore_runtime_state(self, &previous_state);
            return FOC_STATUS_NUMERIC_ERROR;
        }
    }

    requested_voltage.d =
        calculated_output.v_dq_pi.d +
        calculated_output.v_dq_feedforward.d;
    requested_voltage.q =
        calculated_output.v_dq_pi.q +
        calculated_output.v_dq_feedforward.q;

    if (is_fast_path) {
        foc_limit_voltage_fast(
            &requested_voltage,
            input->v_dc,
            self->voltage_utilization,
            &calculated_output.v_dq_applied,
            &calculated_output.is_voltage_saturated
        );
    } else {
        if ((!foc_dq_is_finite(&requested_voltage)) ||
            (!foc_limit_voltage(
                &requested_voltage,
                input->v_dc,
                self->voltage_utilization,
                &calculated_output.v_dq_applied,
                &calculated_output.is_voltage_saturated))) {
            foc_restore_runtime_state(self, &previous_state);
            return FOC_STATUS_NUMERIC_ERROR;
        }
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->voltage_limit,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    if (calculated_output.is_voltage_saturated) {
        const dq_t applied_pi_voltage = {
            .d = calculated_output.v_dq_applied.d -
                calculated_output.v_dq_feedforward.d,
            .q = calculated_output.v_dq_applied.q -
                calculated_output.v_dq_feedforward.q,
        };

        if (is_fast_path) {
            pi_controller_apply_tracking_fast(
                &self->d_axis_pi,
                applied_pi_voltage.d
            );
            pi_controller_apply_tracking_fast(
                &self->q_axis_pi,
                applied_pi_voltage.q
            );
        } else {
            if ((!foc_dq_is_finite(&applied_pi_voltage)) ||
                (pi_controller_apply_tracking(
                    &self->d_axis_pi,
                    applied_pi_voltage.d) != PI_CONTROLLER_STATUS_OK) ||
                (pi_controller_apply_tracking(
                    &self->q_axis_pi,
                    applied_pi_voltage.q) != PI_CONTROLLER_STATUS_OK)) {
                foc_restore_runtime_state(self, &previous_state);
                return FOC_STATUS_PI_ERROR;
            }
        }
    }

    if (profile != NULL) {
        profile_segment_start_cycles = foc_profile_end_segment(
            &profile->tracking,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
    }

    transform_inverse_park(
        &calculated_output.v_dq_applied,
        input->sin_theta,
        input->cos_theta,
        &calculated_output.v_alpha_beta_ref
    );

    if ((!foc_float_is_finite(
            calculated_output.v_alpha_beta_ref.alpha)) ||
        (!foc_float_is_finite(
            calculated_output.v_alpha_beta_ref.beta))) {
        if (!is_fast_path) {
            foc_restore_runtime_state(self, &previous_state);
        }
        return FOC_STATUS_NUMERIC_ERROR;
    }

    foc_copy_output(output, &calculated_output);

    if (profile != NULL) {
        (void)foc_profile_end_segment(
            &profile->output,
            profile_segment_start_cycles,
            cycle_counter_reader
        );
        ++profile->complete_sample_count;
        profile->is_last_sample_complete = true;
    }

    return FOC_STATUS_OK;
}

foc_status_t foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output
)
{
    return foc_update_internal(self, input, output, NULL, NULL, false);
}

foc_status_t foc_update_profiled(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output,
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader
)
{
    if ((profile == NULL) || (cycle_counter_reader == NULL)) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    return foc_update_internal(
        self,
        input,
        output,
        profile,
        cycle_counter_reader,
        false
    );
}

foc_status_t foc_update_fast(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output
)
{
    return foc_update_internal(self, input, output, NULL, NULL, true);
}

foc_status_t foc_update_fast_profiled(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output,
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader
)
{
    if ((profile == NULL) || (cycle_counter_reader == NULL)) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    return foc_update_internal(
        self,
        input,
        output,
        profile,
        cycle_counter_reader,
        true
    );
}
