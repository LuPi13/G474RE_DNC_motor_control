/**
 * @file app.c
 * @brief Open-loop 및 FOC current mode의 App fast loop를 구현한다.
 */

#include "app.h"

#include <float.h>
#include <stddef.h>

#define APP_ZERO_DUTY                 (0.5f)
#define APP_TWO_PI_RAD                (6.28318530717958647692f)

#if APP_FAST_LOOP_DETAILED_PROFILING_ENABLED
#define APP_PROFILE_END_SEGMENT(self_, member_, start_cycles_) \
    (((self_)->config.fast_loop_profile == NULL) ? \
        (start_cycles_) : \
        app_profile_end_segment( \
            (self_), \
            &(self_)->config.fast_loop_profile->member_, \
            (start_cycles_)))

static inline uint32_t app_profile_begin(app_t *self)
{
    if (self->config.fast_loop_profile == NULL) {
        return 0U;
    }

    self->config.fast_loop_profile->is_last_sample_complete = false;
    return self->config.cycle_counter_reader();
}

static inline uint32_t app_profile_end_segment(
    app_t *self,
    app_fast_loop_profile_segment_t *segment,
    uint32_t start_cycles
)
{
    uint32_t end_cycles;
    uint32_t elapsed_cycles;

    if (self->config.fast_loop_profile == NULL) {
        return start_cycles;
    }

    end_cycles = self->config.cycle_counter_reader();
    elapsed_cycles = end_cycles - start_cycles;
    segment->last_cycles = elapsed_cycles;
    if (elapsed_cycles > segment->max_cycles) {
        segment->max_cycles = elapsed_cycles;
    }

    return end_cycles;
}

#else
#define APP_PROFILE_END_SEGMENT(self_, member_, start_cycles_) \
    (start_cycles_)

static inline uint32_t app_profile_begin(app_t *self)
{
    (void)self;
    return 0U;
}

#endif

static bool app_float_is_finite(float value)
{
    return (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float app_wrap_angle_rad(float theta_rad)
{
    if (theta_rad >= APP_TWO_PI_RAD) {
        theta_rad -= APP_TWO_PI_RAD;
    } else if (theta_rad < 0.0f) {
        theta_rad += APP_TWO_PI_RAD;
    }

    return theta_rad;
}

static abc_t app_neutral_duty(void)
{
    const abc_t duty = {
        .a = APP_ZERO_DUTY,
        .b = APP_ZERO_DUTY,
        .c = APP_ZERO_DUTY,
    };

    return duty;
}

static void app_copy_motor_control_output(
    motor_control_output_t *destination,
    const motor_control_output_t *source
)
{
    destination->i_dq_ref.d = source->i_dq_ref.d;
    destination->i_dq_ref.q = source->i_dq_ref.q;
    destination->foc.i_dq_unfiltered.d =
        source->foc.i_dq_unfiltered.d;
    destination->foc.i_dq_unfiltered.q =
        source->foc.i_dq_unfiltered.q;
    destination->foc.i_dq_feedback.d = source->foc.i_dq_feedback.d;
    destination->foc.i_dq_feedback.q = source->foc.i_dq_feedback.q;
    destination->foc.i_dq_error.d = source->foc.i_dq_error.d;
    destination->foc.i_dq_error.q = source->foc.i_dq_error.q;
    destination->foc.v_dq_pi.d = source->foc.v_dq_pi.d;
    destination->foc.v_dq_pi.q = source->foc.v_dq_pi.q;
    destination->foc.v_dq_feedforward.d =
        source->foc.v_dq_feedforward.d;
    destination->foc.v_dq_feedforward.q =
        source->foc.v_dq_feedforward.q;
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

static void app_clear_motor_control_output(motor_control_output_t *output)
{
    output->i_dq_ref.d = 0.0f;
    output->i_dq_ref.q = 0.0f;
    output->foc.i_dq_unfiltered.d = 0.0f;
    output->foc.i_dq_unfiltered.q = 0.0f;
    output->foc.i_dq_feedback.d = 0.0f;
    output->foc.i_dq_feedback.q = 0.0f;
    output->foc.i_dq_error.d = 0.0f;
    output->foc.i_dq_error.q = 0.0f;
    output->foc.v_dq_pi.d = 0.0f;
    output->foc.v_dq_pi.q = 0.0f;
    output->foc.v_dq_feedforward.d = 0.0f;
    output->foc.v_dq_feedforward.q = 0.0f;
    output->foc.v_dq_applied.d = 0.0f;
    output->foc.v_dq_applied.q = 0.0f;
    output->foc.v_alpha_beta_ref.alpha = 0.0f;
    output->foc.v_alpha_beta_ref.beta = 0.0f;
    output->foc.is_voltage_saturated = false;
    output->is_current_reference_rate_limited = false;
    output->is_current_reference_saturated = false;
}

static void app_clear_rotor_feedback(hall_estimator_output_t *output)
{
    output->theta_e_rad = 0.0f;
    output->omega_e_rad_s = 0.0f;
    output->has_valid_angle = false;
    output->has_valid_speed = false;
    output->has_edge_reference = false;
    output->is_sector_limited = false;
    output->is_timed_out = false;
}

static void app_copy_fast_loop_output(
    app_fast_loop_output_t *destination,
    const app_fast_loop_output_t *source
)
{
    destination->raw.phase_a = source->raw.phase_a;
    destination->raw.phase_b = source->raw.phase_b;
    destination->raw.phase_c = source->raw.phase_c;
    destination->raw.dc_link = source->raw.dc_link;
    destination->i_abc.a = source->i_abc.a;
    destination->i_abc.b = source->i_abc.b;
    destination->i_abc.c = source->i_abc.c;
    destination->v_dc = source->v_dc;
    destination->voltage_angle_rad = source->voltage_angle_rad;
    destination->v_alpha_beta.alpha = source->v_alpha_beta.alpha;
    destination->v_alpha_beta.beta = source->v_alpha_beta.beta;
    destination->rotor_feedback.theta_e_rad =
        source->rotor_feedback.theta_e_rad;
    destination->rotor_feedback.omega_e_rad_s =
        source->rotor_feedback.omega_e_rad_s;
    destination->rotor_feedback.has_valid_angle =
        source->rotor_feedback.has_valid_angle;
    destination->rotor_feedback.has_valid_speed =
        source->rotor_feedback.has_valid_speed;
    destination->rotor_feedback.has_edge_reference =
        source->rotor_feedback.has_edge_reference;
    destination->rotor_feedback.is_sector_limited =
        source->rotor_feedback.is_sector_limited;
    destination->rotor_feedback.is_timed_out =
        source->rotor_feedback.is_timed_out;
    app_copy_motor_control_output(
        &destination->motor_control,
        &source->motor_control
    );
    destination->duty.a = source->duty.a;
    destination->duty.b = source->duty.b;
    destination->duty.c = source->duty.c;
    destination->mode = source->mode;
    destination->has_valid_phase_current =
        source->has_valid_phase_current;
    destination->has_applied_duty = source->has_applied_duty;
}

static app_open_loop_command_t app_get_open_loop_command(const app_t *self)
{
    const uint32_t active_index = self->active_command_index & 1U;

    /* Writer가 active index를 publish하기 전에 command 쓰기를 끝냈음을 보장한다. */
    __DMB();
    return self->command_buffer[active_index];
}

static motor_control_current_reference_target_t
app_get_current_reference_target(const app_t *self)
{
    const bool is_speed_mode = self->mode == APP_MODE_SPEED;
    const uint32_t active_index = is_speed_mode ?
        (self->active_speed_current_target_index & 1U) :
        (self->active_current_command_index & 1U);

    __DMB();
    return is_speed_mode ? self->speed_current_target_buffer[active_index] :
        self->current_command_buffer[active_index];
}

static bool app_is_foc_mode(app_mode_t mode)
{
    return (mode == APP_MODE_CURRENT) || (mode == APP_MODE_SPEED);
}

static app_speed_command_t app_get_speed_command(const app_t *self)
{
    const uint32_t active_index = self->active_speed_command_index & 1U;

    __DMB();
    return self->speed_command_buffer[active_index];
}

static app_speed_feedback_t app_get_speed_feedback(const app_t *self)
{
    const uint32_t active_index = self->active_speed_feedback_index & 1U;

    __DMB();
    return self->speed_feedback_buffer[active_index];
}

static void app_publish_speed_feedback(
    app_t *self,
    const hall_estimator_output_t *rotor_feedback,
    const hall_driver_signal_feedback_t *hall_signal
)
{
    const uint32_t active_index = self->active_speed_feedback_index & 1U;
    const app_speed_feedback_t active_feedback =
        self->speed_feedback_buffer[active_index];
    const uint32_t inactive_index =
        (self->active_speed_feedback_index ^ 1U) & 1U;
    const bool has_valid_speed = rotor_feedback->has_valid_speed;

    /* Hall edge/timeout으로 speed observation이 바뀔 때만 1 kHz reader에 publish한다. */
    if ((active_feedback.capture_count == hall_signal->capture_count) &&
        (active_feedback.has_valid_speed == has_valid_speed) &&
        (active_feedback.is_timed_out == hall_signal->is_timed_out)) {
        return;
    }

    self->speed_feedback_buffer[inactive_index] = (app_speed_feedback_t){
        .omega_e_rad_s = has_valid_speed ?
            rotor_feedback->omega_e_rad_s : 0.0f,
        .capture_count = hall_signal->capture_count,
        .has_valid_speed = has_valid_speed,
        .is_timed_out = hall_signal->is_timed_out,
    };
    __DMB();
    self->active_speed_feedback_index = inactive_index;
}

static bool app_commands_are_zero(const app_t *self)
{
    const app_open_loop_command_t open_loop_command =
        app_get_open_loop_command(self);
    const motor_control_current_reference_target_t current_target =
        app_get_current_reference_target(self);
    const app_speed_command_t speed_command = app_get_speed_command(self);

    return (open_loop_command.voltage_magnitude == 0.0f) &&
        (open_loop_command.omega_e_rad_s == 0.0f) &&
        (current_target.i_dq_ref.d == 0.0f) &&
        (current_target.i_dq_ref.q == 0.0f) &&
        (speed_command.omega_m_ref_rad_s == 0.0f);
}

static fault_manager_fault_mask_t app_adc_fault_mask(
    adc_driver_status_t adc_status
)
{
    if (adc_status == ADC_DRIVER_STATUS_SYNC_ERROR) {
        return FAULT_MANAGER_FAULT_ADC_SYNC;
    }

    if (adc_status == ADC_DRIVER_STATUS_OVERRUN) {
        return FAULT_MANAGER_FAULT_ADC_OVERRUN;
    }

    return FAULT_MANAGER_FAULT_ADC;
}

static app_status_t app_disable_for_fault(app_t *self, app_status_t status)
{
    current_sensor_offset_calibration_state_t calibration_state;
    current_sensor_status_t current_sensor_status;
    motor_control_status_t motor_control_status;
    bool has_pwm_disable_error = false;
    const bool should_reset_motor_control = app_is_foc_mode(self->mode);

    self->mode = APP_MODE_DISABLED;
    self->drive_state = APP_DRIVE_STATE_FAULTED;
    current_sensor_status = current_sensor_get_offset_calibration_state(
        self->config.current_sensor,
        &calibration_state
    );
    if ((current_sensor_status == CURRENT_SENSOR_STATUS_OK) &&
        (calibration_state ==
            CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING)) {
        current_sensor_status = current_sensor_abort_offset_calibration(
            self->config.current_sensor
        );
    }
    if (current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        self->last_current_sensor_status = current_sensor_status;
    }
    __DMB();

    if (self->config.pwm_driver->is_initialized &&
        self->config.pwm_driver->is_enabled) {
        self->last_pwm_status = pwm_driver_disable(
            self->config.pwm_driver
        );
        if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
            has_pwm_disable_error = true;
            self->last_fault_manager_status = fault_manager_latch(
                self->config.fault_manager,
                FAULT_MANAGER_FAULT_PWM
            );
        }
    }

    if (should_reset_motor_control) {
        motor_control_status = motor_control_reset(
            self->config.motor_control
        );
        if (motor_control_status != MOTOR_CONTROL_STATUS_OK) {
            self->last_motor_control_status = motor_control_status;
            (void)fault_manager_latch(
                self->config.fault_manager,
                FAULT_MANAGER_FAULT_MOTOR_CONTROL
            );
        }
    }

    if (has_pwm_disable_error) {
        self->last_status = APP_STATUS_PWM_ERROR;
        return APP_STATUS_PWM_ERROR;
    }

    self->last_status = status;
    return status;
}

static app_status_t app_latch_and_stop(
    app_t *self,
    app_status_t status,
    fault_manager_fault_mask_t fault_mask
)
{
    ++self->error_count;
    self->last_fault_manager_status = fault_manager_latch(
        self->config.fault_manager,
        fault_mask
    );
    if (self->last_fault_manager_status != FAULT_MANAGER_STATUS_OK) {
        return app_disable_for_fault(
            self,
            APP_STATUS_FAULT_MANAGER_ERROR
        );
    }

    return app_disable_for_fault(self, status);
}

static void app_process_fault_clear_request(
    app_t *self
)
{
    if (!self->is_fault_clear_requested) {
        return;
    }

    /* 요청 하나는 성공/실패와 관계없이 한 번만 소비한다. */
    self->is_fault_clear_requested = false;
    __DMB();

    self->last_fault_clear_status = fault_manager_clear(
        self->config.fault_manager,
        !self->config.pwm_driver->is_enabled,
        app_commands_are_zero(self)
    );

    if (self->last_fault_clear_status == FAULT_MANAGER_STATUS_OK) {
        ++self->fault_clear_success_count;
    } else {
        ++self->fault_clear_blocked_count;
    }
}

static app_status_t app_update_rotor_feedback(
    app_t *self,
    const hall_estimator_output_t **rotor_feedback
)
{
    hall_driver_signal_feedback_t hall_signal;
    const hall_decoder_output_t *decoded_hall;

    self->last_hall_driver_status = hall_driver_get_signal_feedback(
        self->config.hall_driver,
        &hall_signal
    );
    if (self->last_hall_driver_status != HALL_DRIVER_STATUS_OK) {
        return APP_STATUS_HALL_FEEDBACK_ERROR;
    }

    self->last_hall_decoder_status = hall_decoder_update_fast(
        self->config.hall_decoder,
        &hall_signal
    );
    if (self->last_hall_decoder_status != HALL_DECODER_STATUS_OK) {
        return APP_STATUS_HALL_DECODER_ERROR;
    }
    decoded_hall = hall_decoder_get_latest_output_fast(
        self->config.hall_decoder
    );

    self->last_hall_estimator_status = hall_estimator_update_from_decoder_fast(
        self->config.hall_estimator,
        decoded_hall,
        self->config.sampling_period_s
    );
    if (self->last_hall_estimator_status != HALL_ESTIMATOR_STATUS_OK) {
        return APP_STATUS_ROTOR_ESTIMATOR_ERROR;
    }
    *rotor_feedback = hall_estimator_get_latest_output_fast(
        self->config.hall_estimator
    );
    if (self->mode == APP_MODE_SPEED) {
        app_publish_speed_feedback(self, *rotor_feedback, &hall_signal);
    }

    return APP_STATUS_OK;
}

static app_status_t app_update_current_sensor(
    app_t *self,
    const adc_driver_raw_sample_t *raw,
    abc_t *i_abc,
    bool *has_valid_phase_current
)
{
    current_sensor_offset_calibration_state_t calibration_state;

    *has_valid_phase_current = false;
    if (app_is_foc_mode(self->mode)) {
        current_sensor_convert_fast(
            self->config.current_sensor,
            raw,
            i_abc
        );
        self->last_current_sensor_status = CURRENT_SENSOR_STATUS_OK;
        *has_valid_phase_current = true;
        return APP_STATUS_OK;
    }

    self->last_current_sensor_status =
        current_sensor_get_offset_calibration_state(
            self->config.current_sensor,
            &calibration_state
        );
    if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_CURRENT_SENSOR_ERROR,
            FAULT_MANAGER_FAULT_CURRENT_SENSOR
        );
    }

    if (calibration_state == CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING) {
        if ((self->mode != APP_MODE_DISABLED) ||
            self->config.pwm_driver->is_enabled) {
            return app_latch_and_stop(
                self,
                APP_STATUS_CURRENT_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_CURRENT_SENSOR
            );
        }

        self->last_current_sensor_status =
            current_sensor_process_offset_sample(
                self->config.current_sensor,
                raw
            );
        if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_CURRENT_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_CURRENT_SENSOR
            );
        }

        self->last_current_sensor_status =
            current_sensor_get_offset_calibration_state(
                self->config.current_sensor,
                &calibration_state
            );
        if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_CURRENT_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_CURRENT_SENSOR
            );
        }
    }

    if (calibration_state == CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING) {
        return APP_STATUS_OK;
    }
    if (calibration_state != CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE) {
        return app_latch_and_stop(
            self,
            APP_STATUS_CURRENT_SENSOR_ERROR,
            FAULT_MANAGER_FAULT_CURRENT_SENSOR
        );
    }

    self->last_current_sensor_status = current_sensor_convert(
        self->config.current_sensor,
        raw,
        i_abc
    );
    if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_CURRENT_SENSOR_ERROR,
            FAULT_MANAGER_FAULT_CURRENT_SENSOR
        );
    }

    *has_valid_phase_current = true;
    return APP_STATUS_OK;
}

app_status_t app_init(app_t *self, const app_config_t *config)
{
    if ((self == NULL) ||
        (config == NULL) ||
        (config->adc_driver == NULL) ||
        (config->current_sensor == NULL) ||
        (config->voltage_sensor == NULL) ||
        (config->pwm_driver == NULL) ||
        (config->fault_manager == NULL) ||
        (config->hall_driver == NULL) ||
        (config->hall_decoder == NULL) ||
        (config->hall_estimator == NULL) ||
        (config->motor_control == NULL) ||
        (((config->fast_loop_profile == NULL) &&
            (config->motor_control_profile == NULL)) !=
            (config->cycle_counter_reader == NULL)) ||
        (!app_float_is_finite(config->sampling_period_s)) ||
        (config->sampling_period_s <= 0.0f) ||
        (!app_float_is_finite(config->speed_loop_period_s)) ||
        (config->speed_loop_period_s <= 0.0f) ||
        (config->current_offset_calibration_timeout_ms == 0U) ||
        (!app_float_is_finite(config->speed_stop_omega_m_threshold_rad_s)) ||
        (config->speed_stop_omega_m_threshold_rad_s <= 0.0f) ||
        (config->speed_stop_dwell_ms == 0U) ||
        (config->speed_loop_period_s !=
            config->motor_control->config.speed_controller.pi.sampling_period_s) ||
        (!app_float_is_finite(config->initial_voltage_angle_rad)) ||
        (config->initial_voltage_angle_rad < 0.0f) ||
        (config->initial_voltage_angle_rad >= APP_TWO_PI_RAD)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!config->adc_driver->is_initialized) ||
        (!config->current_sensor->is_initialized) ||
        (!config->voltage_sensor->is_initialized) ||
        (!config->fault_manager->is_initialized) ||
        (!config->hall_driver->is_initialized) ||
        (!config->hall_decoder->is_initialized) ||
        (!config->hall_estimator->is_initialized) ||
        (!config->motor_control->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }

    const app_open_loop_command_t zero_command = {
        .voltage_magnitude = 0.0f,
        .omega_e_rad_s = 0.0f,
    };
    const motor_control_current_reference_target_t zero_current_target = {
        .i_dq_ref = {0.0f, 0.0f},
        .was_saturated = false,
    };
    const app_speed_command_t zero_speed_command = {
        .omega_m_ref_rad_s = 0.0f,
    };
    const app_speed_feedback_t zero_speed_feedback = {
        .omega_e_rad_s = 0.0f,
        .capture_count = 0U,
        .has_valid_speed = false,
        .is_timed_out = false,
    };
    const abc_t neutral_duty = app_neutral_duty();
    const app_t initialized = {
        .config = *config,
        .command_buffer = {zero_command, zero_command},
        .active_command_index = 0U,
        .current_command_buffer = {
            zero_current_target,
            zero_current_target,
        },
        .active_current_command_index = 0U,
        .speed_command_buffer = {zero_speed_command, zero_speed_command},
        .active_speed_command_index = 0U,
        .speed_current_target_buffer = {
            zero_current_target,
            zero_current_target,
        },
        .active_speed_current_target_index = 0U,
        .speed_feedback_buffer = {zero_speed_feedback, zero_speed_feedback},
        .active_speed_feedback_index = 0U,
        .voltage_angle_rad = config->initial_voltage_angle_rad,
        .last_v_alpha_beta = {0.0f, 0.0f},
        .last_duty = neutral_duty,
        .last_adc_status = ADC_DRIVER_STATUS_OK,
        .last_current_sensor_status = CURRENT_SENSOR_STATUS_OK,
        .last_voltage_sensor_status = VOLTAGE_SENSOR_STATUS_OK,
        .last_hall_driver_status = HALL_DRIVER_STATUS_OK,
        .last_hall_decoder_status = HALL_DECODER_STATUS_OK,
        .last_hall_estimator_status = HALL_ESTIMATOR_STATUS_OK,
        .last_motor_control_status = MOTOR_CONTROL_STATUS_OK,
        .last_cordic_status = CORDIC_DRIVER_STATUS_OK,
        .last_svpwm_status = SVPWM_STATUS_OK,
        .last_pwm_status = PWM_DRIVER_STATUS_OK,
        .last_fault_manager_status = FAULT_MANAGER_STATUS_OK,
        .last_fault_clear_status = FAULT_MANAGER_STATUS_OK,
        .last_status = APP_STATUS_OK,
        .last_speed_output = {
            .omega_m_ref_limited_rad_s = 0.0f,
            .omega_m_feedback_rad_s = 0.0f,
            .speed_controller = {
                .omega_m_feedback_filtered_rad_s = 0.0f,
                .omega_m_error_rad_s = 0.0f,
                .i_q_ref = 0.0f,
                .is_i_q_ref_saturated = false,
            },
            .current_reference_target = zero_current_target,
        },
        .fast_loop_count = 0U,
        .duty_update_count = 0U,
        .not_ready_count = 0U,
        .error_count = 0U,
        .fault_clear_request_count = 0U,
        .fault_clear_success_count = 0U,
        .fault_clear_blocked_count = 0U,
        .is_initialized = true,
        .mode = APP_MODE_DISABLED,
        .drive_state = APP_DRIVE_STATE_DISABLED,
        .current_offset_calibration_elapsed_ms = 0U,
        .is_current_offset_calibration_timeout_requested = false,
        .speed_stop_low_speed_elapsed_ms = 0U,
        .is_fault_clear_requested = false,
    };

    *self = initialized;
    return APP_STATUS_OK;
}

app_status_t app_set_current_command(
    app_t *self,
    const app_current_command_t *command
)
{
    motor_control_current_reference_target_t prepared_target;
    uint32_t inactive_index;

    if ((self == NULL) || (command == NULL) ||
        (!app_float_is_finite(command->i_dq_ref.d)) ||
        (!app_float_is_finite(command->i_dq_ref.q))) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    if (motor_control_prepare_current_reference_target(
            self->config.motor_control,
            &command->i_dq_ref,
            &prepared_target) != MOTOR_CONTROL_STATUS_OK) {
        return APP_STATUS_INVALID_STATE;
    }

    inactive_index =
        (self->active_current_command_index ^ 1U) & 1U;
    self->current_command_buffer[inactive_index] = prepared_target;
    __DMB();
    self->active_current_command_index = inactive_index;

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_set_speed_command(
    app_t *self,
    const app_speed_command_t *command
)
{
    uint32_t inactive_index;

    if ((self == NULL) || (command == NULL) ||
        (!app_float_is_finite(command->omega_m_ref_rad_s))) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    inactive_index = (self->active_speed_command_index ^ 1U) & 1U;
    self->speed_command_buffer[inactive_index] = *command;
    __DMB();
    self->active_speed_command_index = inactive_index;

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_set_open_loop_command(
    app_t *self,
    const app_open_loop_command_t *command
)
{
    float angle_step_rad;
    uint32_t inactive_index;

    if ((self == NULL) || (command == NULL)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    angle_step_rad = command->omega_e_rad_s * self->config.sampling_period_s;
    if ((!app_float_is_finite(command->voltage_magnitude)) ||
        (command->voltage_magnitude < 0.0f) ||
        (!app_float_is_finite(command->omega_e_rad_s)) ||
        (!app_float_is_finite(angle_step_rad)) ||
        (angle_step_rad < -APP_TWO_PI_RAD) ||
        (angle_step_rad > APP_TWO_PI_RAD)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    inactive_index = (self->active_command_index ^ 1U) & 1U;
    self->command_buffer[inactive_index] = *command;

    /* Reader가 새 index를 보기 전에 inactive buffer의 두 field를 모두 완성한다. */
    __DMB();
    self->active_command_index = inactive_index;

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_start_current_offset_calibration(app_t *self)
{
    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (!self->config.adc_driver->is_initialized) ||
        (!self->config.adc_driver->is_running) ||
        (self->mode != APP_MODE_DISABLED) ||
        self->config.pwm_driver->is_enabled) {
        return APP_STATUS_INVALID_STATE;
    }

    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    self->last_current_sensor_status =
        current_sensor_start_offset_calibration(
            self->config.current_sensor
        );
    if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        self->last_status = APP_STATUS_CURRENT_SENSOR_ERROR;
        return APP_STATUS_CURRENT_SENSOR_ERROR;
    }

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_handle_current_offset_calibration_timeout(app_t *self)
{
    current_sensor_offset_calibration_state_t calibration_state;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    self->last_current_sensor_status =
        current_sensor_get_offset_calibration_state(
            self->config.current_sensor,
            &calibration_state
        );
    if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        return APP_STATUS_INVALID_STATE;
    }
    if (calibration_state == CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE) {
        self->last_status = APP_STATUS_OK;
        return APP_STATUS_OK;
    }
    if (calibration_state != CURRENT_SENSOR_OFFSET_CALIBRATION_RUNNING) {
        return APP_STATUS_INVALID_STATE;
    }

    self->last_current_sensor_status =
        current_sensor_abort_offset_calibration(
            self->config.current_sensor
        );
    if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
        self->last_current_sensor_status =
            current_sensor_get_offset_calibration_state(
                self->config.current_sensor,
                &calibration_state
            );
        if ((self->last_current_sensor_status == CURRENT_SENSOR_STATUS_OK) &&
            (calibration_state ==
                CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE)) {
            self->last_status = APP_STATUS_OK;
            return APP_STATUS_OK;
        }
        return APP_STATUS_INVALID_STATE;
    }

    return app_latch_and_stop(
        self,
        APP_STATUS_CURRENT_OFFSET_CALIBRATION_TIMEOUT,
        FAULT_MANAGER_FAULT_CURRENT_SENSOR
    );
}

app_status_t app_start_open_loop(app_t *self)
{
    current_sensor_offset_calibration_state_t calibration_state;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }

    if (app_is_foc_mode(self->mode)) {
        return APP_STATUS_INVALID_STATE;
    }
    if (self->mode == APP_MODE_OPEN_LOOP) {
        self->last_status = APP_STATUS_OK;
        return APP_STATUS_OK;
    }

    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    self->last_current_sensor_status =
        current_sensor_get_offset_calibration_state(
            self->config.current_sensor,
            &calibration_state
        );
    if ((self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) ||
        (calibration_state !=
            CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE)) {
        self->last_status = APP_STATUS_INVALID_STATE;
        return APP_STATUS_INVALID_STATE;
    }

    __DMB();
    self->mode = APP_MODE_OPEN_LOOP;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_request_fault_clear(app_t *self)
{
    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (!fault_manager_is_faulted(self->config.fault_manager))) {
        return APP_STATUS_INVALID_STATE;
    }

    __DMB();
    self->is_fault_clear_requested = true;
    ++self->fault_clear_request_count;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_stop_open_loop(app_t *self)
{
    abc_t neutral_duty;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }

    if (app_is_foc_mode(self->mode)) {
        return APP_STATUS_INVALID_STATE;
    }

    self->mode = APP_MODE_DISABLED;
    __DMB();

    neutral_duty = app_neutral_duty();
    self->last_pwm_status = pwm_driver_set_duty(
        self->config.pwm_driver,
        &neutral_duty
    );
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_PWM_ERROR,
            FAULT_MANAGER_FAULT_PWM
        );
    }

    self->last_v_alpha_beta.alpha = 0.0f;
    self->last_v_alpha_beta.beta = 0.0f;
    self->last_duty = neutral_duty;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_start_current_control(app_t *self)
{
    current_sensor_offset_calibration_state_t calibration_state;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }
    if (self->mode == APP_MODE_OPEN_LOOP) {
        return APP_STATUS_INVALID_STATE;
    }
    if (self->mode == APP_MODE_CURRENT) {
        self->last_status = APP_STATUS_OK;
        return APP_STATUS_OK;
    }
    if (self->mode == APP_MODE_SPEED) {
        return APP_STATUS_INVALID_STATE;
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }
    if (!app_commands_are_zero(self)) {
        return APP_STATUS_INVALID_STATE;
    }

    self->last_current_sensor_status =
        current_sensor_get_offset_calibration_state(
            self->config.current_sensor,
            &calibration_state
        );
    if ((self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) ||
        (calibration_state !=
            CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE) ||
        (!self->config.hall_driver->is_running) ||
        (self->last_hall_driver_status != HALL_DRIVER_STATUS_OK) ||
        (self->last_hall_decoder_status != HALL_DECODER_STATUS_OK) ||
        (self->last_hall_estimator_status !=
            HALL_ESTIMATOR_STATUS_OK) ||
        (!self->config.hall_estimator->output.has_valid_angle)) {
        self->last_status = APP_STATUS_INVALID_STATE;
        return APP_STATUS_INVALID_STATE;
    }

    self->last_motor_control_status = motor_control_reset(
        self->config.motor_control
    );
    if (self->last_motor_control_status != MOTOR_CONTROL_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_MOTOR_CONTROL_ERROR,
            FAULT_MANAGER_FAULT_MOTOR_CONTROL
        );
    }

    __DMB();
    self->mode = APP_MODE_CURRENT;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

static app_status_t app_stop_foc_control(
    app_t *self,
    app_mode_t expected_mode
)
{
    abc_t neutral_duty;
    const app_speed_command_t zero_speed_command = {
        .omega_m_ref_rad_s = 0.0f,
    };
    const motor_control_current_reference_target_t zero_current_target = {
        .i_dq_ref = {0.0f, 0.0f},
        .was_saturated = false,
    };

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized) ||
        ((expected_mode != APP_MODE_CURRENT) &&
            (expected_mode != APP_MODE_SPEED)) ||
        (self->mode != expected_mode)) {
        return APP_STATUS_INVALID_STATE;
    }

    self->mode = APP_MODE_DISABLED;
    __DMB();

    self->speed_command_buffer[
        (self->active_speed_command_index ^ 1U) & 1U] = zero_speed_command;
    __DMB();
    self->active_speed_command_index ^= 1U;
    self->speed_current_target_buffer[
        (self->active_speed_current_target_index ^ 1U) & 1U] =
        zero_current_target;
    __DMB();
    self->active_speed_current_target_index ^= 1U;

    self->last_motor_control_status = motor_control_reset(
        self->config.motor_control
    );
    if (self->last_motor_control_status != MOTOR_CONTROL_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_MOTOR_CONTROL_ERROR,
            FAULT_MANAGER_FAULT_MOTOR_CONTROL
        );
    }

    neutral_duty = app_neutral_duty();
    self->last_pwm_status = pwm_driver_set_duty(
        self->config.pwm_driver,
        &neutral_duty
    );
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_PWM_ERROR,
            FAULT_MANAGER_FAULT_PWM
        );
    }

    self->last_v_alpha_beta = (alpha_beta_t){0.0f, 0.0f};
    self->last_duty = neutral_duty;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_stop_current_control(app_t *self)
{
    return app_stop_foc_control(self, APP_MODE_CURRENT);
}

static app_status_t app_stop_speed_control(app_t *self)
{
    return app_stop_foc_control(self, APP_MODE_SPEED);
}

app_status_t app_start_speed_control(app_t *self)
{
    app_status_t status;
    const motor_control_current_reference_target_t zero_current_target = {
        .i_dq_ref = {0.0f, 0.0f},
        .was_saturated = false,
    };
    uint32_t inactive_index;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }
    if (self->mode == APP_MODE_SPEED) {
        self->last_status = APP_STATUS_OK;
        return APP_STATUS_OK;
    }
    if (self->mode != APP_MODE_DISABLED) {
        return APP_STATUS_INVALID_STATE;
    }

    status = app_start_current_control(self);
    if (status != APP_STATUS_OK) {
        return status;
    }

    inactive_index =
        (self->active_speed_current_target_index ^ 1U) & 1U;
    self->speed_current_target_buffer[inactive_index] = zero_current_target;
    __DMB();
    self->active_speed_current_target_index = inactive_index;
    self->mode = APP_MODE_SPEED;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_speed_control_tick(app_t *self)
{
    app_speed_command_t command;
    app_speed_feedback_t feedback;
    motor_control_speed_input_t input;
    uint32_t inactive_index;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }
    if (self->mode != APP_MODE_SPEED) {
        return APP_STATUS_OK;
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    command = app_get_speed_command(self);
    feedback = app_get_speed_feedback(self);
    input = (motor_control_speed_input_t){
        .omega_m_requested_rad_s = command.omega_m_ref_rad_s,
        .omega_e_feedback_rad_s = feedback.has_valid_speed ?
            feedback.omega_e_rad_s : 0.0f,
    };
    self->last_motor_control_status = motor_control_update_speed(
        self->config.motor_control,
        &input,
        &self->last_speed_output
    );
    if (self->last_motor_control_status != MOTOR_CONTROL_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_MOTOR_CONTROL_ERROR,
            FAULT_MANAGER_FAULT_MOTOR_CONTROL
        );
    }

    inactive_index =
        (self->active_speed_current_target_index ^ 1U) & 1U;
    self->speed_current_target_buffer[inactive_index] =
        self->last_speed_output.current_reference_target;
    __DMB();
    self->active_speed_current_target_index = inactive_index;

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_drive_start(app_t *self)
{
    app_status_t status;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (self->drive_state != APP_DRIVE_STATE_DISABLED)) {
        return APP_STATUS_INVALID_STATE;
    }

    status = app_start_current_offset_calibration(self);
    if (status != APP_STATUS_OK) {
        return status;
    }

    self->current_offset_calibration_elapsed_ms = 0U;
    self->is_current_offset_calibration_timeout_requested = false;
    __DMB();
    self->drive_state = APP_DRIVE_STATE_CURRENT_OFFSET_CALIBRATION;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_drive_start_speed(
    app_t *self,
    const app_speed_command_t *command
)
{
    const app_current_command_t zero_current_command = {
        .i_dq_ref = {0.0f, 0.0f},
    };
    const app_speed_command_t zero_speed_command = {
        .omega_m_ref_rad_s = 0.0f,
    };
    app_status_t status;

    if ((self == NULL) || (command == NULL) ||
        (!app_float_is_finite(command->omega_m_ref_rad_s))) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized) ||
        (self->drive_state != APP_DRIVE_STATE_READY)) {
        return APP_STATUS_INVALID_STATE;
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    status = app_set_current_command(self, &zero_current_command);
    if (status != APP_STATUS_OK) {
        return status;
    }
    status = app_set_speed_command(self, &zero_speed_command);
    if (status != APP_STATUS_OK) {
        return status;
    }
    status = app_start_speed_control(self);
    if (status != APP_STATUS_OK) {
        return status;
    }

    self->last_pwm_status = pwm_driver_enable(self->config.pwm_driver);
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_PWM_ERROR,
            FAULT_MANAGER_FAULT_PWM
        );
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        return app_disable_for_fault(self, APP_STATUS_FAULT_ACTIVE);
    }

    status = app_set_speed_command(self, command);
    if (status != APP_STATUS_OK) {
        return app_latch_and_stop(
            self,
            status,
            FAULT_MANAGER_FAULT_MOTOR_CONTROL
        );
    }

    __DMB();
    self->drive_state = APP_DRIVE_STATE_SPEED_RUNNING;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_drive_request_speed_stop(app_t *self)
{
    const app_speed_command_t zero_speed_command = {
        .omega_m_ref_rad_s = 0.0f,
    };
    app_status_t status;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (self->drive_state != APP_DRIVE_STATE_SPEED_RUNNING)) {
        return APP_STATUS_INVALID_STATE;
    }

    status = app_set_speed_command(self, &zero_speed_command);
    if (status != APP_STATUS_OK) {
        return status;
    }

    __DMB();
    self->drive_state = APP_DRIVE_STATE_RAMP_TO_ZERO;
    self->speed_stop_low_speed_elapsed_ms = 0U;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_drive_update(app_t *self)
{
    current_sensor_offset_calibration_state_t calibration_state;
    app_status_t status;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        if (self->drive_state != APP_DRIVE_STATE_FAULTED) {
            return app_disable_for_fault(self, APP_STATUS_FAULT_ACTIVE);
        }
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    if (self->drive_state == APP_DRIVE_STATE_CURRENT_OFFSET_CALIBRATION) {
        if (self->is_current_offset_calibration_timeout_requested) {
            self->is_current_offset_calibration_timeout_requested = false;
            status = app_handle_current_offset_calibration_timeout(self);
            if (status != APP_STATUS_OK) {
                return status;
            }
        }

        self->last_current_sensor_status =
            current_sensor_get_offset_calibration_state(
                self->config.current_sensor,
                &calibration_state
            );
        if (self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_CURRENT_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_CURRENT_SENSOR
            );
        }
        if (calibration_state == CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE) {
            __DMB();
            self->drive_state = APP_DRIVE_STATE_READY;
        } else if (calibration_state == CURRENT_SENSOR_OFFSET_CALIBRATION_FAILED) {
            return app_latch_and_stop(
                self,
                APP_STATUS_CURRENT_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_CURRENT_SENSOR
            );
        }
    } else if (self->drive_state == APP_DRIVE_STATE_RAMP_TO_ZERO) {
        if (self->speed_stop_low_speed_elapsed_ms >=
            self->config.speed_stop_dwell_ms) {
            status = app_stop_speed_control(self);
            if (status != APP_STATUS_OK) {
                return status;
            }
            self->last_pwm_status = pwm_driver_disable(self->config.pwm_driver);
            if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
                return app_latch_and_stop(
                    self,
                    APP_STATUS_PWM_ERROR,
                    FAULT_MANAGER_FAULT_PWM
                );
            }
            __DMB();
            self->drive_state = APP_DRIVE_STATE_READY;
        }
    }

    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

void app_drive_scheduler_tick(app_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return;
    }

    if (self->drive_state == APP_DRIVE_STATE_CURRENT_OFFSET_CALIBRATION) {
        if (self->current_offset_calibration_elapsed_ms <
            self->config.current_offset_calibration_timeout_ms) {
            ++self->current_offset_calibration_elapsed_ms;
        }
        if (self->current_offset_calibration_elapsed_ms >=
            self->config.current_offset_calibration_timeout_ms) {
            self->is_current_offset_calibration_timeout_requested = true;
            __DMB();
        }
    }

    if ((self->drive_state == APP_DRIVE_STATE_SPEED_RUNNING) ||
        (self->drive_state == APP_DRIVE_STATE_RAMP_TO_ZERO)) {
        (void)app_speed_control_tick(self);
    }

    if (self->drive_state == APP_DRIVE_STATE_RAMP_TO_ZERO) {
        const app_speed_feedback_t speed_feedback = app_get_speed_feedback(self);
        const float omega_m_feedback_rad_s =
            self->last_speed_output.omega_m_feedback_rad_s;
        const bool is_below_stop_speed =
            (omega_m_feedback_rad_s >=
                -self->config.speed_stop_omega_m_threshold_rad_s) &&
            (omega_m_feedback_rad_s <=
                self->config.speed_stop_omega_m_threshold_rad_s);
        const bool is_stop_speed_confirmed = speed_feedback.is_timed_out ||
            (speed_feedback.has_valid_speed && is_below_stop_speed);

        if (self->last_speed_output.omega_m_ref_limited_rad_s != 0.0f) {
            self->speed_stop_low_speed_elapsed_ms = 0U;
        } else if ((self->speed_stop_low_speed_elapsed_ms != 0U) ||
                   is_stop_speed_confirmed) {
            if (self->speed_stop_low_speed_elapsed_ms <
                self->config.speed_stop_dwell_ms) {
                ++self->speed_stop_low_speed_elapsed_ms;
            }
        }
    }
}

app_status_t app_drive_recover_after_fault(app_t *self)
{
    current_sensor_offset_calibration_state_t calibration_state;

    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) ||
        (self->drive_state != APP_DRIVE_STATE_FAULTED)) {
        return APP_STATUS_INVALID_STATE;
    }
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        self->last_status = APP_STATUS_FAULT_ACTIVE;
        return APP_STATUS_FAULT_ACTIVE;
    }

    self->last_current_sensor_status = current_sensor_get_offset_calibration_state(
        self->config.current_sensor,
        &calibration_state
    );
    if ((self->last_current_sensor_status != CURRENT_SENSOR_STATUS_OK) ||
        (calibration_state != CURRENT_SENSOR_OFFSET_CALIBRATION_COMPLETE)) {
        return APP_STATUS_INVALID_STATE;
    }

    __DMB();
    self->drive_state = APP_DRIVE_STATE_READY;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}

app_status_t app_handle_adc_error(
    app_t *self,
    adc_driver_status_t adc_status
)
{
    if ((self == NULL) ||
        (adc_status == ADC_DRIVER_STATUS_OK) ||
        (adc_status == ADC_DRIVER_STATUS_NOT_READY)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    self->last_adc_status = adc_status;
    return app_latch_and_stop(
        self,
        APP_STATUS_ADC_ERROR,
        app_adc_fault_mask(adc_status)
    );
}

#define calculated_output (*fast_loop_output)

static app_status_t app_motor_fast_loop_update(
    app_t *self,
    app_fast_loop_output_t *fast_loop_output
)
{
    app_open_loop_command_t open_loop_command = {0};
    motor_control_current_reference_target_t current_target;
    const hall_estimator_output_t *rotor_feedback;
    app_status_t rotor_status;
    bool was_faulted;
    float sin_theta;
    float cos_theta;
    uint32_t profile_segment_start_cycles;
    uint32_t profile_control_start_cycles;

    profile_segment_start_cycles = app_profile_begin(self);

    self->last_adc_status = adc_driver_read_raw(
        self->config.adc_driver,
        &calculated_output.raw
    );
    if (self->last_adc_status == ADC_DRIVER_STATUS_NOT_READY) {
        ++self->not_ready_count;
        self->last_status = APP_STATUS_ADC_NOT_READY;
        return APP_STATUS_ADC_NOT_READY;
    }
    if (self->last_adc_status != ADC_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_ADC_ERROR,
            app_adc_fault_mask(self->last_adc_status)
        );
    }

    calculated_output.i_abc = (abc_t){0.0f, 0.0f, 0.0f};
    calculated_output.has_valid_phase_current = false;
    calculated_output.v_dc = voltage_sensor_convert_fast(
        self->config.voltage_sensor,
        calculated_output.raw.dc_link
    );
    self->last_voltage_sensor_status = VOLTAGE_SENSOR_STATUS_OK;

    const app_status_t current_sensor_status = app_update_current_sensor(
        self,
        &calculated_output.raw,
        &calculated_output.i_abc,
        &calculated_output.has_valid_phase_current
    );
    if (current_sensor_status != APP_STATUS_OK) {
        return current_sensor_status;
    }

    ++self->fast_loop_count;
    calculated_output.voltage_angle_rad = self->voltage_angle_rad;
    calculated_output.v_alpha_beta = (alpha_beta_t){0.0f, 0.0f};
    calculated_output.duty = app_neutral_duty();
    calculated_output.has_applied_duty = false;

    profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
        self,
        sensing,
        profile_segment_start_cycles
    );

    was_faulted = fault_manager_is_faulted(self->config.fault_manager);
    if (calculated_output.has_valid_phase_current) {
        fault_manager_update_measurements_fast(
            self->config.fault_manager,
            &calculated_output.i_abc,
            calculated_output.v_dc
        );
        self->last_fault_manager_status = FAULT_MANAGER_STATUS_OK;
    } else {
        self->last_fault_manager_status =
            fault_manager_update_dc_link_voltage(
                self->config.fault_manager,
                calculated_output.v_dc
            );
    }
    if (self->last_fault_manager_status != FAULT_MANAGER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_FAULT_MANAGER_ERROR,
            FAULT_MANAGER_FAULT_INVALID_MEASUREMENT
        );
    }

    if (fault_manager_is_faulted(self->config.fault_manager)) {
        if (!was_faulted) {
            ++self->error_count;
        }

        if (app_disable_for_fault(
                self,
                APP_STATUS_FAULT_ACTIVE) == APP_STATUS_PWM_ERROR) {
            return APP_STATUS_PWM_ERROR;
        }
        app_process_fault_clear_request(self);

        if (fault_manager_is_faulted(self->config.fault_manager)) {
            self->last_status = APP_STATUS_FAULT_ACTIVE;
            return APP_STATUS_FAULT_ACTIVE;
        }
    }

    profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
        self,
        fault,
        profile_segment_start_cycles
    );

    rotor_status = app_update_rotor_feedback(
        self,
        &rotor_feedback
    );
    if ((rotor_status != APP_STATUS_OK) && app_is_foc_mode(self->mode)) {
        const fault_manager_fault_mask_t fault_mask =
            ((rotor_status == APP_STATUS_HALL_FEEDBACK_ERROR) ||
             (rotor_status == APP_STATUS_HALL_DECODER_ERROR)) ?
                FAULT_MANAGER_FAULT_HALL_FEEDBACK :
                FAULT_MANAGER_FAULT_ROTOR_ESTIMATOR;

        return app_latch_and_stop(self, rotor_status, fault_mask);
    }
    if (rotor_status != APP_STATUS_OK) {
        app_clear_rotor_feedback(&calculated_output.rotor_feedback);
    } else {
        calculated_output.rotor_feedback = *rotor_feedback;
    }

    profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
        self,
        rotor,
        profile_segment_start_cycles
    );
    profile_control_start_cycles = profile_segment_start_cycles;

    calculated_output.mode = self->mode;
    if (!calculated_output.has_valid_phase_current ||
        (self->mode == APP_MODE_DISABLED)) {
        app_clear_motor_control_output(&calculated_output.motor_control);
        self->last_status = APP_STATUS_OK;
        return APP_STATUS_OK;
    }

    if (self->mode == APP_MODE_OPEN_LOOP) {
        app_clear_motor_control_output(&calculated_output.motor_control);
        open_loop_command = app_get_open_loop_command(self);
        if (open_loop_command.voltage_magnitude > 0.0f) {
            self->last_cordic_status = cordic_driver_sin_cos(
                self->voltage_angle_rad,
                &sin_theta,
                &cos_theta
            );
            if (self->last_cordic_status != CORDIC_DRIVER_STATUS_OK) {
                return app_latch_and_stop(
                    self,
                    APP_STATUS_CORDIC_ERROR,
                    FAULT_MANAGER_FAULT_CORDIC
                );
            }

            calculated_output.v_alpha_beta.alpha =
                open_loop_command.voltage_magnitude * cos_theta;
            calculated_output.v_alpha_beta.beta =
                open_loop_command.voltage_magnitude * sin_theta;
        } else {
            /* 0 V open-loop command는 정확한 neutral duty를 바로 사용한다. */
            self->last_cordic_status = CORDIC_DRIVER_STATUS_OK;
            self->last_svpwm_status = SVPWM_STATUS_OK;
        }
    } else if (app_is_foc_mode(self->mode)) {
        motor_control_fast_input_t motor_control_input;

        if (calculated_output.v_dc <= 0.0f) {
            return app_latch_and_stop(
                self,
                APP_STATUS_VOLTAGE_SENSOR_ERROR,
                FAULT_MANAGER_FAULT_INVALID_MEASUREMENT
            );
        }

        if (!calculated_output.rotor_feedback.has_valid_angle) {
            return app_latch_and_stop(
                self,
                APP_STATUS_ROTOR_ESTIMATOR_ERROR,
                FAULT_MANAGER_FAULT_ROTOR_ESTIMATOR
            );
        }

        cordic_driver_sin_cos_fast(
            calculated_output.rotor_feedback.theta_e_rad,
            &sin_theta,
            &cos_theta
        );
        self->last_cordic_status = CORDIC_DRIVER_STATUS_OK;

        profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
            self,
            control_cordic,
            profile_segment_start_cycles
        );

        current_target = app_get_current_reference_target(self);
        motor_control_input = (motor_control_fast_input_t){
            .i_abc = calculated_output.i_abc,
            .current_reference_target = current_target,
            .sin_theta = sin_theta,
            .cos_theta = cos_theta,
            .omega_e_rad_s =
                calculated_output.rotor_feedback.has_valid_speed ?
                    calculated_output.rotor_feedback.omega_e_rad_s : 0.0f,
            .v_dc = calculated_output.v_dc,
        };
        profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
            self,
            control_prepare,
            profile_segment_start_cycles
        );

#if APP_FAST_LOOP_DETAILED_PROFILING_ENABLED
        if (self->config.motor_control_profile != NULL) {
            self->last_motor_control_status =
                motor_control_update_fast_profiled(
                    self->config.motor_control,
                    &motor_control_input,
                    &calculated_output.motor_control,
                    self->config.motor_control_profile,
                    self->config.cycle_counter_reader
                );
        } else {
            self->last_motor_control_status = motor_control_update_fast(
                self->config.motor_control,
                &motor_control_input,
                &calculated_output.motor_control
            );
        }
#else
        self->last_motor_control_status = motor_control_update_fast(
            self->config.motor_control,
            &motor_control_input,
            &calculated_output.motor_control
        );
#endif
        if (self->last_motor_control_status != MOTOR_CONTROL_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_MOTOR_CONTROL_ERROR,
                FAULT_MANAGER_FAULT_MOTOR_CONTROL
            );
        }

        calculated_output.v_alpha_beta =
            calculated_output.motor_control.foc.v_alpha_beta_ref;
    } else {
        return app_latch_and_stop(
            self,
            APP_STATUS_INVALID_STATE,
            FAULT_MANAGER_FAULT_MOTOR_CONTROL
        );
    }

    profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
        self,
        control,
        profile_control_start_cycles
    );

    if (app_is_foc_mode(self->mode) ||
        (open_loop_command.voltage_magnitude > 0.0f)) {
        if (app_is_foc_mode(self->mode)) {
            self->last_svpwm_status = svpwm_calculate_fast(
                &calculated_output.v_alpha_beta,
                calculated_output.v_dc,
                &calculated_output.duty
            );
        } else {
            self->last_svpwm_status = svpwm_calculate(
                &calculated_output.v_alpha_beta,
                calculated_output.v_dc,
                &calculated_output.duty
            );
        }
        if (self->last_svpwm_status != SVPWM_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_SVPWM_ERROR,
                FAULT_MANAGER_FAULT_SVPWM
            );
        }
    }

    if (app_is_foc_mode(self->mode)) {
        pwm_driver_set_duty_fast(
            self->config.pwm_driver,
            &calculated_output.duty
        );
        self->last_pwm_status = PWM_DRIVER_STATUS_OK;
    } else {
        self->last_pwm_status = pwm_driver_set_duty(
            self->config.pwm_driver,
            &calculated_output.duty
        );
    }
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_PWM_ERROR,
            FAULT_MANAGER_FAULT_PWM
        );
    }

    profile_segment_start_cycles = APP_PROFILE_END_SEGMENT(
        self,
        modulation_pwm,
        profile_segment_start_cycles
    );

    if (self->mode == APP_MODE_OPEN_LOOP) {
        const float next_angle_rad = self->voltage_angle_rad +
            (open_loop_command.omega_e_rad_s *
             self->config.sampling_period_s);

        self->voltage_angle_rad = app_wrap_angle_rad(next_angle_rad);
    }
    self->last_v_alpha_beta = calculated_output.v_alpha_beta;
    self->last_duty = calculated_output.duty;
    ++self->duty_update_count;

    calculated_output.has_applied_duty = true;
    self->last_status = APP_STATUS_OK;

    (void)APP_PROFILE_END_SEGMENT(
        self,
        diagnostic,
        profile_segment_start_cycles
    );
#if APP_FAST_LOOP_DETAILED_PROFILING_ENABLED
    if (self->config.fast_loop_profile != NULL) {
        ++self->config.fast_loop_profile->complete_sample_count;
        self->config.fast_loop_profile->is_last_sample_complete = true;
    }
#endif
    return APP_STATUS_OK;
}

#undef calculated_output

app_status_t app_motor_fast_loop(app_t *self, app_fast_loop_output_t *output)
{
    app_fast_loop_output_t calculated_output;

    if ((self == NULL) || (output == NULL)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    const app_status_t status = app_motor_fast_loop_update(
        self,
        &calculated_output
    );
    if (status == APP_STATUS_OK) {
        app_copy_fast_loop_output(output, &calculated_output);
    }
    return status;
}

app_status_t app_motor_fast_loop_fast(
    app_t *self,
    app_fast_loop_output_t *output
)
{
    if ((self == NULL) || (output == NULL)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

    return app_motor_fast_loop_update(self, output);
}

app_status_t app_motor_current_fast_loop_drive_fast(app_t *self)
{
    adc_driver_raw_sample_t raw;
    abc_t i_abc;
    float v_dc;
    const hall_estimator_output_t *rotor_feedback;
    motor_control_fast_input_t control_input;
    alpha_beta_t v_alpha_beta;
    abc_t duty;
    float sin_theta;
    float cos_theta;
    bool has_valid_phase_current;
    app_status_t status;

    if ((self == NULL) || !self->is_initialized ||
        !app_is_foc_mode(self->mode)) {
        return APP_STATUS_INVALID_STATE;
    }
    self->last_adc_status = adc_driver_read_raw(self->config.adc_driver, &raw);
    if (self->last_adc_status == ADC_DRIVER_STATUS_NOT_READY) {
        ++self->not_ready_count;
        self->last_status = APP_STATUS_ADC_NOT_READY;
        return APP_STATUS_ADC_NOT_READY;
    }
    if (self->last_adc_status != ADC_DRIVER_STATUS_OK) {
        return app_latch_and_stop(self, APP_STATUS_ADC_ERROR,
                                  app_adc_fault_mask(self->last_adc_status));
    }
    v_dc = voltage_sensor_convert_fast(self->config.voltage_sensor, raw.dc_link);
    self->last_voltage_sensor_status = VOLTAGE_SENSOR_STATUS_OK;
    status = app_update_current_sensor(
        self,
        &raw,
        &i_abc,
        &has_valid_phase_current
    );
    if (status != APP_STATUS_OK) {
        return status;
    }
    ++self->fast_loop_count;
    fault_manager_update_measurements_fast(self->config.fault_manager, &i_abc, v_dc);
    self->last_fault_manager_status = FAULT_MANAGER_STATUS_OK;
    if (fault_manager_is_faulted(self->config.fault_manager)) {
        ++self->error_count;
        return app_disable_for_fault(self, APP_STATUS_FAULT_ACTIVE);
    }
    status = app_update_rotor_feedback(self, &rotor_feedback);
    if (status != APP_STATUS_OK) {
        return app_latch_and_stop(self, status,
            (status == APP_STATUS_HALL_FEEDBACK_ERROR || status == APP_STATUS_HALL_DECODER_ERROR) ?
            FAULT_MANAGER_FAULT_HALL_FEEDBACK : FAULT_MANAGER_FAULT_ROTOR_ESTIMATOR);
    }
    if (v_dc <= 0.0f) {
        return app_latch_and_stop(self, APP_STATUS_VOLTAGE_SENSOR_ERROR,
                                  FAULT_MANAGER_FAULT_INVALID_MEASUREMENT);
    }
    if (!rotor_feedback->has_valid_angle) {
        return app_latch_and_stop(self, APP_STATUS_ROTOR_ESTIMATOR_ERROR,
                                  FAULT_MANAGER_FAULT_ROTOR_ESTIMATOR);
    }
    cordic_driver_sin_cos_fast(rotor_feedback->theta_e_rad, &sin_theta, &cos_theta);
    self->last_cordic_status = CORDIC_DRIVER_STATUS_OK;
    control_input = (motor_control_fast_input_t){
        .i_abc = i_abc, .current_reference_target = app_get_current_reference_target(self),
        .sin_theta = sin_theta, .cos_theta = cos_theta,
        .omega_e_rad_s = rotor_feedback->has_valid_speed ? rotor_feedback->omega_e_rad_s : 0.0f,
        .v_dc = v_dc,
    };
    self->last_motor_control_status = motor_control_update_fast_voltage(
        self->config.motor_control, &control_input, &v_alpha_beta);
    if (self->last_motor_control_status != MOTOR_CONTROL_STATUS_OK) {
        return app_latch_and_stop(self, APP_STATUS_MOTOR_CONTROL_ERROR,
                                  FAULT_MANAGER_FAULT_MOTOR_CONTROL);
    }
    self->last_svpwm_status = svpwm_calculate_fast(&v_alpha_beta, v_dc, &duty);
    if (self->last_svpwm_status != SVPWM_STATUS_OK) {
        return app_latch_and_stop(self, APP_STATUS_SVPWM_ERROR, FAULT_MANAGER_FAULT_SVPWM);
    }
    pwm_driver_set_duty_fast(self->config.pwm_driver, &duty);
    self->last_pwm_status = PWM_DRIVER_STATUS_OK;
    self->last_v_alpha_beta = v_alpha_beta;
    self->last_duty = duty;
    ++self->duty_update_count;
    self->last_status = APP_STATUS_OK;
    return APP_STATUS_OK;
}
