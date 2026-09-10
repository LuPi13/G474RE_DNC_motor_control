/**
 * @file app.c
 * @brief Open-loop inverter bring-up용 App fast loop를 구현한다.
 */

#include "app.h"

#include <float.h>
#include <stddef.h>

#define APP_ZERO_DUTY                 (0.5f)
#define APP_TWO_PI_RAD                (6.28318530717958647692f)

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

static app_open_loop_command_t app_get_open_loop_command(const app_t *self)
{
    const uint32_t active_index = self->active_command_index & 1U;

    /* Writer가 active index를 publish하기 전에 command 쓰기를 끝냈음을 보장한다. */
    __DMB();
    return self->command_buffer[active_index];
}

static bool app_command_is_zero(const app_open_loop_command_t *command)
{
    return (command->voltage_magnitude == 0.0f) &&
        (command->omega_e_rad_s == 0.0f);
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

    self->is_open_loop_active = false;
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

    if ((!self->config.pwm_driver->is_initialized) ||
        (!self->config.pwm_driver->is_enabled)) {
        self->last_status = status;
        return status;
    }

    self->last_pwm_status = pwm_driver_disable(self->config.pwm_driver);
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        self->last_fault_manager_status = fault_manager_latch(
            self->config.fault_manager,
            FAULT_MANAGER_FAULT_PWM
        );
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
    app_t *self,
    const app_open_loop_command_t *command
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
        app_command_is_zero(command)
    );

    if (self->last_fault_clear_status == FAULT_MANAGER_STATUS_OK) {
        ++self->fault_clear_success_count;
    } else {
        ++self->fault_clear_blocked_count;
    }
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
        if (self->is_open_loop_active || self->config.pwm_driver->is_enabled) {
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
        (!app_float_is_finite(config->sampling_period_s)) ||
        (config->sampling_period_s <= 0.0f) ||
        (!app_float_is_finite(config->initial_voltage_angle_rad)) ||
        (config->initial_voltage_angle_rad < 0.0f) ||
        (config->initial_voltage_angle_rad >= APP_TWO_PI_RAD)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!config->adc_driver->is_initialized) ||
        (!config->current_sensor->is_initialized) ||
        (!config->voltage_sensor->is_initialized) ||
        (!config->fault_manager->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }

    const app_open_loop_command_t zero_command = {
        .voltage_magnitude = 0.0f,
        .omega_e_rad_s = 0.0f,
    };
    const abc_t neutral_duty = app_neutral_duty();
    const app_t initialized = {
        .config = *config,
        .command_buffer = {zero_command, zero_command},
        .active_command_index = 0U,
        .voltage_angle_rad = config->initial_voltage_angle_rad,
        .last_v_alpha_beta = {0.0f, 0.0f},
        .last_duty = neutral_duty,
        .last_adc_status = ADC_DRIVER_STATUS_OK,
        .last_current_sensor_status = CURRENT_SENSOR_STATUS_OK,
        .last_voltage_sensor_status = VOLTAGE_SENSOR_STATUS_OK,
        .last_cordic_status = CORDIC_DRIVER_STATUS_OK,
        .last_svpwm_status = SVPWM_STATUS_OK,
        .last_pwm_status = PWM_DRIVER_STATUS_OK,
        .last_fault_manager_status = FAULT_MANAGER_STATUS_OK,
        .last_fault_clear_status = FAULT_MANAGER_STATUS_OK,
        .last_status = APP_STATUS_OK,
        .fast_loop_count = 0U,
        .duty_update_count = 0U,
        .not_ready_count = 0U,
        .error_count = 0U,
        .fault_clear_request_count = 0U,
        .fault_clear_success_count = 0U,
        .fault_clear_blocked_count = 0U,
        .is_initialized = true,
        .is_open_loop_active = false,
        .is_fault_clear_requested = false,
    };

    *self = initialized;
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
        self->is_open_loop_active ||
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
    self->is_open_loop_active = true;
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

    self->is_open_loop_active = false;
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

app_status_t app_motor_fast_loop(app_t *self, app_fast_loop_output_t *output)
{
    app_fast_loop_output_t calculated_output;
    app_open_loop_command_t command;
    bool was_faulted;
    float sin_theta;
    float cos_theta;
    float next_angle_rad;

    if ((self == NULL) || (output == NULL)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return APP_STATUS_INVALID_STATE;
    }

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
    self->last_voltage_sensor_status = voltage_sensor_convert(
        self->config.voltage_sensor,
        calculated_output.raw.dc_link,
        &calculated_output.v_dc
    );
    if (self->last_voltage_sensor_status != VOLTAGE_SENSOR_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_VOLTAGE_SENSOR_ERROR,
            FAULT_MANAGER_FAULT_INVALID_MEASUREMENT
        );
    }

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
    calculated_output.v_alpha_beta.alpha = 0.0f;
    calculated_output.v_alpha_beta.beta = 0.0f;
    calculated_output.duty = app_neutral_duty();
    calculated_output.has_applied_duty = false;

    was_faulted = fault_manager_is_faulted(self->config.fault_manager);
    if (calculated_output.has_valid_phase_current) {
        self->last_fault_manager_status = fault_manager_update_measurements(
            self->config.fault_manager,
            &calculated_output.i_abc,
            calculated_output.v_dc
        );
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
        command = app_get_open_loop_command(self);
        app_process_fault_clear_request(self, &command);

        if (fault_manager_is_faulted(self->config.fault_manager)) {
            self->last_status = APP_STATUS_FAULT_ACTIVE;
            return APP_STATUS_FAULT_ACTIVE;
        }
    }

    if (!calculated_output.has_valid_phase_current ||
        !self->is_open_loop_active) {
        self->last_status = APP_STATUS_OK;
        *output = calculated_output;
        return APP_STATUS_OK;
    }

    command = app_get_open_loop_command(self);
    if (command.voltage_magnitude > 0.0f) {
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
            command.voltage_magnitude * cos_theta;
        calculated_output.v_alpha_beta.beta =
            command.voltage_magnitude * sin_theta;

        self->last_svpwm_status = svpwm_calculate(
            &calculated_output.v_alpha_beta,
            calculated_output.v_dc,
            &calculated_output.duty
        );
        if (self->last_svpwm_status != SVPWM_STATUS_OK) {
            return app_latch_and_stop(
                self,
                APP_STATUS_SVPWM_ERROR,
                FAULT_MANAGER_FAULT_SVPWM
            );
        }
    } else {
        /* 0 V에서는 v_dc가 0이어도 정의되는 neutral duty를 직접 사용한다. */
        self->last_cordic_status = CORDIC_DRIVER_STATUS_OK;
        self->last_svpwm_status = SVPWM_STATUS_OK;
    }

    self->last_pwm_status = pwm_driver_set_duty(
        self->config.pwm_driver,
        &calculated_output.duty
    );
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        return app_latch_and_stop(
            self,
            APP_STATUS_PWM_ERROR,
            FAULT_MANAGER_FAULT_PWM
        );
    }

    next_angle_rad = self->voltage_angle_rad +
        (command.omega_e_rad_s * self->config.sampling_period_s);
    self->voltage_angle_rad = app_wrap_angle_rad(next_angle_rad);
    self->last_v_alpha_beta = calculated_output.v_alpha_beta;
    self->last_duty = calculated_output.duty;
    ++self->duty_update_count;

    calculated_output.has_applied_duty = true;
    self->last_status = APP_STATUS_OK;
    *output = calculated_output;
    return APP_STATUS_OK;
}
