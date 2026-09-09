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

static app_status_t app_fail_stop(app_t *self, app_status_t status)
{
    ++self->error_count;
    self->last_status = status;

    if (!self->is_open_loop_active) {
        return status;
    }

    self->is_open_loop_active = false;
    __DMB();

    self->last_pwm_status = pwm_driver_disable(self->config.pwm_driver);
    if (self->last_pwm_status != PWM_DRIVER_STATUS_OK) {
        self->last_status = APP_STATUS_PWM_ERROR;
        return APP_STATUS_PWM_ERROR;
    }

    return status;
}

app_status_t app_init(app_t *self, const app_config_t *config)
{
    if ((self == NULL) ||
        (config == NULL) ||
        (config->adc_driver == NULL) ||
        (config->pwm_driver == NULL) ||
        (!app_float_is_finite(config->sampling_period_s)) ||
        (config->sampling_period_s <= 0.0f) ||
        (!app_float_is_finite(config->initial_voltage_angle_rad)) ||
        (config->initial_voltage_angle_rad < 0.0f) ||
        (config->initial_voltage_angle_rad >= APP_TWO_PI_RAD)) {
        return APP_STATUS_INVALID_ARGUMENT;
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
        .last_cordic_status = CORDIC_DRIVER_STATUS_OK,
        .last_svpwm_status = SVPWM_STATUS_OK,
        .last_pwm_status = PWM_DRIVER_STATUS_OK,
        .last_status = APP_STATUS_OK,
        .fast_loop_count = 0U,
        .duty_update_count = 0U,
        .not_ready_count = 0U,
        .error_count = 0U,
        .is_initialized = true,
        .is_open_loop_active = false,
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

app_status_t app_start_open_loop(app_t *self)
{
    if (self == NULL) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (!self->config.pwm_driver->is_initialized)) {
        return APP_STATUS_INVALID_STATE;
    }

    __DMB();
    self->is_open_loop_active = true;
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
        ++self->error_count;
        self->last_status = APP_STATUS_PWM_ERROR;
        (void)pwm_driver_disable(self->config.pwm_driver);
        return APP_STATUS_PWM_ERROR;
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
    return app_fail_stop(self, APP_STATUS_ADC_ERROR);
}

app_status_t app_motor_fast_loop(app_t *self, app_fast_loop_output_t *output)
{
    app_fast_loop_output_t calculated_output;
    app_open_loop_command_t command;
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
        return app_fail_stop(self, APP_STATUS_ADC_ERROR);
    }

    self->last_adc_status = adc_driver_convert(
        self->config.adc_driver,
        &calculated_output.raw,
        &calculated_output.i_abc,
        &calculated_output.v_dc
    );
    if (self->last_adc_status != ADC_DRIVER_STATUS_OK) {
        return app_fail_stop(self, APP_STATUS_ADC_ERROR);
    }

    ++self->fast_loop_count;
    calculated_output.voltage_angle_rad = self->voltage_angle_rad;
    calculated_output.v_alpha_beta.alpha = 0.0f;
    calculated_output.v_alpha_beta.beta = 0.0f;
    calculated_output.duty = app_neutral_duty();
    calculated_output.has_applied_duty = false;

    if (!self->is_open_loop_active) {
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
            return app_fail_stop(self, APP_STATUS_CORDIC_ERROR);
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
            return app_fail_stop(self, APP_STATUS_SVPWM_ERROR);
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
        return app_fail_stop(self, APP_STATUS_PWM_ERROR);
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
