/**
 * @file fault_manager.c
 * @brief Motor drive의 software fault 감시와 latch 상태를 구현한다.
 */

#include "fault_manager.h"

#include <float.h>
#include <stddef.h>

static bool fault_manager_float_is_finite(float value)
{
    return (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool fault_manager_abs_trip(float value, float trip_threshold)
{
    return (value >= trip_threshold) || (value <= -trip_threshold);
}

static bool fault_manager_abs_remains_active(float value, float clear_threshold)
{
    return (value > clear_threshold) || (value < -clear_threshold);
}

static void fault_manager_capture_first_fault(
    fault_manager_t *self,
    fault_manager_fault_mask_t fault_mask
)
{
    self->first_fault_snapshot.fault_mask = fault_mask;
    self->first_fault_snapshot.i_abc = self->latest_i_abc;
    self->first_fault_snapshot.v_dc = self->latest_v_dc;
    self->first_fault_snapshot.has_valid_measurement =
        self->has_valid_measurement;
}

static void fault_manager_latch_internal(
    fault_manager_t *self,
    fault_manager_fault_mask_t fault_mask
)
{
    if (self->latched_fault_mask == FAULT_MANAGER_FAULT_NONE) {
        fault_manager_capture_first_fault(self, fault_mask);
        ++self->trip_count;
    }

    self->latched_fault_mask |= fault_mask;
}

static bool fault_manager_update_current_condition(
    float current_a,
    bool was_active,
    const fault_manager_config_t *config
)
{
    if (was_active) {
        return fault_manager_abs_remains_active(
            current_a,
            config->phase_current_clear_abs_a
        );
    }

    return fault_manager_abs_trip(
        current_a,
        config->phase_current_trip_abs_a
    );
}

static bool fault_manager_update_voltage_condition(
    float v_dc,
    bool was_active,
    const fault_manager_config_t *config
)
{
    if (was_active) {
        return v_dc > config->dc_link_overvoltage_clear_v;
    }

    return v_dc >= config->dc_link_overvoltage_trip_v;
}

fault_manager_status_t fault_manager_init(
    fault_manager_t *self,
    const fault_manager_config_t *config
)
{
    if ((self == NULL) || (config == NULL)) {
        return FAULT_MANAGER_STATUS_INVALID_ARGUMENT;
    }

    if ((!fault_manager_float_is_finite(config->phase_current_trip_abs_a)) ||
        (!fault_manager_float_is_finite(config->phase_current_clear_abs_a)) ||
        (!fault_manager_float_is_finite(config->dc_link_overvoltage_trip_v)) ||
        (!fault_manager_float_is_finite(config->dc_link_overvoltage_clear_v)) ||
        (config->phase_current_trip_abs_a <= 0.0f) ||
        (config->phase_current_clear_abs_a < 0.0f) ||
        (config->phase_current_clear_abs_a >= config->phase_current_trip_abs_a) ||
        (config->dc_link_overvoltage_trip_v <= 0.0f) ||
        (config->dc_link_overvoltage_clear_v < 0.0f) ||
        (config->dc_link_overvoltage_clear_v >= config->dc_link_overvoltage_trip_v)) {
        return FAULT_MANAGER_STATUS_INVALID_CONFIG;
    }

    const fault_manager_t initialized = {
        .config = *config,
        .active_fault_mask = FAULT_MANAGER_FAULT_NONE,
        .latched_fault_mask = FAULT_MANAGER_FAULT_NONE,
        .first_fault_snapshot = {
            .fault_mask = FAULT_MANAGER_FAULT_NONE,
            .i_abc = {0.0f, 0.0f, 0.0f},
            .v_dc = 0.0f,
            .has_valid_measurement = false,
        },
        .latest_i_abc = {0.0f, 0.0f, 0.0f},
        .latest_v_dc = 0.0f,
        .has_valid_measurement = false,
        .last_status = FAULT_MANAGER_STATUS_OK,
        .trip_count = 0U,
        .clear_count = 0U,
        .blocked_clear_count = 0U,
        .is_initialized = true,
    };

    *self = initialized;
    return FAULT_MANAGER_STATUS_OK;
}

fault_manager_status_t fault_manager_update_measurements(
    fault_manager_t *self,
    const abc_t *i_abc,
    float v_dc
)
{
    fault_manager_fault_mask_t active_mask;

    if ((self == NULL) || (i_abc == NULL)) {
        return FAULT_MANAGER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FAULT_MANAGER_STATUS_INVALID_STATE;
    }

    active_mask = self->active_fault_mask &
        ~FAULT_MANAGER_MEASUREMENT_FAULT_MASK;

    if ((!fault_manager_float_is_finite(i_abc->a)) ||
        (!fault_manager_float_is_finite(i_abc->b)) ||
        (!fault_manager_float_is_finite(i_abc->c)) ||
        (!fault_manager_float_is_finite(v_dc))) {
        self->has_valid_measurement = false;
        active_mask |= FAULT_MANAGER_FAULT_INVALID_MEASUREMENT;
        self->active_fault_mask = active_mask;
        fault_manager_latch_internal(
            self,
            FAULT_MANAGER_FAULT_INVALID_MEASUREMENT
        );
        self->last_status = FAULT_MANAGER_STATUS_OK;
        return FAULT_MANAGER_STATUS_OK;
    }

    self->latest_i_abc = *i_abc;
    self->latest_v_dc = v_dc;
    self->has_valid_measurement = true;

    if (fault_manager_update_current_condition(
            i_abc->a,
            (self->active_fault_mask &
                FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT) != 0U,
            &self->config)) {
        active_mask |= FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT;
    }

    if (fault_manager_update_current_condition(
            i_abc->b,
            (self->active_fault_mask &
                FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT) != 0U,
            &self->config)) {
        active_mask |= FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT;
    }

    if (fault_manager_update_current_condition(
            i_abc->c,
            (self->active_fault_mask &
                FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT) != 0U,
            &self->config)) {
        active_mask |= FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT;
    }

    if (fault_manager_update_voltage_condition(
            v_dc,
            (self->active_fault_mask &
                FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE) != 0U,
            &self->config)) {
        active_mask |= FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE;
    }

    self->active_fault_mask = active_mask;
    if (active_mask != FAULT_MANAGER_FAULT_NONE) {
        fault_manager_latch_internal(self, active_mask);
    }

    self->last_status = FAULT_MANAGER_STATUS_OK;
    return FAULT_MANAGER_STATUS_OK;
}

fault_manager_status_t fault_manager_update_dc_link_voltage(
    fault_manager_t *self,
    float v_dc
)
{
    if (self == NULL) {
        return FAULT_MANAGER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return FAULT_MANAGER_STATUS_INVALID_STATE;
    }

    if (!fault_manager_float_is_finite(v_dc)) {
        self->has_valid_measurement = false;
        self->active_fault_mask |= FAULT_MANAGER_FAULT_INVALID_MEASUREMENT;
        fault_manager_latch_internal(
            self,
            FAULT_MANAGER_FAULT_INVALID_MEASUREMENT
        );
        self->last_status = FAULT_MANAGER_STATUS_OK;
        return FAULT_MANAGER_STATUS_OK;
    }

    const bool was_overvoltage_active =
        (self->active_fault_mask &
            FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE) != 0U;
    self->latest_v_dc = v_dc;
    self->has_valid_measurement = false;
    self->active_fault_mask &= ~FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE;
    if (fault_manager_update_voltage_condition(
            v_dc,
            was_overvoltage_active,
            &self->config)) {
        self->active_fault_mask |= FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE;
        fault_manager_latch_internal(
            self,
            FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE
        );
    }

    self->last_status = FAULT_MANAGER_STATUS_OK;
    return FAULT_MANAGER_STATUS_OK;
}

fault_manager_status_t fault_manager_latch(
    fault_manager_t *self,
    fault_manager_fault_mask_t fault_mask
)
{
    if ((self == NULL) ||
        (fault_mask == FAULT_MANAGER_FAULT_NONE) ||
        ((fault_mask & ~FAULT_MANAGER_FAULT_ALL_MASK) != 0U)) {
        return FAULT_MANAGER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FAULT_MANAGER_STATUS_INVALID_STATE;
    }

    fault_manager_latch_internal(self, fault_mask);
    self->last_status = FAULT_MANAGER_STATUS_OK;
    return FAULT_MANAGER_STATUS_OK;
}

fault_manager_status_t fault_manager_clear(
    fault_manager_t *self,
    bool is_pwm_disabled,
    bool is_command_zero
)
{
    if (self == NULL) {
        return FAULT_MANAGER_STATUS_INVALID_ARGUMENT;
    }

    if ((!self->is_initialized) ||
        (self->latched_fault_mask == FAULT_MANAGER_FAULT_NONE)) {
        return FAULT_MANAGER_STATUS_INVALID_STATE;
    }

    if ((!is_pwm_disabled) ||
        (!is_command_zero) ||
        (!self->has_valid_measurement) ||
        (self->active_fault_mask != FAULT_MANAGER_FAULT_NONE)) {
        ++self->blocked_clear_count;
        self->last_status = FAULT_MANAGER_STATUS_CLEAR_BLOCKED;
        return FAULT_MANAGER_STATUS_CLEAR_BLOCKED;
    }

    self->latched_fault_mask = FAULT_MANAGER_FAULT_NONE;
    ++self->clear_count;
    self->last_status = FAULT_MANAGER_STATUS_OK;
    return FAULT_MANAGER_STATUS_OK;
}

bool fault_manager_is_faulted(const fault_manager_t *self)
{
    return (self != NULL) &&
        self->is_initialized &&
        (self->latched_fault_mask != FAULT_MANAGER_FAULT_NONE);
}
