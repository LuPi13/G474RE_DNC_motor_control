/**
 * @file canopen_service.c
 * @brief CANopenNode 기반 CiA 402 Profile Torque subset 구현.
 * @ingroup app_canopen_service
 */

#include "canopen_service.h"

#include <math.h>
#include <string.h>

#include "../ThirdParty/CANopenNode/CANopen.h"
#include "../Communication/object_dictionary/OD.h"
#include "canopen_fdcan_adapter.h"

#define CANOPEN_SERVICE_PROFILE_TORQUE_MODE       ((int8_t)4)
#define CANOPEN_SERVICE_TORQUE_PER_MILLE          (1000.0f)
#define CANOPEN_SERVICE_NMT_CONTROL \
    (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | \
     CO_ERR_REG_COMMUNICATION)
#define CANOPEN_SERVICE_FIRST_HEARTBEAT_MS         (500U)
#define CANOPEN_SERVICE_SDO_SERVER_TIMEOUT_MS      (1000U)
#define CANOPEN_SERVICE_ERROR_CODE_NONE            (0x0000U)
#define CANOPEN_SERVICE_ERROR_CODE_OVERCURRENT     (0x2310U)
#define CANOPEN_SERVICE_ERROR_CODE_OVERVOLTAGE     (0x3210U)
#define CANOPEN_SERVICE_ERROR_CODE_SOFTWARE        (0xFF01U)
#define CANOPEN_SERVICE_ERROR_CODE_OVERSPEED       (0xFF02U)
#define CANOPEN_SERVICE_ERROR_CODE_RPDO_TIMEOUT    (0x8250U)
#define CANOPEN_SERVICE_OVERSPEED_CLEAR_RATIO       (0.95f)
#define CANOPEN_SERVICE_TARGET_WINDOW_PER_MILLE     (10.0f)

#define CIA402_STATUS_READY_TO_SWITCH_ON            (1U << 0)
#define CIA402_STATUS_SWITCHED_ON                    (1U << 1)
#define CIA402_STATUS_OPERATION_ENABLED              (1U << 2)
#define CIA402_STATUS_FAULT                          (1U << 3)
#define CIA402_STATUS_VOLTAGE_ENABLED                (1U << 4)
#define CIA402_STATUS_QUICK_STOP                     (1U << 5)
#define CIA402_STATUS_SWITCH_ON_DISABLED             (1U << 6)
#define CIA402_STATUS_REMOTE                         (1U << 9)
#define CIA402_STATUS_TARGET_REACHED                 (1U << 10)
#define CIA402_STATUS_INTERNAL_LIMIT                 (1U << 11)

static bool canopen_service_controlword_matches(
    uint16_t controlword,
    uint16_t mask,
    uint16_t value
)
{
    return (controlword & mask) == value;
}

static float canopen_service_absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t canopen_service_clamp_i16(float value)
{
    if (value > 32767.0f) {
        return INT16_MAX;
    }
    if (value < -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)lroundf(value);
}

static uint32_t canopen_service_round_nonnegative_u32(float value)
{
    if ((!isfinite(value)) || (value <= 0.0f)) {
        return 0U;
    }
    if (value >= 4294967040.0f) {
        return UINT32_MAX;
    }
    return (uint32_t)lroundf(value);
}

static bool canopen_service_is_valid_motor_profile(
    const canopen_service_motor_profile_t *profile
)
{
    return (profile != NULL) && (profile->pole_pairs > 0U) &&
        isfinite(profile->permanent_magnet_flux_linkage_wb) &&
        (profile->permanent_magnet_flux_linkage_wb > 0.0f) &&
        isfinite(profile->torque_reference_current_peak_a) &&
        (profile->torque_reference_current_peak_a > 0.0f) &&
        isfinite(profile->maximum_mechanical_speed_rad_s) &&
        (profile->maximum_mechanical_speed_rad_s > 0.0f);
}

static void canopen_service_publish_motor_profile(
    const canopen_service_motor_profile_t *profile
)
{
    const float torque_coefficient_nm_per_a = 1.5f *
        (float)profile->pole_pairs *
        profile->permanent_magnet_flux_linkage_wb;
    const float reference_torque_nm = torque_coefficient_nm_per_a *
        profile->torque_reference_current_peak_a;
    const float rpm_per_rad_s = 60.0f /
        (2.0f * 3.14159265358979323846f);

    OD_RAM.x2000_motorPolePairs = profile->pole_pairs;
    OD_RAM.x2001_permanentMagnetFluxLinkage =
        profile->permanent_magnet_flux_linkage_wb;
    OD_RAM.x2002_ratedPhaseCurrentPeak =
        profile->torque_reference_current_peak_a;
    OD_RAM.x2003_maximumMechanicalSpeed =
        profile->maximum_mechanical_speed_rad_s;
    OD_RAM.x2004_torqueCoefficient = torque_coefficient_nm_per_a;
    OD_RAM.x6075_motorRatedCurrent = canopen_service_round_nonnegative_u32(
        profile->torque_reference_current_peak_a * 1000.0f
    );
    OD_RAM.x6076_motorRatedTorque = canopen_service_round_nonnegative_u32(
        reference_torque_nm * 1000.0f
    );
    OD_RAM.x6080_maximumMotorSpeed = canopen_service_round_nonnegative_u32(
        profile->maximum_mechanical_speed_rad_s * rpm_per_rad_s
    );
}

static void canopen_service_try_apply_motor_profile(
    canopen_service_t *self
)
{
    /*
     * OD 0x2000..0x2004는 현재 motor profile의 read-only mirror다.
     * 통신으로 live motor parameter를 바꾸면 App/FOC state와 어긋날 수 있으므로
     * 명시적인 정지/재초기화 절차가 설계되기 전에는 write를 적용하지 않는다.
     */
    canopen_service_publish_motor_profile(&self->motor_profile);
}

static uint16_t canopen_service_fault_error_code(
    const canopen_service_t *self
)
{
    const fault_manager_t *fault_manager = self->config.fault_manager;
    const fault_manager_fault_mask_t overcurrent_mask =
        FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT |
        FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT |
        FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT;

    if ((fault_manager->latched_fault_mask & overcurrent_mask) != 0U) {
        return CANOPEN_SERVICE_ERROR_CODE_OVERCURRENT;
    }
    if ((fault_manager->latched_fault_mask &
         FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE) != 0U) {
        return CANOPEN_SERVICE_ERROR_CODE_OVERVOLTAGE;
    }
    if (self->is_overspeed_fault_latched) {
        return CANOPEN_SERVICE_ERROR_CODE_OVERSPEED;
    }
    if (self->is_rpdo_timeout_fault_latched) {
        return CANOPEN_SERVICE_ERROR_CODE_RPDO_TIMEOUT;
    }
    if (fault_manager->latched_fault_mask != 0U) {
        return CANOPEN_SERVICE_ERROR_CODE_SOFTWARE;
    }
    return CANOPEN_SERVICE_ERROR_CODE_NONE;
}

static void canopen_service_update_overspeed(
    canopen_service_t *self,
    float omega_e_rad_s,
    bool has_valid_speed
)
{
    const bool fault_reset_rising =
        ((OD_RAM.x6040_controlword & 0x0080U) != 0U) &&
        ((self->previous_controlword & 0x0080U) == 0U);
    const float omega_m_abs_rad_s = has_valid_speed ?
        canopen_service_absolute(omega_e_rad_s) /
            (float)self->motor_profile.pole_pairs : 0.0f;

    if (has_valid_speed &&
        (omega_m_abs_rad_s >
         self->motor_profile.maximum_mechanical_speed_rad_s)) {
        self->is_overspeed_fault_latched = true;
    } else if (self->is_overspeed_fault_latched &&
               fault_reset_rising && has_valid_speed &&
               (omega_m_abs_rad_s <=
                self->motor_profile.maximum_mechanical_speed_rad_s *
                    CANOPEN_SERVICE_OVERSPEED_CLEAR_RATIO)) {
        self->is_overspeed_fault_latched = false;
    }
}

static void canopen_service_update_rpdo_timeout(
    canopen_service_t *self,
    CO_t *canopen
)
{
    const bool fault_reset_rising =
        ((OD_RAM.x6040_controlword & 0x0080U) != 0U) &&
        ((self->previous_controlword & 0x0080U) == 0U);
    const bool is_timed_out = CO_isError(
        canopen->em,
        CO_EM_RPDO_TIME_OUT
    );

    if (is_timed_out) {
        self->is_rpdo_timeout_fault_latched = true;
    } else if (self->is_rpdo_timeout_fault_latched &&
               fault_reset_rising) {
        self->is_rpdo_timeout_fault_latched = false;
    }
}

static void canopen_service_update_emergency(
    canopen_service_t *self,
    CO_t *canopen,
    uint16_t error_code
)
{
    if (error_code == CANOPEN_SERVICE_ERROR_CODE_RPDO_TIMEOUT) {
        if (self->is_emergency_reported) {
            CO_errorReset(
                canopen->em,
                CO_EM_GENERIC_SOFTWARE_ERROR,
                0U
            );
            self->is_emergency_reported = false;
        }
        return;
    }

    const bool should_report =
        (error_code != CANOPEN_SERVICE_ERROR_CODE_NONE) ||
        (self->drive_state == CANOPEN_SERVICE_DRIVE_FAULT);

    if (should_report && !self->is_emergency_reported) {
        const uint16_t emergency_code =
            (error_code != CANOPEN_SERVICE_ERROR_CODE_NONE) ?
                error_code : CANOPEN_SERVICE_ERROR_CODE_SOFTWARE;
        CO_errorReport(
            canopen->em,
            CO_EM_GENERIC_SOFTWARE_ERROR,
            emergency_code,
            self->config.fault_manager->latched_fault_mask
        );
        self->is_emergency_reported = true;
    } else if ((!should_report) && self->is_emergency_reported) {
        CO_errorReset(
            canopen->em,
            CO_EM_GENERIC_SOFTWARE_ERROR,
            0U
        );
        self->is_emergency_reported = false;
    }
}

static uint16_t canopen_service_statusword(
    const canopen_service_t *self,
    bool is_nmt_operational
)
{
    uint16_t statusword = 0U;

    switch (self->drive_state) {
        case CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED:
            statusword = CIA402_STATUS_SWITCH_ON_DISABLED;
            break;
        case CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON:
            statusword = CIA402_STATUS_READY_TO_SWITCH_ON |
                CIA402_STATUS_QUICK_STOP;
            break;
        case CANOPEN_SERVICE_DRIVE_SWITCHED_ON:
            statusword = CIA402_STATUS_READY_TO_SWITCH_ON |
                CIA402_STATUS_SWITCHED_ON |
                CIA402_STATUS_QUICK_STOP;
            break;
        case CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED:
            statusword = CIA402_STATUS_READY_TO_SWITCH_ON |
                CIA402_STATUS_SWITCHED_ON |
                CIA402_STATUS_OPERATION_ENABLED |
                CIA402_STATUS_VOLTAGE_ENABLED |
                CIA402_STATUS_QUICK_STOP;
            break;
        case CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE:
            statusword = CIA402_STATUS_READY_TO_SWITCH_ON |
                CIA402_STATUS_SWITCHED_ON |
                CIA402_STATUS_OPERATION_ENABLED |
                CIA402_STATUS_VOLTAGE_ENABLED;
            break;
        case CANOPEN_SERVICE_DRIVE_FAULT:
        default:
            statusword = CIA402_STATUS_FAULT;
            break;
    }

    if (is_nmt_operational) {
        statusword |= CIA402_STATUS_REMOTE;
    }
    if (self->is_target_reached) {
        statusword |= CIA402_STATUS_TARGET_REACHED;
    }
    if (self->is_internal_limit_active) {
        statusword |= CIA402_STATUS_INTERNAL_LIMIT;
    }
    return statusword;
}

/**
 * @brief CANopen service가 현재 App current lifecycle을 소유하는지 확인한다.
 *
 * @note Pre-operational 또는 switch-on 단계는 App을 시작하지 않았으므로 debugger 등 다른
 *       command source의 current 운전을 중지하지 않는다.
 */
static bool canopen_service_owns_drive(const canopen_service_t *self)
{
    return (self->drive_state == CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED) ||
        (self->drive_state == CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE);
}

static void canopen_service_stop_drive(canopen_service_t *self)
{
    const drive_command_t command = {
        .type = DRIVE_COMMAND_STOP,
        .i_dq_ref = {.d = 0.0f, .q = 0.0f},
        .omega_m_ref_rad_s = 0.0f,
    };

    if (canopen_service_owns_drive(self) &&
        (self->config.app->drive_state ==
         APP_DRIVE_STATE_CURRENT_RUNNING)) {
        (void)drive_command_router_execute(
            self->config.drive_command_router,
            self->config.app,
            &command
        );
    }
    self->commanded_i_q_a = 0.0f;
}

static bool canopen_service_start_drive(canopen_service_t *self)
{
    const drive_command_t command = {
        .type = DRIVE_COMMAND_START_CURRENT,
        .i_dq_ref = {.d = 0.0f, .q = 0.0f},
        .omega_m_ref_rad_s = 0.0f,
    };

    return drive_command_router_execute(
        self->config.drive_command_router,
        self->config.app,
        &command
    ) == APP_STATUS_OK;
}

static void canopen_service_update_drive_state(
    canopen_service_t *self,
    bool is_nmt_operational
)
{
    const uint16_t controlword = OD_RAM.x6040_controlword;
    const bool fault_reset_rising =
        ((controlword & 0x0080U) != 0U) &&
        ((self->previous_controlword & 0x0080U) == 0U);
    const bool is_app_faulted = fault_manager_is_faulted(
        self->config.fault_manager
    );
    const bool is_faulted = is_app_faulted ||
        self->is_overspeed_fault_latched ||
        self->is_rpdo_timeout_fault_latched;

    self->previous_controlword = controlword;
    if (is_faulted) {
        const drive_command_t clear_command = {
            .type = DRIVE_COMMAND_REQUEST_FAULT_CLEAR,
            .i_dq_ref = {.d = 0.0f, .q = 0.0f},
            .omega_m_ref_rad_s = 0.0f,
        };

        canopen_service_stop_drive(self);
        self->drive_state = CANOPEN_SERVICE_DRIVE_FAULT;
        if (fault_reset_rising && is_app_faulted &&
            (drive_command_router_execute(
                 self->config.drive_command_router,
                 self->config.app,
                 &clear_command
             ) == APP_STATUS_OK)) {
            self->is_fault_reset_pending = true;
        }
        return;
    }

    if (self->drive_state == CANOPEN_SERVICE_DRIVE_FAULT) {
        if (self->config.app->drive_state == APP_DRIVE_STATE_READY) {
            self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
            self->is_fault_reset_pending = false;
        } else if (self->is_fault_reset_pending) {
            const drive_command_t recover_command = {
                .type = DRIVE_COMMAND_RECOVER_FAULT,
                .i_dq_ref = {.d = 0.0f, .q = 0.0f},
                .omega_m_ref_rad_s = 0.0f,
            };

            if (drive_command_router_execute(
                    self->config.drive_command_router,
                    self->config.app,
                    &recover_command
                ) == APP_STATUS_OK) {
                self->drive_state =
                    CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
                self->is_fault_reset_pending = false;
            }
        }
        return;
    }

    if (!is_nmt_operational) {
        canopen_service_stop_drive(self);
        self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
        return;
    }

    switch (self->drive_state) {
    case CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED:
        if (canopen_service_controlword_matches(
                controlword, 0x0087U, 0x0006U)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON;
        }
        break;

    case CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON:
        if (canopen_service_controlword_matches(
                controlword, 0x0082U, 0x0000U)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x008FU, 0x0007U)) {
            self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCHED_ON;
        }
        break;

    case CANOPEN_SERVICE_DRIVE_SWITCHED_ON:
        if (canopen_service_controlword_matches(
                controlword, 0x0082U, 0x0000U)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x0087U, 0x0006U)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON;
        } else if ((OD_RAM.x6060_modesOfOperation ==
                    CANOPEN_SERVICE_PROFILE_TORQUE_MODE) &&
                   canopen_service_controlword_matches(
                       controlword, 0x008FU, 0x000FU)) {
            if (canopen_service_start_drive(self)) {
                self->drive_state =
                    CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED;
            } else {
                ++self->drive_error_count;
                self->drive_state = CANOPEN_SERVICE_DRIVE_FAULT;
            }
        }
        break;

    case CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED:
        if (OD_RAM.x6060_modesOfOperation !=
            CANOPEN_SERVICE_PROFILE_TORQUE_MODE) {
            canopen_service_stop_drive(self);
            self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCHED_ON;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x0086U, 0x0002U)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x0082U, 0x0000U)) {
            canopen_service_stop_drive(self);
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x008FU, 0x0007U)) {
            canopen_service_stop_drive(self);
            self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCHED_ON;
        } else if (canopen_service_controlword_matches(
                       controlword, 0x0087U, 0x0006U)) {
            canopen_service_stop_drive(self);
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON;
        }
        break;

    case CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE:
        if (canopen_service_controlword_matches(
                controlword, 0x0082U, 0x0000U)) {
            canopen_service_stop_drive(self);
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
        } else if ((OD_RAM.x6060_modesOfOperation ==
                    CANOPEN_SERVICE_PROFILE_TORQUE_MODE) &&
                   canopen_service_controlword_matches(
                       controlword, 0x008FU, 0x000FU)) {
            self->drive_state =
                CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED;
        }
        break;

    case CANOPEN_SERVICE_DRIVE_FAULT:
    default:
        self->drive_state = CANOPEN_SERVICE_DRIVE_FAULT;
        break;
    }
}

static bool canopen_service_update_torque_command(
    canopen_service_t *self,
    uint32_t elapsed_us
)
{
    float target_per_mille = (float)OD_RAM.x6071_targetTorque;
    float maximum_per_mille = (float)OD_RAM.x6072_maximumTorque;
    float requested_i_q_a;
    float max_step_a;
    drive_command_t command;

    if (maximum_per_mille > CANOPEN_SERVICE_TORQUE_PER_MILLE) {
        maximum_per_mille = CANOPEN_SERVICE_TORQUE_PER_MILLE;
    }
    if (target_per_mille > maximum_per_mille) {
        target_per_mille = maximum_per_mille;
    } else if (target_per_mille < -maximum_per_mille) {
        target_per_mille = -maximum_per_mille;
    }

    requested_i_q_a = self->motor_profile.torque_reference_current_peak_a *
        target_per_mille / CANOPEN_SERVICE_TORQUE_PER_MILLE;
    if (self->drive_state == CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE) {
        requested_i_q_a = 0.0f;
    }

    max_step_a = self->motor_profile.torque_reference_current_peak_a *
        (float)OD_RAM.x6087_torqueSlope *
        ((float)elapsed_us * 1.0e-6f) /
        CANOPEN_SERVICE_TORQUE_PER_MILLE;
    if (max_step_a <= 0.0f) {
        self->commanded_i_q_a = requested_i_q_a;
    } else if (requested_i_q_a > self->commanded_i_q_a + max_step_a) {
        self->commanded_i_q_a += max_step_a;
    } else if (requested_i_q_a < self->commanded_i_q_a - max_step_a) {
        self->commanded_i_q_a -= max_step_a;
    } else {
        self->commanded_i_q_a = requested_i_q_a;
    }

    self->is_internal_limit_active =
        canopen_service_absolute(self->commanded_i_q_a) >
        self->config.app->config.motor_control->config
            .current_reference_magnitude_limit;
    command = (drive_command_t) {
        .type = DRIVE_COMMAND_SET_CURRENT,
        .i_dq_ref = {.d = 0.0f, .q = self->commanded_i_q_a},
        .omega_m_ref_rad_s = 0.0f,
    };
    return drive_command_router_execute(
        self->config.drive_command_router,
        self->config.app,
        &command
    ) == APP_STATUS_OK;
}

canopen_service_status_t canopen_service_init(
    canopen_service_t *self,
    const canopen_service_config_t *config,
    const canopen_service_motor_profile_t *motor_profile
)
{
    CO_t *canopen;
    CO_ReturnError_t error;
    uint32_t error_info = 0U;

    if ((self == NULL) || (config == NULL) ||
        (motor_profile == NULL) || (config->fdcan_driver == NULL) ||
        (config->app == NULL) || (config->drive_command_router == NULL) ||
        (config->fault_manager == NULL) || (config->node_id < 1U) ||
        (config->node_id > 127U) || (config->bit_rate_kbit_s != 500U)) {
        return CANOPEN_SERVICE_STATUS_INVALID_ARGUMENT;
    }
    if ((!config->app->is_initialized) ||
        (!config->fdcan_driver->is_initialized) ||
        config->app->config.pwm_driver->is_enabled ||
        (!canopen_service_is_valid_motor_profile(motor_profile))) {
        return CANOPEN_SERVICE_STATUS_INVALID_STATE;
    }

    (void)memset(self, 0, sizeof(*self));
    self->config = *config;
    self->motor_profile = *motor_profile;
    self->drive_state = CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED;
    canopen_service_publish_motor_profile(&self->motor_profile);
    OD_RAM.x6040_controlword = 0U;
    OD_RAM.x6041_statusword = CIA402_STATUS_SWITCH_ON_DISABLED |
        CIA402_STATUS_QUICK_STOP;
    OD_RAM.x6060_modesOfOperation = 0;
    OD_RAM.x6061_modesOfOperationDisplay = 0;
    OD_RAM.x6071_targetTorque = 0;
    OD_RAM.x6077_torqueActualValue = 0;

    canopen = CO_new(NULL, NULL);
    if (canopen == NULL) {
        return CANOPEN_SERVICE_STATUS_STACK_ERROR;
    }
    error = CO_CANinit(
        canopen,
        config->fdcan_driver,
        config->bit_rate_kbit_s
    );
    if (error != CO_ERROR_NO) {
        return CANOPEN_SERVICE_STATUS_STACK_ERROR;
    }
    error = CO_CANopenInit(
        canopen,
        NULL,
        NULL,
        OD,
        NULL,
        CANOPEN_SERVICE_NMT_CONTROL,
        CANOPEN_SERVICE_FIRST_HEARTBEAT_MS,
        CANOPEN_SERVICE_SDO_SERVER_TIMEOUT_MS,
        0U,
        false,
        config->node_id,
        &error_info
    );
    if (error != CO_ERROR_NO) {
        return CANOPEN_SERVICE_STATUS_STACK_ERROR;
    }
    error = CO_CANopenInitPDO(
        canopen,
        canopen->em,
        OD,
        config->node_id,
        &error_info
    );
    if (error != CO_ERROR_NO) {
        return CANOPEN_SERVICE_STATUS_STACK_ERROR;
    }

    CO_CANsetNormalMode(canopen->CANmodule);
    if (!canopen->CANmodule->CANnormal) {
        return CANOPEN_SERVICE_STATUS_STACK_ERROR;
    }

    self->canopen_instance = canopen;
    self->is_initialized = true;
    self->last_status = CANOPEN_SERVICE_STATUS_OK;
    return CANOPEN_SERVICE_STATUS_OK;
}

canopen_service_status_t canopen_service_process(
    canopen_service_t *self,
    uint32_t elapsed_us,
    float i_q_feedback_a,
    bool has_valid_i_q_feedback,
    float omega_e_rad_s,
    bool has_valid_speed
)
{
    CO_t *canopen;
    CO_NMT_reset_cmd_t reset_command;
    bool is_nmt_operational;
    float actual_per_mille = 0.0f;

    if ((self == NULL) || (elapsed_us == 0U) ||
        (!isfinite(i_q_feedback_a)) || (!isfinite(omega_e_rad_s))) {
        return CANOPEN_SERVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return CANOPEN_SERVICE_STATUS_INVALID_STATE;
    }

    canopen = (CO_t *)self->canopen_instance;
    reset_command = CO_process(canopen, false, elapsed_us, NULL);
    if (reset_command != CO_RESET_NOT) {
        self->last_status = CANOPEN_SERVICE_STATUS_RESET_REQUESTED;
        return CANOPEN_SERVICE_STATUS_RESET_REQUESTED;
    }

    is_nmt_operational =
        canopen->NMT->operatingState == CO_NMT_OPERATIONAL;
    CO_process_RPDO(canopen, false, elapsed_us, NULL);

    canopen_service_try_apply_motor_profile(self);
    OD_RAM.x6061_modesOfOperationDisplay =
        (OD_RAM.x6060_modesOfOperation ==
         CANOPEN_SERVICE_PROFILE_TORQUE_MODE) ?
            CANOPEN_SERVICE_PROFILE_TORQUE_MODE : 0;
    canopen_service_update_overspeed(
        self,
        omega_e_rad_s,
        has_valid_speed
    );
    canopen_service_update_rpdo_timeout(self, canopen);
    canopen_service_update_drive_state(self, is_nmt_operational);

    if ((self->drive_state == CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED) ||
        (self->drive_state == CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE)) {
        if (!canopen_service_update_torque_command(self, elapsed_us)) {
            canopen_service_stop_drive(self);
            self->drive_state = CANOPEN_SERVICE_DRIVE_FAULT;
            ++self->drive_error_count;
            self->last_status = CANOPEN_SERVICE_STATUS_DRIVE_ERROR;
        }
    } else {
        self->commanded_i_q_a = 0.0f;
        self->is_internal_limit_active = false;
        self->is_target_reached = false;
    }

    OD_RAM.x603F_errorCode = canopen_service_fault_error_code(self);
    if ((OD_RAM.x603F_errorCode == CANOPEN_SERVICE_ERROR_CODE_NONE) &&
        (self->drive_state == CANOPEN_SERVICE_DRIVE_FAULT)) {
        OD_RAM.x603F_errorCode = CANOPEN_SERVICE_ERROR_CODE_SOFTWARE;
    }
    canopen_service_update_emergency(
        self,
        canopen,
        OD_RAM.x603F_errorCode
    );
    if (has_valid_i_q_feedback) {
        actual_per_mille = i_q_feedback_a /
            self->motor_profile.torque_reference_current_peak_a *
            CANOPEN_SERVICE_TORQUE_PER_MILLE;
    }
    self->is_target_reached =
        has_valid_i_q_feedback &&
        ((self->drive_state == CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED) ||
         (self->drive_state == CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE)) &&
        (canopen_service_absolute(
            actual_per_mille -
            (self->commanded_i_q_a /
             self->motor_profile.torque_reference_current_peak_a *
             CANOPEN_SERVICE_TORQUE_PER_MILLE)
        ) <= CANOPEN_SERVICE_TARGET_WINDOW_PER_MILLE);
    OD_RAM.x6077_torqueActualValue = canopen_service_clamp_i16(
        actual_per_mille
    );
    OD_RAM.x6041_statusword = canopen_service_statusword(
        self,
        is_nmt_operational
    );

    if (is_nmt_operational) {
        CO_process_TPDO(canopen, false, elapsed_us, NULL);
    }

    ++self->process_count;
    if (self->last_status != CANOPEN_SERVICE_STATUS_DRIVE_ERROR) {
        self->last_status = CANOPEN_SERVICE_STATUS_OK;
    }
    return self->last_status;
}
