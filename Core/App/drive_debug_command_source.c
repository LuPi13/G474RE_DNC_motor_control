/**
 * @file drive_debug_command_source.c
 * @brief debugger Live Expression용 current/speed command source를 구현한다.
 */

#include "drive_debug_command_source.h"

#include "drive_command.h"

#define DRIVE_DEBUG_COMMAND_SOURCE_MAX_ABS_SPEED_RPM (3000.0f)
#define DRIVE_DEBUG_COMMAND_SOURCE_RPM_TO_RAD_S      (0.104719758f)

volatile drive_debug_command_source_t drive_debug_command_source;

static drive_command_router_t drive_debug_command_router;

static float drive_debug_command_source_limit_speed_rpm(float speed_rpm)
{
    if (speed_rpm > DRIVE_DEBUG_COMMAND_SOURCE_MAX_ABS_SPEED_RPM) {
        return DRIVE_DEBUG_COMMAND_SOURCE_MAX_ABS_SPEED_RPM;
    }
    if (speed_rpm < -DRIVE_DEBUG_COMMAND_SOURCE_MAX_ABS_SPEED_RPM) {
        return -DRIVE_DEBUG_COMMAND_SOURCE_MAX_ABS_SPEED_RPM;
    }
    return speed_rpm;
}

void drive_debug_command_source_init(void)
{
    drive_command_router_init(&drive_debug_command_router);
    drive_debug_command_source = (drive_debug_command_source_t) {
        .requested_mode = DRIVE_DEBUG_COMMAND_MODE_SPEED,
        .requested_i_d_a = 0.0f,
        .requested_i_q_a = 0.0f,
        .requested_speed_rpm = 0.0f,
        .start_requested = false,
        .stop_requested = false,
        .applied_i_dq_a = {0.0f, 0.0f},
        .applied_speed_rpm = 0.0f,
        .is_running = false,
        .last_status = APP_STATUS_OK,
    };
}

void drive_debug_command_source_update(app_t *app)
{
    app_drive_state_t drive_state;
    float requested_speed_rpm;

    if (app == NULL) {
        return;
    }

    drive_state = app->drive_state;
    if ((drive_state != APP_DRIVE_STATE_CURRENT_RUNNING) &&
        (drive_state != APP_DRIVE_STATE_SPEED_RUNNING) &&
        (drive_state != APP_DRIVE_STATE_RAMP_TO_ZERO)) {
        drive_debug_command_source.stop_requested = false;
    }
    requested_speed_rpm = drive_debug_command_source_limit_speed_rpm(
        drive_debug_command_source.requested_speed_rpm
    );

    if ((drive_state == APP_DRIVE_STATE_READY) &&
        drive_debug_command_source.start_requested) {
        drive_command_t command;

        if (drive_debug_command_source.requested_mode ==
            DRIVE_DEBUG_COMMAND_MODE_CURRENT) {
            command = (drive_command_t) {
                .type = DRIVE_COMMAND_START_CURRENT,
                .i_dq_ref = {
                    drive_debug_command_source.requested_i_d_a,
                    drive_debug_command_source.requested_i_q_a,
                },
            };
        } else {
            command = (drive_command_t) {
                .type = DRIVE_COMMAND_START_SPEED,
                .omega_m_ref_rad_s =
                    requested_speed_rpm * DRIVE_DEBUG_COMMAND_SOURCE_RPM_TO_RAD_S,
            };
        }

        drive_debug_command_source.last_status =
            drive_command_router_execute(
                &drive_debug_command_router,
                app,
                &command
            );
        if (drive_debug_command_source.last_status == APP_STATUS_OK) {
            if (command.type == DRIVE_COMMAND_START_CURRENT) {
                drive_debug_command_source.applied_i_dq_a = command.i_dq_ref;
            } else {
                drive_debug_command_source.applied_speed_rpm = requested_speed_rpm;
            }
        }
        drive_debug_command_source.start_requested = false;
        drive_debug_command_source.stop_requested = false;
    } else if (drive_state == APP_DRIVE_STATE_CURRENT_RUNNING) {
        if (drive_debug_command_source.stop_requested) {
            const drive_command_t command = {
                .type = DRIVE_COMMAND_STOP,
            };

            drive_debug_command_source.last_status =
                drive_command_router_execute(
                    &drive_debug_command_router,
                    app,
                    &command
                );
            if (drive_debug_command_source.last_status == APP_STATUS_OK) {
                drive_debug_command_source.applied_i_dq_a = (dq_t){0.0f, 0.0f};
            }
            drive_debug_command_source.requested_i_d_a = 0.0f;
            drive_debug_command_source.requested_i_q_a = 0.0f;
            drive_debug_command_source.stop_requested = false;
        } else if ((drive_debug_command_source.requested_i_d_a !=
                    drive_debug_command_source.applied_i_dq_a.d) ||
                   (drive_debug_command_source.requested_i_q_a !=
                    drive_debug_command_source.applied_i_dq_a.q)) {
            const drive_command_t command = {
                .type = DRIVE_COMMAND_SET_CURRENT,
                .i_dq_ref = {
                    drive_debug_command_source.requested_i_d_a,
                    drive_debug_command_source.requested_i_q_a,
                },
            };

            drive_debug_command_source.last_status =
                drive_command_router_execute(
                    &drive_debug_command_router,
                    app,
                    &command
                );
            if (drive_debug_command_source.last_status == APP_STATUS_OK) {
                drive_debug_command_source.applied_i_dq_a = command.i_dq_ref;
            }
        }
    } else if (drive_state == APP_DRIVE_STATE_SPEED_RUNNING) {
        if (drive_debug_command_source.stop_requested) {
            const drive_command_t command = {
                .type = DRIVE_COMMAND_STOP,
                .omega_m_ref_rad_s = 0.0f,
            };

            drive_debug_command_source.last_status =
                drive_command_router_execute(
                    &drive_debug_command_router,
                    app,
                    &command
                );
            drive_debug_command_source.requested_speed_rpm = 0.0f;
            drive_debug_command_source.stop_requested = false;
        } else if (requested_speed_rpm !=
                   drive_debug_command_source.applied_speed_rpm) {
            const drive_command_t command = {
                .type = DRIVE_COMMAND_SET_SPEED,
                .omega_m_ref_rad_s =
                    requested_speed_rpm * DRIVE_DEBUG_COMMAND_SOURCE_RPM_TO_RAD_S,
            };

            drive_debug_command_source.last_status =
                drive_command_router_execute(
                    &drive_debug_command_router,
                    app,
                    &command
                );
            if (drive_debug_command_source.last_status == APP_STATUS_OK) {
                drive_debug_command_source.applied_speed_rpm = requested_speed_rpm;
            }
        }
    }

    drive_debug_command_source.is_running =
        (app->drive_state == APP_DRIVE_STATE_CURRENT_RUNNING) ||
        (app->drive_state == APP_DRIVE_STATE_SPEED_RUNNING) ||
        (app->drive_state == APP_DRIVE_STATE_RAMP_TO_ZERO);
}
