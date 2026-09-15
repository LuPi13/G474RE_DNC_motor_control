/**
 * @file drive_command.c
 * @brief 외부 drive command를 App lifecycle API로 전달한다.
 */

#include "drive_command.h"

#include <math.h>

void drive_command_router_init(drive_command_router_t *self)
{
    if (self == NULL) {
        return;
    }

    *self = (drive_command_router_t) {
        .last_status = APP_STATUS_OK,
        .accepted_count = 0U,
        .rejected_count = 0U,
    };
}

app_status_t drive_command_router_execute(
    drive_command_router_t *self,
    app_t *app,
    const drive_command_t *command
)
{
    app_status_t status;

    if ((self == NULL) || (app == NULL) || (command == NULL)) {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    switch (command->type) {
    case DRIVE_COMMAND_START_SPEED:
        if (!isfinite(command->omega_m_ref_rad_s)) {
            status = APP_STATUS_INVALID_ARGUMENT;
        } else {
            const app_speed_command_t speed_command = {
                .omega_m_ref_rad_s = command->omega_m_ref_rad_s,
            };
            status = app_drive_start_speed(app, &speed_command);
        }
        break;

    case DRIVE_COMMAND_SET_SPEED:
        if (!isfinite(command->omega_m_ref_rad_s)) {
            status = APP_STATUS_INVALID_ARGUMENT;
        } else if (app->drive_state != APP_DRIVE_STATE_SPEED_RUNNING) {
            status = APP_STATUS_INVALID_STATE;
        } else {
            const app_speed_command_t speed_command = {
                .omega_m_ref_rad_s = command->omega_m_ref_rad_s,
            };
            status = app_set_speed_command(app, &speed_command);
        }
        break;

    case DRIVE_COMMAND_STOP:
        status = app_drive_request_speed_stop(app);
        break;

    case DRIVE_COMMAND_RECOVER_FAULT:
        status = app_drive_recover_after_fault(app);
        break;

    default:
        status = APP_STATUS_INVALID_ARGUMENT;
        break;
    }

    self->last_status = status;
    if (status == APP_STATUS_OK) {
        ++self->accepted_count;
    } else {
        ++self->rejected_count;
    }
    return status;
}
