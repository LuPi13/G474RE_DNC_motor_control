/**
 * @file drive_parameter_manager.c
 * @brief Drive parameter Flash lifecycle과 debugger mailbox 구현.
 */

#include "drive_parameter_manager.h"

#include <string.h>

#include "flash_storage_driver.h"

volatile drive_parameter_debug_t drive_parameter_debug;

static void drive_parameter_manager_refresh_dirty(void)
{
    drive_parameter_debug.is_dirty = !drive_parameters_are_equal(
        (const drive_parameters_t *)&drive_parameter_debug.working_set,
        (const drive_parameters_t *)&drive_parameter_debug.active_set
    );
}

static void drive_parameter_manager_set_status(drive_parameter_manager_status_t status)
{
    drive_parameter_debug.last_status = (uint32_t)status;
    drive_parameter_manager_refresh_dirty();
}

static bool drive_parameter_manager_read_latest(drive_parameters_t *parameters, drive_parameter_record_selection_t *selection)
{
    uint8_t slot_a[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    uint8_t slot_b[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    if ((flash_storage_driver_read(FLASH_STORAGE_SLOT_A, 0U, slot_a, sizeof(slot_a)) != FLASH_STORAGE_STATUS_OK) ||
        (flash_storage_driver_read(FLASH_STORAGE_SLOT_B, 0U, slot_b, sizeof(slot_b)) != FLASH_STORAGE_STATUS_OK)) return false;
    return drive_parameter_record_select_latest(slot_a, slot_b, parameters, selection) == DRIVE_PARAMETER_RECORD_STATUS_OK;
}

void drive_parameter_manager_init(drive_parameter_manager_t *self, const drive_parameters_t *defaults)
{
    drive_parameters_t loaded;
    drive_parameter_record_selection_t selection;
    if ((self == NULL) || (defaults == NULL) || !drive_parameters_is_valid(defaults)) return;
    memset(self, 0, sizeof(*self));
    self->defaults = *defaults;
    drive_parameter_debug = (drive_parameter_debug_t){0};
    if (drive_parameter_manager_read_latest(&loaded, &selection)) {
        drive_parameter_debug.working_set = loaded;
        drive_parameter_debug.active_set = loaded;
        drive_parameter_debug.active_generation = selection.generation;
        drive_parameter_debug.is_flash_record_valid = true;
        self->active_slot = selection.slot;
        drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ACTIVE);
    } else {
        drive_parameter_debug.working_set = *defaults;
        drive_parameter_debug.active_set = *defaults;
        self->active_slot = DRIVE_PARAMETER_SLOT_NONE;
        drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_DEFAULTS_ACTIVE);
    }
    self->is_initialized = true;
}

static bool drive_parameter_manager_save_active(drive_parameter_manager_t *self)
{
    uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    const drive_parameter_slot_t target = (self->active_slot == DRIVE_PARAMETER_SLOT_A) ?
        DRIVE_PARAMETER_SLOT_B : DRIVE_PARAMETER_SLOT_A;
    const uint32_t next_generation = drive_parameter_debug.active_generation + 1U;
    if (!drive_parameter_record_encode((const drive_parameters_t *)&drive_parameter_debug.active_set, next_generation, record)) return false;
    if ((flash_storage_driver_erase((flash_storage_slot_t)target) != FLASH_STORAGE_STATUS_OK) ||
        (flash_storage_driver_program((flash_storage_slot_t)target, 0U, record, DRIVE_PARAMETER_RECORD_BODY_SIZE_BYTES) != FLASH_STORAGE_STATUS_OK) ||
        (flash_storage_driver_program((flash_storage_slot_t)target, DRIVE_PARAMETER_RECORD_BODY_SIZE_BYTES, &record[DRIVE_PARAMETER_RECORD_BODY_SIZE_BYTES], 8U) != FLASH_STORAGE_STATUS_OK)) return false;
    self->active_slot = target;
    drive_parameter_debug.active_generation = next_generation;
    drive_parameter_debug.is_flash_record_valid = true;
    return true;
}

void drive_parameter_manager_process(drive_parameter_manager_t *self, bool is_safe_state, drive_parameter_apply_callback_t apply_callback, void *apply_context)
{
    if ((self == NULL) || !self->is_initialized) return;
    drive_parameter_manager_refresh_dirty();
    if (drive_parameter_debug.load_request != self->handled_load_request) {
        drive_parameters_t loaded; drive_parameter_record_selection_t selection;
        self->handled_load_request = drive_parameter_debug.load_request;
        if (drive_parameter_manager_read_latest(&loaded, &selection)) {
            drive_parameter_debug.working_set = loaded;
            drive_parameter_debug.active_generation = selection.generation;
            drive_parameter_debug.is_flash_record_valid = true;
            self->active_slot = selection.slot;
            drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_WORKING_SET_LOADED);
        } else drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ERROR);
        return;
    }
    if (drive_parameter_debug.defaults_request != self->handled_defaults_request) {
        self->handled_defaults_request = drive_parameter_debug.defaults_request;
        drive_parameter_debug.working_set = self->defaults;
        drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_WORKING_SET_DEFAULTED);
        return;
    }
    if (drive_parameter_debug.apply_request != self->handled_apply_request) {
        if (!is_safe_state) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_PENDING_SAFE_STATE); return; }
        self->handled_apply_request = drive_parameter_debug.apply_request;
        if (!drive_parameters_is_valid((const drive_parameters_t *)&drive_parameter_debug.working_set)) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_INVALID_WORKING_SET); return; }
        if ((apply_callback == NULL) || !apply_callback(apply_context, (const drive_parameters_t *)&drive_parameter_debug.working_set)) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_APPLY_FAILED); return; }
        drive_parameter_debug.active_set = drive_parameter_debug.working_set;
        drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_APPLIED);
        return;
    }
    if (drive_parameter_debug.save_request != self->handled_save_request) {
        if (!is_safe_state) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_PENDING_SAFE_STATE); return; }
        self->handled_save_request = drive_parameter_debug.save_request;
        if (!drive_parameters_are_equal((const drive_parameters_t *)&drive_parameter_debug.working_set, (const drive_parameters_t *)&drive_parameter_debug.active_set)) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_DIRTY_NOT_APPLIED); return; }
        drive_parameter_manager_set_status(drive_parameter_manager_save_active(self) ? DRIVE_PARAMETER_MANAGER_STATUS_SAVED : DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ERROR);
        return;
    }
    if (drive_parameter_debug.erase_request != self->handled_erase_request) {
        if (!is_safe_state) { drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_PENDING_SAFE_STATE); return; }
        self->handled_erase_request = drive_parameter_debug.erase_request;
        if ((flash_storage_driver_erase(FLASH_STORAGE_SLOT_A) == FLASH_STORAGE_STATUS_OK) && (flash_storage_driver_erase(FLASH_STORAGE_SLOT_B) == FLASH_STORAGE_STATUS_OK)) { self->active_slot = DRIVE_PARAMETER_SLOT_NONE; drive_parameter_debug.is_flash_record_valid = false; drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_ERASED); }
        else drive_parameter_manager_set_status(DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ERROR);
    }
}
