/**
 * @file drive_parameter_manager.h
 * @brief Debugger 요청과 Flash slot을 연결하는 drive parameter lifecycle API.
 * @ingroup app_drive_parameters
 */

#ifndef APP_DRIVE_PARAMETER_MANAGER_H
#define APP_DRIVE_PARAMETER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "drive_parameters.h"

/** @defgroup app_drive_parameters Drive parameter manager
 * @brief Flash load/save와 안전한 controller apply를 main context에서만 처리한다.
 * @{ */

typedef enum {
    DRIVE_PARAMETER_MANAGER_STATUS_DEFAULTS_ACTIVE = 0,
    DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ACTIVE,
    DRIVE_PARAMETER_MANAGER_STATUS_WORKING_SET_LOADED,
    DRIVE_PARAMETER_MANAGER_STATUS_WORKING_SET_DEFAULTED,
    DRIVE_PARAMETER_MANAGER_STATUS_APPLIED,
    DRIVE_PARAMETER_MANAGER_STATUS_SAVED,
    DRIVE_PARAMETER_MANAGER_STATUS_ERASED,
    DRIVE_PARAMETER_MANAGER_STATUS_PENDING_SAFE_STATE,
    DRIVE_PARAMETER_MANAGER_STATUS_INVALID_WORKING_SET,
    DRIVE_PARAMETER_MANAGER_STATUS_APPLY_FAILED,
    DRIVE_PARAMETER_MANAGER_STATUS_FLASH_ERROR,
    DRIVE_PARAMETER_MANAGER_STATUS_DIRTY_NOT_APPLIED
} drive_parameter_manager_status_t;

/** @brief Live Expression에서 읽고 쓰는 파라미터 mailbox. */
typedef struct {
    drive_parameters_t working_set; /**< Tuning 중 debugger가 수정하는 RAM parameter set. */
    drive_parameters_t active_set; /**< 실제 controller에 적용된 parameter set. */
    uint32_t active_generation; /**< 마지막으로 load/save한 Flash generation. */
    uint32_t last_status; /**< drive_parameter_manager_status_t 값. */
    uint32_t load_request; /**< 값을 증가시키면 Flash -> working_set load. */
    uint32_t apply_request; /**< 값을 증가시키면 working_set을 controller에 적용. */
    uint32_t save_request; /**< 값을 증가시키면 active_set을 Flash에 저장. */
    uint32_t defaults_request; /**< 값을 증가시키면 working_set을 컴파일 기본값으로 복원. */
    uint32_t erase_request; /**< 값을 증가시키면 두 Flash slot을 erase. */
    bool is_flash_record_valid; /**< boot 시 유효 Flash record를 찾았으면 true. */
    bool is_dirty; /**< working_set과 active_set이 다르면 true. */
} drive_parameter_debug_t;

typedef bool (*drive_parameter_apply_callback_t)(void *context, const drive_parameters_t *parameters);

typedef struct {
    drive_parameters_t defaults;
    drive_parameter_slot_t active_slot;
    uint32_t handled_load_request;
    uint32_t handled_apply_request;
    uint32_t handled_save_request;
    uint32_t handled_defaults_request;
    uint32_t handled_erase_request;
    bool is_initialized;
} drive_parameter_manager_t;

extern volatile drive_parameter_debug_t drive_parameter_debug;

void drive_parameter_manager_init(drive_parameter_manager_t *self, const drive_parameters_t *defaults);
void drive_parameter_manager_process(
    drive_parameter_manager_t *self,
    bool is_safe_state,
    drive_parameter_apply_callback_t apply_callback,
    void *apply_context
);

/** @} */

#endif /* APP_DRIVE_PARAMETER_MANAGER_H */
