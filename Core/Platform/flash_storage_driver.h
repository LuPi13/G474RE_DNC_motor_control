/**
 * @file flash_storage_driver.h
 * @brief STM32 internal Flash의 예약된 파라미터 slot 접근 API.
 * @ingroup platform_flash_storage
 */

#ifndef PLATFORM_FLASH_STORAGE_DRIVER_H
#define PLATFORM_FLASH_STORAGE_DRIVER_H

#include <stddef.h>
#include <stdint.h>

/** @defgroup platform_flash_storage Flash storage driver
 * @brief 마지막 8 KiB 예약 영역의 두 개 4 KiB slot을 raw byte buffer로 읽고 쓴다.
 * @{ */

#define FLASH_STORAGE_SLOT_SIZE_BYTES (4096U)

typedef enum {
    FLASH_STORAGE_STATUS_OK = 0,
    FLASH_STORAGE_STATUS_INVALID_ARGUMENT,
    FLASH_STORAGE_STATUS_HAL_ERROR,
    FLASH_STORAGE_STATUS_VERIFY_ERROR
} flash_storage_status_t;

typedef enum {
    FLASH_STORAGE_SLOT_A = 0,
    FLASH_STORAGE_SLOT_B
} flash_storage_slot_t;

flash_storage_status_t flash_storage_driver_read(
    flash_storage_slot_t slot,
    uint32_t offset_bytes,
    uint8_t *destination,
    size_t size_bytes
);
flash_storage_status_t flash_storage_driver_erase(flash_storage_slot_t slot);
flash_storage_status_t flash_storage_driver_program(
    flash_storage_slot_t slot,
    uint32_t offset_bytes,
    const uint8_t *source,
    size_t size_bytes
);

/** @} */

#endif /* PLATFORM_FLASH_STORAGE_DRIVER_H */
