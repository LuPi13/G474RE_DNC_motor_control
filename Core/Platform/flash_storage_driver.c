/**
 * @file flash_storage_driver.c
 * @brief STM32G474 내부 Flash 파라미터 slot의 HAL 기반 구현.
 */

#include "flash_storage_driver.h"

#include <stdbool.h>
#include <string.h>

#include "stm32g4xx_hal.h"

#define FLASH_STORAGE_SLOT_A_ADDRESS (0x0807E000UL)
#define FLASH_STORAGE_SLOT_B_ADDRESS (0x0807F000UL)

static uint32_t flash_storage_driver_slot_address(flash_storage_slot_t slot)
{
    return (slot == FLASH_STORAGE_SLOT_A) ? FLASH_STORAGE_SLOT_A_ADDRESS : FLASH_STORAGE_SLOT_B_ADDRESS;
}

static bool flash_storage_driver_range_is_valid(uint32_t offset_bytes, size_t size_bytes)
{
    return (offset_bytes <= FLASH_STORAGE_SLOT_SIZE_BYTES) &&
           (size_bytes <= (FLASH_STORAGE_SLOT_SIZE_BYTES - offset_bytes));
}

flash_storage_status_t flash_storage_driver_read(flash_storage_slot_t slot, uint32_t offset_bytes, uint8_t *destination, size_t size_bytes)
{
    if ((destination == NULL) || !flash_storage_driver_range_is_valid(offset_bytes, size_bytes)) return FLASH_STORAGE_STATUS_INVALID_ARGUMENT;
    memcpy(destination, (const void *)(flash_storage_driver_slot_address(slot) + offset_bytes), size_bytes);
    return FLASH_STORAGE_STATUS_OK;
}

flash_storage_status_t flash_storage_driver_erase(flash_storage_slot_t slot)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    const uint32_t address = flash_storage_driver_slot_address(slot);
    const bool is_dual_bank = (FLASH->OPTR & FLASH_OPTR_DBANK) != 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    if (is_dual_bank) {
        erase.Banks = FLASH_BANK_2;
        erase.Page = (address - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
        erase.NbPages = FLASH_STORAGE_SLOT_SIZE_BYTES / FLASH_PAGE_SIZE;
    } else {
        erase.Banks = FLASH_BANK_1;
        erase.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE_128_BITS;
        erase.NbPages = 1U;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_FLASH_Unlock() != HAL_OK) {
        if (primask == 0U) __enable_irq();
        return FLASH_STORAGE_STATUS_HAL_ERROR;
    }
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    (void)HAL_FLASH_Lock();
    if (primask == 0U) __enable_irq();
    return (status == HAL_OK) ? FLASH_STORAGE_STATUS_OK : FLASH_STORAGE_STATUS_HAL_ERROR;
}

flash_storage_status_t flash_storage_driver_program(flash_storage_slot_t slot, uint32_t offset_bytes, const uint8_t *source, size_t size_bytes)
{
    if ((source == NULL) || ((offset_bytes % 8U) != 0U) || ((size_bytes % 8U) != 0U) || !flash_storage_driver_range_is_valid(offset_bytes, size_bytes)) return FLASH_STORAGE_STATUS_INVALID_ARGUMENT;
    const uint32_t address = flash_storage_driver_slot_address(slot) + offset_bytes;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_FLASH_Unlock() != HAL_OK) {
        if (primask == 0U) __enable_irq();
        return FLASH_STORAGE_STATUS_HAL_ERROR;
    }
    for (size_t offset = 0U; offset < size_bytes; offset += 8U) {
        uint64_t word = 0U;
        for (uint32_t byte = 0U; byte < 8U; ++byte) word |= ((uint64_t)source[offset + byte]) << (8U * byte);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (uint32_t)offset, word) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            if (primask == 0U) __enable_irq();
            return FLASH_STORAGE_STATUS_HAL_ERROR;
        }
    }
    (void)HAL_FLASH_Lock();
    if (primask == 0U) __enable_irq();
    if (memcmp((const void *)address, source, size_bytes) != 0) return FLASH_STORAGE_STATUS_VERIFY_ERROR;
    return FLASH_STORAGE_STATUS_OK;
}
