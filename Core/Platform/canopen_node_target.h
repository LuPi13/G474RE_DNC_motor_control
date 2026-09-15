/**
 * @file canopen_node_target.h
 * @brief STM32G474용 CANopenNode target type과 critical-section 계약을 정의한다.
 *
 * CANopenNode upstream의 generic `CO_driver.h`가 요구하는 target layer이다.
 * 실제 FDCAN HAL 접근은 `canopen_fdcan_adapter.c`만 수행한다.
 */

#ifndef CANOPEN_NODE_TARGET_H
#define CANOPEN_NODE_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define CO_USE_GLOBALS

/* CiA 301/CiA 402 Profile Torque에 필요한 service만 활성화한다. */
#define CO_CONFIG_NMT               (0)
#define CO_CONFIG_HB_CONS           (0)
#define CO_CONFIG_NODE_GUARDING     (0)
#define CO_CONFIG_EM \
    (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_PROD_CONFIGURABLE | \
     CO_CONFIG_EM_HISTORY)
#define CO_CONFIG_SDO_SRV           (CO_CONFIG_SDO_SRV_SEGMENTED)
#define CO_CONFIG_SDO_CLI           (0)
#define CO_CONFIG_TIME              (0)
#define CO_CONFIG_SYNC              (0)
#define CO_CONFIG_PDO \
    (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | \
     CO_CONFIG_RPDO_TIMERS_ENABLE | CO_CONFIG_TPDO_TIMERS_ENABLE)
#define CO_CONFIG_STORAGE           (0)
#define CO_CONFIG_LEDS              (0)
#define CO_CONFIG_GFC               (0)
#define CO_CONFIG_SRDO              (0)
#define CO_CONFIG_LSS               (0)
#define CO_CONFIG_GTW               (0)
#define CO_CONFIG_CRC16             (0)
#define CO_CONFIG_FIFO              (0)
#define CO_CONFIG_TRACE             (0)
#define CO_CONFIG_DEBUG             (0)

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(value) (value)
#define CO_SWAP_32(value) (value)
#define CO_SWAP_64(value) (value)

typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

typedef struct {
    uint16_t ident;
    uint8_t dlc;
    uint8_t data[8];
} CO_CANrxMsg_t;

static inline uint16_t CO_CANrxMsg_readIdent(void *message)
{
    return ((CO_CANrxMsg_t *)message)->ident;
}

static inline uint8_t CO_CANrxMsg_readDLC(void *message)
{
    return ((CO_CANrxMsg_t *)message)->dlc;
}

static inline const uint8_t *CO_CANrxMsg_readData(void *message)
{
    return ((CO_CANrxMsg_t *)message)->data;
}

typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

#define CO_MemoryBarrier() __DMB()
#define CO_FLAG_READ(rx_new) ((rx_new) != NULL)
#define CO_FLAG_SET(rx_new) \
    do { \
        __DMB(); \
        (rx_new) = (void *)1L; \
    } while (0)
#define CO_FLAG_CLEAR(rx_new) \
    do { \
        __DMB(); \
        (rx_new) = NULL; \
    } while (0)

/* Priority 0 ADC와 priority 1 Hall은 CANopen lock으로 막지 않는다. */
#define CO_CANOPEN_BASEPRI_VALUE (2UL << (8UL - __NVIC_PRIO_BITS))
#define CO_LOCK_GENERIC(saved_basepri) \
    do { \
        uint32_t saved_basepri = __get_BASEPRI(); \
        __set_BASEPRI_MAX(CO_CANOPEN_BASEPRI_VALUE); \
        __DSB(); \
        __ISB();
#define CO_UNLOCK_GENERIC(saved_basepri) \
        __set_BASEPRI(saved_basepri); \
        __DSB(); \
        __ISB(); \
    } while (0)

#define CO_LOCK_CAN_SEND(module) CO_LOCK_GENERIC(co_saved_basepri_send)
#define CO_UNLOCK_CAN_SEND(module) CO_UNLOCK_GENERIC(co_saved_basepri_send)
#define CO_LOCK_EMCY(module) CO_LOCK_GENERIC(co_saved_basepri_emcy)
#define CO_UNLOCK_EMCY(module) CO_UNLOCK_GENERIC(co_saved_basepri_emcy)
#define CO_LOCK_OD(module) CO_LOCK_GENERIC(co_saved_basepri_od)
#define CO_UNLOCK_OD(module) CO_UNLOCK_GENERIC(co_saved_basepri_od)

#endif /* CANOPEN_NODE_TARGET_H */
