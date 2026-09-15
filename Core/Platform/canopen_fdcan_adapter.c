/**
 * @file canopen_fdcan_adapter.c
 * @brief CANopenNode CAN driver contract를 generic fdcan_driver로 구현한다.
 */

#include "canopen_fdcan_adapter.h"

#include <string.h>

#define CANOPEN_FDCAN_STANDARD_ID_MASK (0x07FFU)
#define CANOPEN_FDCAN_BIT_RATE_KBIT_S  (500U)

static bool canopen_fdcan_adapter_try_send(
    CO_CANmodule_t *module,
    CO_CANtx_t *buffer
)
{
    fdcan_driver_t *driver;
    fdcan_frame_t frame;

    if ((module == NULL) || (buffer == NULL) || (buffer->DLC > 8U) ||
        (module->CANptr == NULL)) {
        return false;
    }

    driver = (fdcan_driver_t *)module->CANptr;
    frame = (fdcan_frame_t) {
        .standard_id = (uint16_t)(buffer->ident & CANOPEN_FDCAN_STANDARD_ID_MASK),
        .length = buffer->DLC,
        .data = {0U},
    };
    (void)memcpy(frame.data, buffer->data, frame.length);
    return fdcan_driver_send(driver, &frame) == FDCAN_DRIVER_STATUS_OK;
}

static void canopen_fdcan_adapter_flush_pending(CO_CANmodule_t *module)
{
    if ((module == NULL) || (module->CANtxCount == 0U)) {
        return;
    }

    for (uint16_t index = 0U; index < module->txSize; ++index) {
        CO_CANtx_t *const buffer = &module->txArray[index];

        if (!buffer->bufferFull) {
            continue;
        }
        if (!canopen_fdcan_adapter_try_send(module, buffer)) {
            break;
        }

        buffer->bufferFull = false;
        --module->CANtxCount;
        module->bufferInhibitFlag = buffer->syncFlag;
    }
}

static void canopen_fdcan_adapter_receive(
    void *context,
    const fdcan_frame_t *frame
)
{
    CO_CANmodule_t *const module = (CO_CANmodule_t *)context;
    CO_CANrxMsg_t message;

    if ((module == NULL) || (frame == NULL) || (frame->length > 8U)) {
        return;
    }

    message = (CO_CANrxMsg_t) {
        .ident = frame->standard_id,
        .dlc = frame->length,
        .data = {0U},
    };
    (void)memcpy(message.data, frame->data, message.dlc);

    for (uint16_t index = 0U; index < module->rxSize; ++index) {
        CO_CANrx_t *const buffer = &module->rxArray[index];

        if ((((message.ident ^ buffer->ident) & buffer->mask) == 0U) &&
            (buffer->CANrx_callback != NULL)) {
            buffer->CANrx_callback(buffer->object, &message);
            break;
        }
    }
}

void CO_CANsetConfigurationMode(void *CANptr)
{
    if (CANptr != NULL) {
        (void)fdcan_driver_stop((fdcan_driver_t *)CANptr);
    }
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    fdcan_driver_t *driver;

    if ((CANmodule == NULL) || (CANmodule->CANptr == NULL)) {
        return;
    }

    driver = (fdcan_driver_t *)CANmodule->CANptr;
    if (fdcan_driver_start(driver) == FDCAN_DRIVER_STATUS_OK) {
        CANmodule->CANnormal = true;
    }
}

CO_ReturnError_t canopen_fdcan_adapter_bind(
    CO_CANmodule_t *module,
    fdcan_driver_t *driver,
    uint16_t bit_rate_kbit_s
)
{
    if ((module == NULL) || (driver == NULL) || !driver->is_initialized ||
        ((bit_rate_kbit_s != 0U) &&
         (bit_rate_kbit_s != CANOPEN_FDCAN_BIT_RATE_KBIT_S))) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    module->CANptr = driver;
    module->CANerrorStatus = 0U;
    module->CANnormal = false;
    module->useCANrxFilters = false;
    module->bufferInhibitFlag = false;
    module->firstCANtxMessage = true;
    module->CANtxCount = 0U;
    module->errOld = 0U;

    if (fdcan_driver_set_receive_callback(
            driver,
            canopen_fdcan_adapter_receive,
            module
        ) != FDCAN_DRIVER_STATUS_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    return CO_ERROR_NO;
}

CO_ReturnError_t CO_CANmodule_init(
    CO_CANmodule_t *CANmodule,
    void *CANptr,
    CO_CANrx_t rxArray[],
    uint16_t rxSize,
    CO_CANtx_t txArray[],
    uint16_t txSize,
    uint16_t CANbitRate
)
{
    if ((CANmodule == NULL) || (CANptr == NULL) || (rxArray == NULL) ||
        (txArray == NULL) || (rxSize == 0U) || (txSize == 0U)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    for (uint16_t index = 0U; index < rxSize; ++index) {
        rxArray[index] = (CO_CANrx_t) {
            .ident = 0U,
            .mask = 0xFFFFU,
            .object = NULL,
            .CANrx_callback = NULL,
        };
    }
    for (uint16_t index = 0U; index < txSize; ++index) {
        txArray[index].bufferFull = false;
    }

    return canopen_fdcan_adapter_bind(
        CANmodule,
        (fdcan_driver_t *)CANptr,
        CANbitRate
    );
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    fdcan_driver_t *driver;

    if ((CANmodule == NULL) || (CANmodule->CANptr == NULL)) {
        return;
    }

    driver = (fdcan_driver_t *)CANmodule->CANptr;
    (void)fdcan_driver_set_receive_callback(driver, NULL, NULL);
    (void)fdcan_driver_stop(driver);
    CANmodule->CANnormal = false;
}

CO_ReturnError_t CO_CANrxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    uint16_t mask,
    bool_t rtr,
    void *object,
    void (*CANrx_callback)(void *object, void *message)
)
{
    CO_CANrx_t *buffer;

    if ((CANmodule == NULL) || (index >= CANmodule->rxSize) ||
        (object == NULL) || (CANrx_callback == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    buffer = &CANmodule->rxArray[index];
    buffer->ident = (uint16_t)(ident & CANOPEN_FDCAN_STANDARD_ID_MASK);
    buffer->mask = (uint16_t)(mask & CANOPEN_FDCAN_STANDARD_ID_MASK);
    if (rtr) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;
    return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    bool_t rtr,
    uint8_t noOfBytes,
    bool_t syncFlag
)
{
    CO_CANtx_t *buffer;

    if ((CANmodule == NULL) || (index >= CANmodule->txSize) ||
        (noOfBytes > 8U) || rtr) {
        return NULL;
    }

    buffer = &CANmodule->txArray[index];
    *buffer = (CO_CANtx_t) {
        .ident = (uint16_t)(ident & CANOPEN_FDCAN_STANDARD_ID_MASK),
        .DLC = noOfBytes,
        .data = {0U},
        .bufferFull = false,
        .syncFlag = syncFlag,
    };
    return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t result = CO_ERROR_NO;

    if ((CANmodule == NULL) || (buffer == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        result = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    if (canopen_fdcan_adapter_try_send(CANmodule, buffer)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    } else if (!buffer->bufferFull) {
        buffer->bufferFull = true;
        ++CANmodule->CANtxCount;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    return result;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    bool deleted = false;

    if (CANmodule == NULL) {
        return;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    if (CANmodule->bufferInhibitFlag) {
        CANmodule->bufferInhibitFlag = false;
        deleted = true;
    }
    for (uint16_t index = 0U; index < CANmodule->txSize; ++index) {
        CO_CANtx_t *const buffer = &CANmodule->txArray[index];

        if (buffer->bufferFull && buffer->syncFlag) {
            buffer->bufferFull = false;
            --CANmodule->CANtxCount;
            deleted = true;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (deleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    fdcan_driver_t *driver;
    uint32_t protocol_status;
    uint16_t canopen_status;

    if ((CANmodule == NULL) || (CANmodule->CANptr == NULL)) {
        return;
    }

    driver = (fdcan_driver_t *)CANmodule->CANptr;
    protocol_status = driver->config.hfdcan->Instance->PSR &
        (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);
    if (protocol_status != CANmodule->errOld) {
        CANmodule->errOld = protocol_status;
        canopen_status = CANmodule->CANerrorStatus;
        canopen_status &= (uint16_t)~(
            CO_CAN_ERRTX_BUS_OFF |
            CO_CAN_ERRRX_WARNING |
            CO_CAN_ERRRX_PASSIVE |
            CO_CAN_ERRTX_WARNING |
            CO_CAN_ERRTX_PASSIVE
        );
        if ((protocol_status & FDCAN_PSR_BO) != 0U) {
            canopen_status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            if ((protocol_status & FDCAN_PSR_EW) != 0U) {
                canopen_status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }
            if ((protocol_status & FDCAN_PSR_EP) != 0U) {
                canopen_status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }
        }
        CANmodule->CANerrorStatus = canopen_status;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    canopen_fdcan_adapter_flush_pending(CANmodule);
    CO_UNLOCK_CAN_SEND(CANmodule);
}
