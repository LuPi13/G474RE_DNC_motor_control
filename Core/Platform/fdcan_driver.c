/**
 * @file fdcan_driver.c
 * @brief STM32 FDCAN Classic CAN frame 송수신 Platform driver를 구현한다.
 */

#include "fdcan_driver.h"

static uint32_t fdcan_driver_dlc_from_length(uint8_t length)
{
    static const uint32_t dlc_by_length[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };

    return dlc_by_length[length];
}

static uint8_t fdcan_driver_length_from_dlc(uint32_t dlc)
{
    return (dlc <= FDCAN_DLC_BYTES_8) ? (uint8_t)(dlc >> 16U) : 0U;
}

static bool fdcan_driver_is_valid_frame(const fdcan_frame_t *frame)
{
    return (frame != NULL) && (frame->standard_id <= 0x07FFU) &&
        (frame->length <= 8U);
}

fdcan_driver_status_t fdcan_driver_init(
    fdcan_driver_t *self,
    const fdcan_driver_config_t *config
)
{
    if ((self == NULL) || (config == NULL) || (config->hfdcan == NULL) ||
        (config->hfdcan->Init.FrameFormat != FDCAN_FRAME_CLASSIC)) {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    if ((HAL_FDCAN_ConfigGlobalFilter(
            config->hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
        (HAL_FDCAN_ActivateNotification(
            config->hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING |
                FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF,
            0U) != HAL_OK)) {
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    *self = (fdcan_driver_t) {
        .config = *config,
        .receive_binding = {
            {
                .callback = config->receive_callback,
                .context = config->receive_context,
            },
            {
                .callback = config->receive_callback,
                .context = config->receive_context,
            },
        },
        .active_receive_binding_index = 0U,
        .last_received_frame = {0U, 0U, {0U}},
        .received_count = 0U,
        .transmitted_count = 0U,
        .error_count = 0U,
        .last_hal_error = HAL_FDCAN_ERROR_NONE,
        .last_status = FDCAN_DRIVER_STATUS_OK,
        .is_initialized = true,
        .is_started = false,
    };
    return FDCAN_DRIVER_STATUS_OK;
}

fdcan_driver_status_t fdcan_driver_start(fdcan_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (self->is_started) {
        self->last_status = FDCAN_DRIVER_STATUS_OK;
        return FDCAN_DRIVER_STATUS_OK;
    }
    if (HAL_FDCAN_Start(self->config.hfdcan) != HAL_OK) {
        self->last_status = FDCAN_DRIVER_STATUS_HAL_ERROR;
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    __DMB();
    self->is_started = true;
    self->last_status = FDCAN_DRIVER_STATUS_OK;
    return FDCAN_DRIVER_STATUS_OK;
}

fdcan_driver_status_t fdcan_driver_stop(fdcan_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_started) {
        self->last_status = FDCAN_DRIVER_STATUS_OK;
        return FDCAN_DRIVER_STATUS_OK;
    }
    if (HAL_FDCAN_Stop(self->config.hfdcan) != HAL_OK) {
        self->last_status = FDCAN_DRIVER_STATUS_HAL_ERROR;
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    __DMB();
    self->is_started = false;
    self->last_status = FDCAN_DRIVER_STATUS_OK;
    return FDCAN_DRIVER_STATUS_OK;
}

fdcan_driver_status_t fdcan_driver_set_receive_callback(
    fdcan_driver_t *self,
    fdcan_driver_receive_callback_t callback,
    void *context
)
{
    uint8_t inactive_index;

    if ((self == NULL) || !self->is_initialized) {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    inactive_index =
        (uint8_t)(self->active_receive_binding_index ^ 1U);
    self->receive_binding[inactive_index] =
        (fdcan_driver_receive_binding_t) {
            .callback = callback,
            .context = context,
        };
    __DMB();
    self->active_receive_binding_index = inactive_index;
    __DMB();
    self->last_status = FDCAN_DRIVER_STATUS_OK;
    return FDCAN_DRIVER_STATUS_OK;
}

fdcan_driver_status_t fdcan_driver_send(
    fdcan_driver_t *self,
    const fdcan_frame_t *frame
)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((self == NULL) || (!fdcan_driver_is_valid_frame(frame))) {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if ((!self->is_initialized) || (!self->is_started)) {
        return FDCAN_DRIVER_STATUS_INVALID_STATE;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(self->config.hfdcan) == 0U) {
        self->last_status = FDCAN_DRIVER_STATUS_TX_BUSY;
        return FDCAN_DRIVER_STATUS_TX_BUSY;
    }

    header.Identifier = frame->standard_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = fdcan_driver_dlc_from_length(frame->length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            self->config.hfdcan, &header, frame->data) != HAL_OK) {
        self->last_status = FDCAN_DRIVER_STATUS_HAL_ERROR;
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    ++self->transmitted_count;
    self->last_status = FDCAN_DRIVER_STATUS_OK;
    return FDCAN_DRIVER_STATUS_OK;
}

void fdcan_driver_handle_rx_fifo0(
    fdcan_driver_t *self,
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t interrupt_flags
)
{
    if ((self == NULL) || (!self->is_initialized) || (!self->is_started) ||
        (hfdcan != self->config.hfdcan) ||
        ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header;
        fdcan_frame_t frame = {0};

        if (HAL_FDCAN_GetRxMessage(
                hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            self->last_status = FDCAN_DRIVER_STATUS_HAL_ERROR;
            return;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME)) {
            continue;
        }

        frame.standard_id = (uint16_t)header.Identifier;
        frame.length = fdcan_driver_length_from_dlc(header.DataLength);
        self->last_received_frame = frame;
        ++self->received_count;
        self->last_status = FDCAN_DRIVER_STATUS_OK;

        const fdcan_driver_receive_binding_t *const binding =
            &self->receive_binding[self->active_receive_binding_index];
        if (binding->callback != NULL) {
            binding->callback(binding->context, &frame);
        }
    }
}

void fdcan_driver_handle_error(
    fdcan_driver_t *self,
    FDCAN_HandleTypeDef *hfdcan
)
{
    if ((self == NULL) || (!self->is_initialized) ||
        (hfdcan != self->config.hfdcan)) {
        return;
    }

    self->last_hal_error = HAL_FDCAN_GetError(hfdcan);
    ++self->error_count;
    self->last_status = FDCAN_DRIVER_STATUS_HAL_ERROR;
}
