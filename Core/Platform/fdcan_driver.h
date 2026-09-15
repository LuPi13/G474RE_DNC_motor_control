/**
 * @file fdcan_driver.h
 * @brief STM32 FDCAN Classic CAN frame 송수신 Platform API를 정의한다.
 */

#ifndef FDCAN_DRIVER_H
#define FDCAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/**
 * @brief FDCAN driver 처리 결과.
 */
typedef enum {
    FDCAN_DRIVER_STATUS_OK = 0,
    FDCAN_DRIVER_STATUS_INVALID_ARGUMENT,
    FDCAN_DRIVER_STATUS_INVALID_STATE,
    FDCAN_DRIVER_STATUS_TX_BUSY,
    FDCAN_DRIVER_STATUS_HAL_ERROR
} fdcan_driver_status_t;

/**
 * @brief Classic CAN standard data frame.
 */
typedef struct {
    uint16_t standard_id; /**< 11-bit standard identifier [0, 0x7ff]. */
    uint8_t length; /**< data byte 수 [0, 8]. */
    uint8_t data[8]; /**< frame payload. */
} fdcan_frame_t;

/**
 * @brief RX FIFO0에서 꺼낸 frame의 ISR consumer.
 *
 * @param[in] context config에서 지정한 consumer context.
 * @param[in] frame 수신 frame. callback 반환 뒤에는 이 pointer가 유효하지 않다.
 * @warning FDCAN ISR context에서 호출된다. blocking, HAL 지연 호출 또는 motor-control API 호출은 금지한다.
 */
typedef void (*fdcan_driver_receive_callback_t)(
    void *context,
    const fdcan_frame_t *frame
);

/**
 * @brief FDCAN hardware handle과 RX consumer 설정.
 */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan; /**< CubeMX가 Classic CAN으로 초기화한 FDCAN handle. */
    fdcan_driver_receive_callback_t receive_callback; /**< ISR RX consumer. NULL이면 frame을 기록만 한다. */
    void *receive_context; /**< receive_callback의 context. */
} fdcan_driver_config_t;

/**
 * @brief FDCAN RX callback과 context의 atomic publication 단위.
 */
typedef struct {
    fdcan_driver_receive_callback_t callback;
    void *context;
} fdcan_driver_receive_binding_t;

/**
 * @brief FDCAN driver runtime state와 diagnostic.
 */
typedef struct {
    fdcan_driver_config_t config; /**< 초기화 시 복사한 hardware/consumer 설정. */
    fdcan_driver_receive_binding_t receive_binding[2];
    volatile uint8_t active_receive_binding_index;
    volatile fdcan_frame_t last_received_frame; /**< 가장 최근 RX FIFO0 frame. */
    volatile uint32_t received_count; /**< 성공적으로 FIFO0에서 꺼낸 frame 수. */
    volatile uint32_t transmitted_count; /**< TX FIFO에 정상 등록한 frame 수. */
    volatile uint32_t error_count; /**< HAL error callback 수. */
    volatile uint32_t last_hal_error; /**< 마지막 HAL FDCAN error mask. */
    volatile fdcan_driver_status_t last_status; /**< 마지막 driver 처리 결과. */
    bool is_initialized; /**< init 완료 여부. */
    bool is_started; /**< 마지막 start/stop 결과로 기록한 peripheral 실행 상태. */
} fdcan_driver_t;

/**
 * @brief FDCAN global filter와 interrupt notification을 준비한다.
 *
 * @param[out] self 초기화할 driver instance.
 * @param[in] config CubeMX handle과 RX consumer 설정.
 * @pre CubeMX FDCAN 초기화와 FDCAN2_IT0 NVIC 설정이 완료되어 있어야 한다.
 * @note standard data frame은 RX FIFO0으로 받고 extended/remote frame은 거부한다. 이 함수는
 *       peripheral을 start하지 않는다.
 */
fdcan_driver_status_t fdcan_driver_init(
    fdcan_driver_t *self,
    const fdcan_driver_config_t *config
);

/**
 * @brief 준비된 FDCAN peripheral을 시작한다.
 *
 * @param[in,out] self 초기화된 driver instance.
 * @pre main context에서 호출한다.
 */
fdcan_driver_status_t fdcan_driver_start(fdcan_driver_t *self);

/**
 * @brief FDCAN peripheral을 정지한다.
 *
 * @param[in,out] self 초기화된 driver instance.
 * @pre main context에서 호출하며 FDCAN ISR과 동시에 호출하지 않는다.
 */
fdcan_driver_status_t fdcan_driver_stop(fdcan_driver_t *self);

/**
 * @brief FDCAN RX ISR consumer를 런타임에 교체한다.
 *
 * @param[in,out] self 초기화된 driver instance.
 * @param[in] callback 수신 frame consumer. NULL이면 raw diagnostic만 기록한다.
 * @param[in] context callback context.
 * @pre main context에서만 호출한다.
 * @note FDCAN ISR가 callback/context의 서로 다른 binding을 보지 않도록
 *       double buffer와 memory barrier를 사용한다.
 */
fdcan_driver_status_t fdcan_driver_set_receive_callback(
    fdcan_driver_t *self,
    fdcan_driver_receive_callback_t callback,
    void *context
);

/**
 * @brief standard data frame을 TX FIFO에 등록한다.
 *
 * @param[in,out] self 시작된 driver instance.
 * @param[in] frame 전송할 Classic CAN frame.
 * @retval FDCAN_DRIVER_STATUS_TX_BUSY TX FIFO가 가득 차 frame을 등록하지 못함.
 * @note 성공은 bus transmission 완료가 아니라 TX FIFO 등록 완료를 뜻한다.
 */
fdcan_driver_status_t fdcan_driver_send(
    fdcan_driver_t *self,
    const fdcan_frame_t *frame
);

/**
 * @brief HAL RX FIFO0 callback에서 수신 frame을 처리한다.
 *
 * @param[in,out] self 초기화된 driver instance.
 * @param[in] hfdcan callback을 발생시킨 HAL handle.
 * @param[in] interrupt_flags HAL이 전달한 RX FIFO0 interrupt flag.
 * @note FDCAN ISR에서만 호출한다.
 */
void fdcan_driver_handle_rx_fifo0(
    fdcan_driver_t *self,
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t interrupt_flags
);

/**
 * @brief HAL FDCAN error callback diagnostic을 갱신한다.
 *
 * @param[in,out] self 초기화된 driver instance.
 * @param[in] hfdcan callback을 발생시킨 HAL handle.
 * @note FDCAN ISR에서만 호출한다. fault policy는 상위 communication service가 결정한다.
 */
void fdcan_driver_handle_error(
    fdcan_driver_t *self,
    FDCAN_HandleTypeDef *hfdcan
);

#endif /* FDCAN_DRIVER_H */
