/**
 * @file drive_command.h
 * @brief 외부 drive command를 App lifecycle API로 전달하는 interface를 정의한다.
 *
 * CAN, UART, debugger test source는 protocol 해석 뒤 이 module의 command만 만든다.
 * 이 module은 통신 peripheral이나 HAL에 의존하지 않으며, PWM lifecycle의 owner인 App API만 호출한다.
 */

#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include <stdint.h>

#include "app.h"

/**
 * @brief 외부 command source가 요청할 수 있는 정상 drive 동작.
 */
typedef enum {
    DRIVE_COMMAND_START_CURRENT = 0, /**< READY에서 current control과 PWM output을 시작한다. */
    DRIVE_COMMAND_SET_CURRENT, /**< 실행 중인 d/q current reference를 갱신한다. */
    DRIVE_COMMAND_START_SPEED, /**< READY에서 speed control과 PWM output을 시작한다. */
    DRIVE_COMMAND_SET_SPEED, /**< 실행 중인 speed reference를 갱신한다. */
    DRIVE_COMMAND_STOP, /**< 현재 운전 mode의 정상 정지를 요청한다. */
    DRIVE_COMMAND_REQUEST_FAULT_CLEAR, /**< 다음 유효 fast-loop sample에서 fault latch clear를 요청한다. */
    DRIVE_COMMAND_RECOVER_FAULT /**< fault clear 뒤 FAULTED에서 READY로 복귀를 요청한다. */
} drive_command_type_t;

/**
 * @brief protocol-independent drive command payload.
 */
typedef struct {
    drive_command_type_t type; /**< 요청할 lifecycle 동작. */
    dq_t i_dq_ref; /**< START_CURRENT 또는 SET_CURRENT의 d/q current 지령 [A]. */
    float omega_m_ref_rad_s; /**< START_SPEED 또는 SET_SPEED의 기계각속도 지령 [rad/s]. */
} drive_command_t;

/**
 * @brief command dispatch 진단 상태.
 *
 * command source마다 별도 instance를 둘 수 있으며, App의 command나 lifecycle state를 복제하지 않는다.
 */
typedef struct {
    app_status_t last_status; /**< 마지막 dispatch 결과. */
    uint32_t accepted_count; /**< APP_STATUS_OK로 처리한 command 수. */
    uint32_t rejected_count; /**< 유효하지 않은 command 또는 현재 상태에서 거부된 command 수. */
} drive_command_router_t;

/**
 * @brief drive command router의 진단 상태를 초기화한다.
 *
 * @param[out] self 초기화할 router instance.
 */
void drive_command_router_init(drive_command_router_t *self);

/**
 * @brief 하나의 외부 drive command를 현재 App lifecycle에 적용한다.
 *
 * @param[in,out] self 초기화된 router instance.
 * @param[in,out] app 초기화된 App instance. main context에서만 호출한다.
 * @param[in] command protocol 해석이 완료된 command.
 * @return App lifecycle API가 반환한 처리 결과.
 *
 * @note 이 함수는 ISR에서 호출하지 않는다. 통신 ISR은 command를 queue에 넣고, main context가
 *       dequeue한 뒤 이 함수를 호출해야 한다.
 */
app_status_t drive_command_router_execute(
    drive_command_router_t *self,
    app_t *app,
    const drive_command_t *command
);

#endif /* DRIVE_COMMAND_H */
