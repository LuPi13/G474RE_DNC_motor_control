/**
 * @file drive_debug_command_source.h
 * @brief debugger Live Expression용 선택형 speed command source를 정의한다.
 *
 * 이 module은 board bring-up에서만 사용한다. 제품 CAN/UART command source와 달리 debugger가
 * 전역 변수를 갱신하지만, 실제 lifecycle 전이는 drive_command_router_execute()로만 요청한다.
 */

#ifndef DRIVE_DEBUG_COMMAND_SOURCE_H
#define DRIVE_DEBUG_COMMAND_SOURCE_H

#include <stdbool.h>

#include "app.h"

/**
 * @brief debugger가 읽고 쓰는 speed command source 상태.
 *
 * requested_speed_rpm, start_requested, stop_requested만 debugger에서 쓴다. 나머지는 관찰용
 * 결과이며 App 또는 motor control의 canonical state를 복제하지 않는다.
 */
typedef struct {
    volatile float requested_speed_rpm; /**< 다음 start 또는 실행 중 SET_SPEED의 요청 속도 [rpm]. */
    volatile bool start_requested; /**< true이면 READY에서 speed control 시작을 한 번 요청한다. */
    volatile bool stop_requested; /**< true이면 speed control의 ramp-to-zero를 한 번 요청한다. */
    volatile float applied_speed_rpm; /**< 마지막으로 수락된 speed target [rpm]. */
    volatile bool is_running; /**< App이 SPEED_RUNNING 또는 RAMP_TO_ZERO이면 true. */
    volatile app_status_t last_status; /**< 마지막 command router 처리 결과. */
} drive_debug_command_source_t;

/**
 * @brief debugger가 접근할 board bring-up command source instance.
 */
extern volatile drive_debug_command_source_t drive_debug_command_source;

/**
 * @brief debugger command source를 초기화한다.
 *
 * @pre main context에서 한 번 호출한다.
 */
void drive_debug_command_source_init(void);

/**
 * @brief debugger 요청을 읽고 필요한 drive command를 main context에서 실행한다.
 *
 * @param[in,out] app 초기화된 App instance.
 *
 * @pre main context에서만 호출한다.
 * @note 이 함수는 speed command를 ±3000 rpm으로 제한한다. ISR에서는 호출하지 않는다.
 */
void drive_debug_command_source_update(app_t *app);

#endif /* DRIVE_DEBUG_COMMAND_SOURCE_H */
