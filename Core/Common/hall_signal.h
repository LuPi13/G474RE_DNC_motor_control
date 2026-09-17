/**
 * @file hall_signal.h
 * @brief Platform Hall driver와 Control Hall decoder가 공유하는 raw signal snapshot을 정의한다.
 */

#ifndef HALL_SIGNAL_H
#define HALL_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 3-bit Hall raw state와 edge timing의 hardware-independent snapshot.
 *
 * Platform driver는 GPIO/TIM capture 결과를 이 값으로 publish하고, Control decoder는
 * motor profile에 따라 sector, angle, speed를 해석한다. raw signal 자체는 motor profile이나
 * peripheral handle을 포함하지 않으므로 Common의 계층 독립 value type으로 둔다.
 */
typedef struct {
    uint8_t hall_state;       /**< A/B/C = bit 2/1/0인 raw state, 범위 [0, 7]. */
    uint32_t capture_count;   /**< 수락한 Hall state transition마다 증가하는 capture sequence. */
    uint32_t invalid_capture_count; /**< State 불일치, 0 tick 또는 overcapture raw event 누적 횟수. */
    float edge_interval_s;    /**< 유효할 때 직전 edge부터 현재 edge까지의 시간 [s]. */
    bool has_state_sample;    /**< hall_state가 실제 GPIO sample임. */
    bool has_valid_interval;  /**< edge_interval_s를 speed 계산에 사용할 수 있음. */
    bool is_timed_out;        /**< Hall edge 없이 driver timeout이 발생함. */
} hall_signal_t;

#endif /* HALL_SIGNAL_H */
