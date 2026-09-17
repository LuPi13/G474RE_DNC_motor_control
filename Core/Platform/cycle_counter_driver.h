/**
 * @file cycle_counter_driver.h
 * @brief Cortex-M DWT cycle counter의 최소 Platform 접근 API를 정의한다.
 * @ingroup platform_cycle_counter
 */

#ifndef PLATFORM_CYCLE_COUNTER_DRIVER_H
#define PLATFORM_CYCLE_COUNTER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/** @defgroup platform_cycle_counter Cycle counter driver
 * @brief DWT의 free-running 32-bit CPU cycle counter를 App 진단에 제공한다.
 * @{ */

/**
 * @brief DWT CPU cycle counter를 0부터 시작한다.
 *
 * @return counter enable bit가 설정되면 true.
 * @note Debug/diagnostic build에서만 호출한다. 이 API는 counter를 reset한다.
 */
bool cycle_counter_driver_init(void);

/**
 * @brief 현재 CPU cycle counter를 읽는다.
 *
 * @return Wrap-around 가능한 32-bit CPU cycle count.
 * @pre cycle_counter_driver_init()가 성공했어야 한다.
 */
uint32_t cycle_counter_driver_read(void);

/** @} */

#endif /* PLATFORM_CYCLE_COUNTER_DRIVER_H */
