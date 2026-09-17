/**
 * @file cycle_counter_driver.c
 * @brief Cortex-M DWT cycle counter Platform 접근을 구현한다.
 */

#include "cycle_counter_driver.h"

#include "stm32g4xx_hal.h"

bool cycle_counter_driver_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

uint32_t cycle_counter_driver_read(void)
{
    return DWT->CYCCNT;
}
