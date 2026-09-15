/**
 * @file canopen_fdcan_adapter.h
 * @brief CANopenNode CAN driver contract를 fdcan_driver에 연결한다.
 */

#ifndef CANOPEN_FDCAN_ADAPTER_H
#define CANOPEN_FDCAN_ADAPTER_H

#include "../ThirdParty/CANopenNode/301/CO_driver.h"
#include "fdcan_driver.h"

/**
 * @brief CANopenNode CAN module이 사용할 FDCAN driver를 준비한다.
 *
 * @param[in,out] module CANopenNode가 소유하는 CAN module.
 * @param[in,out] driver 초기화된 generic FDCAN driver.
 * @param[in] bit_rate_kbit_s CubeMX nominal bit rate와 일치해야 하는 bit rate [kbit/s].
 * @return CANopenNode driver status.
 * @note 실제 호출은 CANopenNode의 `CO_CANmodule_init()`을 통해 이뤄진다.
 */
CO_ReturnError_t canopen_fdcan_adapter_bind(
    CO_CANmodule_t *module,
    fdcan_driver_t *driver,
    uint16_t bit_rate_kbit_s
);

#endif /* CANOPEN_FDCAN_ADAPTER_H */
