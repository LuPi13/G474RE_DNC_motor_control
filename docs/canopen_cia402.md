# CANopen / CiA 402 통합 기준

## 목적과 범위

이 firmware는 FDCAN2의 Classic CAN 500 kbit/s에서 CANopenNode 기반의 CiA 301과
CiA 402 Profile Torque subset을 제공한다. 초기 범위는 Node-ID 1, NMT/heartbeat,
SDO server, RPDO1/TPDO1, EMCY, Profile Torque mode(0x6060 = 4)다.

현재는 transport, CANopenNode subset, object dictionary, CiA 402 service와 1 kHz
`main.c` wiring까지 구현되어 있다. CiA 402 hardware validation은 다음 단계다.

CANopen ISR은 frame을 CANopenNode RX buffer에 전달만 한다. NMT, SDO, PDO,
CiA 402 상태전이와 drive command 실행은 1 kHz main-context service가 수행한다.
CANopen ISR과 CANopen service는 PWM, FOC, HAL PWM API를 직접 호출하지 않는다.

```text
FDCAN2 IRQ
  -> fdcan_driver
  -> canopen_fdcan_adapter
  -> CANopenNode RX pre-process

1 kHz main context
  -> CO_process / RPDO / TPDO
  -> canopen_service CiA 402 state machine
  -> drive_command_router
  -> App lifecycle
  -> motor_control / FOC
```

`fdcan_driver`는 HAL과 FDCAN peripheral을 소유한다. `canopen_fdcan_adapter`는
CANopenNode의 `CO_CAN*` contract만 구현하며, HAL callback을 별도로 소유하지 않는다.

## 코드 배치와 의존성

- `Core/Platform/fdcan_driver.*`: raw Classic CAN transport와 HAL boundary
- `Core/Platform/canopen_fdcan_adapter.*`: CANopenNode CAN-driver adapter
- `Core/ThirdParty/CANopenNode/`: Apache-2.0 upstream source의 필요한 header와 CiA 301 source
- `Core/Communication/object_dictionary/OD.*`: CANopenEditor generated object dictionary
- `Core/App/canopen_service.*`: CiA 402 application adapter

CANopenNode source 전체를 빌드하지 않는다. 현재 build 대상은 `CANopen.c`와 Emergency,
NMT/Heartbeat, OD interface, SDO server, PDO 및 그 최소 보조 source다. LSS, SRDO,
gateway, SDO client, storage는 비활성화한다.

CubeIDE에는 `Core/ThirdParty/CANopenNode`를 C compiler include path로 추가해야 한다.
ThirdParty와 Communication source directory를 처음 추가한 뒤에는 Debug와 Release를 각각
Clean Build하여 CubeIDE generated source list를 갱신한다.

## Drive lifecycle과 fault

CiA 402의 Enable operation은 `DRIVE_COMMAND_START_CURRENT`을 통해서만 current mode를
시작한다. Disable voltage와 Quick stop은 `DRIVE_COMMAND_STOP`으로 current command를 0 A로
publish한 뒤 PWM을 끄는 App lifecycle을 사용한다.

Fault reset controlword edge는 먼저 `DRIVE_COMMAND_REQUEST_FAULT_CLEAR`로 App의 비동기
fault clear를 요청한다. 유효 ADC sample에서 latch가 실제 해제된 뒤에만
`DRIVE_COMMAND_RECOVER_FAULT`가 FAULTED에서 READY로 복귀시킨다. reset 전의 current command로
자동 재기동하지 않는다.

## Torque scaling profile

`canopen_service_motor_profile_t`가 pole pairs, PM flux linkage, maximum mechanical speed와
`torque_reference_current_peak_a`를 소유한다. 마지막 값은 0x6071 target torque의 1000 permille가
가리키는 q-axis peak current다.

현재 보드 설정은 `motor_config_canopen_torque_reference_current_peak_a = 2.0f`로,
1000 permille = 2 A peak다. 이후 이 Config 값만 바꾸면 CANopen target conversion과
OD의 0x2002/0x6075/0x6076 표시가 함께 갱신된다.

이 값은 motor nameplate rated current와 firmware current-reference limit이 서로 다를 수 있으므로
별도로 명시해야 한다. CANopen profile current가 FOC 2 A command limit을 넘으면 FOC가 제한하며,
CiA 402 statusword의 internal-limit bit를 설정한다. 통신으로 0x2000..0x2004를 쓴다고 live FOC
parameter를 변경하지 않는다. 현재 이 object들은 active profile의 read-only mirror다. live parameter
update는 PWM-off 상태의 atomic 재초기화/validation/storage 정책을 별도 설계한 뒤 추가한다.

## 초기 PDO

Node-ID가 `n`일 때 RPDO1은 `0x200 + n`, TPDO1은 `0x180 + n`이다.

```text
RPDO1: 0x6040 Controlword (u16), 0x6060 Mode (i8), 0x6071 Target torque (i16)
TPDO1: 0x6041 Statusword (u16), 0x6061 Mode display (i8), 0x6077 Actual torque (i16)
```

multi-byte PDO data는 CANopen little-endian이다. RPDO watchdog, EMCY error mapping,
power-drive state transition은 통신 hardware test에서 별도로 검증해야 하며, raw FDCAN frame
송수신 성공만으로 CiA 402 동작이 검증된 것은 아니다.
