# Flash Drive Parameter Storage

## 목적과 범위

`drive_parameters_t`는 모터 모델, current/speed PI, reference 제한, 속도 제한 및 CANopen torque
conversion scale을 저장한다. PCB 배선/ADC/HRTIM mapping, sensor gain, ADC offset, Hall wiring profile,
runtime state, command, fault는 저장하지 않는다. ADC offset은 매 부팅마다 다시 calibration한다.

## Flash 예약 영역

`STM32G474RETX_FLASH.ld`는 마지막 8 KiB를 firmware에서 제외한다.

```text
0x0807_E000 .. 0x0807_EFFF : slot A (4 KiB)
0x0807_F000 .. 0x0807_FFFF : slot B (4 KiB)
```

두 slot은 DBANK option에 따른 erase granularity 차이에도 독립적으로 erase할 수 있도록 각각 4 KiB다.
Flash driver는 STM32G474의 현재 DBANK 설정을 읽어 single-bank 4 KiB page 또는 dual-bank의 2 KiB
pages 두 개를 erase한다. Flash program은 64-bit double-word 단위다.

정상 firmware download가 storage 영역을 보존하는지는 programmer erase 설정에 따라 다르다. full-chip erase를
실행하면 parameter도 사라지며, 다음 boot에서는 컴파일 기본값을 사용한다.

## Record와 전원 차단 복구

각 slot에는 고정 길이 160-byte record가 있다.

```text
magic | schema version + payload size | generation | CRC32 | payload | commit marker
```

payload는 explicit little-endian IEEE-754 32-bit field serialization을 사용한다. C struct padding이나 field
layout에는 의존하지 않는다. CRC는 header의 `magic/version/size/generation`과 payload에 적용한다.

저장은 항상 반대 slot에 수행한다.

1. target slot erase
2. record body program 및 read-back verify
3. 마지막 64-bit commit marker program 및 verify

boot은 commit marker, magic, schema version, CRC, parameter range가 모두 유효한 record만 사용하고, 둘 다
유효하면 wrap-around-safe generation이 큰 쪽을 선택한다. 저장 중 reset/power loss가 발생한 새 slot은
uncommitted로 무시되고 이전 slot이 유지된다.

## Boot와 debugger workflow

`main.c`는 controller initialization 전에 Flash를 읽는다. 유효 record가 없으면 `drive_parameters_get_defaults()`
기본값을 사용한다. Live Expression/SWV에서는 전역 `drive_parameter_debug`을 사용한다.

1. `working_set` 값을 수정한다.
2. drive를 `APP_DRIVE_STATE_READY`, PWM output disabled 상태로 둔다.
3. `apply_request`를 증가시킨다. 전체 range validation 후 `motor_control`을 reset/reinitialize한다.
4. 실제 동작을 확인한다.
5. `save_request`를 증가시킨다. `working_set`과 동일한 `active_set`만 Flash에 기록한다.

`load_request`는 Flash 값을 `working_set`으로만 가져오며 자동 apply하지 않는다. `defaults_request`도 working
set만 컴파일 기본값으로 바꾼다. `erase_request`는 두 slot을 지우며 active controller setting은 바꾸지 않는다.

drive가 running, ramp-to-zero, calibration 또는 fault 상태이면 apply/save/erase request는
`DRIVE_PARAMETER_MANAGER_STATUS_PENDING_SAFE_STATE`로 유지되고 READY가 될 때 처리된다. 이 작업은 main
context에서만 수행하며 ADC ISR, SysTick, 40 kHz FOC loop에서는 Flash API를 호출하지 않는다.

## 검증

Host test는 `tests/test_drive_parameters.c`에 있다. clang 또는 gcc가 설치된 환경에서 다음을 실행한다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File ./tests/run_drive_parameter_tests.ps1
```

round trip, commit marker 없는 torn write, CRC corruption, range rejection, A/B 최신 generation 선택을 확인한다.
보드에서는 save 후 reset, slot을 번갈아 쓰는 반복 save, 저장 중 전원 차단 후 이전 record boot를 별도로 확인해야
한다. 이 문서는 hardware validation 완료를 주장하지 않는다.
