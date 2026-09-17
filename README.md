# STM32G474RE DNC Motor Control

STM32G474RE 기반 3상 PMSM FOC firmware다. HRTIM PWM, 동기 ADC 전류 샘플링,
Hall rotor feedback, FOC current control, speed control 및 CANopen CiA 402
Profile Torque/Profile Velocity subset을 제공한다.

이 문서는 **모터, PCB 또는 운전 한계를 바꿀 때 수정해야 하는 위치**를 찾기 위한
최상위 안내서다. 모듈 설계와 런타임 규칙은 [docs/README.md](docs/README.md)를
source of truth로 사용한다.

## 코드 로직/API 참고

제어 로직, module public API 또는 source-level 동작을 수정하기 전에는
[Doxygen API 문서](https://lupi13.github.io/G474RE_DNC_motor_control/api/html/index.html)를 참고한다.

API HTML은 GitHub Actions가 `main` 변경 시 생성해 GitHub Pages에 배포한다. 생성물인
`docs/api/`와 로컬 출력 경로 `build/doxygen/`은 Git에 추가하지 않는다.

## 빠른 변경표

| 변경하려는 것 | 먼저 바꿀 곳 | 반드시 같이 확인할 곳 |
|---|---|---|
| d/q 전류 지령 상한, 하한, 변화율 | [`Core/Src/main.c`](Core/Src/main.c)의 `motor_control_config` | FOC 전류 제한, fault overcurrent threshold, CANopen torque 기준 전류 |
| 속도 지령 상한, 하한, 가감속 | `motor_control_config`의 `speed_reference_*` | CANopen 최대 기계 속도, speed PI 출력 한계, Hall feedback 범위 |
| d/q current PI, R/L, PM flux | `motor_control_config.foc` | PWM/ADC 주기, DC-link 전압, CANopen torque coefficient |
| speed PI, speed filter | [`Core/Config/motor_config.c`](Core/Config/motor_config.c)의 `motor_config_speed_controller` | speed-loop 1 kHz 주기, q-axis current limit |
| CANopen 1000 permille 토크 기준 | `motor_config_canopen_torque_reference_current_peak_a` | FOC current magnitude limit, `0x6075`, `0x6076`, `0x6072` |
| CANopen PDO, OD 기본값 | [`Core/Communication/object_dictionary/OD.c`](Core/Communication/object_dictionary/OD.c) | [`Core/App/canopen_service.c`](Core/App/canopen_service.c), CANopen master 설정 |
| ADCx/channel/rank 또는 전류 상 매핑 | [`.ioc`](G474RE_DNC_motor_control.ioc)와 CubeMX 생성 ADC 초기화 | `main.c`의 `adc_config`, 센서 gain/offset, ADC completion IRQ |
| HRTIM timer/channel, PWM pin, dead time, PWM 주파수 | `.ioc`와 CubeMX 생성 HRTIM 초기화 | `main.c`의 `pwm_config`, ADC trigger, 40 kHz 관련 모든 sampling period |
| Hall pin 또는 배선 순서 | `.ioc`/GPIO 설정과 `main.c`의 `hall_config` | `motor_config_hall_profile`, TIM2 clock/prescaler, 정·역방향 검증 |
| CAN pin, bit rate, Node-ID | `.ioc`/FDCAN2 설정과 `main.c`의 `canopen_config` | CANopen master의 bit rate/COB-ID/PDO 설정 |

## 설정 소유권

설정은 한 파일에 모두 있지 않다. 아래 순서를 지켜야 같은 물리량의 설정이
엇갈리지 않는다.

```text
.ioc / CubeMX generated peripheral initialization
    -> main.c USER CODE: driver wiring, board-dependent integration
    -> Core/Config: motor-specific reusable configuration
    -> Control: limiters, PI, FOC and speed-loop execution
    -> CANopen service / Object Dictionary: network representation
```

- `.ioc`와 CubeMX 생성 구역은 **핀, peripheral, timer, ADC rank, trigger, interrupt**의
  source of truth다.
- `main.c` USER CODE는 생성된 peripheral을 logical phase/current/feedback과 연결하고,
  현재 board와 motor의 통합 설정을 만든다.
- `Core/Config/motor_config.c`는 Hall profile, speed PI 및 CANopen torque reference처럼
  motor-specific이지만 hardware-independent인 값을 둔다.
- CANopen OD는 통신 표현이며, live FOC parameter의 source of truth가 아니다.

## 제어 주기와 지령 흐름

| 경로 | 실행 주기 | 역할 |
|---|---:|---|
| ADC injected completion → fast loop | 40 kHz | phase current/DC-link 변환, Hall angle, current reference slew, FOC, SVPWM, PWM write |
| SysTick → `app_drive_scheduler_tick()` | 1 kHz | speed reference slew, speed PI, speed-current target publish |
| main context → `canopen_service_process()` | 약 1 kHz | CANopenNode, CiA 402 state machine, PDO/SDO command dispatch, TPDO publish |

```text
CANopen RPDO / SDO
    -> canopen_service
    -> drive_command_router
    -> App command buffer
    -> 1 kHz speed loop 또는 current target prepare
    -> 40 kHz current limiter / FOC
    -> SVPWM / HRTIM PWM
```

통신 ISR은 frame을 CANopenNode RX buffer에 전달만 한다. CANopen service는 PWM/FOC를
직접 호출하지 않는다.

## Debug 관측: Live Expression / SWV

`drive_debug_command_source`는 debugger에서 전류/속도 지령과 start/stop을 입력하는 bring-up 전용
경로다. 결과 관측에는 전역 `drive_debug_snapshot`을 사용한다. ADC fast loop가 40번째 유효 sample마다
(현재 40 kHz / 1 kHz) `Idq`, `Vdq`, 3상 전류, DC-link 전압, duty, electrical/mechanical speed,
speed PI 결과, mode/state/fault를 한 묶음으로 갱신한다.

- **Live Expression**: `drive_debug_snapshot`을 펼쳐 원하는 필드를 읽는다. 값은 debugger에서 쓰지 않는다.
- **SWV Data Trace**: `i_q_ref_a`/`i_q_a`, `v_q_applied_v`, `omega_m_rad_s`, `v_dc_v`처럼 필요한 소수의
  field만 선택한다. snapshot 갱신률은 1 kHz다.
- **일관성 확인**: `drive_debug_snapshot_sequence`이 짝수이고 snapshot 읽기 전후 같은 값이면 완성된
  snapshot이다.
- **위치의 범위**: `theta_e_rad`는 electrical angle이고
  `theta_m_rad_per_electrical_cycle`은 한 electrical cycle 내 기계각 성분이다. multi-turn/absolute
  mechanical position은 encoder 또는 homing이 추가되기 전에는 제공하지 않는다.

40 kHz switching ripple, ADC sampling timing 및 한 PWM 주기의 과도현상은 이 observer 대신
oscilloscope 또는 추후 fault-triggered fast capture로 확인한다.

## 모터와 제어기 설정

### Current command 및 FOC

`Core/Src/main.c`의 `motor_control_config`가 현재 FOC와 지령 limiter의 source다.

| 필드 | 현재 값 | 단위/의미 |
|---|---:|---|
| `current_reference_min/max` | d/q 각각 -2 / +2 | 축별 지령 범위 [A] |
| `current_reference_magnitude_limit` | 2 | d/q vector 최종 크기 상한 [A] |
| `current_reference_rise/fall_rate_per_s` | d/q 각각 100 | 40 kHz 지령 변화율 [A/s] |
| `foc.d_axis_pi`, `q_axis_pi` | `kp`, `ki`, anti-windup, ±100 V | d/q current PI 설정 |
| `foc.current_filter.cutoff_frequency_hz` | 5000 | d/q current feedback filter [Hz] |
| `foc.voltage_utilization` | 0.9 | SVPWM 선형 전압 사용률 |
| `foc.d_axis_inductance_h` | 546 µH | d-axis inductance |
| `foc.q_axis_inductance_h` | 592 µH | q-axis inductance |
| `foc.permanent_magnet_flux_linkage_wb` | 6.74 mWb | PM flux linkage |
| `foc.is_decoupling_enabled` | false | d/q feedforward/decoupling enable |
| `pole_pairs` | 5 | electrical/mechanical speed conversion |

적용 순서는 `axis clamp -> current magnitude clamp -> d/q rate limiter -> final
magnitude clamp -> FOC`다. 따라서 CANopen Torque, CANopen Velocity, debug command를
포함한 모든 경로는 최종적으로 이 전류 한계를 통과한다.

FOC 전압은 PI의 ±100 V 설정만으로 결정되지 않는다. 실제 적용 d/q voltage vector는
`v_dc * voltage_utilization / sqrt(3)` 이내로 제한된다. DC-link 전압 또는 PWM 조건을
바꾸면 이 한계와 current PI anti-windup 거동을 재검증해야 한다.

### Speed control

| 위치 | 필드 | 현재 값 | 의미 |
|---|---|---:|---|
| `main.c` | `speed_reference_min/max_rad_s` | -314.159 / +314.159 | 기계각속도 request 범위 [rad/s], 약 ±3000 rpm |
| `main.c` | `speed_reference_rise/fall_rate_rad_s2` | 31.416 | 1 kHz speed reference 가감속 제한 [rad/s²] |
| `motor_config.c` | `motor_config_speed_controller.pi` | output ±0.5 | speed PI가 만드는 q-axis current 범위 [A] |
| `motor_config.c` | speed PI sampling period | 0.001 | speed PI 실행 주기 [s] |
| `motor_config.c` | speed feedback filter cutoff | 30 | mechanical speed feedback filter [Hz] |

Speed PI output은 우선 ±0.5 A로 제한되고, 이후 current-reference axis/vector limit과
40 kHz current slew limit이 한 번 더 적용된다. 속도 제어에서 허용할 토크를 높이려면
speed PI `output_min/max`와 FOC current limits를 함께 검토해야 한다.

## CANopen CiA 402 설정과 완전한 추적

지원 모드는 Profile Velocity (`0x6060 = 3`)와 Profile Torque (`0x6060 = 4`)다.
기본 RPDO1/TPDO1은 속도 운전에 맞춰져 있다.

```text
RPDO1: 0x6040 Controlword (u16), 0x6060 Mode (i8),
       0x60FF Target velocity (i32, rpm)
TPDO1: 0x6041 Statusword (u16), 0x6061 Mode display (i8),
       0x606C Velocity actual value (i32, rpm)
```

PDO byte order는 little-endian이다. Torque PDO가 필요하면 표준 PDO remapping 절차로
`0x1600`/`0x1A00`을 수정해야 한다. 기본 PDO에서는 `0x6071 Target torque`를 보내지
않는다.

### CANopen motor profile: 한 곳에서 출발하는 값

`main.c`의 `canopen_motor_profile`은 다음 값을 이미 `motor_control_config`에서
가져온다. **이 세 값은 CANopen OD에서 별도로 복제해 수정하지 않는다.**

| CANopen profile field | 원래 설정 | service가 publish하는 OD |
|---|---|---|
| `pole_pairs` | `motor_control_config.pole_pairs` | `0x2000 Motor pole pairs` |
| PM flux linkage | `motor_control_config.foc.permanent_magnet_flux_linkage_wb` | `0x2001`, `0x2004 Torque coefficient` |
| maximum mechanical speed | `motor_control_config.speed_reference_max_rad_s` | `0x2003`, `0x6080 Maximum motor speed` |

Torque coefficient는 `Kt = 1.5 * pole_pairs * PM_flux_linkage` [Nm/A]로 계산된다.
현재 값은 약 `0.05055 Nm/A`다.

`0x2000`~`0x2004`, `0x6075`, `0x6076`, `0x6080`은 CANopen service가 init 및 1 kHz
processing 때 motor profile에서 다시 publish하는 mirror다. `OD.c`의 초기값만 바꿔서는
유지되지 않는다.

### Profile Torque (`0x6060 = 4`)

```text
0x6071 Target torque [permille]
    -> 0x6072 Maximum torque [permille] clamp (최대 1000)
    -> torque_reference_current_peak_a와 비례 변환
    -> 0x6087 Torque slope로 1 kHz slew
    -> DRIVE_COMMAND_SET_CURRENT (i_d = 0, i_q = command)
    -> current axis/vector/rate limit
    -> FOC
```

| 값 | 설정/기본값 | 실제 의미 |
|---|---|---|
| `motor_config_canopen_torque_reference_current_peak_a` | 2 A peak | `0x6071 = 1000`이 요청하는 q-axis current |
| `0x6072 Maximum torque` | 1000 permille | CANopen torque command 상한. OD runtime write 가능 |
| `0x6087 Torque slope` | 1000 | 1 kHz torque command slew. OD runtime write 가능 |
| `0x6075 Motor rated current` | service가 2000 mA로 mirror | CANopen 표시값 |
| `0x6076 Motor rated torque` | service가 약 101 mNm로 mirror | `Kt * torque_reference_current_peak_a` 표시값 |
| FOC final limit | 2 A magnitude | CANopen 변환 이후의 안전 상한 |

`torque_reference_current_peak_a`는 motor nominal current와 반드시 같을 필요는 없지만,
FOC current magnitude limit보다 크게 잡으면 command가 후단에서 잘린다. 제품에서
`0x6071 = 1000`을 “허용 최대 토크”로 약속하려면 두 값을 같은 안전 기준으로 맞춘다.

현재 main loop는 CANopen service에 q-axis current feedback을 전달하지 않는다
(`i_q_feedback_a = 0`, `has_valid_i_q_feedback = false`). 따라서 현재 `0x6077 Torque
actual value`는 0이고 Torque mode의 Statusword target-reached 판단도 true가 되지
않는다. 이는 current control 자체와 별개의 **CANopen torque feedback 미연결 상태**다.

### Profile Velocity (`0x6060 = 3`)

```text
0x60FF Target velocity [rpm]
    -> rpm to mechanical rad/s conversion
    -> speed-reference min/max clamp
    -> motor-profile maximum-speed clamp
    -> DRIVE_COMMAND_SET_SPEED
    -> 1 kHz speed reference slew
    -> speed PI (i_q target)
    -> current axis/vector/rate limit
    -> 40 kHz FOC
```

| 값 | 현재 동작 |
|---|---|
| `0x60FF Target velocity` | signed mechanical rpm request |
| `speed_reference_min/max_rad_s` | CANopen 속도 request의 최종 범위 |
| `speed_reference_rise/fall_rate_rad_s2` | CANopen speed command에도 적용되는 가감속 제한 |
| `0x6080 Maximum motor speed` | motor profile에서 rpm으로 표시하는 read-only mirror |
| `0x606C Velocity actual value` | 유효한 Hall electrical speed를 pole-pair로 나눠 mechanical rpm으로 publish |
| target reached window | actual과 limited target의 차가 10 rpm 이하일 때 Statusword bit 10 |

속도 지령이 범위를 넘으면 Statusword bit 11(internal limit active)이 설정된다. 현재
Profile Velocity에는 `0x6083 Profile acceleration`/`0x6084 Profile deceleration` OD를
구현하지 않았고, 가감속은 위의 내부 speed-reference rate limiter가 담당한다.

## PCB 또는 peripheral 변경

### ADC / current / voltage sensing

ADCx, ADC channel, injected rank, phase wiring 또는 DC-link sensing divider를 바꿀 때:

1. `.ioc`에서 ADC pin/channel/rank/trigger/sample time을 변경하고 CubeMX를 재생성한다.
2. `main.c`의 `adc_config`에서 `phase_a`, `phase_b`, `phase_c`, `dc_link`,
   `injected_completion_adc`가 새 CubeMX 설정과 정확히 일치하도록 바꾼다.
3. `current_sensor_config.gain_a_per_count`와 offset calibration 허용 범위를 새 회로에
   맞춰 갱신한다. 전류 sensor type 또는 ADC reference가 달라지면 gain도 다시 계산한다.
4. `voltage_sensor_config.offset_counts`, `gain_v_per_count`를 새 divider/offset에 맞춘다.
5. phase current 및 DC-link overvoltage fault threshold도 실제 hardware safe limit으로
   다시 설정한다.
6. PWM trigger부터 ADC JEOC, `adc_config.injected_completion_adc`, ADC3 IRQ fast-loop까지
   scope 또는 debug signal로 검증한다.

ADC logical mapping만 바꾸고 CubeMX injected configuration을 그대로 두거나, 반대로
CubeMX rank만 바꾸고 `adc_config`를 그대로 두면 다른 상의 전류를 제어하게 된다.

### HRTIM / PWM

현재 `pwm_config` logical mapping은 `phase_a -> TIMER_F`, `phase_b -> TIMER_D`,
`phase_c -> TIMER_C`, 모두 compare unit 1이다.

PWM pin, HRTIM timer/channel, complementary output, dead time, PWM frequency 또는 ADC
trigger를 바꿀 때:

1. `.ioc`에서 HRTIM output, dead time, period, trigger source와 ADC trigger를 함께 변경한다.
2. `main.c`의 `pwm_config` timer index/compare unit을 logical A/B/C phase와 맞춘다.
3. PWM frequency가 바뀌면 `APP_FAST_LOOP_FREQUENCY_HZ`, fast-loop sampling period,
   d/q current reference rate, current PI/filter sampling period을 같은 실제 주기로
   맞춘다.
4. dead time과 duty mapping은 무전력 또는 낮은 DC-link 전압에서 먼저 scope로 확인한다.

### Hall feedback

현재 Hall GPIO mapping은 `main.c`의 `hall_config`에서 A/B/C = `PA0/PA1/PA2`이고,
TIM2 kernel clock은 170 MHz로 전달한다. Hall pin, timer clock/prescaler, phase order,
Hall polarity 또는 motor가 바뀌면 다음을 모두 검토한다.

1. `.ioc`의 GPIO alternate function, TIM2 XOR/input capture/overflow 설정
2. `hall_config`의 GPIO port/pin 및 `timer_clock_hz`
3. `motor_config_hall_profile.sector_by_state`
4. `motor_config_hall_profile.forward_edge_angle_rad`
5. 정방향/역방향 sector sequence, speed sign, timeout, electrical angle 검증

### FDCAN / CANopen

FDCAN2는 Classic CAN 500 kbit/s로 구성되어 있고, `canopen_config`의 Node-ID는 1이다.
bit rate나 CAN pin을 바꾸면 `.ioc`, FDCAN generated init, CANopen master 설정을 함께
바꾼다. `canopen_service`는 현재 500 kbit/s 외 값은 init에서 거부한다.

## 안전 한계와 정지

| 항목 | 위치 | 현재 값 |
|---|---|---:|
| phase overcurrent trip / clear | `main.c` macro | 3.0 / 1.0 A |
| DC-link overvoltage trip / clear | `main.c` macro | 79.2 / 75.0 V |
| normal speed-stop threshold | `main.c` macro | 52.36 rad/s |
| normal speed-stop dwell | `main.c` macro | 10 ms |
| CANopen overspeed | CANopen motor profile | speed-reference maximum 초과 시 fault latch |

Fault/emergency shutdown은 normal current/speed rate limiter를 우회해 PWM을 차단한다.
새 PCB 또는 motor의 전류/전압 정격에 맞춰 fault threshold를 먼저 조정하고 검증한 뒤
제어기 상한을 올린다.

## 변경 순서와 검증

1. **CubeMX/PCB 변경**: `.ioc` 변경 → CubeMX 재생성 → generated diff 검토 → `main.c`
   logical mapping 갱신.
2. **motor parameter 변경**: pole pairs, R/L, PM flux, Hall profile, current/voltage
   limits, speed PI를 함께 검토.
3. **CANopen 변경**: motor profile mirror, torque reference, PDO mapping과 master
   configuration을 일치시킨다.
4. **검증**: PWM/ADC timing → current polarity/scaling → Hall sign/angle → low-current FOC
   → low-speed speed loop → CANopen Enable/Quick Stop/Disable 순서로 수행한다.

변경 후에는 최소한 아래를 확인한다.

- PWM A/B/C가 의도한 HRTIM output에 대응하고 dead time이 유지되는가
- ADC raw phase와 logical phase A/B/C가 맞고, 0 A offset과 gain이 맞는가
- current, DC-link fault threshold가 hardware rating 안에 있는가
- `0x6060 = 3`, `0x60FF`, `0x606C`가 의도한 rpm으로 동작하는가
- `0x6060 = 4`, `0x6071`, `0x6072`, `0x6087`이 의도한 q-axis current로 변환되는가
- Torque actual value가 필요한 제품이면 위에 명시한 q-axis feedback 연결 작업을
  별도로 완료했는가

## 관련 내부 문서

- [Runtime/data flow](docs/runtime_and_dataflow.md)
- [CANopen CiA 402 integration](docs/canopen_cia402.md)
- [Numeric representation and units](docs/numeric_representation.md)
- [Development and hardware bring-up process](docs/development_process.md)
- [File structure and dependency rules](docs/file_structure.md)
