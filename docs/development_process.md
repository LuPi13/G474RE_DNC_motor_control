# Development Process

## 1. 목적

이 문서는 STM32G474 기반 3상 SVPWM/FOC 모터드라이브 펌웨어를 실제 하드웨어에서 단계적으로 bring-up하고 검증하기 위한 개발 순서를 정의한다.

핵심 원칙은 다음과 같다.

> 한 계층을 전부 완성한 뒤 다음 계층으로 넘어가지 않는다.  
> 작은 기능 단위로 아래 계층부터 구현하고, 실제 하드웨어 또는 unit test로 검증한 뒤 바로 위 계층과 연결한다.

즉, **bottom-up + vertical slice** 방식으로 개발한다.

---

## 2. 개발 기본 원칙

### MUST

- hardware driver는 실제 하드웨어 동작을 확인한 뒤 다음 단계로 넘어간다.
- Control algorithm은 가능한 한 hardware-independent test를 먼저 수행한다.
- 실제 전력 인가 전에 PWM disable/fault path를 검증한다.
- current loop를 speed/position loop보다 먼저 완성한다.
- 구현 중 module boundary가 부적절한 것이 확인되면 임시방편으로 dependency를 깨지 말고 architecture를 재검토한다.
- 각 milestone에서 "어디까지 검증되었는지"를 Git commit 또는 tag로 식별 가능하게 만든다.

### SHOULD

- 한 번에 많은 module을 구현하지 않는다.
- driver, algorithm, control을 작은 vertical slice로 연결하며 진행한다.
- 오실로스코프, debugger, GPIO timing pin, unit test 등을 적극적으로 사용한다.
- 실제 모터를 연결하기 전에 가능한 범위의 open-loop 검증을 완료한다.

---

# 3. 전체 개발 단계

권장 순서:

```text
0. Project skeleton / conventions
        ↓
1. PWM driver
        ↓
2. ADC driver + PWM synchronization
        ↓
3. Current / voltage sensing conversion
        ↓
4. Rotor feedback driver
        ↓
5. CORDIC driver
        ↓
6. Transform / PI controller
        ↓
7. SVPWM
        ↓
8. Open-loop inverter integration
        ↓
9. Fault / safe shutdown
        ↓
10. FOC current loop
        ↓
11. Speed loop
        ↓
12. Position loop
        ↓
13. State machine / communication / diagnostics
```

모든 Platform module을 먼저 완성할 필요는 없다.

예를 들어 CAN/UART가 모터 구동 bring-up에 당장 필요하지 않다면 뒤로 미룬다.

---

# 4. Stage 0 — Project Skeleton

처음에 구현하는 것은 동작 코드보다 프로젝트 구조다.

예:

```text
Core/
    Inc/        # CubeMX 생성 영역
    Src/        # CubeMX 생성 영역
    App/        # 이하 사용자 작성 계층
    Control/
    Algorithm/
    Platform/
    Common/
    Config/
docs/
```

이하 계층 경로는 `Core/` 기준이다. 배치와 생성 코드의 경계는
[`file_structure.md`](file_structure.md)를 따른다.

최소한 다음 interface/header를 먼저 잡을 수 있다.

```text
Common/
    vector_types.h

Platform/
    pwm_driver.h
    adc_driver.h
    cordic_driver.h
    hall_driver.h

Algorithm/
    transform.h
    pi_controller.h
    svpwm.h

Control/
    foc.h
    speed_controller.h
    position_controller.h
    motor_control.h
```

이 단계의 목적은 구현을 미리 확정하는 것이 아니라:

- module responsibility
- dependency direction
- input/output type
- naming

을 먼저 명확히 하는 것이다.

### 완료 조건

- 프로젝트가 clean build 된다.
- 기본 디렉터리와 문서가 존재한다.
- 주요 module의 책임을 설명할 수 있다.

---

# 5. Stage 1 — PWM Driver

가장 먼저 실제 하드웨어에서 검증할 module로 `pwm_driver`를 권장한다.

현재 API 형태 (전체 계약은 [`pwm_driver.h`](../Core/Platform/pwm_driver.h) 참조):

```c
pwm_driver_status_t pwm_driver_init(pwm_driver_t *self, const pwm_driver_config_t *config);
pwm_driver_status_t pwm_driver_enable(pwm_driver_t *self);
pwm_driver_status_t pwm_driver_disable(pwm_driver_t *self);
pwm_driver_status_t pwm_driver_set_duty(pwm_driver_t *self, const abc_t *duty);
```

FOC나 SVPWM을 붙이지 않고 고정 duty를 직접 넣는다.
다음은 `pwm_driver` instance의 초기화 성공 후 호출하는 예다.

```c
abc_t duty = {
    .a = 0.25f,
    .b = 0.50f,
    .c = 0.75f,
};

pwm_driver_status_t status = pwm_driver_set_duty(&pwm_driver, &duty);
if (status != PWM_DRIVER_STATUS_OK) {
    /* Bring-up 오류를 기록하고 output을 활성화하지 않는다. */
    return;
}
```

`pwm_driver_init()`은 counter를 시작하지만 output은 비활성 상태로 둔다.
초기 duty의 update 반영을 확인한 뒤 `pwm_driver_enable()`을 호출하고 반환값을 확인한다.
ADC와 연동할 때는 counter 시작 전 ADC를 준비해야 하므로
[`runtime_and_dataflow.md`](runtime_and_dataflow.md)의 시작 순서를 따른다.

### 확인 항목

- PWM frequency
- center-aligned 동작
- 3상 timer synchronization
- preload/update 설정과 세 compare 쓰기의 update deadline
- high-side / low-side channel mapping
- complementary output
- dead time
- duty 0 / 1 근처 behavior
- enable / disable
- safe output state
- fault/break 입력이 있다면 즉시 차단 여부

### 검증 방법

- 오실로스코프
- logic analyzer
- 필요 시 GPIO timing pin

### 완료 조건

PWM driver API만 사용해서 예상한 3상 PWM을 안정적으로 출력할 수 있다.

---

# 6. Stage 2 — ADC Driver + PWM Synchronization

ADC 자체보다 **PWM에 동기화된 sampling**이 핵심이다.

현재 구현의 수집 경로:

```text
HRTIM 전류 trigger -> 세 ADC injected 변환 -> 각 완료 callback
                                                ↓
                                 ADC driver가 3상 완료 취합
                                                ↓
HRTIM 전압 trigger -> 단일 regular 변환 -> 완료된 Vdc 결과를 DR에서 읽기
                                                ↓
                                       raw sample 묶음
```

전류와 전압의 trigger 시점은 같다고 가정하지 않는다. Vdc는 DMA 없이 읽으며,
읽는 함수가 변환을 시작하거나 polling 대기하지 않는다.
`adc_driver_init()`은 설정 검증과 ADC 자체 calibration을 수행하고,
`adc_driver_start()`가 regular 및 injected 그룹을 외부 trigger 대기 상태로 준비한다.
보정은 신호 수가 아니라 실제 ADC와 사용 입력 모드에 맞춰 수행한다.

초기에는 `adc_driver_raw_sample_t`의 `phase_a`, `phase_b`, `phase_c`, `dc_link`만 검증해도 된다.
현재 보드의 매핑/환산 계수 예와 지원 구성은
[`adc_driver.h`](../Core/Platform/adc_driver.h)를 참조한다.
Callback 연결, sample 소비 및 오류 처리 조건은
[`runtime_and_dataflow.md`](runtime_and_dataflow.md)를 따른다.

### 확인 항목

- trigger source
- sampling point
- conversion complete timing
- channel ordering
- ADC saturation 여부
- ISR execution timing
- 한 전류 수집 주기당 3상 완료 판정/묶음 소비가 한 번씩 이루어지는지
- Vdc 변환 완료와 DR 읽기 시점, NOT_READY/overrun 발생 여부
- callback 누락/중복 및 실행 deadline 위반의 검출/복구 경로

ISR 진입 시 debug GPIO를 toggle하여 PWM과 timing을 함께 관찰하는 방법을 권장한다.

### 완료 조건

지정한 PWM 위치에서 ADC sample이 재현성 있게 취득된다.

---

# 7. Stage 3 — Sensor Conversion

ADC count를 실제 물리량으로 변환한다.

```text
ADC raw count
    ↓
offset correction
    ↓
gain / scaling
    ↓
current [A], voltage [V]
```

예:

```text
i_a = ...
i_b = ...
i_c = ...
v_dc = ...
```

현재 `adc_driver`는 ADC peripheral 매핑과 raw sample 취합만 담당한다.
`current_sensor`는 상별 `(code - offset_counts) * gain_a_per_count`를 계산하여
`abc_t` 전류 [A]를 반환하고, `voltage_sensor`는 DC-link 전압 [V]를 반환한다.
App은 이 module의 성공한 SI 결과만 feedback으로 전달한다.

ADC 자체 calibration과 센서 영점/이득 보정은 별개다.
무전류 영점, 기준 전류/전압 대비 gain과 극성을 확인해야 하며,
raw 값이 들어온다는 사실만으로 센서 정확도 검증이 끝난 것은 아니다.

현재 기동 경로는 PWM output을 끈 채 HRTIM counter의 ADC trigger만 실행한다. App은 raw
sample을 `current_sensor`에 전달하고, `current_sensor`가 settling sample을 버린 뒤 3상
offset을 평균하여 허용 범위 안의 결과만 소유한다. 평균 완료나 timeout 확인 전에는
open-loop와 PWM output을 활성화하지 않는다. 범위 오류나 timeout은 current-sensor fault로
latch한다. 운전 중 재보정이 필요하면 PWM 차단만으로 충분하다고 가정하지 말고 실제 상전류
감쇠와 rotor 정지를 확인한 뒤 같은 절차를 수행한다.

Offset calibration timeout은 임의의 고정값으로 두지 않는다. 최소 필요시간은

```text
(settling_sample_count + averaging_sample_count) / actual_sample_rate
```

이며 timeout은 이 값보다 반드시 커야 한다. Integer ms로 계산할 때는 올림하고 startup
scheduling과 일시적인 sample 누락을 위한 명시적 margin을 더한다. Sample 수나 ADC rate를
변경하면 timeout도 같은 설정에서 다시 계산되도록 한다. 현재 128 settling + 2048 averaging,
40 kHz 조건의 이상적인 최소시간은 54.4 ms이므로 10 ms timeout은 유효하지 않다.

센서 보정과 환산이 ADC peripheral 수집과 독립적으로 바뀌므로 현재 구현은 다음처럼 분리한다.

```text
adc_driver.c
current_sensor.c
voltage_sensor.c
```

`current_sensor`만 raw offset, 보정 누적 상태와 gain을 소유한다. App이나 `adc_driver`에 같은
offset 또는 누적 상태를 복제하지 않는다.

### 완료 조건

Debugger에서 raw count가 아니라 실제 SI 값으로 신뢰 가능한 feedback을 읽을 수 있다.

---

# 8. Stage 4 — Rotor Feedback

사용할 sensor부터 최소 기능으로 검증한다.

현재 Hall 구성의 수집/추정 경로:

```text
TIM2 CH1/CH2/CH3의 Hall A/B/C
    ↓
XOR edge capture / counter overflow timeout
    ↓
GPIO state (A/B/C = bit 2/1/0)
    ↓
motor-specific Hall profile / decoder
    ↓
valid Hall state / sector
    ↓
direction
    ↓
edge-to-edge electrical speed
    ↓
sector boundary electrical angle
    ↓
완성 feedback snapshot publish
```

현재 API 형태의 전체 계약은
[`hall_driver.h`](../Core/Platform/hall_driver.h)를 따른다.

```c
hall_driver_status_t hall_driver_init(
    hall_driver_t *self,
    const hall_driver_config_t *config);

hall_driver_status_t hall_driver_start(hall_driver_t *self);

hall_driver_status_t hall_driver_handle_capture(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim);

hall_driver_status_t hall_driver_handle_timeout(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim);

hall_driver_status_t hall_driver_get_feedback(
    const hall_driver_t *self,
    hall_driver_feedback_t *feedback);

hall_driver_status_t hall_driver_get_signal_feedback(
    const hall_driver_t *self,
    hall_driver_signal_feedback_t *feedback);

hall_decoder_status_t hall_decoder_init(
    hall_decoder_t *self,
    const hall_decoder_profile_t *profile);

hall_decoder_status_t hall_decoder_update(
    hall_decoder_t *self,
    const hall_decoder_observation_t *observation,
    hall_decoder_output_t *output);

hall_estimator_status_t hall_estimator_init(hall_estimator_t *self);

hall_estimator_status_t hall_estimator_update(
    hall_estimator_t *self,
    const hall_estimator_observation_t *observation,
    float elapsed_s,
    hall_estimator_output_t *output);
```

Board config에는 TIM handle, Hall A/B/C GPIO mapping과 prescaler 적용 전 TIM kernel clock을
둔다. Motor/phase/Hall 배선별 profile에는 raw Hall state-to-sector lookup과 여섯 정방향
sector 진입 electrical edge angle을 둔다. 현재 motor profile의 정방향 sequence는 다음과 같다.

```text
101 -> 100 -> 110 -> 010 -> 011 -> 001 -> 101
sector 0   1      2      3      4      5
```

정방향은 물리적인 시계/반시계 방향으로 고정된 의미가 아니라 이 sequence가 증가하는
사용자 정의 방향이다. Sector는 이 순서에 붙인 논리 번호이며 절대 기계 위치가 아니다.

이상적인 Hall 배치는 sector 0 정방향 진입 edge offset과 `pi/3` 간격으로 profile을 만들 수
있다. 다른 motor에서는 정·역방향 저속 commissioning으로 여섯 실제 경계각을 측정한다.
Decoder는 인접 경계각의 차이로 실제 sector span과 edge-to-edge speed를 계산하며,
`hall_estimator`도 같은 span을 sector 제한에 사용한다.

Raw state `000`과 `111`의 유효성은 driver가 고정하지 않는다. Profile의 8-entry lookup에서
정확히 여섯 state를 sector 0부터 5에 중복 없이 mapping하고 나머지 두 state를 invalid로 둔다.
따라서 Hall channel 순서, polarity 및 60-degree/120-degree coding 차이는 driver 수정이 아니라
profile 교체로 처리한다.

현재 bring-up의 TIM2 counter는 170 MHz timer kernel clock과 prescaler `16`으로
10 MHz이며, auto-reload `9,999,999`를 사용해 약 1초의 timeout을 만든다. 이 값은
저속 측정 범위와 정지 판정 지연의 trade-off이므로 motor/application 요구에 따라
CubeMX config와 driver 입력을 함께 검토한다.

### 확인 순서

1. Hall A/B/C raw bit mapping과 profile에서 invalid로 지정한 두 state 검출을 확인한다.
2. 손으로 정·역회전하며 Hall sequence, sector 및 direction을 확인한다.
3. 연속된 두 유효 edge에서 signed `omega_e_rad_s`의 크기와 부호를 확인한다.
4. Timeout 시 `omega_e_rad_s == 0`, `is_timed_out == true`가 되는지 확인한다.
5. Timeout 직후 첫 edge에서는 속도가 무효이고 다음 edge부터 다시 유효해지는지 확인한다.
6. Decoder의 `invalid_state_count`, `invalid_transition_count`, `missed_capture_count`와 driver의
   `invalid_capture_count`, `timeout_count`가 의도한 사건에만 증가하는지 확인한다.
7. ADC가 TIM2 ISR을 선점하는 구성에서 `hall_driver_get_signal_feedback()`과
   `hall_driver_get_feedback()`이 완성된 snapshot만 반환하는지 확인한다.
8. `hall_decoder`가 profile에 따라 raw state, sector, direction과 edge speed를 일관되게 만드는지 확인한다.
9. `hall_estimator`가 정방향에서는 증가, 역방향에서는 감소하는 연속 전기각을 만드는지 확인한다.
10. `0`과 `2*pi` 경계에서 전기각이 [0, 2*pi) 범위로 정상 wrap되는지 확인한다.
11. 다음 Hall edge가 늦으면 이동량이 현재 profile sector span으로 제한되고 `is_sector_limited`가 true가 되는지 확인한다.

Hall edge 사이의 continuous angle extrapolation은 현재 Control의 `hall_estimator`가
담당한다. 직전 signed electrical speed를 적분하고 새 edge에서 동기화하며, 추정 이동량은
decoder가 제공한 현재 sector span을 넘지 않는다. Filtering, hysteresis 및
Hall/encoder/EEMF 공통 선택은 아직 지원하지 않으며, 필요해지면 별도 estimator 정책 또는
공통 `rotor_estimator`로 확장한다.

### Hall electrical offset commissioning

Raw Hall state mapping, sector 순서, 정·역방향과 timeout 검증은 electrical offset 검증과
별도 단계다. FOC에 사용할 `forward_edge_angle_rad[k]`는 정방향으로 sector `k`에 진입하는
물리 Hall 경계의 rotor electrical angle [rad]이며, 초기 ideal `pi/3` profile은 이 기준을
보장하지 않는다.

임시 board test는 PWM을 활성화한 low-voltage open-loop에서 다음 계약으로 측정한다.

1. DC-link 최소 전압, 무부하, 고정된 보호 조건에서 작은 전압 vector와 저속 양의
   `omega_e_rad_s`를 적용한다. 전류는 trip threshold보다 충분히 낮게 유지한다.
2. 처음 한 electrical revolution의 edge는 동기화/가속 구간으로 버린다. 이후 정방향으로
   sector `k`에 진입할 때 사용한 `voltage_angle_rad`를 `k`번 경계 sample로 기록한다.
3. 정지 settling 뒤 같은 조건의 음의 `omega_e_rad_s`를 적용한다. 역방향으로 sector `k`에
   진입한 edge는 정방향 경계 `(k + 1) % 6`을 넘은 사건이므로 그 index에 기록한다.
4. 각 경계의 정·역방향 sample을 circular mean하여 `[0, 2*pi)`의
   `forward_edge_angle_rad[k]`를 만들고, 인접 차이로 sector span을 계산한다.

양의 stator `voltage_angle_rad` sweep에서 decoder가 `HALL_DECODER_DIRECTION_FORWARD`를
보고해야 profile의 증가 방향이 FOC electrical-angle 증가 방향과 일치한다. 반대 방향이면
측정값을 적용하지 않고 logical sector numbering을 반전한 뒤 처음부터 다시 측정한다.

이 시험은 rotor가 low-speed voltage vector를 따라가며 stator field angle과 rotor d-axis가
일치한다는 commissioning 가정을 사용한다. 연속 sweep에서는 torque-angle과 마찰 때문에
정방향과 역방향의 command angle이 공통적으로 벌어질 수 있다. 이 directional hysteresis의
절대값은 단독으로 실패 조건이 아니며, 같은 방향 안의 반복편차와 여섯 경계에서 hysteresis가
공통 offset으로 유지되는지를 따로 확인한다. 정·역방향 circular mean은 이 공통 offset을
상쇄한 profile 후보로 사용한다. 방향 내부 반복편차 또는 경계별 hysteresis 편차가 크면
전압/속도를 조정해 재측정한다. 결과는 먼저 debugger에서 검토하며, 임시 시험은 Flash 저장이나
`motor_config_hall_profile` 자동 변경을 수행하지 않는다.

현재 motor profile에는 같은 조건에서 반복한 두 측정의 경계별 평균을 적용했다.

```text
forward_edge_angle_rad =
    [0.616236031, 1.610801515, 2.666652325,
     3.741505025, 4.795457365, 5.775905845]
```

이 값은 현재 motor/phase/Hall 배선 조합에만 유효하다. Motor 또는 phase/Hall 배선을 바꾸면
raw state mapping과 함께 다시 측정하며, commissioning 임시 코드는 측정 완료 후 production
binary에서 제거한다.

### Hall과 encoder 선택

Encoder는 향후 별도 `encoder_driver.c/.h`로 구현한다. Hall mode와 encoder mode가
같은 TIM2를 사용하므로 한 firmware configuration에서 동시에 시작하지 않는다.
CubeMX에서 선택한 mode에 맞는 driver 하나만 App에서 초기화한다.

Hall feedback은 절대 기계 원점, multi-turn 위치 또는 위치제어에 필요한 연속 기계각을
제공하지 않는다. 따라서 현재 Hall-only configuration의 목표는 전기각/전기각속도 기반
FOC와 속도제어까지이며 위치제어는 지원 범위에서 제외한다. 위치제어 단계에서는 encoder
등 필요한 위치 관측 가능성을 제공하는 feedback 구성을 먼저 마련해야 한다.

Encoder도 raw acquisition → position → speed 순으로 작은 단계부터 검증한다.

### 완료 조건

선택한 feedback 장치가 제공하는 범위 안에서 회전 방향, 전기각/기계각 및 속도의 의미와
단위가 일관되고 실제 회전과 맞는다. Hall 구성에서는 invalid state/transition, timeout,
재동기화 동작과 FOC에 사용할 electrical offset까지 별도로 검증한다.

---

# 9. Stage 5 — CORDIC Driver

CORDIC은 독립적으로 검증한다.

권장 API:

```c
cordic_driver_status_t cordic_driver_init(void);

cordic_driver_status_t cordic_driver_sin_cos(
    float theta_rad,
    float *sin_theta,
    float *cos_theta);

cordic_driver_status_t cordic_driver_cartesian_to_polar(
    float x,
    float y,
    float *magnitude,
    float *theta_rad);
```

Q31 변환은 driver 내부에 가둔다. `sin_cos`는 [rad]를 받고 무차원 sine/cosine을
반환한다. `cartesian_to_polar`는 Q1.31 포화를 피하도록 두 입력을 같은 비율로 내부
scaling하고, 입력과 같은 단위의 magnitude 및 [-pi, pi) [rad] phase를 반환한다.

### 최소 test point

```text
0
pi/2
pi
-pi/2
```

`math.h`의 `sinf/cosf`와 비교하는 reference test를 권장한다.
직교-극좌표 변환은 축 방향, 사분면, 영벡터와 `(3, 4)`처럼 Q1.31 범위를 넘는
입력을 포함해 `sqrtf/atan2f` reference와 비교한다.

### 완료 조건

허용 오차 내에서 `float rad -> float sin/cos` interface가 신뢰 가능하다.

---

# 10. Stage 6 — Algorithm Layer

Hardware 없이 가능한 module을 먼저 검증한다.

`vector_types.h`는 Algorithm이 아니라 `Common/`의 공용 타입 정의이며 Stage 0에서 준비한다.

Algorithm 구현 권장 순서:

```text
transform
    ↓
limiter / rate_limiter
    ↓
pi_controller
    ↓
svpwm
```

## Transform test

최소 round-trip:

```text
abc -> alpha_beta -> abc
alpha_beta -> dq -> alpha_beta
```

Clarke scaling, sign convention, zero-sequence 가정을 명시한다.

현재 `transform` module은 a상 축과 alpha축을 일치시키는 amplitude-invariant
Clarke 변환을 사용한다. inverse Clarke는 `a + b + c = 0`을 가정하며,
Park 변환은 양의 전기각에 대해 alpha-beta vector를 `-theta`만큼 회전하는
부호 규약을 사용한다. Park와 inverse Park에는 같은 각도에서 한 번 계산한
`sin_theta`, `cos_theta`를 전달해 재사용한다.

## Limiter test

`limiter_clamp()`은 범위 안의 값과 상·하한 밖의 값을 확인한다.

`rate_limiter`는 다음을 확인한다.

- 비대칭 상승·하강 rate
- target 도달 시 overshoot 없이 정확히 정착
- runtime rate 변경 시 output 연속성 유지
- 0 rate 방향의 출력 정지
- reset
- 잘못된 rate/sampling period 거부

## PI controller test

확인:

- proportional response
- integral accumulation
- saturation
- scalar saturation의 back-calculation anti-windup
- 외부에서 추가 제한된 applied output tracking
- reset
- output min/max

### 완료 조건

각 module이 STM32 peripheral 없이 독립적으로 검증 가능하다.

---

# 11. Stage 7 — SVPWM

입력:

```c
alpha_beta_t v_ref;
float v_dc;
```

출력:

```c
abc_t duty;
```

SVPWM은 HRTIM register를 직접 쓰지 않는다.

현재 `svpwm` module은 inverse Clarke 뒤 `-(v_max + v_min) / 2`의 common-mode를
주입하는 min-max 방식을 사용한다. 상전압 span이 `v_dc`를 초과하면 duty를 상별로
clamp하지 않고 overmodulation을 반환한다. d/q voltage vector limitation과 PI tracking은
FOC에서 먼저 수행한다.

### 확인 항목

- sector transition
- duty continuity
- duty range
- modulation limit
- zero vector distribution
- 0 ~ 2*pi angle sweep에서 3상 duty 형태

가능하면 Python reference와 비교한다.

### 완료 조건

SVPWM 출력 duty가 수치적으로 검증된다.

---

# 12. Stage 8 — Open-Loop Inverter Integration

첫 번째 중요한 vertical slice:

```text
theta
  ↓
sin/cos
  ↓
v_alpha_beta
  ↓
SVPWM
  ↓
PWM driver
```

그 다음:

```c
theta += omega * sampling_period_s;
```

형태로 open-loop rotating voltage vector를 만든다.

현재 구현은 `Core/App/app.c/.h`의 `app_motor_fast_loop()`에서 ADC sample 소비/환산부터
CORDIC, SVPWM, PWM duty 기록까지 연결한다. `voltage_angle_rad`는 rotor angle이 아니라
인가할 alpha-beta 전압 vector의 phase이며, command는 전압 크기 [V]와 signed 전기각속도
[rad/s]로 구성한다. 초기 command는 0 V, 0 rad/s이고 `app_start_open_loop()` 전에는
ADC feedback만 갱신한다.

Main 쪽 command writer가 ADC ISR에 선점되어도 두 command field가 섞이지 않도록
double buffer로 전달한다. Writer는 단일 실행 문맥이고 ADC ISR보다 낮은 preemption
priority여야 한다. 범위를 벗어난 command는 이전 command를 유지하며, 실행 중 하위 module
오류는 open-loop 정지와 software PWM disable로 이어진다.

### 검증 목적

FOC current loop 없이도 다음 stack을 확인할 수 있다.

```text
angle generation
    +
SVPWM
    +
HRTIM PWM
```

### 주의

실제 모터/전력 인가는 낮은 voltage/current 조건에서 점진적으로 진행한다.

---

# 13. Stage 9 — Fault / Safe Shutdown

실제 전력 인가 전 반드시 검증한다.

최소 항목:

- software PWM disable
- HRTIM fault/break
- over-current input
- driver fault
- invalid sensor state
- DC-link over-voltage
- emergency stop path

개념:

```text
fault detected
    ↓
safe output
    ↓
fault latch / state transition
```

hardware emergency path는 software state machine보다 빠르게 동작할 수 있어야 한다.

현재 PCB에는 별도의 gate-driver fault/HRTIM fault 입력이 연결되어 있지 않다. 따라서 현재
vertical slice는 App 계층 `fault_manager`에서 다음 software 보호를 먼저 제공한다.

- 각 상 `|i_phase| >= 3.0 A`에서 과전류 latch
- `v_dc >= 79.2 V`에서 DC-link 과전압 latch
- ADC/rotor/motor-control/CORDIC/SVPWM/PWM 오류 원인 latch
- Fault 발생 시 active mode 중지와 `pwm_driver_disable()`
- PWM 비활성, 0 command, 정상 측정 조건을 확인한 명령 기반 latch 해제
- 해제 뒤 자동 재시작 금지

상전류 `1.0 A`, DC-link `75.0 V`를 현재 clear hysteresis 기준으로 사용한다. 이 값들은
제품/보드 설정이며 향후 Config 계층으로 이동할 수 있다. ADC 동기 오류처럼 정상 sample이
재개되지 않는 경우에는 명령 clear보다 먼저 ADC 재동기화나 MCU reset이 필요하다.

현재 fault manager에는 DC-link 저전압 trip이 없다. 따라서 10 V 저전압 차단값도 아직
적용하지 않으며, gate-driver 동작 범위와 brownout/회생 조건을 포함한 별도 보호 정책으로
추가할 때 trip/clear hysteresis와 mode start 조건을 함께 정의한다.

이 software 보호를 hardware 과전류 보호로 간주하지 않는다. 향후 PCB revision에서는
gate-driver fault 또는 COMP 출력에서 HRTIM fault까지 이어지는 CPU 독립 경로를 추가하고
실제 신호를 강제로 인가해 output safe state를 검증해야 한다.

### 완료 조건

현재 PCB에서는 software fault를 강제로 발생시켜 PWM disable, latch, 차단된 clear,
정상 clear 및 자동 재시작 금지를 검증한다. HRTIM hardware fault까지 추가되는 보드에서는
software 실행과 무관하게 PWM이 safe state로 들어가는 것도 별도로 검증한다.

---

# 14. Stage 10 — FOC Current Loop

FOC는 current-control subsystem으로 구현한다.

FOC에 전달할 d/q 전류 지령은 `motor_control`에서 먼저 조정한다.

```text
Current command 또는 Speed PI output
  ↓
d/q axis 및 magnitude saturation
  ↓
d/q rate limiter
  ↓
이전 지령 기준 최종 magnitude saturation과 limiter state 동기화
  ↓
i_dq_ref
```

현재 `motor_control`은 reference conditioning 뒤 자신이 소유한 FOC를 실행하며 App fast loop가
Hall estimator의 rotor feedback, `i_abc`, `v_dc`를 전달한다. 최종 `v_alpha_beta_ref`는 App에서
SVPWM과 PWM driver로 이어진다. 초기 제품 설정은 d/q vector 지령 상한 2 A, 변화율 100 A/s,
software 상전류 trip 3 A다. 정상 stop은 0 A target으로 ramp할 수 있고 Fault/emergency stop은
reference ramp를 기다리지 않고 즉시 PWM을 차단한다.

```text
i_abc
  ↓
Clarke
  ↓
Park
  ↓
i_d / i_q
  ↓
d/q IIR low-pass (제어 feedback 경로)
  ↓
d/q PI
  ↓
decoupling / feedforward
  ↓
voltage limitation
  ↓
inverse Park
  ↓
v_alpha_beta_ref
```

Digital current filter는 Clarke/Park 뒤의 `i_d`, `i_q` feedback에 적용한다. 상전류
과전류 보호는 지연된 filter 출력이 아니라 offset 보정만 끝난 unfiltered `i_abc`를 사용한다.
센서 출력과 MCU ADC 사이의 analog RC filter는 anti-alias/noise 제한용 hardware 경계로
취급하며, digital IIR의 상태나 coefficient를 `current_sensor` 또는 `adc_driver`가 소유하지 않는다.

`Core/Algorithm/filter.c/.h`에는 fixed-step scalar 1차 저역통과 IIR 계산과 상태 관리가
구현되어 있다. 이 module은 cutoff 기본값이나 d/q 의미를 소유하지 않는다. FOC 통합 시
FOC가 `i_d`, `i_q`용 instance를 각각 소유하고 첫 유효 feedback으로 초기화한다.

`Core/Control/foc.c/.h`는 위 current-control 계산을 hardware-independent subsystem으로
구현한다. d/q filter/PI state, 선택적 motor-model decoupling, 원형 전압 제한과 external
anti-windup tracking을 소유하며 출력은 `v_alpha_beta_ref`까지다. 40 kHz 실행, 500 Hz/1 kHz
current-loop bandwidth, motor parameter ±30%, 2-sample voltage delay 및 3 A 포화 후 복귀를
포함한 standalone 수치 검증을 완료했다. `motor_control`과 App fast loop, SVPWM/PWM 경로의
software 통합을 완료했고, 보정한 Hall profile을 사용한 저전류 board 폐루프 검증도 수행했다.
Current mode는 여전히 일반 기동 시 자동으로 시작하지 않으며 명시적인 상위 명령으로 진입한다.

현재 PCB의 ACS725 VIOUT와 MCU ADC 사이에는 47 Ω series resistor와 ADC 입력의 1 nF
capacitor가 있으며 계산상 RC cutoff는 약 3.39 MHz다. 이 RC는 40 kHz sampling의 주된
anti-alias filter가 아니라 ADC sampling kickback과 고주파 EMI를 줄이는 hardware 경계로
취급한다. 제어에 사용할 digital IIR cutoff는 측정 noise와 목표 current-loop bandwidth를
기준으로 정한 뒤 제품별 config에서 전달하며 Algorithm/FOC 내부 기본값으로 숨기지 않는다.

### 저전류 current-mode board 검증 결과

무부하에서 `i_d_ref = 0`을 유지하고 작은 양·음 `i_q_ref`를 적용했다. 측정된 q축 전류는
지령을 같은 부호로 추종했고 정·역방향 회전이 모두 정상적으로 확인됐다. 약 `0.28 A` 이상에서
축이 자력으로 회전하기 시작했으며, 이는 작은 지령에서 Hall edge가 갱신되지 않는 구간과
정지마찰을 넘는 기동 토크가 필요함을 보여준다.

회전 속도가 올라간 뒤 `i_q_ref = 0.3 A` 부근에서는 `is_voltage_saturated`가 간헐적으로
동작했고, `0.4 A` 지령에서는 계속 true였다. 이때 실제 q축 전류는 약 `0.3 A` 부근에 머물렀다.
이는 속도제어 없이 torque current를 계속 인가하여 회전수가 올라가고, 역기전력 때문에 사용
가능한 전압 한계에 도달한 조건과 일치한다. 시험 중 software fault와 fast-loop deadline miss는
발생하지 않았다.

이 결과로 저전류 범위의 current-feedback 부호, Park/Hall angle 방향, 양방향 torque 생성,
전압 제한 진입은 확인했다. 정량적인 1 kHz bandwidth, 2 A 지령 전 범위, 부하 급변 및 최소·정격·
최대 DC-link 조건은 아직 검증하지 않았으므로 이 시험 결과로 확장해 주장하지 않는다. 시험용
debugger 지령과 FOC snapshot 코드는 확인 후 production `main.c`에서 제거한다.

그 다음:

```text
SVPWM
  ↓
PWM
```

처음에는 Current mode만 구현한다.

```text
i_d_ref = 0
i_q_ref = small value
```

### 확인 항목

- d/q sign
- rotor angle convention
- current-loop stability
- current limit
- voltage saturation
- anti-windup
- ISR execution time

### 완료 조건

작은 `i_q_ref` step을 안정적으로 추종한다.

---

# 15. Stage 11 — Speed Loop

Current loop이 충분히 안정화된 뒤 진행한다.

현재 motor의 초기 speed-loop 설계 기준은 다음과 같다.

```text
mechanical speed range: 300 to 3000 rpm (초기 검증 범위)
speed-loop rate: 1 kHz
initial bandwidth: 5 Hz
speed-feedback low-pass cutoff: 30 Hz
acceleration/deceleration limit: 300 rpm/s
bring-up q-axis current limit: +/-0.5 A
product q-axis current limit: +/-2.0 A
```

Motor rotor inertia는 `8.6e-6 kg*m^2`, 초기 최대 load inertia는 그 5배로 두어 총 관성을
`5.16e-5 kg*m^2`로 사용한다. `K_t = 1.5 * pole_pairs * lambda_f = 0.05055 N*m/A`와
5 Hz, damping ratio 약 0.707을 적용한 초기 speed PI 후보는 `Kp = 0.0453 A/(rad/s)`,
`Ki = 1.007 A/rad`, `Kaw = 22.2 1/s`다. 실제 발전기 부하와 관성 변화에 따라 config에서
gain과 reference rate를 교체할 수 있어야 한다.

Speed PI는 40 kHz ADC ISR에 직접 추가하지 않고 별도 1 kHz scheduler context에서 실행하여
double-buffered q축 current target을 publish한다. Fast current loop는 마지막으로 완성된 target만
소비한다. 정지 명령은 speed reference를 0까지 ramp하며 회생 제동한 뒤 q축 current를 0으로
만들고 PWM을 비활성화한다. 정지 후 shaft position hold는 제공하지 않는다.

`speed_controller` 자체는 mechanical-speed feedback의 30 Hz low-pass filter와 PI, q축 전류
scalar 제한 및 downstream current 제한용 external tracking만 소유한다. 300 rpm/s speed-reference
rate limiter, electrical-to-mechanical speed 변환과 1 kHz 실행/publish는 `motor_control`과 App
integration 단계에서 연결한다. 따라서 standalone controller의 수치 검증은 먼저 진행하되,
40 kHz current fast path에는 speed PI를 직접 삽입하지 않는다. 초기 gain, 1 ms 주기, 30 Hz
feedback cutoff와 ±0.5 A bring-up 출력 범위는 `motor_config_speed_controller`에 둔다.

2026-09-15 board shadow 시험에서는 PWM과 App drive mode를 비활성 상태로 유지하고 실제 Hall
속도를 1 kHz main-loop 시험 경로에서 controller에 입력했다. 손으로 축을 정·역회전했을 때
기계속도 feedback, filtered speed, speed error와 `i_q_ref`의 부호가 예상과 일치했고, ±0.5 A
PI 포화, ±3000 rpm command clamp, reset 및 Hall timeout 처리도 정상 동작했다. 시험용
controller/output은 current-reference 또는 PWM에 전달하지 않았다. 해당 임시 코드는 검증 후
`main.c`에서 제거했다.

3000 rpm에서 Hall edge rate는 1500 Hz이고 rotor electrical speed는 약 `1571 rad/s`다.
24 V DC-link에서는 현재 0.9 SVPWM voltage utilization 기준으로 3000 rpm을 우선 검증한다.
저속 한계는 아직 확정하지 않았으므로 300 rpm부터 낮추며 Hall edge-to-edge speed의 품질을
확인하고, 필요하면 Hall PLL 또는 별도 speed estimator를 후속 단계로 추가한다.

```text
omega_ref
  ↓
speed controller
  ↓
i_q_ref
  ↓
FOC
```

확인:

- acceleration/deceleration
- torque/current limit
- speed PI saturation
- anti-windup
- sign convention

### 완료 조건

속도 reference를 안정적으로 추종한다.

---

# 16. Stage 12 — Position Loop

마지막 cascade loop:

```text
theta_ref
  ↓
position controller
  ↓
omega_ref
  ↓
speed controller
  ↓
i_q_ref
  ↓
FOC
```

위치제어기가 speed controller를 직접 include하지 않는다.

`motor_control`이 cascade를 orchestration한다.

이 단계는 feedback 장치가 위치제어에 필요한 연속 기계각과 원점 기준을 제공할 때만
진행한다. 현재 Hall-only configuration은 이 조건을 만족하지 않으므로 위치제어 mode를
제공하지 않는다. 향후 encoder를 사용할 때 원점/index 처리와 multi-turn 정책을 먼저 정한다.

---

# 17. Stage 13 — State Machine / Communication / Diagnostics

기본 구동 stack이 안정화된 뒤:

- CAN
- UART
- command protocol
- telemetry
- calibration mode
- fault reporting
- parameter configuration
- startup sequence
- system state machine

을 고도화한다.

통신이 bring-up에 꼭 필요하면 필요한 최소 기능만 앞 단계에서 먼저 구현할 수 있다.

---

# 18. Milestone 예

```text
M0  project skeleton
M1  three-phase PWM verified
M2  synchronized ADC verified
M3  SI-unit sensing verified
M4  rotor feedback verified
M5  SVPWM open-loop verified
M6  safe shutdown verified
M7  current control verified
M8  speed control verified
M9  position control verified
```

Git tag를 사용할 경우 예:

```text
bringup-pwm-v1
bringup-adc-sync-v1
bringup-hall-v1
open-loop-svpwm-v1
current-control-v1
speed-control-v1
```

---

# 19. 한 단계에서 막혔을 때

다음 단계로 넘어가서 문제를 덮지 않는다.

예를 들어 current control이 이상하다면 speed controller를 추가하지 않는다.

원인을 다음 계층 순서로 다시 확인한다.

```text
measurement
    ↓
angle
    ↓
transform
    ↓
controller
    ↓
SVPWM
    ↓
PWM
```

한 단계 아래의 검증 가능한 interface부터 다시 확인한다.

---

# 20. Definition of Done

각 기능의 "완료"는 단순히 코드가 작성되었다는 뜻이 아니다.

최소한:

- build 성공
- 예상 동작 검증
- 단위/범위 확인
- fault/edge case 검토
- 필요한 Doxygen 작성
- 관련 문서 변경 반영
- 의미 있는 Git commit 생성

까지를 하나의 개발 단위로 본다.

PWM 동기 fast loop에 영향을 주는 변경은 위 항목에 더해 다음을 모두 만족해야 한다.

- `real_time_execution_budget.md`에 정의된 변경 전후 cycle 측정
- 정상 및 worst-case 전체 maximum 기준 통과
- deadline miss 0 확인
- linked image disassembly와 stack usage 검토

이 timing gate를 통과하기 전에는 current-loop 실구동 확대, speed/position loop의 40 kHz
fast-path 통합 또는 speed-mode PWM 활성화로 진행하지 않는다. Hardware-independent controller와
PWM 비활성 shadow 시험은 `real_time_execution_budget.md`에 정의한 분리 조건에서 먼저 수행할
수 있다.
