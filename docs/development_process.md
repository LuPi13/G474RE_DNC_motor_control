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

현재는 `adc_driver_convert()`가 config의 영점과 gain으로
`(code - offset_counts) * gain_per_count`를 계산하여 `abc_t` 전류 [A]와
`float` DC-link 전압 [V]를 반환한다. App은 성공한 결과만 feedback으로 전달한다.

ADC 자체 calibration과 센서 영점/이득 보정은 별개다.
무전류 영점, 기준 전류/전압 대비 gain과 극성을 확인해야 하며,
raw 값이 들어온다는 사실만으로 센서 정확도 검증이 끝난 것은 아니다.

초기에는 이처럼 `adc_driver`에 conversion을 둘 수 있다.

복잡도가 증가하면:

```text
adc_driver.c
current_sensor.c
voltage_sensor.c
```

로 분리한다.

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

hall_driver_status_t hall_driver_get_rotor_feedback(
    const hall_driver_t *self,
    hall_driver_rotor_feedback_t *feedback);

hall_estimator_status_t hall_estimator_init(hall_estimator_t *self);

hall_estimator_status_t hall_estimator_update(
    hall_estimator_t *self,
    const hall_estimator_observation_t *observation,
    float elapsed_s,
    hall_estimator_output_t *output);
```

Board/motor별 config에는 TIM handle, Hall A/B/C GPIO mapping, 정방향 Hall sequence,
prescaler 적용 전 TIM kernel clock 및 electrical angle offset을 둔다. 현재 정방향
sequence는 다음과 같다.

```text
101 -> 100 -> 110 -> 010 -> 011 -> 001 -> 101
sector 0   1      2      3      4      5
```

정방향은 물리적인 시계/반시계 방향으로 고정된 의미가 아니라 이 sequence가 증가하는
사용자 정의 방향이다. Sector는 이 순서에 붙인 논리 번호이며 절대 기계 위치가 아니다.

현재 기본 각도 모델은 sensor가 정확히 60 electrical degree 간격으로 배치되었다고
가정한다. 유효 transition 뒤 sector 중심에서 정방향은 `-pi/6`, 역방향은 `+pi/6`인
경계각을 사용한다. Electrical offset은 실제 FOC 전에 rotor flux와 phase 기준으로
별도 보정해야 한다.

현재 bring-up의 TIM2 counter는 170 MHz timer kernel clock과 prescaler `16`으로
10 MHz이며, auto-reload `9,999,999`를 사용해 약 1초의 timeout을 만든다. 이 값은
저속 측정 범위와 정지 판정 지연의 trade-off이므로 motor/application 요구에 따라
CubeMX config와 driver 입력을 함께 검토한다.

### 확인 순서

1. Hall A/B/C mapping과 `000`, `111` invalid state 검출을 확인한다.
2. 손으로 정·역회전하며 Hall sequence, sector 및 direction을 확인한다.
3. 연속된 두 유효 edge에서 signed `omega_e_rad_s`의 크기와 부호를 확인한다.
4. Timeout 시 `omega_e_rad_s == 0`, `is_timed_out == true`가 되는지 확인한다.
5. Timeout 직후 첫 edge에서는 속도가 무효이고 다음 edge부터 다시 유효해지는지 확인한다.
6. `invalid_state_count`, `invalid_transition_count`, `timeout_count`가 의도한 사건에만 증가하는지 확인한다.
7. ADC가 TIM2 ISR을 선점하는 구성에서 `hall_driver_get_rotor_feedback()`과
   `hall_driver_get_feedback()`이 완성된 snapshot만 반환하는지 확인한다.
8. `hall_estimator`가 정방향에서는 증가, 역방향에서는 감소하는 연속 전기각을 만드는지 확인한다.
9. `0`과 `2*pi` 경계에서 전기각이 [0, 2*pi) 범위로 정상 wrap되는지 확인한다.
10. 다음 Hall edge가 늦으면 이동량이 `pi/3`으로 제한되고 `is_sector_limited`가 true가 되는지 확인한다.

Hall edge 사이의 continuous angle extrapolation은 현재 Control의 `hall_estimator`가
담당한다. 직전 signed electrical speed를 적분하고 새 edge에서 동기화하며, 추정 이동량은
이상적인 한 Hall sector 폭인 `pi/3`을 넘지 않는다. Filtering, hysteresis, sensor별 위치
보정 및 Hall/encoder/EEMF 공통 선택은 아직 지원하지 않으며, 필요해지면 별도 estimator
정책 또는 공통 `rotor_estimator`로 확장한다.

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

### 완료 조건

잘못된 상태에서 PWM이 확실히 safe state로 들어간다.

---

# 14. Stage 10 — FOC Current Loop

FOC는 current-control subsystem으로 구현한다.

```text
i_abc
  ↓
Clarke
  ↓
Park
  ↓
i_d / i_q
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
