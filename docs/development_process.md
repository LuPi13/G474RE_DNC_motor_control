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
App/
Control/
Algorithm/
Platform/
Common/
Config/
docs/
```

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

최소 API 예:

```c
void pwm_driver_init(void);
void pwm_driver_enable(void);
void pwm_driver_disable(void);
void pwm_driver_set_duty(const abc_t *duty);
```

FOC나 SVPWM을 붙이지 않고 고정 duty를 직접 넣는다.

```c
abc_t duty = {
    .a = 0.25f,
    .b = 0.50f,
    .c = 0.75f,
};

pwm_driver_set_duty(&duty);
```

### 확인 항목

- PWM frequency
- center-aligned 동작
- 3상 timer synchronization
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

개념적 흐름:

```text
HRTIM event
    ↓
ADC trigger
    ↓
injected conversion
    ↓
conversion complete ISR
```

초기에는 raw sample만 검증해도 된다.

```c
typedef struct {
    uint16_t phase_a;
    uint16_t phase_b;
    uint16_t phase_c;
    uint16_t dc_link;
} adc_raw_sample_t;
```

### 확인 항목

- trigger source
- sampling point
- conversion complete timing
- channel ordering
- ADC saturation 여부
- ISR execution timing

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

초기에는 `adc_driver`에 conversion을 둘 수 있다.

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

Hall 예:

```text
GPIO state
    ↓
valid Hall state
    ↓
sector
    ↓
direction
    ↓
speed
    ↓
electrical angle
```

처음부터 estimator 전체를 만들 필요는 없다.

먼저:

```c
uint8_t hall_driver_get_state(void);
```

가 정확한지 확인한다.

그 다음 speed/angle estimation을 추가한다.

Encoder도 동일한 방식으로 raw acquisition → position → speed 순으로 진행한다.

### 완료 조건

회전 방향, 기계각/전기각, 속도의 의미와 단위가 일관되고 실제 회전과 맞는다.

---

# 9. Stage 5 — CORDIC Driver

CORDIC은 독립적으로 검증한다.

권장 API:

```c
void cordic_driver_sin_cos(
    float theta_rad,
    float *sin_theta,
    float *cos_theta);
```

Q31 변환은 driver 내부에 가둔다.

### 최소 test point

```text
0
pi/2
pi
-pi/2
```

`math.h`의 `sinf/cosf`와 비교하는 reference test를 권장한다.

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

## PI controller test

확인:

- proportional response
- integral accumulation
- saturation
- anti-windup
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
