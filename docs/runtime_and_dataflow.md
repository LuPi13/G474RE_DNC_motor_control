# Runtime, Data Flow, and Scheduling

## 1. Command / Feedback / Output 분리

외부 명령, feedback, control runtime state를 가능한 한 구분한다.

예:

```c
typedef struct {
    float theta_m_ref_rad;
    float omega_m_ref_rad_s;
    float i_d_ref;
    float i_q_ref;
} motor_command_t;

typedef struct {
    abc_t i_abc;
    float v_dc;
    float theta_m_rad;
    float omega_m_rad_s;
    float theta_e_rad;
    float omega_e_rad_s;
} motor_feedback_t;

typedef struct {
    abc_t duty;
} motor_control_output_t;
```

`motor_control_t`에는 controller state를 둔다.

```c
typedef struct {
    motor_control_mode_t mode;

    position_controller_t position_controller;
    speed_controller_t speed_controller;
    foc_t foc;
} motor_control_t;
```

하나의 거대한 `sMotor`에 hardware instance, sensor raw data, command, controller state, fault/state machine을 모두 몰아넣지 않는다.

---

## 2. Single source of truth

같은 의미의 runtime 값을 여러 subsystem에 중복 저장하고 수동 동기화하지 않는다.

지양:

```text
hall.electrical_angle
motor.electrical_angle
foc.electrical_angle
```

중 하나가 갱신되지 않으면 즉시 inconsistency가 생긴다.

권장:

```text
rotor estimator
      |
      v
motor_feedback.theta_e_rad
      |
      v
motor_control / FOC
```

각 module의 내부 state가 필요한 경우에도 외부에 전달되는 canonical output의 owner를 명확히 한다.

현재 Hall 구성에서는 `hall_driver`가 TIM capture/timeout으로부터 만든 최신
`hall_driver_feedback_t` snapshot의 owner다. App은 fast loop가 시작될 때
`hall_driver_get_feedback()`으로 이 snapshot을 읽어 해당 제어 주기의
`motor_feedback`을 구성한다. Control/FOC는 driver 내부 buffer를 직접 참조하거나
별도의 Hall 각도를 독립적으로 갱신하지 않는다.

`main.c`의 `hall_test_feedback`은 hardware bring-up 중 debugger 관찰을 위한 임시
복사본이며 canonical runtime feedback으로 사용하지 않는다.

---

## 3. Fast loop

개념적 흐름:

```text
PWM timing event
   ->
ADC sampling
   ->
ISR/callback
   ->
ADC driver가 3상 injected 완료 취합
   ->
app_motor_fast_loop()
   ->
feedback acquire/convert
   ->
Hall feedback snapshot acquire
   ->
rotor estimate/update when configured
   ->
motor_control fast update
   ->
SVPWM
   ->
PWM duty write
```

HAL callback에는 로직을 길게 작성하지 않는다.

현재 전류는 서로 다른 세 ADC의 injected 변환으로 수집한다. 따라서 callback마다
fast loop를 실행하지 않고, driver가 세 상의 완료를 취합한 뒤 한 번만 호출한다.

향후 App 연결 예 (`adc_driver`는 초기화/시작된 instance):

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    bool is_complete = false;
    adc_driver_status_t status = adc_driver_handle_injected_complete(
        &adc_driver, hadc, &is_complete);

    if (status != ADC_DRIVER_STATUS_OK) {
        /* App의 오류 기록/보호 경로로 전달한다. 제어 계산은 실행하지 않는다. */
        return;
    }

    if (is_complete) {
        app_motor_fast_loop();
    }
}
```

위 예시는 연결 구조이며 오류 처리 구현까지 포함하지 않는다. HAL 오류 callback도
App의 오류 처리 경로에 연결해야 한다.

HAL callback 정의는 `Core/Src/main.c`의 CubeMX USER CODE 영역에 두고,
실제 orchestration은 `Core/App/app.c`에 둔다. 현재 bring-up에서는 App 진입점 대신
`main.c`의 `adc_test_update()`가 raw 수집과 SI 환산만 수행한다.
App 함수로 분리해도 실행 문맥은 같은 ADC ISR이며, main loop로 실행이 이동하지 않는다.

### Sample 소비와 실행 조건

- App fast loop는 완료 판정 직후 같은 ISR 흐름에서 `adc_driver_read_raw()`로 묶음을 한 번 소비한다.
- Vdc는 외부 trigger에 의한 단일 regular 변환 결과를 DMA 없이 읽는다.
  읽을 때 새 변환을 시작하거나 완료를 polling하지 않는다.
- 전압 변환은 읽기 전에 완료되어야 하고, 읽는 동안 다음 전압 변환이 완료되지 않아야 한다.
  전류와 전압이 같은 시점에 샘플링된다고 가정하지 말고 trigger 간 시간 관계를 검증한다.
- `adc_driver_read_raw()`와 `adc_driver_convert()`가 모두 성공한 경우에만 feedback을 갱신하고
  제어/SVPWM 계산을 진행한다. NOT_READY나 오류가 발생한 묶음으로 제어를 진행하지 않는다.
  반복 오류에 대한 보호 동작은 App이 결정한다.
- 해당 전압 DR은 driver만 읽는다. 다른 코드에서 먼저 읽으면 EOC 등 상태 판정에 영향을 준다.
- 전류 ADC ISR끼리는 서로 선점하지 않도록 구성하고, 묶음 소비를 다음 수집 주기 전에 끝낸다.
  완료 bitmask는 PWM 주기 번호를 증명하지 않으므로 trigger 누락과 실행 deadline은 App에서 별도로 감시한다.

지원 ADC 구성, sample 폐기 조건 및 동기 오류 복구 절차의 상세 계약은
[`adc_driver.h`](../Core/Platform/adc_driver.h)를 따른다.

### Hall feedback 갱신과 전달

Hall feedback은 ADC sample 주기마다 새로 측정되는 값이 아니다. TIM2의 XOR Hall
edge 또는 counter overflow가 발생할 때 비동기적으로 갱신되고, ADC fast loop는
그 시점까지 publish된 최신 완성본을 읽는다.

```text
TIM2 XOR Hall edge
 -> HAL_TIM_IC_CaptureCallback()
 -> hall_driver_handle_capture()
 -> inactive feedback buffer 완성
 -> active index publish

TIM2 counter overflow
 -> HAL_TIM_PeriodElapsedCallback()
 -> hall_driver_handle_timeout()
 -> 정지/0 rad/s 상태 publish

세 ADC injected 변환 완료
 -> app_motor_fast_loop()
 -> hall_driver_get_feedback()
 -> 최신 완성 Hall snapshot 소비
```

현재 NVIC preemption priority는 ADC1/2와 ADC3가 `0`, TIM2가 `1`이다. 따라서 ADC
fast loop가 Hall feedback 작성 중 TIM2 ISR을 선점할 수 있다. `hall_driver`는 inactive
buffer의 모든 field를 완성하고 memory barrier 뒤 active index 하나를 바꾸는 double
buffer 방식으로 publish한다. ADC reader는 선점 시점에 따라 이전 또는 새 완성본을
읽지만 두 시점의 field가 섞인 snapshot은 노출되지 않는다.

다음 계약을 유지한다.

- Hall feedback은 `hall_driver_get_feedback()`으로만 읽고 내부 buffer/index를 직접 읽거나 수정하지 않는다.
- Hall capture/timeout handler를 호출하는 writer는 해당 TIM ISR 하나뿐이다.
- Hall ISR에서는 FOC/SVPWM을 실행하지 않는다. 제어 stack은 세 ADC 완료로 시작되는 fast loop에서 실행한다.
- Driver는 Hall edge 사이의 angle extrapolation이나 filtering을 수행하지 않는다.
- Fast loop는 `has_valid_state`, `has_valid_angle`, `has_valid_speed`, `is_timed_out`을 확인하고
  사용할 수 없는 feedback으로 제어를 진행하지 않는 정책을 App에서 결정한다.
- Timeout 뒤 첫 Hall edge의 capture 시간은 완전한 edge-to-edge 간격이 아니므로 속도를
  계산하지 않는다. 다음 유효 edge부터 속도 계산을 재개한다.
- Priority 관계나 실행 문맥을 바꾸면 double buffer의 single-writer/reader 선점 전제를
  다시 검토한다. 자세한 계약은 [`hall_driver.h`](../Core/Platform/hall_driver.h)를 따른다.

### Hall / ADC / PWM 시작과 정지

현재 driver 조합의 시작 순서는 다음과 같다.

```text
CubeMX peripheral 초기화 (ADC trigger가 발생하지 않는 상태)
 -> hall_driver_init(): TIM/GPIO mapping, 정방향 sequence 및 timer 설정 검증
 -> hall_driver_start(): Hall capture와 overflow timeout interrupt 시작
 -> adc_driver_init(): 매핑 검증과 ADC 자체 calibration
 -> adc_driver_start(): regular 및 세 injected 그룹을 trigger 대기 상태로 준비
 -> pwm_driver_init(): HRTIM counter 시작과 동기화
 -> 초기 duty 준비 및 update 반영 확인
 -> pwm_driver_enable(): PWM output 활성화
```

`pwm_driver_init()`의 counter 시작/software reset부터 ADC trigger가 발생할 수 있다.
따라서 callback이 사용하는 상태를 먼저 준비해야 한다. 향후 제어를 연결할 때는
App이 startup 상태를 구분하여 PWM driver 초기화 완료 전에 duty 갱신을 호출하지 않게 한다.

`pwm_driver_disable()`은 output만 끄며 counter와 ADC trigger를 정지하지 않는다.
ADC 정지/재동기화 시에는 App/Platform 통합 경로에서 trigger를 막고 진행 중인 ISR 처리가
끝난 뒤 `adc_driver_stop()`을 호출한다. 재시작도 trigger를 막은 상태에서 ADC를 먼저 준비한다.
ADC driver는 PWM/time base를 직접 제어하지 않는다.

Hall timer는 PWM/ADC trigger와 독립적으로 먼저 시작할 수 있지만, rotor feedback을 사용하는
fast loop가 시작되기 전에는 준비되어 있어야 한다. `hall_driver_stop()`은 마지막 Hall
state/sector/angle과 diagnostic counter를 보존하면서 속도를 `0 rad/s`로 만들고
`has_valid_speed`를 false로 설정한다. 정지 중에도 rotor 움직임을 관찰해야 하는 시스템이면
PWM output 정지와 Hall timer 정지를 동일한 동작으로 묶지 않고 App state policy로 결정한다.

---

## 4. ISR 규칙

ISR/callback은:

- 이벤트 source 확인
- timestamp/capture가 필요하면 즉시 저장
- 필요한 상위 entry point 호출
- 가능한 한 빠르게 종료

를 담당한다.

다음은 피한다.

- ISR 안에 위치/속도/FOC 코드를 직접 길게 작성
- HAL callback에서 global variable를 여기저기 갱신
- controller 간 실행 순서를 callback 파일에 분산
- blocking communication

현재 HAL TIM callback은 `main.c`의 CubeMX USER CODE 영역에서 event source를 확인하고
대응하는 Hall handler만 호출한다. 향후 App 통합 후에도 Hall callback 안에 motor-control
stack을 직접 넣지 않고, 세 ADC 완료 callback에서 시작하는 fast-loop entry point를 유지한다.

---

## 5. Multi-rate scheduling

loop rate는 controller 내부 prescaler에 숨기기보다 scheduler/integration layer에서 보이도록 한다.

예:

```text
fast loop
- current sensing
- rotor angle update when required
- FOC
- SVPWM
- PWM update

speed loop
- speed controller

position loop
- position controller

slow loop
- state machine
- communication
- diagnostics
- thermal/fault supervision
```

향후 scheduling의 개념 예 (`app_update_feedback_fast()`는 raw 읽기와 환산이
모두 성공했는지를 bool로 반환하는 App helper로 가정):

```c
void app_motor_fast_loop(void)
{
    if (!app_update_feedback_fast()) {
        return;
    }
    motor_control_fast_update(&motor_control, ...);

    if (++speed_divider >= SPEED_LOOP_DIVIDER) {
        speed_divider = 0U;
        motor_control_speed_update(&motor_control, ...);
    }

    if (++position_divider >= POSITION_LOOP_DIVIDER) {
        position_divider = 0U;
        motor_control_position_update(&motor_control, ...);
    }
}
```

실제 rate와 scheduling method는 향후 구현에 따라 바뀔 수 있다.

핵심은 **실행 rate를 알고리즘 내부의 숨은 counter로 분산시키지 않는 것**이다.

---

## 6. Control mode routing

Position mode:

```text
command.theta_m_ref
  -> position controller
  -> omega_ref
  -> speed controller
  -> i_q_ref
  -> FOC
```

Speed mode:

```text
command.omega_m_ref
  -> speed controller
  -> i_q_ref
  -> FOC
```

Current mode:

```text
command.i_d_ref / i_q_ref
  -> FOC
```

controller끼리 서로 호출하지 않고 `motor_control`이 routing한다.

위 Position mode는 필요한 위치 feedback이 제공되는 구성의 일반적인 routing이다.
현재 Hall-only configuration은 절대 기계 원점과 연속 기계각을 제공하지 않으므로
Position mode를 지원하지 않는다.

---

## 7. PWM / SVPWM boundary

```text
FOC
 -> v_alpha_beta_ref

SVPWM
 -> duty_abc [normalized]

PWM driver
 -> HRTIM compare register
```

SVPWM은 HRTIM period count, channel register, dead-time register를 몰라야 한다.

PWM driver는 FOC의 `i_d`, `i_q`, speed reference 등을 몰라야 한다.

### Preload와 duty 반영 deadline

현재 CubeMX 설정은 Timer C/D/F의 preload와 reset/roll-over update를 활성화한다.
Compare 쓰기는 preload에 저장되고, 실제 동작 값으로의 반영은 timer update 때 이루어진다.

`pwm_driver_set_duty()`는 세 상 compare를 순서대로 쓴다. **세 타이머의 update 시점이
동기화되어 있고 해당 update 전에 모든 쓰기가 완료되면** 세 상 duty가 같은 시점에 반영된다.
Preload는 CPU가 세 값을 모두 쓸 때까지 update를 기다려주는 기능은 아니다.
쓰는 도중 update 경계를 넘으면 일부 상만 먼저 새 duty를 반영할 수 있다.

따라서 App은 ADC 완료부터 제어/SVPWM 계산과 마지막 compare 쓰기까지의 실행 시간을
목표 update deadline 안에 확보해야 한다. 순차 쓰기 자체를 동시 반영 불가로 해석하지 않는다.
API 상세 계약은 [`pwm_driver.h`](../Core/Platform/pwm_driver.h)를 따른다.

---

## 8. Fault와 state machine

Fault detection source는 여러 곳일 수 있다.

```text
hardware driver
sensor validation
control saturation monitor
state machine timeout
communication watchdog
```

하지만 system response는 통합된 fault/state logic이 결정한다.

예:

```text
fault detected
   ->
fault manager records/latches
   ->
state machine enters FAULT
   ->
PWM disable
```

low-level driver가 임의로 system state를 변경하지 않는다. 단, hardware emergency shutdown 같은 안전 path는 예외이며 architecture에 명시한다.
