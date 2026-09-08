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
rotor estimate update
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

### ADC / PWM 시작과 정지

현재 driver 조합의 시작 순서는 다음과 같다.

```text
CubeMX peripheral 초기화 (ADC trigger가 발생하지 않는 상태)
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
