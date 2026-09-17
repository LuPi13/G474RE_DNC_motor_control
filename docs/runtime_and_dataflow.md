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

외부 통신이 step 형태의 command를 갱신하더라도 controller reference에 직접 전달하지 않고,
필요한 command 범위를 먼저 제한한 뒤 고정 주기 rate limiter를 거쳐 전달할 수 있다.

```text
external command
  -> command range clamp
  -> rate limiter
  -> limited reference
  -> controller
```

통신 수신 경로는 target만 갱신하며 rate limiter는 통신 packet 도착 시점이 아니라 해당
reference를 소비하는 scheduler의 고정 주기에서 실행한다. Runtime rate 변경도 limiter
instance를 통신/ISR 양쪽에서 직접 수정하지 않고 같은 scheduler 문맥에서 적용한다.
정상 stop ramp와 달리 fault/emergency shutdown은 rate limiter를 우회해 즉시 안전 출력을 적용한다.

Current mode의 d/q 전류 지령은 `motor_control`이 다음 순서로 처리한다.

```text
requested i_dq
  -> d/q axis clamp
  -> current magnitude saturation
  -> d/q rate limiter
  -> final current magnitude saturation
  -> FOC i_dq_ref
```

두 scalar rate limiter가 서로 다른 비율로 이동하면 중간 d/q vector가 원형 제한을 벗어날
수 있으므로 마지막 saturation을 방어선으로 둔다. 이때 원점 방향으로 단순 축소하지 않고
직전 적용 지령에서 중간 지령으로 향하는 이동량을 원 경계까지만 허용한다. 그러면 축별
rate limit도 보존된다. 이 제한이 개입하면 limiter state도 실제 적용한 `i_dq_ref`로
동기화한다. 외부 target은 command source가 소유하고,
`motor_control.i_dq_ref`가 FOC에 전달되는 최종 reference의 source of truth다.

Speed mode는 1 kHz SysTick scheduler에서만 다음 순서로 처리한다.

```text
published omega_m target
  -> speed-reference range/rate limit
  -> hall omega_e snapshot / pole-pair conversion
  -> speed PI and external tracking
  -> double-buffered i_d = 0, i_q target
  -> 40 kHz current fast loop / FOC
```

ADC ISR은 Hall speed feedback snapshot의 단일 writer이고 SysTick은 그 reader다. 새 Hall capture,
speed-validity 변화 또는 timeout일 때만 speed snapshot을 publish하며, 같은 capture의 반복
fast-loop에서는 재-publish하지 않는다. SysTick은 speed current target의 단일 writer이고 ADC ISR은
그 reader다. 두 방향 모두 inactive buffer를 완성한 뒤 memory barrier와 active index 하나로
publish한다. 따라서 speed PI를 ADC ISR에 넣거나 SysTick에서 FOC/PWM register를 직접 갱신하지
않는다.

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

현재 Hall 구성에서는 `hall_driver`가 TIM capture/timeout으로부터 만든 최신 raw signal
snapshot의 owner다. App은 fast loop가 시작될 때 경량 snapshot을 읽어 `hall_decoder`에
전달하고, decoder는 선택된 motor profile에 따라 sector/edge angle/speed를 만든다.
`hall_estimator`는 이 decoded observation을 이어 받아 edge 사이 연속 전기각을 소유한다.
Control/FOC는 driver 내부 buffer를 직접 참조하거나 별도의 Hall 각도를 독립적으로 갱신하지
않는다.

`main.c`는 Hall feedback의 별도 복사본을 유지하지 않는다. 관찰이 필요하면 driver/App이
소유한 snapshot API를 사용하며, 이를 product runtime state로 중복 보관하지 않는다.

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
지정된 completion ADC의 injected 완료 IRQ
   ->
ADC driver가 같은 trigger의 3상 JDR을 일괄 수집
   ->
fast-loop pending 표시
   ->
HAL IRQ 처리 종료 및 현재 ADC flag 정리
   ->
ADC IRQ 후처리
   ->
app_motor_fast_loop()
   ->
feedback acquire/convert
   ->
raw Hall snapshot acquire
   ->
motor profile decode / continuous rotor estimate
   ->
motor_control fast update
   ->
SVPWM
   ->
PWM duty write
```

HAL callback에는 로직을 길게 작성하지 않는다.

현재 전류는 서로 다른 세 ADC의 injected 변환으로 수집하지만, 세 변환은 같은 trigger,
sampling time과 oversampling 설정을 사용한다. 세 ADC에서 각각 완료 interrupt를 발생시키지
않고 `adc_driver_config_t::injected_completion_adc` 하나만 완료 interrupt를 발생시킨다.
정상 JEOC 경로에서 App IRQ entry는 driver를 통해 세 ADC의 JEOC를 확인하고 rank 1 JDR을
일괄 수집한다. 현재 JEOC/JEOS를 정리하고 pending을 표시한 뒤 fast loop를 실행한다. 그 외
비정상 interrupt source는 `HAL_ADC_IRQHandler()` fallback으로 넘긴다.

현재 App 연결의 축약 예 (`adc_driver`는 초기화/시작된 instance):

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
        adc_fast_loop_pending = true;
    }
}

void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}

void ADC3_IRQHandler(void)
{
    if (app_adc_injected_irq_try_handle_fast(&hadc3)) {
        app_adc_irq_epilogue();
        return;
    }
    HAL_ADC_IRQHandler(&hadc3);
    app_adc_irq_epilogue();
}
```

현재 보드에서는 ADC3를 completion ADC로 사용한다. ADC1/ADC2 injected 완료 interrupt는
driver start에서 활성화하지 않으므로 정상 경로에서는 shared `ADC1_2_IRQHandler()`에
진입하지 않는다. 위 예시는 연결 구조를 줄여 쓴 것이며 실제 `main.c`는 ADC 수집 오류와 HAL 오류 callback도
App의 `app_handle_adc_error()` 경로에 연결한다.

HAL callback 정의는 `Core/Src/main.c`의 CubeMX USER CODE 영역에 두고,
실제 orchestration은 `Core/App/app.c`에 둔다. 현재 open-loop bring-up에서는
`main.c`의 IRQ 후처리 helper가 ISR 전용 `app_motor_fast_loop_fast()`를 호출하며, App이 raw sample
소비와 SI 환산부터 CORDIC/SVPWM/PWM duty 갱신까지 수행한다. `main.c`는 fast-loop raw sample,
환산 결과 또는 Hall 추정 결과의 debugger용 복사본을 만들지 않는다.
`app_adc_irq_epilogue()`는 completion ADC인 `ADC3_IRQHandler()`의 JEOC 정리 뒤 CubeMX USER
CODE 영역에서 호출한다. App 함수로 분리해도 실행 문맥은 같은 ADC ISR이며, main loop로
실행이 이동하지 않는다.

Hall snapshot 취득, motor profile decoding과 continuous-angle estimator 실행은 App
orchestration에 있다. `main.c`에는 IRQ 경계와 초기화 연결만 유지한다.

정상 fast IRQ는 세 JDR을 읽은 직후 현재 JEOC/JEOS를 직접 정리하고 pending을 표시한다.
HAL fallback을 탄 경우에는 `HAL_ADCEx_InjectedConvCpltCallback()`이 pending만 표시하고,
HAL이 현재 flag를 정리한 뒤 fast loop를 실행한다. 어느 경로에서도 callback 안에서 제어
계산을 실행하지 않는다.

### Sample 소비와 실행 조건

- App fast loop는 완료 판정 직후 같은 ADC ISR의 JEOC 정리 후에서
  `adc_driver_read_raw()`로 묶음을 한 번 소비한다.
- Vdc는 외부 trigger에 의한 단일 regular 변환 결과를 DMA 없이 읽는다.
  읽을 때 새 변환을 시작하거나 완료를 polling하지 않는다.
- 전압 변환은 읽기 전에 완료되어야 하고, 읽는 동안 다음 전압 변환이 완료되지 않아야 한다.
  전류와 전압이 같은 시점에 샘플링된다고 가정하지 말고 trigger 간 시간 관계를 검증한다.
- `adc_driver_read_raw()`, `current_sensor_convert()`, `voltage_sensor_convert()`가 모두
  성공한 경우에만 완전한 feedback을 갱신하고 제어/SVPWM 계산을 진행한다. 전류 영점 보정
  중에는 `current_sensor_process_offset_sample()`로 raw sample을 소비하고 DC-link 전압 보호만
  갱신한다. NOT_READY나 오류가 발생한 묶음으로 제어를 진행하지 않으며 보호 동작은 App이 결정한다.
- 해당 전압 DR은 driver만 읽는다. 다른 코드에서 먼저 읽으면 EOC 등 상태 판정에 영향을 준다.
- Completion callback에서 follower ADC의 JEOC가 모두 설정되지 않았거나 이전 묶음이 미소비된
  경우 동기 오류로 처리한다. 묶음 소비를 다음 수집 주기 전에 끝내며 trigger 누락과 실행
  deadline은 App에서 별도로 감시한다.

지원 ADC 구성, sample 폐기 조건 및 동기 오류 복구 절차의 상세 계약은
[`adc_driver.h`](../Core/Platform/adc_driver.h)를 따른다.

### Hall feedback 갱신과 전달

Hall feedback은 ADC sample 주기마다 새로 측정되는 값이 아니다. TIM2의 XOR Hall
edge 또는 counter overflow가 발생할 때 비동기적으로 갱신되고, ADC fast loop는
그 시점까지 publish된 최신 완성본을 읽는다.

현재 TIM2 counter는 10 MHz, auto-reload `999999`로 설정되어 Hall edge timeout은 100 ms다.
이 값은 Hall speed feedback을 무효/0으로 전이하는 시간이며, 정상 정지의 유일한 조건으로 쓰지 않는다.

```text
TIM2 XOR Hall edge
 -> HAL_TIM_IC_CaptureCallback()
 -> hall_driver_handle_capture()
 -> raw state/edge interval inactive buffer 완성
 -> active index publish

TIM2 counter overflow
 -> HAL_TIM_PeriodElapsedCallback()
 -> hall_driver_handle_timeout()
 -> raw timeout 상태와 무효 interval publish

completion ADC injected 변환 완료
 -> 세 ADC JDR 일괄 수집
 -> fast-loop pending 표시
 -> HAL ADC flag 정리
 -> ADC IRQ 후처리
 -> app_motor_fast_loop()
 -> hall_driver_get_signal_feedback()
 -> hall_decoder_update_fast()
 -> hall_estimator_update_from_decoder_fast()
 -> 최신 연속 Hall rotor feedback 소비
```

현재 NVIC preemption priority는 ADC1/2와 ADC3가 `0`, TIM2가 `1`이다. 따라서 ADC
fast loop가 Hall feedback 작성 중 TIM2 ISR을 선점할 수 있다. `hall_driver`는 inactive
buffer의 모든 field를 완성하고 memory barrier 뒤 active index 하나를 바꾸는 double
buffer 방식으로 publish한다. ADC reader는 선점 시점에 따라 이전 또는 새 완성본을
읽지만 두 시점의 field가 섞인 snapshot은 노출되지 않는다.

다음 계약을 유지한다.

- Fast loop는 `hall_driver_get_signal_feedback()`으로 raw signal snapshot을 읽고, 전체
  hardware diagnostic이 필요한 경로는 `hall_driver_get_feedback()`을 사용한다. 내부
  buffer/index는 직접 읽거나 수정하지 않는다.
- `hall_driver_signal_feedback_t`와 `hall_decoder_observation_t`는 Common의 같은
  `hall_signal_t` alias다. App은 signal snapshot을 복사하거나 재포장하지 않고 decoder에
  전달하며, raw signal의 owner는 계속 Platform driver다.
- Hall capture/timeout handler를 호출하는 writer는 해당 TIM ISR 하나뿐이다.
- Hall ISR에서는 FOC/SVPWM을 실행하지 않는다. 제어 stack은 세 ADC 완료로 시작되는 fast loop에서 실행한다.
- Driver는 raw Hall state의 유효성, sector, direction, electrical angle 또는 speed를 판단하지
  않는다. 이 의미는 `hall_decoder`와 motor profile이 소유한다.
- Decoder profile은 8개 raw state 각각을 sector 0부터 5 또는 invalid로 mapping하고, 여섯
  정방향 진입 경계각을 제공한다. 실제 sector span과 edge-to-edge speed도 이 경계각으로
  계산한다.
- 현재 Hall estimator는 마지막 유효 edge 이후 `abs(omega_e) * elapsed_s`를 적분하되,
  이동량을 profile의 현재 Hall sector span으로 제한한다. Signed speed가 양수이면
  edge 각도에서 증가하고 음수이면 감소하며, 다음 Hall edge가 없으면 해당 방향의 sector
  출구 경계에서 대기한다. 따라서 Hall state 변화 없이 추정각만 다음 sector로 넘어가지 않는다.
- Decoder output은 최신 sector와 실제 sector span을 함께 전달한다. Estimator는 transition
  count와 validity flag뿐 아니라 sector 변화도 관측 갱신으로 취급하므로, decoder resync에서
  sector 중심각이 바뀌어도 이전 관측을 잘못 재사용하지 않는다.
- Fast loop는 `has_valid_state`, `has_valid_angle`, `has_valid_speed`, `is_timed_out`을 확인하고
  사용할 수 없는 feedback으로 제어를 진행하지 않는 정책을 App에서 결정한다.
- `hall_estimator_update_fast()`는 이미 검증된 범용 observation을 사용할 수 있는 fast API이며,
  pointer/초기화/observation 전체 검사를 생략하지만 transition 순서 오류는 계속 반환한다.
  범용 입력과 수치 test는 checked `hall_estimator_update()`를 사용한다.
- App current-mode fast loop는 `hall_decoder_update_fast()`로 Hall driver가 보장한 raw
  snapshot을 처리한다. 이 경로는 pointer/초기화/raw field 검증과 output 구조체 복사를
  생략하지만, state sample 부재, profile-invalid state, capture 누락, 비인접 transition은
  계속 오류로 반환한다. `hall_decoder_get_latest_output_fast()`의 반환 포인터는 같은 ISR에서
  다음 decoder update 전까지만 읽고, 범용 입력과 수치 test는 checked `hall_decoder_update()`를
  사용한다.
- Current-mode에서는 `hall_estimator_update_from_decoder_fast()`가 위 decoder output을 직접
  읽고 estimator가 소유하는 output을 in-place로 갱신한다. App은 같은 ISR에서
  `hall_estimator_get_latest_output_fast()`의 읽기 전용 포인터만 소비한다. Generic fast-loop의
  diagnostic output은 필요할 때만 이 결과를 복사하며, decoder와 estimator 사이에 새 runtime
  source of truth를 만들지 않는다.
- Timeout 뒤 첫 Hall edge의 capture 시간은 완전한 edge-to-edge 간격이 아니므로 속도를
  계산하지 않는다. 다음 유효 edge부터 속도 계산을 재개한다.
- Priority 관계나 실행 문맥을 바꾸면 double buffer의 single-writer/reader 선점 전제를
  다시 검토한다. 자세한 계약은 [`hall_driver.h`](../Core/Platform/hall_driver.h)를 따른다.

### Hall / ADC / PWM 시작과 정지

현재 driver 조합의 시작 순서는 다음과 같다.

```text
CubeMX peripheral 초기화 (ADC trigger가 발생하지 않는 상태)
 -> hall_driver_init(): TIM/GPIO mapping 및 timer 설정 검증
 -> hall_decoder_init(): motor별 raw-state mapping과 electrical edge angle 검증
 -> hall_driver_start(): Hall capture와 overflow timeout interrupt 시작
 -> adc_driver_init(): 매핑 검증과 ADC 자체 calibration
 -> adc_driver_start(): regular 및 세 injected 그룹을 trigger 대기 상태로 준비
 -> app_drive_start(): PWM 비활성 조건에서 무전류 평균 수집 lifecycle 시작
 -> pwm_driver_init(): HRTIM counter 시작과 동기화
 -> ADC fast loop: settling sample 폐기 및 3상 영점 평균 누적
 -> App main-context update: calibration COMPLETE를 READY로 전이
 -> app_drive_start_speed(): 0 A target 준비, speed mode 진입, PWM output 활성화
```

`pwm_driver_init()`의 counter 시작/software reset부터 ADC trigger가 발생할 수 있다.
따라서 callback이 사용하는 상태를 먼저 준비해야 한다. `app_drive_state_t`는 PWM driver
초기화 완료 전에는 duty 갱신을 호출하지 않도록 startup lifecycle을 구분한다.
현재 App은 보정 중 open-loop 또는 PWM output 활성화를 거부한다. `current_sensor`는 settling
sample 폐기, 3상 평균 누적, 허용 ADC code 범위 검사와 완료 offset의 유일한 owner다.
결과가 범위를 벗어나거나 timeout이 발생하면 current-sensor fault를 latch하고 PWM을
fail-stop 처리한다. App은 raw count나 누적 합을 별도로 소유하지 않으며, 보정 완료 뒤에는
`current_sensor_convert()`가 `abc_t` 전류 [A]를 반환한다. `adc_driver`는 센서 영점이나 gain을
알지 않고 ADC peripheral 매핑과 raw sample 취합만 담당한다.

`pwm_driver_disable()`은 output만 끄며 counter와 ADC trigger를 정지하지 않는다.
ADC 정지/재동기화 시에는 App/Platform 통합 경로에서 trigger를 막고 진행 중인 ISR 처리가
끝난 뒤 `adc_driver_stop()`을 호출한다. 재시작도 trigger를 막은 상태에서 ADC를 먼저 준비한다.
ADC driver는 PWM/time base를 직접 제어하지 않는다.

Hall timer는 PWM/ADC trigger와 독립적으로 먼저 시작할 수 있지만, rotor feedback을 사용하는
fast loop가 시작되기 전에는 driver와 decoder가 모두 준비되어 있어야 한다.
`hall_driver_stop()`은 마지막 raw Hall state와 diagnostic counter를 보존하면서 edge interval을
무효화한다. 정지 중에도 rotor 움직임을 관찰해야 하는 시스템이면
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
- fast loop에서 큰 controller/output 구조체 전체를 transactional snapshot으로 반복 복사

정상 fast path에서는 filter, PI, limiter runtime state도 매번 snapshot/rollback하지 않는다.
초기화와 command 경로에서 입력을 검증하고 fast path는 검증된 precondition 아래 필요한
runtime state만 직접 갱신한다. 진단 output의 scalar 복사로 library call만 피하는 것도 충분한
최적화가 아니며, hot path가 생성·전달하는 데이터 자체를 최소화한다.

40 kHz 실행시간 예산, checked/fast path 경계와 필수 검증 절차는
[`real_time_execution_budget.md`](real_time_execution_budget.md)를 따른다.

현재 HAL TIM callback은 `main.c`의 CubeMX USER CODE 영역에서 event source를 확인하고
대응하는 Hall handler만 호출한다. Hall callback 안에는 motor-control stack을 직접 넣지 않는다.
Completion ADC IRQ가 세 JDR을 수집하고 현재 flag를 정리한 뒤, 같은 IRQ 후처리에서 fast-loop
entry point를 실행하는 구조를 유지한다.

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
    app_publish_rotor_feedback_for_speed_loop(...);
    motor_control_fast_update(&motor_control, ...);
}

void app_speed_loop_1khz(void)
{
    app_read_latest_rotor_feedback(...);
    motor_control_speed_update(&motor_control, ...);
    app_publish_current_target(...);
}
```

Speed loop는 40 kHz ADC ISR에 직접 넣지 않는다. 최신 rotor feedback snapshot과 q축 current
target은 한쪽에서 완성되지 않은 값을 읽지 않도록 App이 명시적인 publish/read 경계를 제공한다.
실제 1 kHz timer/event source와 interrupt priority는 통합 단계에서 결정하고, 그 주기와
`speed_controller`의 PI/filter sampling period를 일치시킨다.

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

FOC 구현 시 v_dc와 voltage margin에 맞게 d/q 전압 vector를 제한하고 실제 적용 가능한
각 축 성분을 PI back-calculation에 tracking한 뒤 v_alpha_beta_ref를 만든다. SVPWM은
입력 vector의 방향을 바꾸는 별도 overmodulation을 수행하지 않으며, 선형 modulation
범위를 벗어난 입력은 오류로 반환한다. PWM driver의 최종 duty clamp는 register 보호를
위한 방어선이지 정상적인 modulation 제한 경로가 아니다.

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

Fast-loop execution budget을 변경하거나 board timing을 재검증할 때는 전용 검증 build에서
Cortex-M DWT cycle counter로 첫 ADC IRQ 진입부터 fast-loop entry point 종료까지의 cycle을
측정할 수 있다. 예를 들어
CPU 170 MHz, fast loop 40 kHz의 한 주기는
`170000000 / 40000 = 4250 cycles`이다. 측정 구간은 HAL IRQ 진입/처리 비용을
포함해야 하며, fast-loop body만의 값도 별도로 측정해 비용을 구분한다. IRQ 진입 직전 hardware latency와 최종
interrupt 복귀 비용은 포함되지 않으므로 interrupt jitter와 duty write deadline을
위한 margin을 남겨야 한다.

공유 기본 build의 `main.c`에는 DWT enable, cycle 최대값, deadline miss counter 같은
검증용 상태를 두지 않는다. 검증 계측은 App의 선택형 profile interface 또는 별도 test build에서
켜며, 검증 종료 뒤 product build에 남기지 않는다.

현재 170 MHz/40 kHz timing contract의 hard deadline은 4250 cycles이며, 통합 통과 목표는
전체 worst-case 3200 cycles 이하와 deadline miss 0이다. Body, 정상 경로 또는 평균값만으로
통과시키지 않는다. 세부 구간과 시험 조건은
[`real_time_execution_budget.md`](real_time_execution_budget.md)를 따른다.

### 현재 App drive-mode 연결

`app_motor_fast_loop()`은 공통 feedback/fault 경로 뒤 active mode에 맞는 계산을 실행한다.

```text
adc_driver_read_raw / convert
 -> unfiltered i_abc / v_dc software fault 검사
 -> raw Hall snapshot / hall_decoder_update_fast / hall_estimator_update_from_decoder_fast
 -> open-loop: voltage angle -> CORDIC -> v_alpha_beta
    current: current command -> motor_control -> FOC -> v_alpha_beta_ref
 -> SVPWM
 -> PWM duty write
```

Open-loop command는 전압 vector 크기 [V]와 signed electrical angular velocity [rad/s]이며,
rotor electrical angle과 별개의 값이다. 0 V command는 DC-link가 0 V인 bring-up 상태에서도
정의되도록 CORDIC/SVPWM을 생략하고 0.5 duty를 사용한다. Command writer와 ADC ISR reader
사이에는 App이 소유한 double buffer를 사용한다. Writer는 하나이고 ADC ISR보다 낮은
preemption priority에서 실행해야 한다.

Current command는 App writer가 `motor_control`의 checked prepare API로 axis/vector 2 A 제한을
먼저 적용한 뒤 별도 double buffer에 publish한다. 40 kHz fast loop는 준비된 target에
100 A/s rate limit과 이동 중 최종 vector 제한만 적용한다. FOC와 하위 PI/filter는 App의
measurement/fault/rotor/CORDIC 검증을 precondition으로 하는 fast API를 사용하며 최종
alpha-beta 전압의 유한성 검사는 유지한다. 초기 board bring-up FOC 설정은 500 Hz current-loop
bandwidth, 5 kHz d/q IIR, 0.9 voltage utilization과 decoupling OFF다. Current mode start는
0 A command, current offset calibration 완료와 유효 Hall electrical angle을 요구한다.
Hall electrical offset 검증 전에는 current mode를 자동 시작하지 않는다.

App은 PWM counter 시작 중 발생할 수 있는 ADC event를 소비하되 mode start 전에는 PWM
compare를 갱신하지 않는다. 각 유효 sample에서 `fault_manager`가 3상 과전류와 DC-link
과전압을 먼저 검사한다. 실행 중 threshold 위반이나 ADC/rotor/control/CORDIC/SVPWM/PWM
오류가 발생하면 원인을 latch하고 active mode를 중지한 뒤 software PWM disable을 시도한다. 이 경로는
현재 PCB에 없는 HRTIM break/fault나 COMP 기반 hardware 긴급 차단을 대신하지 않는다.

Injected 완료 취합 단계의 동기 오류나 HAL ADC 오류는 fast loop가 실행되지 않을 수 있으므로
`main.c` callback이 `app_handle_adc_error()`에 전달한다. Callback 안에서는 ADC fault latch,
active mode 중지와 software PWM disable만 수행하며 control/SVPWM 계산은 실행하지 않는다.

---

## 8. Fault와 state machine

현재 App의 정상 lifecycle은 다음과 같다.

```text
DISABLED
 -> CURRENT_OFFSET_CALIBRATION  (PWM output off, ADC fast loop가 sample 누적)
 -> READY                       (offset COMPLETE, PWM output off)
 -> CURRENT_RUNNING             (0 A target 준비 후 PWM output on)
 -> PWM output off
 -> READY
 -> SPEED_RUNNING               (0 A target 준비 후 PWM output on)
 -> RAMP_TO_ZERO                (0 rad/s command, speed rate limiter/PI 유지)
 -> Hall timeout 또는 저속 dwell 확인
 -> PWM output off
 -> READY
```

`READY`는 별도 PWM-disabled enum을 중복으로 두지 않고 **보정 완료 + PWM output off**를
의미한다. `app_drive_update()`는 main context에서 calibration 완료/timeout과 PWM enable/disable만
처리한다. `app_drive_scheduler_tick()`은 1 kHz SysTick에서 calibration timeout 시간을 세고 speed PI만
실행하며, ADC ISR은 기존 40 kHz feedback/FOC/SVPWM 경로를 유지한다.

공유용 기본 firmware의 `main.c`는 CubeMX peripheral 초기화, App/driver wiring, IRQ entry와
`app_drive_update()` 호출만 수행한다. 따라서 기동 뒤 offset 보정이 완료되면 `READY`에서 안전하게 대기하며
PWM을 자동으로 enable하지 않는다. CAN, UART 또는 debugger test source는 protocol을 해석한 뒤 main context에서
`drive_command_router_execute()`에 `START_CURRENT`, `SET_CURRENT`, `START_SPEED`, `SET_SPEED`, `STOP`,
`RECOVER_FAULT` command를 전달한다. `STOP`은 current mode에서는 PWM을 즉시 끄고 READY로 복귀하며,
speed mode에서는 기존 ramp-to-zero 경로를 사용한다. 통신 ISR은 직접 PWM/FOC를 조작하지 않고 command를 queue에 넣는다.

FDCAN2는 500 kbit/s Classic CAN과 interrupt priority 2로 설정한다. `fdcan_driver`는 HAL RX
FIFO0 callback에서 standard data frame을 꺼내 ISR consumer에만 전달하며, CANopen protocol 처리와
drive command routing은 main context의 communication service가 수행한다. 따라서 priority 0 ADC fast
loop와 priority 1 Hall timer는 CAN 수신 처리에 의해 지연되지 않는다.
기본 firmware는 FDCAN driver를 초기화 뒤 start하여 raw frame 수신 diagnostic을 유지하지만, 이 동작은
PWM output 또는 App drive lifecycle을 변경하지 않는다.

현재 board bring-up에는 `drive_debug_command_source`를 연결한다. 이 module은 `main.c` 밖에 있으며
`requested_mode`, `requested_i_d_a`, `requested_i_q_a`, `requested_speed_rpm`, `start_requested`,
`stop_requested`를 Live Expression 입력으로 제공한다. 기본 mode는 speed이고 모든 reference와 request의
기본값은 0/false이므로 자동 기동하지 않는다.

`drive_debug_observer`는 이 입력 경로와 별도로, Live Expression/SWV가 읽는 전역
`drive_debug_snapshot`을 제공한다. 40 kHz current/speed fast-loop는 매 sample telemetry를 복사하지
않고, 초기화 시 정한 정확한 분주 주기(현재 40 kHz / 1 kHz = 40 sample)마다 한 번만 상세 FOC output을
계산해 snapshot을 publish한다. 정상 sample은 계속 전압만 반환하는 fast API를 사용한다. 따라서 observer는
제어값의 owner가 아니며 debugger가 snapshot을 쓰면 안 된다. `drive_debug_snapshot_sequence`이 짝수이고
읽기 전후 같은 값일 때 snapshot은 일관된 값이다.

`theta_e_rad`는 wrapped electrical angle이다. `theta_m_rad_per_electrical_cycle`은 이를 pole-pair
수로 나눈 한 electrical cycle 내 기계각 성분일 뿐, multi-turn position이나 전원 재인가 뒤에도 유지되는
absolute position이 아니다. 그런 위치 기능에는 encoder 또는 homing 기준이 필요하다. SWV에는 필요한
신호 몇 개만 선택해 사용하며, 40 kHz switching ripple, ADC sampling timing 또는 cycle-level transient는
SWV 문자열 출력 대신 oscilloscope 또는 별도 triggered fast-sample capture로 검증한다.

정상 정지는 reference가 0이 되었다는 사실만으로 PWM을 끄지 않는다. `RAMP_TO_ZERO`에서 Hall timeout 또는
기계속도 절댓값이 `speed_stop_omega_m_threshold_rad_s` 이하임을 한 번 확인한 뒤
`speed_stop_dwell_ms` 동안 능동 감속하고 PWM을 비활성화한다. 저속 확인 뒤에는 다음 Hall edge의 양자화된
속도값이 커져도 dwell을 되돌리지 않는다. 현재 bring-up 값은 500 rpm, 10 ms다. Hall의 저속 양자화로 edge가
반복될 때 연속 조건이나 timeout만 기다리면 speed PI가 rotor를 경계에서 왕복시킬 수 있으므로, 제어가 안정적인
최저속도에서 PWM을 끄고 마지막 구간은 coast로 맡긴다. 정지 뒤 속도 hold는 하지 않는다.

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

### 현재 software fault manager

현재 App은 별도 `fault_manager_t` instance를 연결해 다음 fault를 bitmask로 latch한다.

- ADC 일반 오류, 수집 동기 오류, DC-link overrun
- 유효하지 않은 측정값
- a/b/c상 각각의 과전류
- DC-link 과전압
- Hall feedback, rotor estimator, motor control 오류
- CORDIC, SVPWM, PWM 오류

현재 보드의 초기 설정은 상전류 절댓값 `3.0 A`와 DC-link `79.2 V`에서 trip한다.
Latch 해제를 위한 hysteresis는 상전류 절댓값 `1.0 A` 이하, DC-link `75.0 V` 이하로
설정한다. Threshold는 보드/제품 설정이므로 `main.c`의 App 통합 config에서 전달하며
fault manager 구현에 숨은 기본값을 두지 않는다.

현재 fault manager에는 DC-link 저전압 보호가 없다. 따라서 `10 V` 저전압 차단
threshold도 아직 적용하지 않는다. 저전압 보호를 추가할 때는 ADC 측정 유효 범위,
기동/정지 상태와 회생 중 DC-link 거동을 함께 정의한 뒤 별도 fault로 구현한다.

Fault가 latch되면 App은 PWM output을 disable하고 활성 drive mode를 해제한다.
ADC trigger와 counter는 계속 동작하므로 ADC가 정상인 fault에서는 측정값을 계속 갱신해
active 원인이 사라졌는지 판단할 수 있다. 외부에서는 open-loop와 current command를 모두
0으로 먼저 publish하고 `app_request_fault_clear()`로 일회성 해제를 요청한다. 다음 유효 fast loop에서
다음 조건을 모두 만족할 때만 latch를 해제한다.

```text
PWM output disabled
open-loop command = 0 V, 0 rad/s
current command = 0 A
latest measurement valid
active measurement fault 없음
```

요청은 성공/실패와 관계없이 한 번만 소비한다. 해제 성공 후에도 drive mode와 PWM은
비활성 상태이며 `app_drive_recover_after_fault()`로 READY 복귀를 명시적으로 요청한 뒤 별도의 새 command와
시작 절차가 필요하다. 따라서 이전 nonzero command로 자동 재시작하지 않는다. ADC 동기 오류처럼 유효 sample 자체가 재개되지 않는 fault는
명령 해제만으로 처리하지 않고 trigger 차단, ADC stop/start 재동기화 또는 MCU reset이 필요하다.

현재 fault manager는 ADC 결과를 사용하는 software 보호다. 실제 hardware revision에서는
gate-driver fault 또는 COMP/window comparator를 HRTIM fault에 연결해 CPU와 무관한 긴급 차단을
추가하고, software latch는 원인 기록과 재시작 정책을 담당하게 한다.
