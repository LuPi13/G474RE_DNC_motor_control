# Real-Time Execution Budget

## 1. 목적

이 문서는 PWM 동기 fast loop의 실행시간 예산, hot path 설계 규칙과 검증 절차를 정의한다.
`runtime_and_dataflow.md`가 실행 순서와 데이터 흐름을 정의한다면, 이 문서는 그 구현이
hard real-time deadline을 만족하는지 판단하는 기준이다.

기능이 수치적으로 맞더라도 이 문서의 deadline 기준을 만족하지 못하면 fast-loop 통합은
완료된 것으로 보지 않는다.

---

## 2. 현재 timing contract

현재 기준 clock과 실행률은 다음과 같다.

| 항목 | 값 |
|---|---:|
| CPU clock | 170 MHz |
| PWM/ADC trigger rate | 40 kHz |
| fast-loop period | 25 us |
| hard deadline | 4250 cycles |

첫 ADC IRQ 진입부터 마지막 PWM compare write와 fast-loop 종료까지의 **전체 경로**는 다음
trigger 전에 끝나야 한다. Body 측정값만으로 deadline 통과를 판정하지 않는다. IRQ 진입
latency, 다른 interrupt의 blocking, 측정 코드와 interrupt 복귀 비용도 별도 margin에 포함한다.

### 통과 기준

- 전체 worst-case 실행시간은 **MUST** `4250 cycles` 미만이어야 한다.
- 현재 bring-up 목표는 전체 최대 **MUST** `3200 cycles` 이하로 두어 최소 25% margin을
  확보한다.
- fast-loop body는 정상 반복 경로에서 **SHOULD** `2200 cycles` 이하, saturation이나
  Hall 전환을 포함한 worst-case에서 **MUST** `2800 cycles` 이하로 한다.
- deadline miss counter는 각 worst-case 시험에서 **MUST** 0이어야 한다.
- `last` 또는 평균값만으로 통과시키지 않고 `max`와 deadline miss를 사용한다.

다른 실행률을 채택하면 scheduler에서 실행률을 명시하고 이 표와 controller sample period,
filter/PI coefficient를 함께 변경한다. 알고리즘 내부에 숨은 prescaler만 추가해서 deadline
문제를 우회하지 않는다.

---

## 3. 알려진 실패 기준선

2026-09-11 current-control dry-run 보드 측정에서 다음 값이 관찰되었다.

| 항목 | 측정값 |
|---|---:|
| fast-loop body last | 8331 cycles |
| 첫 ADC IRQ부터 종료까지 max | 9346 cycles |
| deadline miss count | 25 |

이 값은 40 kHz timing contract를 만족하지 않는다. 따라서 현재 current-mode fast loop는
실구동 검증 단계로 진행하지 않는다. 이 표는 최적화 완료를 주장하는 근거가 아니라 이후
변경의 before/after 기준선이다.

첫 구간 계측에서는 128 settling + 2048 averaging sample에 필요한 이상적 최소 54.4 ms보다
짧은 10 ms current-offset timeout 때문에 current mode에 진입하지 못한 구성 오류도 확인했다.
Profiling 시작 전에는 시험 대상 mode에 실제로 진입했는지, complete sample count가 증가하는지와
최초 fault mask를 함께 확인한다. Fault shutdown 경로의 maximum을 정상 control 경로의
실행시간으로 잘못 해석하지 않는다.

Offset timeout과 반복 fault-reset을 수정한 뒤 같은 Debug `-O3` dry-run에서 최초로 완성된
current-mode 경로는 다음 maximum을 보였다.

| 구간 | 최대 cycle |
|---|---:|
| sensing | 495 |
| fault | 390 |
| rotor | 346 |
| control | 2272 |
| modulation + PWM | 772 |
| diagnostic | 192 |
| fast-loop body | 4726 |
| 첫 ADC IRQ부터 종료까지 | 5749 |
| deadline miss count | 2 |

구간별 maximum은 서로 다른 sample에서 발생할 수 있으므로 단순 합계를 전체 worst-case로
간주하지 않는다. 이 측정에서는 control 구간이 가장 큰 단일 병목이며, body 바깥 ADC/HAL
경로도 약 1000 cycles의 별도 최적화 대상임을 확인했다. Checked/fast path 분리 범위는
하위 구간을 다시 계측해 원인을 확인한 뒤 결정한다.

Control 변경 전에는 해당 구간을 rotor CORDIC, command/input 준비, current-reference 제한,
FOC와 output snapshot으로 한 번 더 나눠 계측한다. 상위 control maximum만으로 내부 병목을
추정해 checked/fast API 범위를 결정하지 않는다.

같은 dry-run binary에서 control 내부를 계측한 결과는 다음과 같다. 모든 maximum이 같은
sample의 합은 아니며, 완성 sample 수가 2이므로 순위 판별용 중간 결과로만 사용한다.

| Control 하위 구간 | 최대 cycle |
|---|---:|
| rotor CORDIC | 285 |
| command/input 준비 | 89 |
| current-reference 처리 | 532 |
| FOC update | 1374 |
| output snapshot | 77 |
| complete sample count | 2 |

FOC가 control 내부에서 가장 큰 구간임을 확인했다. FOC 구현 변경 전에는 입력 검증,
Clarke/Park, rollback snapshot, current filter, PI, voltage limit, tracking, inverse Park/output으로
다시 나눠 계측한다. 이 결과로 정상 경로의 반복 검증·복사 비용과 실제 제어 연산 비용을
구분한다.

FOC 내부를 추가 계측한 결과는 다음과 같다.

| FOC 하위 구간 | 최대 cycle |
|---|---:|
| runtime 입력/상태 검증 | 252 |
| Clarke/Park와 결과 검증 | 186 |
| rollback snapshot | 39 |
| d/q current filter | 230 |
| d/q error와 PI update | 378 |
| feedforward와 voltage limit | 201 |
| saturation tracking | 31 |
| inverse Park와 output snapshot | 174 |
| complete sample count | 2 |

`tracking`의 31 cycles는 포화되지 않은 빈 branch와 계측 경계 비용에 해당한다. 따라서 이
조건에서는 voltage saturation용 `sqrtf`와 PI external tracking이 병목이 아니다. FOC의 가장
큰 단일 구간은 두 축 PI update이며, 입력 검증과 두 축 filter도 큰 비중을 차지한다. 각 구간
소스에는 유한성·pointer·초기화 검사가 계층마다 반복된다. 실제 multiply/add보다 checked API
검사와 status 처리 비용이 누적되는 구조가 주원인이다. Rollback snapshot 자체는 39 cycles로
관찰되어 이번 조건의 주병목이 아니다.

구간마다 cycle counter read와 maximum 갱신 비용이 포함되고 각 maximum도 서로 다른
sample에서 발생할 수 있으므로 위 표를 단순 합산하지 않는다. 최적화 전에는 checked public
API의 contract를 유지하면서, 검증된 ISR 전용 fast update를 별도 제공해 중복 검사를 제거하는
방향을 사용한다. PI, filter, transform의 실제 계산식과 제어 계수는 이 단계에서 바꾸지 않는다.

### 적용한 첫 최적화

위 측정 뒤 다음 경계를 적용했다. 이 항목은 build 검증을 마쳤지만 아직 board cycle 결과를
기록하지 않았으므로 deadline 개선값을 주장하지 않는다.

- App current-command publish 시 axis/vector target 제한을 한 번 계산한다.
- 40 kHz reference path는 준비된 target의 rate step과 최종 vector 제한만 계산한다.
- 원 안에 있는 정상 reference step에서는 정규화용 float 나눗셈을 실행하지 않는다.
- Checked filter, PI와 rate-limiter API는 유지하고 검증 완료 ISR 전용 fast API를 추가한다.
- FOC fast path는 App의 measurement/fault/rotor/CORDIC 검증을 precondition으로 사용한다.
- 중간값마다 반복하던 유한성 검사는 제거하되 최종 alpha-beta 전압 검사는 유지한다.
- Fast FOC 오류는 App fault 전이에서 motor-control reset으로 정리하므로 정상 경로에서
  controller runtime snapshot을 만들지 않는다.
- Motor-control fast path는 App의 미공개 계산 output에 직접 기록해 중간 전체 snapshot 복사를
  한 번 제거한다.

다음 board 측정에서는 기존 dry-run 기능 결과와 함께 전체/body/control/reference/FOC 및 FOC
하위 구간을 같은 조건으로 기록한다. 계측을 켠 결과로 병목 순위와 기능 동등성을 확인한 뒤,
계측을 끈 binary에서 hard deadline과 margin을 최종 판정한다.

첫 최적화 뒤 상세 계측을 유지한 dry-run에서 다음 값이 관찰되었다.

| 항목 | 최대 cycle |
|---|---:|
| current reference | 211 |
| FOC | 1034 |
| control | 1891 |
| fast-loop body | 4369 |
| 첫 ADC IRQ부터 종료까지 | 5397 |

Reference와 FOC는 줄었지만 전체 deadline은 여전히 초과한다. Body 바깥 경로가 약 1028 cycles이고,
당시 구조는 세 ADC가 각각 injected 완료 interrupt를 발생시켜 ADC1/2 shared handler와 ADC3
handler를 모두 거쳤다. 다음 변경은 동일 trigger와 변환 timing을 init에서 검증한 뒤 하나의
completion ADC interrupt에서 세 JDR을 일괄 수집한다. 정상 timing 판정 binary에서는 원인 분석용
상세 profiler를 끄고 전체/body DWT 계측만 유지한다. 상세 profiler가 켜진 측정값으로 최종
deadline 통과를 판정하지 않는다.

Completion ADC의 정상 JEOC 경로는 범용 `HAL_ADC_IRQHandler()`가 수행하는 상태 검사 전체를
반복하지 않는다. App IRQ entry가 driver의 검증된 세 JDR 수집을 호출하고 현재 JEOC/JEOS를
정리한 뒤 fast loop를 실행한다. 예상하지 않은 interrupt source는 HAL fallback으로 남긴다.
또한 ISR 전용 App API는 caller output에 직접 기록하여 정상 경로의 큰 임시 output과 마지막
전체 복사를 만들지 않는다.

Current-mode modulation 경로도 checked API와 ISR fast API를 분리한다. FOC가 유한한
alpha-beta 전압과 양의 `v_dc`를 보장한 뒤 SVPWM fast API는 실제 inverse Clarke, min-max,
overmodulation 경계와 duty clamp만 수행한다. PWM driver는 init에서 각 상의 period와 compare
register 주소를 검증·저장하고, fast write에서는 이미 [0, 1]인 duty의 compare 변환과 세 register
write만 수행한다. 일반 checked API와 open-loop bring-up 경로의 검증 계약은 유지한다.

Sensor 환산도 ADC driver가 이미 보장한 12-bit raw와 init에서 검증한 gain을 fast-path
precondition으로 사용한다. Current mode 진입 시 offset calibration 완료를 이미 확인했으므로
매 sample마다 calibration state를 다시 조회하지 않고 current sensor fast 환산을 사용한다.
보정 중/disabled 경로는 기존 checked state machine을 그대로 사용한다.
같은 전제에서 current-mode fault fast update는 phase trip/clear hysteresis, DC-link overvoltage,
active/latched mask와 first-fault snapshot을 모두 유지하되 pointer/init/finite 중복 검사만 생략한다.
Current mode의 Hall estimator가 보장한 `[0, 2*pi)` 전기각은 CORDIC fast API로 전달한다.
이 경로는 wrap, Q31 변환과 sine/cosine hardware 연산만 수행하고 init/pointer/range 및 이미 선택된
SINE function의 반복 검사를 생략한다.

위 변경을 모두 적용하고 상세 구간 profiler를 끈 current-control dry-run 보드 측정 결과는
다음과 같다. 기능 판정은 800 sample을 완료했으며 ADC/App 오류와 latched fault는 없었다.

| 항목 | 측정값 |
|---|---:|
| fast-loop body max | 2925 cycles |
| 첫 ADC IRQ부터 종료까지 max | 3220 cycles |
| deadline miss count | 0 |
| current-control update count | 800 |

기존 실패 기준선과 비교하면 body는 8331 cycles에서 2925 cycles로, 전체는 9346 cycles에서
3220 cycles로 감소했다. Body 바깥 ADC/IRQ 처리 차이도 약 1015 cycles에서 295 cycles로
감소했다. Hard deadline 4250 cycles는 만족하지만 bring-up 목표 3200 cycles는 20 cycles,
body MUST 기준 2800 cycles는 125 cycles 초과하므로 최적화 완료 기준으로 간주하지 않는다.
기능 확인에 사용한 자동 지령 주입과 800-sample 판정 코드는 측정 후 `main.c`에서 제거하고,
전체/body cycle 및 deadline miss 계측은 이후 실구동 검증을 위해 유지한다.

이 측정 뒤 Hall 해석을 Platform driver에서 Control의 motor별 `hall_decoder`로 분리했다.
따라서 위 `3220 cycles`는 변경 전 기준선이며 현재 binary의 timing 통과 근거로 재사용하지
않는다. 새 capture가 없는 decoder 정상 경로는 precomputed output 복사만 수행하도록 유지하되,
실구동 전 Hall transition/timeout을 포함한 전체 worst-case cycle을 보드에서 다시 측정해야 한다.

Hall decoder와 보정 profile을 통합한 뒤 축을 손으로 정·역회전한 board 시험에서는 첫 ADC IRQ부터
종료까지 `2530 cycles`, deadline miss `0`이 관찰됐다. 이후 PWM을 활성화한 저전류 current-mode
시험에서도 양방향 회전, `0.3 A` 부근의 간헐적 voltage saturation과 `0.4 A` 지령에서의 지속
saturation 동안 deadline miss는 `0`이었다. 다만 이 current-mode 시험의 정확한 maximum cycle과
관찰 시간은 기록되지 않았고 시험용 FOC snapshot 복사도 포함되어 있었으므로, `3200 cycles`
bring-up 목표의 최종 통과 근거로 사용하지 않는다. 시험 코드를 제거한 production current-mode
경로에서 maximum과 관찰 시간을 다시 측정해야 한다.

---

## 4. Fast path와 checked path

설정·명령 경로와 매 sample 실행되는 경로를 구분한다.

### Checked path

다음 작업은 초기화, command publish, 상태 전이 또는 느린 관리 loop에서 수행한다.

- pointer, 설정 범위와 `isfinite()` 검사
- motor/filter/PI/PWM 고정 parameter 계산과 검증
- 변하지 않은 command의 axis/vector clamp
- fault latch 원인 정리, hysteresis bookkeeping과 진단 snapshot 작성
- 상세 Hall observation 검증과 diagnostic snapshot 작성
- public API의 상세 status 생성

### Fast path

Fast path는 checked path가 보장한 precondition을 전제로 하며 다음의 최소 작업만 수행한다.

```text
ADC sample acquire
 -> current/voltage conversion
 -> 즉시 software over-current check
 -> electrical-angle extrapolation / sin-cos
 -> Clarke/Park / d-q filter / PI / voltage limit
 -> inverse Park / SVPWM
 -> precomputed PWM compare write
```

Fast path에는 다음을 두지 않는다.

- 큰 input/output/controller 구조체의 전체 초기화나 반복 복사
- 정상 성공 경로에서의 controller state snapshot/rollback
- 계층마다 반복되는 동일한 유효성 검사
- 문자열 생성, blocking 통신 또는 전체 telemetry publish
- 초기화 후 변하지 않는 peripheral mapping의 반복 분기
- 다수의 module을 거치는 범용 fault-manager bookkeeping

즉시 차단이 필요한 phase over-current와 measurement validity의 최소 검사는 fast path에
남긴다. 이를 느린 loop로 옮겨 timing을 맞추지 않는다. Checked/fast API를 분리하더라도
Control/Algorithm이 HAL 또는 peripheral register를 직접 접근해서는 안 된다.

---

## 5. 구간별 cycle 계측

최적화 전에 다음 구간을 독립적으로 측정한다.

1. ADC raw 획득과 sensor 환산
2. 즉시 fault 검사
3. Hall snapshot, estimator와 CORDIC
4. current reference 처리와 FOC
5. SVPWM과 PWM compare write
6. diagnostic/test output 처리

각 구간에 대해 `last`와 `max`를 보존한다. 계측값 합계와 전체 body 값의 차이를 확인해
계측 자체의 비용과 누락 구간을 검토한다. 임시 계측을 제거하기 전에도 전체 timing을 다시
측정한다.

최소 시험 조건은 다음과 같다.

- PWM 비활성 dry-run과 PWM 활성 상태
- 0 A reference 정상 경로
- current-reference rate limit 동작
- voltage-vector saturation과 anti-windup 동작
- Hall sector transition과 timeout 경계
- phase current trip 직전과 trip 발생
- 지원하는 최소/nominal/최대 DC-link 전압

각 조건은 Release와 실제 배포에 사용할 compiler option으로 최소 60초 실행하고 deadline
miss가 0인지 확인한다. Debug build를 사용했다면 해당 결과는 기능 관찰용으로만 기록한다.

---

## 6. 변경 절차와 회귀 방지

Fast-loop 경로를 바꾸는 변경은 다음 순서를 **MUST** 따른다.

1. 변경 전 build option, 실행 조건, 전체/구간별 max cycle을 기록한다.
2. 변경할 책임과 예상 cycle 영향을 명시한다.
3. 한 번에 하나의 병목 또는 하나의 module boundary만 변경한다.
4. 기존 수치/reference test로 계산 결과가 같음을 확인한다.
5. linked image의 disassembly와 stack usage를 확인한다.
6. 보드에서 정상 및 worst-case cycle을 다시 측정한다.
7. 이 문서의 통과 기준을 만족한 뒤 다음 기능을 fast loop에 추가한다.

다음 중 하나라도 해당하면 current-loop 실구동이나 상위 loop 개발로 진행하지 않는다.

- deadline miss가 1회 이상 발생
- 전체 maximum이 3200 cycles를 초과
- worst-case 분기를 계측하지 않음
- compiler option이나 clock 조건이 불명확함
- 기능 결과와 timing 결과를 같은 binary에서 확인하지 않음

기능 추가가 timing budget을 초과하면 우선 checked/slow/event path로 책임을 이동한다.
그래도 만족하지 못하면 20 kHz current-loop 같은 multi-rate 구조를 검토하되, scheduler에
명시하고 sample period에 의존하는 filter와 controller coefficient를 다시 설계·검증한다.

---

## 7. 결과 기록 형식

측정 결과는 다음 항목을 남긴다.

```text
Build/optimization:
CPU/PWM/control rate:
Test condition:
Total cycles max:
Body cycles max:
Segment cycles max:
Deadline miss count / observation time:
Functional result:
```

실제 보드에서 측정하지 않은 값을 hardware-verified 결과로 기록하지 않는다.
