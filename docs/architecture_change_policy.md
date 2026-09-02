# Architecture Change Policy

## 1. 목적

현재 architecture는 최종 정답이 아니라 유지보수 가능한 **기본값**이다.

구현 중 다음과 같은 상황은 반드시 생길 수 있다.

- 모듈 경계가 실제 데이터 흐름과 맞지 않음
- 성능상 함수 호출/변환 비용이 큼
- 하드웨어 accelerator 때문에 abstraction이 어색함
- 새로운 estimator나 modulation을 추가해야 함
- 기존 controller가 더 큰 composite subsystem이 되는 것이 자연스러움
- ownership/source-of-truth가 현재 구조로 표현하기 어려움

이때 "규칙을 깨도 되는가?"보다 먼저 **책임, dependency, ownership을 다시 정의할 수 있는가**를 본다.

---

## 2. 변경 판단 순서

구조가 불편할 때 다음 순서로 판단한다.

1. 현재 module의 책임이 잘못 정의되었는가?
2. interface에 필요한 데이터가 빠졌는가?
3. 데이터를 전달하기 싫어서 직접 include/global access를 하려는 것인가?
4. 서로 독립적인 module을 composite subsystem으로 재정의하는 것이 더 자연스러운가?
5. 실제 측정된 성능 문제인가, 예상만 한 문제인가?
6. testability와 source-of-truth가 악화되는가?
7. 변경 후 dependency 방향을 한 문장으로 설명할 수 있는가?

---

## 3. controller끼리 직접 include하고 싶어질 때

기본적으로 다음은 피한다.

```text
position_controller -> speed_controller
speed_controller -> foc
```

대신 `motor_control`이 wiring한다.

정말 position module이 speed loop까지 소유해야 한다면 responsibility를 변경한다.

```text
position_controller
```

를 유지한 채 내부에 speed PI를 몰래 넣지 말고:

```text
position_servo
```

같이 composite subsystem임을 이름으로 나타낸다.

---

## 4. 두 module이 서로 state를 필요로 할 때

양방향 dependency를 바로 만들지 않는다.

지양:

```text
A <----> B
```

먼저 다음을 검토한다.

- 공통 data type을 하위 module로 추출
- 상위 coordinator가 값을 전달
- read-only snapshot/input struct 사용
- 하나의 owner를 정하고 다른 쪽은 복사본이 아닌 입력으로 받기
- 두 module이 실제로 하나의 subsystem인지 재평가

circular include/dependency는 architecture warning으로 취급한다.

---

## 5. hardware accelerator 때문에 계층이 깨질 때

예: CORDIC.

기본:

```text
Control/Algorithm float
   ->
cordic_driver float API
   ->
Q31 / HAL / peripheral
```

성능 문제가 실제로 측정되면:

```text
optimized transform backend
   ->
CORDIC Q31
```

같은 최적화를 허용할 수 있다.

이 경우에도 hardware-specific representation을 전체 Control 상태로 확산시키지 않는 것을 우선한다.

---

## 6. 성능 때문에 module 경계를 합칠 때

profiling 없이 "함수 호출이 느릴 것 같다"는 이유로 module을 합치지 않는다.

권장 순서:

1. reference implementation 완성
2. timing/profile 측정
3. 병목 확인
4. 특정 hot path 최적화
5. 결과 비교 test
6. 필요하면 interface/backend 변경

static inline, LTO, compiler optimization으로 해결되는지 먼저 확인한다.

---

## 7. 알고리즘 교체에 대비한 경계

교체 가능성이 높은 부분은 hardware-independent interface를 유지한다.

예:

```text
SVPWM
 -> DPWM
 -> SPWM

Hall estimator
 -> encoder estimator
 -> EEMF/PLL estimator

PI
 -> 다른 controller
```

상위 module은 가능한 한 구현 세부사항보다 입력/출력 contract에 의존한다.

단, 모든 것을 추상화하기 위해 function pointer/virtual interface를 남발하지 않는다. 실제 교체 필요성이 있을 때 abstraction을 도입한다.

---

## 8. 문서 갱신 규칙

architecture 변경이 다음 중 하나에 해당하면 관련 `.md`를 함께 수정한다.

- dependency 방향 변경
- module 책임 변경
- public API 의미 변경
- canonical numeric representation 변경
- source-of-truth owner 변경
- scheduler/rate 구조 변경
- directory/file naming 정책 변경
- safety/fault handling path 변경

Git commit에서 코드와 문서 변경을 함께 남기는 것을 권장한다.

---

## 9. Architecture Decision 기록

큰 변경은 별도 ADR까지 만들 필요가 없더라도 최소한 관련 문서에 다음을 남긴다.

```text
Decision:
무엇을 바꿨는가?

Reason:
기존 구조의 어떤 문제가 실제로 확인되었는가?

Trade-off:
무엇을 얻고 무엇을 잃는가?

Compatibility/Migration:
기존 module/API/state를 어떻게 옮길 것인가?
```

짧은 변경은 Git commit message로 충분할 수 있다. 장기적으로 다시 논쟁할 가능성이 있는 결정만 문서에 남긴다.

---

## 10. AI agent에게 요구할 규칙

AI agent가 코드를 생성/수정할 때:

### MUST

- 기존 module의 책임과 dependency를 먼저 확인한다.
- 기존 convention을 이유 없이 변경하지 않는다.
- circular dependency를 추가하지 않는다.
- Control/Algorithm에 HAL 호출을 추가하지 않는다.
- 동일 의미 state의 새 복사본을 만들기 전에 owner를 확인한다.
- public API 의미나 architecture가 바뀌면 관련 문서 변경도 제안한다.

### SHOULD

- 필요한 최소 범위만 변경한다.
- 새로운 abstraction은 실제 문제를 해결할 때만 도입한다.
- 구조가 기존 규칙과 맞지 않으면 임시 우회보다 책임 재정의를 제안한다.
- 성능 최적화는 profiling 또는 명확한 constraint를 근거로 한다.

### MAY

- 구현 중 더 나은 구조가 확인되면 기존 문서를 수정할 수 있다.
- 단, 기존 규칙을 조용히 무시하지 말고 변경 이유를 명시한다.
