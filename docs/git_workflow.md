# Git Workflow

## 1. 기본 철학

이 프로젝트는 혼자 또는 소수 인원이 장기간 수정하는 embedded/control 프로젝트를 전제로 한다.

Git의 목적은 단순 백업이 아니라:

- 정상 동작했던 상태로 돌아가기
- 문제를 발생시킨 변경을 찾기
- generated code와 사용자 변경을 구분하기
- tuning과 algorithm 변경을 구분하기
- architecture 결정의 history를 남기기

이다.

복잡한 Git flow를 기본으로 사용하지 않는다.

---

# 2. Branch를 꼭 써야 하는가?

아니다.

이 프로젝트를 혼자 개발하고 있고 `main`의 각 commit이 항상 비교적 안정적인 상태라면 **초기 bring-up 단계에서는 `main` 하나만으로도 충분하다.**

즉 다음 방식도 정상적이다.

```text
main
  A -- B -- C -- D -- E
```

작은 embedded 개인 프로젝트에서 의미 없이 feature branch를 계속 만드는 것은 오히려 관리 부담이 될 수 있다.

## 기본 권장안

### 초기 bring-up

```text
main 중심
```

짧고 검증 가능한 변경을 자주 commit한다.

### branch를 쓰는 경우

다음 상황에서는 branch가 유용하다.

1. 며칠 이상 걸릴 실험적 기능
2. 현재 정상 동작 상태를 유지하면서 대규모 refactor
3. 실패 가능성이 높은 sensorless/observer 알고리즘 실험
4. CubeMX/peripheral architecture를 크게 바꾸는 작업
5. 서로 다른 접근법을 비교해야 하는 작업
6. release/hardware test를 위해 현재 `main`을 보존해야 하는 경우
7. 여러 사람이 동시에 작업하는 경우

예:

```text
main
  |
  +--- feature/sensorless-esmo
  |
  +--- refactor/motor-control
```

---

# 3. 추천 Branch 정책

복잡한 `develop`, `release`, `hotfix` branch를 기본으로 두지 않는다.

혼자 개발하는 현재 프로젝트에서는 다음이면 충분하다.

```text
main
feature/<topic>
refactor/<topic>
experiment/<topic>
```

예:

```text
feature/hall-estimator
feature/current-control
refactor/foc-interface
experiment/sensorless-esmo
```

짧은 작업은 branch 없이 `main`에서 바로 commit해도 된다.

---

# 4. Branch를 만들지 말아야 하는 경우

다음 정도의 작업마다 branch를 만들 필요는 없다.

```text
변수 rename
주석 수정
작은 driver bug fix
PI gain 변경
Doxygen 추가
header include 정리
```

이런 변경은 논리적으로 독립된 commit으로 남기는 것으로 충분하다.

---

# 5. Branch의 수명

feature branch는 장기 보관용이 아니다.

권장:

```text
branch 생성
    ↓
작업 / commit
    ↓
검증
    ↓
main에 merge
    ↓
branch 삭제
```

몇 달 동안 branch를 유지하면서 `main`과 크게 벌어지는 것은 피한다.

실험을 보존하려면 필요 시 tag 또는 Git commit hash로 충분하다.

---

# 6. Commit 시점

기본 원칙:

> 한 가지 논리적인 변경이 완료되었고, 그 상태로 돌아와도 의미가 있을 때 commit한다.

시간이나 줄 수를 기준으로 하지 않는다.

좋은 commit 시점:

- 새 module interface 완성
- driver 기능 하나가 실제 hardware에서 검증됨
- unit test 통과
- bug 하나 수정
- refactoring 하나 완료
- CubeMX 설정 변경 완료
- control loop 하나가 동작
- tuning 결과를 고정
- architecture/documentation decision 변경

---

# 7. Commit 전에 최소 확인

가능하면 다음을 확인한다.

```text
[ ] build 성공
[ ] compiler warning 확인
[ ] 관련 기능 검증
[ ] unrelated file이 섞이지 않았는지 확인
[ ] generated code diff가 의도한 것인지 확인
[ ] 필요한 문서/Doxygen 갱신
```

hardware 기능이면:

```text
[ ] 실제 board에서 기본 동작 확인
```

까지 하는 것을 권장한다.

---

# 8. Commit 크기

Commit 하나에는 하나의 목적을 둔다.

좋은 예:

```text
feat(platform): implement HRTIM PWM driver
fix(platform): correct ADC trigger source
refactor(control): separate SVPWM from FOC
docs: document Q31 boundary policy
```

나쁜 예:

```text
update code
various fixes
fix pwm adc hall foc and comments
```

---

# 9. Commit Message Convention

Conventional Commits를 단순화해서 사용한다.

권장 type:

```text
feat       기능 추가
fix        버그 수정
refactor   동작 변경 없는 구조 개선
test       테스트 추가/수정
docs       문서 변경
chore      build, CubeMX, project 설정
tune       제어 파라미터 tuning
perf       실제 성능 최적화
```

`tune`은 Conventional Commits 표준 type은 아니지만 모터제어 프로젝트에서 tuning history를 구분하기 위해 사용한다.

형식:

```text
<type>(<scope>): <summary>
```

예:

```text
feat(platform): add HRTIM PWM driver
feat(algorithm): add SVPWM calculation
feat(control): add d-q current controller

fix(platform): correct ADC injected trigger
fix(control): correct electrical angle sign

refactor(control): separate SVPWM from FOC

tune(control): update current PI gains

docs: add software development process
```

scope는 필요할 때만 사용한다.

---

# 10. Refactor와 기능 변경 분리

이 원칙은 중요하다.

예를 들어:

```text
pi_ctrl
  ->
pi_controller
```

로 rename하면서 anti-windup algorithm까지 동시에 바꾸지 않는다.

권장:

```text
refactor(control): rename pi_ctrl to pi_controller
feat(control): add back-calculation anti-windup
```

이렇게 해야 regression 원인을 쉽게 찾을 수 있다.

---

# 11. CubeMX 변경은 별도 Commit

CubeMX regeneration은 diff가 크고 generated code가 섞이므로 별도 commit을 권장한다.

예:

```text
chore(cubemx): configure HRTIM timers F G H
```

그 다음 사용자 코드:

```text
feat(platform): add three-phase PWM wrapper
```

이렇게 분리한다.

가능하면 한 commit 안에서:

```text
CubeMX generated changes
+
FOC algorithm changes
+
documentation
```

를 한꺼번에 섞지 않는다.

---

# 12. Tuning Commit

제어 gain 변경은 기능 변경과 구분한다.

예:

```text
tune(control): update current PI gains
tune(control): reduce speed-loop bandwidth
```

Bug fix와 tuning도 분리한다.

```text
fix(control): correct anti-windup feedback sign
tune(control): retune current PI after anti-windup fix
```

이렇게 하면 나중에 실험 결과와 commit을 대응시키기 쉽다.

---

# 13. WIP Commit

작업이 길어질 경우 local/feature branch에서 임시 WIP commit을 사용해도 된다.

```text
wip: initial Hall estimator integration
```

단, `main` history를 깔끔하게 유지하고 싶다면 merge 전에 squash 또는 rebase로 정리할 수 있다.

혼자 쓰는 프로젝트라면 WIP commit을 무조건 정리할 필요는 없다. 실제로 도움이 되는 history라면 유지해도 된다.

---

# 14. Merge 방식

초기 개인 프로젝트에서는 과도한 정책이 필요 없다.

## 작은 feature branch

commit history가 의미 있으면 일반 merge 또는 fast-forward.

```text
git switch main
git merge feature/current-control
```

## 실험 중 WIP commit이 많은 branch

main에 넣기 전에 squash를 고려한다.

예:

```text
experiment/sensorless-esmo
  - wip
  - debug
  - debug2
  - temp fix
```

를:

```text
feat(control): add initial ESMO rotor estimator
```

정도의 의미 있는 history로 정리할 수 있다.

단, 원인을 추적할 가치가 있는 intermediate commit은 무조건 squash할 필요가 없다.

---

# 15. Rebase를 써야 하는가?

필수 아니다.

혼자 개발하면서 branch가 짧다면:

```text
git rebase main
```

을 사용해 history를 단순하게 만들 수 있다.

하지만 Git 사용 자체가 개발보다 복잡해진다면 단순 merge를 사용한다.

이 프로젝트에서 Git history의 목적은 "예쁜 그래프"보다 **검증 가능한 변경 기록**이다.

---

# 16. Tag

Hardware bring-up milestone에는 tag가 매우 유용하다.

예:

```text
bringup-pwm-v1
bringup-adc-sync-v1
open-loop-svpwm-v1
current-control-v1
speed-control-v1
```

또는 semantic-like version:

```text
v0.1.0-pwm
v0.2.0-open-loop
v0.3.0-current-control
```

Tag는 다음 상황에서 특히 유용하다.

> "이 버전에서는 분명 PWM이 정상 동작했다."

> "current loop을 붙이기 전 open-loop 버전으로 돌아가고 싶다."

---

# 17. Hardware Test와 Commit

가능하면 hardware 검증 결과를 commit message 또는 tag 주변 기록과 대응시킨다.

예:

```text
feat(platform): synchronize ADC sampling with HRTIM
```

이 commit은 최소한:

- build 성공
- scope/debug GPIO로 trigger timing 확인
- sample sequence 확인

후 생성하는 것이 좋다.

모든 측정 결과를 commit message에 길게 넣을 필요는 없다.

복잡한 bring-up 결과는 `docs/` 또는 issue/log에 기록할 수 있다.

---

# 18. `.gitignore`

반드시 생성물/IDE 임시 파일을 적절히 제외한다.

일반적으로 검토할 대상:

```text
Debug/
Release/
build/
*.o
*.elf
*.map
*.list
.vscode/
.settings/
.metadata/
```

단, 프로젝트 재현에 필요한 IDE project file까지 무조건 제외하지 않는다.

CubeMX `.ioc` 파일은 반드시 version control에 포함한다.

```text
*.ioc  -> track
```

---

# 19. Generated Code

Generated code도 재현 가능성과 diff 확인을 위해 보통 repository에 포함한다.

특히 STM32CubeMX 프로젝트에서는:

```text
.ioc
generated Core files
startup
linker script
```

를 함께 관리하는 것이 실용적이다.

단, generated file을 사람이 직접 수정해야 하는 경우 CubeMX user-code section 또는 별도 wrapper module을 우선한다.

---

# 20. `main`의 안정성 수준

이 프로젝트에서는 `main`을 반드시 release 수준으로 유지할 필요는 없다.

하지만 최소한 다음은 권장한다.

```text
main:
- build 가능
- 알려진 치명적 regression 없음
- 해당 commit message가 주장하는 기능은 검증됨
```

며칠 동안 깨진 상태가 필요하다면 branch를 만드는 것이 낫다.

---

# 21. 추천 실제 Workflow

## 작은 작업

```text
main
  ↓
코드 수정
  ↓
build/test
  ↓
commit
```

예:

```text
feat(platform): add PWM disable function
```

## 큰 작업

```text
main
  ↓
git switch -c feature/current-control
  ↓
여러 commit
  ↓
hardware test
  ↓
main merge
  ↓
branch delete
```

## 위험한 실험

```text
git switch -c experiment/sensorless-esmo
```

정상 동작 중인 `main`을 건드리지 않고 자유롭게 실험한다.

---

# 22. 초기 예상 Commit Sequence

예:

```text
chore: initialize STM32G474 motor drive project

docs: add software architecture and coding conventions

refactor: add App Control Algorithm Platform Common directories

feat(common): add shared motor-control vector types

chore(cubemx): configure HRTIM timers

feat(platform): add HRTIM PWM driver interface

feat(platform): implement synchronized three-phase PWM

feat(platform): add PWM safe disable

chore(cubemx): configure injected ADC channels

feat(platform): add synchronized ADC acquisition

feat(algorithm): add Clarke and Park transforms

test(algorithm): add transform round-trip tests

feat(algorithm): add PI controller

feat(algorithm): add SVPWM

feat(app): add open-loop rotating voltage test

feat(control): add d-q current control

tune(control): tune current-loop PI gains
```

---

# 23. 언제 Branch를 만들지 판단하는 간단한 기준

다음 질문 중 하나라도 `yes`면 branch를 고려한다.

```text
이 작업이 하루 이상 깨진 상태를 만들 가능성이 큰가?
현재 정상 동작 버전을 계속 보존하고 싶은가?
여러 접근법을 비교할 것인가?
대규모 refactor인가?
실패해도 버릴 수 있는 실험인가?
여러 사람이 동시에 작업하는가?
```

전부 `no`라면 `main`에서 바로 작업해도 된다.

---

# 24. 결론

이 프로젝트의 기본 Git 전략은:

```text
짧고 검증 가능한 commit
        +
main 중심 개발
        +
필요할 때만 짧은 feature/experiment branch
        +
hardware milestone tag
```

이다.

Git 사용법 자체가 firmware 개발보다 복잡해지지 않도록 한다.
