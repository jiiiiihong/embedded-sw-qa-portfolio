# STM32 Embedded System

## 프로젝트 개요
STM32F103RB MCU를 활용하여 키패드 입력, Dot Matrix 출력, 온도 센서 측정, 타이머, 잠금장치 기능을 포함한 임베디드 시스템을 구현했습니다.

업로드 자료 기준 핵심 구현 파일은 `final_project.c`이며, 레지스터 직접 설정 방식으로 GPIO, Timer, DMA, ADC, USART, NVIC를 구성했습니다.

---

## 사용 기술
- C
- STM32F103RB / Cortex-M3
- GPIO
- Timer Interrupt
- ADC1 + DMA1 Channel1
- USART1 Interrupt
- NVIC
- AFIO Remap
- 4x4 Keypad
- Dot Matrix Display

---

## 시스템 흐름

```text
4x4 Keypad 입력
→ TIM4 기반 row/column scan
→ Queue에 입력값 저장
→ TIM1 기반 Dot Matrix multiplexing 출력
→ TIM2 기반 온도 센서 ADC/DMA 값 갱신
→ TIM3 기반 시간 카운트 및 timeout 처리
→ 인증 입력/오류 상태 표시
```

---

## 주요 구현 기능

### 1. Queue 기반 입력 처리
키패드 입력, 시간 설정, 인증 설정/입력 데이터를 구분하기 위해 Queue 구조를 구현했습니다.

| Queue | 역할 |
|---|---|
| `myQueue` | 일반 키패드 입력 표시 |
| `time_set_que` | 시간 설정 입력 저장 |
| `passcode_que` | 인증값 설정 데이터 저장 |
| `my_passcode_que` | 사용자가 입력한 인증값 저장 |
| `r_data_que` | USART 수신 데이터 저장 |

### 2. Dot Matrix Multiplexing
TIM1 interrupt에서 row scan과 column 출력값을 갱신하여 Dot Matrix를 구동했습니다.

- PC[7:0], PC[15:8]을 column 출력으로 사용
- PB[7:0]을 row 출력으로 사용
- 숫자, 상태 문자, 온도 표시를 font table로 구성

### 3. Timer 역할 분리
각 Timer에 역할을 분리해 시스템 동작을 구성했습니다.

| Timer | 역할 |
|---|---|
| TIM1 | Dot Matrix scan / display refresh |
| TIM2 | ADC/DMA 기반 온도값 갱신 및 일부 timeout count |
| TIM3 | 시계 카운트, scrolling, 인증 timeout, O/X 표시 시간 관리 |
| TIM4 | Keypad row/column scan 및 debounce 처리 |

### 4. ADC + DMA 기반 온도 측정
ADC1 변환 결과를 DMA로 메모리에 저장하고, TIM2 interrupt에서 일정 주기마다 전압과 온도 표시값을 계산했습니다.

### 5. 인증 및 잠금 기능
특정 입력 패턴을 통해 인증값 설정 모드에 진입하고, 입력 길이와 종료 문자를 기준으로 저장/비교하도록 구성했습니다.

- 설정 모드 진입
- 4~8자리 입력 처리
- 설정 timeout 처리
- 입력 timeout 처리
- 정답/오답 표시
- 3회 연속 실패 시 잠금 상태 표시

---

## 트러블슈팅

### 문제 1. PB3/PB4 입력이 정상적으로 동작하지 않음
**원인**  
STM32F103RB의 PB3, PB4는 기본적으로 JTAG 기능에 묶여 있어 일반 GPIO처럼 사용할 수 없었습니다.

**해결**  
AFIO remap 설정으로 JTAG 기능을 비활성화했습니다.

**결과**  
키패드 입력에 필요한 핀을 GPIO로 활용할 수 있게 되었습니다.

---

### 문제 2. Dot Matrix 일부 핀이 불량하여 8x16 출력이 깨짐
**원인**  
실습 환경의 Dot Matrix 일부 핀이 정상적으로 동작하지 않아 전체 8x16 표시를 그대로 사용할 수 없었습니다.

**해결**  
정상 동작하는 핀을 기준으로 표시 범위를 재구성하고, 8x15 형태로 출력 구조를 우회했습니다.

**결과**  
하드웨어 결함 상황에서도 타이머, 잠금장치, 온도 표시 등 핵심 기능을 완성했습니다.

---

### 문제 3. Keypad 입력 bouncing으로 중복 입력 발생
**원인**  
TIM4 interrupt에서 row/column을 빠르게 scan하면서 하나의 키 입력이 여러 번 Queue에 들어갈 수 있었습니다.

**해결**  
row scan 직후 짧은 delay를 두고, 입력 확정 후 추가 delay를 적용해 중복 입력을 줄였습니다.

**결과**  
키패드 입력이 Queue에 중복 저장되는 현상을 줄였습니다.

---

### 문제 4. 시간 설정/인증 설정 모드가 서로 충돌
**원인**  
일반 입력, 시간 설정, 인증 설정, 인증 검증 모드가 모두 Keypad 입력을 공유하면서 상태 충돌이 발생할 수 있었습니다.

**해결**  
각 모드별 Queue와 상태 플래그를 분리했습니다.

**결과**  
동일한 키패드 입력을 여러 기능에서 사용하면서도 상태 전이를 관리할 수 있게 되었습니다.

---

### 문제 5. ADC 값 표시가 실시간 출력과 맞물려 흔들림
**원인**  
온도 변환과 Dot Matrix 출력이 동시에 진행되면서 표시 갱신 타이밍이 불안정할 수 있었습니다.

**해결**  
ADC 변환값은 DMA로 갱신하고, TIM2 interrupt에서 일정 주기마다 온도 자릿수만 계산하도록 분리했습니다.

**결과**  
센서 데이터 취득과 화면 출력 갱신의 역할을 분리하여 표시 안정성을 높였습니다.

---

## 배운 점
- 임베디드 시스템에서는 코드 로직뿐 아니라 핀맵, 리맵 설정, 주변장치 클럭, 인터럽트 주기까지 함께 검증해야 한다.
- 하드웨어 결함이 있을 때는 기능 전체를 포기하기보다 요구 기능을 만족하는 우회 구조를 설계해야 한다.
- 하나의 입력 장치를 여러 기능에서 공유할 경우 Queue와 상태 플래그를 명확히 분리해야 한다.
- 타이머 기반 시스템은 각 Timer의 책임을 분리해야 디버깅 가능성이 높아진다.

---

## 직무 연결 포인트
Embedded SW QA 관점에서 이 프로젝트는 다음 역량과 연결됩니다.

- MCU peripheral 설정과 실제 하드웨어 동작 간 불일치 원인 분석
- Timer, Interrupt, DMA 기반 동작의 주기적 검증 경험
- 입력, 상태 전이, 출력 표시를 분리해 오류를 추적한 경험
- 핀 충돌, 하드웨어 결함, debounce 등 실제 임베디드 환경의 문제 해결 경험
