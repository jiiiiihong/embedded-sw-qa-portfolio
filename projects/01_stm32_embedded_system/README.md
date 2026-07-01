# STM32 Embedded System

## 프로젝트 개요
STM32F103RB MCU를 활용하여 키패드 입력, Dot Matrix 출력, 온도 센서 측정, 타이머, 잠금장치 기능을 포함한 임베디드 시스템을 구현했습니다.

## 사용 기술
- C
- STM32F103RB
- GPIO
- Timer
- Interrupt
- ADC
- DMA
- EXTI
- Keypad
- Dot Matrix

## 주요 구현 기능
- 4x4 Keypad 입력 처리
- Dot Matrix 기반 시간 및 상태 출력
- ADC/DMA 기반 온도 센서 데이터 처리
- Timer Interrupt 기반 주기 동작 제어
- 비밀번호 입력 및 잠금장치 기능 구현
- 상태 전이에 따른 표시 로직 구현

## 문제 해결
### JTAG 핀 충돌 문제
PB3, PB4 핀이 JTAG 기능으로 할당되어 Keypad 입력이 정상 동작하지 않는 문제가 발생했습니다.  
AFIO 설정을 통해 JTAG 기능을 비활성화하고 GPIO로 사용할 수 있도록 수정했습니다.

### Dot Matrix 핀 결함 대응
Dot Matrix 일부 핀이 정상 동작하지 않아 8x16 출력이 어려운 상황이 발생했습니다.  
출력 구조를 8x15로 우회하여 핵심 표시 기능을 완성했습니다.

## 배운 점
임베디드 시스템에서는 코드 로직뿐 아니라 핀맵, 주변장치 설정, 인터럽트 우선순위, 하드웨어 결함 가능성까지 함께 고려해야 한다는 점을 경험했습니다.
