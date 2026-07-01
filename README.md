# Embedded SW QA Portfolio

## 지원 직무
**Embedded SW QA Engineer / 차량제어 SW 플랫폼 품질 점검**

본 포트폴리오는 임베디드 시스템, 로봇 제어, 비전 기반 검사, 시뮬레이션 프로젝트를 통해 쌓은 **시스템 구현 및 검증 경험**을 정리한 자료입니다.

저는 단순히 기능을 구현하는 것보다, **센서 입력, 제어 판단, 하드웨어 동작, 결과 검증**이 하나의 흐름으로 연결되어야 실제 시스템 품질이 확보된다고 생각합니다. STM32 기반 임베디드 시스템과 로봇·비전 프로젝트를 수행하며 오류 원인을 추적하고, 동작 결과를 검증 가능한 구조로 만드는 경험을 쌓았습니다.

---

## 핵심 역량

### Embedded SW
- STM32F103RB 기반 MCU 제어 경험
- GPIO, Timer, Interrupt, ADC, DMA, EXTI 활용
- Keypad, Dot Matrix, 온도 센서 기반 임베디드 시스템 구현
- C 기반 상태 제어 및 인터럽트 처리 경험

### Robotics & Sensor Integration
- AMR 기반 도난 방지 시스템 구현
- RGB Camera, Depth Camera, LiDAR 기반 객체 탐지 및 추적
- 로봇팔 기반 점자 제작 및 나사 체결 검사 프로젝트 수행
- RealSense Camera 기반 비전 검사 흐름 구현

### SW QA / Validation
- 기능 구현 후 실제 동작 결과 검증
- 센서 입력값과 로봇 동작 결과 간 불일치 원인 분석
- 단계별 통합 과정에서 발생한 오류 추적 및 수정
- 점자 검증, 나사 체결 불량 감지, Isaac Sim 시뮬레이션 검증 경험

### Programming
- C / C++ / Python
- OpenCV
- ROS 기반 로봇 시스템 경험
- Isaac Sim 기반 시뮬레이션 개발 경험

---

## 주요 프로젝트

| No. | Project | Keywords | Summary |
|---|---|---|---|
| 01 | [STM32 Embedded System](./projects/01_stm32_embedded_system/README.md) | STM32, C, Timer, Interrupt, DMA, ADC | 키패드, 온도센서, Dot Matrix를 활용한 임베디드 시스템 구현 |
| 02 | [AMR Security System](./projects/02_amr_security_system/README.md) | AMR, Camera, LiDAR, Tracking | 박물관 환경에서 AMR 기반 도난 방지 및 추적 시스템 구현 |
| 03 | [Braille Robot Validation](./projects/03_braille_robot_validation/README.md) | Robot Arm, OpenCV, Validation | 로봇팔 기반 맞춤형 점자 제작 및 점형 검증 |
| 04 | [Screw Defect Detection](./projects/04_screw_defect_detection/README.md) | RealSense, Vision, Robot Arm | 나사 체결 불량 감지 및 조치 시스템 구현 |
| 05 | [Isaac Sim Sorting](./projects/05_isaac_sim_sorting/README.md) | Isaac Sim, QR, Simulation | 물류 창고 박스 분류 시뮬레이션 구현 |

---

## 차량제어 SW 플랫폼 품질 점검 직무와의 연결

이 직무는 기능안전 및 A-SPICE 기준에 따라 SW 개발 프로세스 준수 여부를 점검하고, SW 품질 지표 수립, 필드 이슈 원인 분석, 재발 방지 대책 이행을 확인하는 역할입니다.

제가 수행한 프로젝트들은 단순 구현에서 끝나지 않고, 실제 동작 결과를 확인하고 오류 원인을 추적하는 과정을 포함했습니다.

- STM32 프로젝트: 주변장치 설정, 인터럽트, GPIO, ADC/DMA를 활용한 임베디드 동작 검증
- AMR 프로젝트: 센서 입력과 추적 동작 간 연결 검증
- 점자 프로젝트: 로봇팔 출력 결과를 비전 기반으로 검증
- 나사 체결 프로젝트: 검사 결과와 로봇 조치 흐름 연결
- Isaac Sim 프로젝트: QR 인식, 박스 방향, 로봇 분류 동작의 단계별 통합 검증

---

## 문제 해결 경험

프로젝트 수행 중 단순 기능 구현보다 오류 원인을 추적하고 재발을 방지하는 과정에 집중했습니다.

- STM32 프로젝트에서 JTAG 핀 설정 문제로 GPIO 입력이 정상 동작하지 않는 원인을 분석하고 AFIO 설정을 수정
- Dot Matrix 일부 핀 결함 상황에서 8x15 출력 구조로 우회하여 기능 완성
- Isaac Sim 프로젝트에서 QR 인식, 박스 방향, 로봇 동작 불일치 문제를 단계별로 분리하여 디버깅
- 로봇팔/비전 프로젝트에서 실제 출력 결과와 인식 결과 간 차이를 검증 로직으로 보완

자세한 내용은 [Troubleshooting](./docs/troubleshooting.md)에 정리했습니다.

---

## 추가 문서

- [Skill Summary](./docs/skill_summary.md)
- [Troubleshooting](./docs/troubleshooting.md)
- [Task Test Preparation](./docs/task_test_preparation.md)

---

## Portfolio PDF

포트폴리오 PDF는 추후 `portfolio/` 폴더에 추가할 예정입니다.

---

## Contact

- GitHub: https://github.com/jiiiiihong
