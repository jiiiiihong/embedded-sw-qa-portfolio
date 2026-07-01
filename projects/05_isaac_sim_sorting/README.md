# Isaac Sim Sorting Simulation

## 프로젝트 개요
NVIDIA Isaac Sim의 `finalfac1.usd` 물류 창고 환경에서 QR 코드가 부착된 박스를 컨베이어로 공급하고, 두 대의 로봇이 출고일 기준으로 박스를 분류하는 시뮬레이션을 구현했습니다.

초기에는 단일 로봇이 박스를 밀어 분류하는 동작을 검증했고, 이후 QR 인식, 패키지 CSV 매칭, 2대 로봇 협동 분류, 최종 목적지 도달 판정까지 단계적으로 통합했습니다.

---

## 사용 기술
- NVIDIA Isaac Sim
- Python
- USD / USD Physics
- OpenCV QRCodeDetector
- Isaac Camera Sensor
- Multi-Robot Control
- Conveyor 기반 물류 흐름 구성
- CSV 기반 패키지 라우팅
- ROS 2 외부 서비스 연동 보조 로직

---

## 입력 자료 및 구현 파일
업로드한 자료 기준 주요 구현 파일은 다음과 같습니다.

| 자료 | 내용 |
|---|---|
| `Step4_Stage1_01_finalfac1_two_robot_sort_v17.py` | 최종 통합 코드. QR 인식, CSV 라우팅, 2대 로봇 분류, 최종 게이트 판정 포함 |
| `packages_2026-06-08.csv` 등 | `package_id`, `customer_name`, `route_zone`, `qr_id` 기반 배송 라우팅 데이터 |
| `box_assets/`, `qr_codes/` | QR 코드가 부착된 패키지 USD 및 QR 이미지 |
| `qr_debug/` | robot1 상단 카메라에서 캡처한 QR 디버그 이미지 |
| `finalfac1.usd` | 물류 창고 시뮬레이션 환경 |
| `isaacsim분류.mkv` | 최종 분류 시연 영상 |

대용량 USD, 영상, QR 이미지 전체는 GitHub 저장소에 직접 업로드하지 않고, 포트폴리오 설명과 핵심 구조 중심으로 정리했습니다.

---

## 시스템 흐름

```text
패키지 CSV 로드
→ QR 코드가 포함된 박스 USD 선택
→ 컨베이어 위에 박스 스폰
→ robot1 상단 카메라로 QR 인식
→ QR payload를 CSV의 route_zone과 매칭
→ today / day2 / day3 target 결정
→ robot1이 today / not_today 1차 분류
→ day2, day3 박스는 robot2가 2차 분류
→ 최종 목적지 영역과 박스 bbox overlap 확인
→ 다음 박스 스폰
```

---

## 핵심 구현 기능

### 1. QR 기반 패키지 라우팅
`packages_2026-06-08.csv` 파일의 `qr_id`와 `route_zone`을 읽어 박스의 목적지를 결정했습니다.

- `route_zone == today-date` → `today`
- 다음 출고일 → `day2`
- 그 이후 출고일 → `day3`

QR payload가 CSV에 없을 경우에도 실험이 중단되지 않도록 기존 QR 날짜 규칙 기반 fallback을 두었습니다.

### 2. robot1 상단 QR 카메라
robot1 작업대 상단에 USD Camera를 생성하고 Isaac Camera Sensor로 RGB 이미지를 캡처했습니다.

- 기본 카메라 해상도: `1024x1024`
- focal length 및 aperture 조정으로 QR 영역 확대
- OpenCV `QRCodeDetector`로 실제 QR decode 시도
- decode 실패 시 `user:qr_payload` fallback 사용
- `qr_debug/BOX_xxxx_robot1_qr_camera.png`로 실패 화면 저장

### 3. 2단계 분류 시나리오
robot1과 robot2의 역할을 분리했습니다.

| 단계 | 로봇 | 판단 기준 | 동작 |
|---|---|---|---|
| 1차 분류 | robot1 | QR target | today면 왼팔, day2/day3이면 오른팔 |
| 2차 분류 | robot2 | robot1에서 읽은 target | day2와 day3를 최종 분기 |

robot2는 QR을 다시 읽지 않고 robot1에서 이미 판정한 target 정보를 전달받아 동작하도록 구성했습니다.

### 4. 최종 목적지 도달 판정
초기에는 박스 중심점이 목적지 영역 안에 들어와야 성공으로 판정했습니다. 하지만 실제 컨베이어 분류에서는 박스가 목적지 영역을 일부 통과하거나 걸쳐도 분류 성공으로 보는 것이 자연스러웠습니다.

따라서 최종 판정을 `center inside` 방식에서 `bbox overlap` 방식으로 바꾸었습니다.

```text
기존: box 중심점이 final gate 내부에 있어야 성공
개선: box bounding box와 final gate bounding box가 겹치면 성공
```

이 변경으로 박스가 목적지 영역을 통과했지만 중심점 조건 때문에 실패 처리되던 문제를 줄였습니다.

### 5. 박스 QR 면 방향 고정
box asset 내부에 이미 QR 이미지가 박혀 있었지만, 스폰 방향에 따라 QR이 카메라에서 보이지 않는 문제가 발생했습니다.

별도 테스트 결과 QR이 보이는 면은 asset의 local `-Y` 면임을 확인했고, 스폰 시 `y_neg` 면이 위로 오도록 회전값을 강제했습니다.

```text
QR face test 결과: local -Y face에 QR 존재
적용: --box-qr-face-up y_neg
```

### 6. 대형 박스 충돌 프록시
일부 asset은 시각적 높이가 로봇팔 분류 동작보다 높게 측정되어 gripper와 충돌하거나 동작이 불안정해질 수 있었습니다.

이를 위해 시각 asset은 유지하되 충돌체 높이를 제한하는 proxy 정책을 추가했습니다.

- asset collision 비활성화
- 낮은 proxy collider 생성
- gripper edge snag 방지를 위해 XY scale 축소

---

## 트러블슈팅

### 문제 1. QR이 카메라에 보이지 않음
**원인**  
초기에는 새 QR plane을 상자 윗면에 붙이는 방식과 box asset에 이미 포함된 QR을 쓰는 방식이 섞여 있었습니다. 또한 asset의 QR 면 방향이 일정하지 않아 카메라가 QR 없는 면을 보는 경우가 있었습니다.

**해결**  
box asset에 이미 포함된 QR을 기준으로 통일하고, `qr_face_terminal_tester` 실험 결과를 바탕으로 `local -Y` 면이 위로 오도록 `force_box_qr_face_up()`을 적용했습니다.

**결과**  
상단 카메라에서 QR 디버그 이미지가 안정적으로 저장되었고, decode 실패 시에도 원인 화면을 확인할 수 있게 되었습니다.

---

### 문제 2. 실제 QR decode 실패 시 시뮬레이션이 멈춤
**원인**  
카메라 위치, 해상도, 조명, QR 크기에 따라 OpenCV QR decode가 실패하면 이후 라우팅 결정이 불가능했습니다.

**해결**  
실제 이미지 decode를 1순위로 사용하되, 실패 시 box prim의 `user:qr_payload` 속성을 fallback으로 사용했습니다.

```text
camera decode 성공 → decoded payload 사용
camera decode 실패 → user:qr_payload fallback
```

**결과**  
실제 QR 인식 검증과 전체 물류 흐름 안정성을 동시에 확보했습니다.

---

### 문제 3. 다음 박스 스폰이 늦어짐
**원인**  
초기 final gate 조건은 박스 중심점이 목적지 영역 내부에 있어야 성공으로 처리했습니다. 실제로는 박스가 목적지 영역에 걸쳐 지나가도 중심점이 조건을 만족하지 못해 다음 스폰이 지연되었습니다.

**해결**  
박스 bbox와 final gate bbox의 overlap 판정으로 변경했습니다.

**결과**  
분류 성공 판정이 실제 컨베이어 동작과 더 가까워졌고, 다음 박스 공급 타이밍이 안정화되었습니다.

---

### 문제 4. robot2가 today 박스에도 반응할 가능성
**원인**  
2대 로봇을 단순 순차 실행하면 today 박스도 robot2 단계로 넘어갈 수 있었습니다.

**해결**  
`today`는 robot1 왼팔 분류 후 최종 목적지 도달만 확인하고, robot2는 건너뛰도록 분기했습니다. `day2/day3`의 경우에만 robot1 오른팔 동작 후 robot2가 동작하도록 구성했습니다.

**결과**  
분류 시나리오가 실제 요구사항과 맞게 정리되었습니다.

---

### 문제 5. ROS 2 외부 서비스 호출 환경 불일치
**원인**  
Isaac Sim 프로세스와 ROS 2 서비스 클라이언트의 환경변수, DDS 설정, workspace setup이 다르면 외부 서비스 호출이 실패할 수 있었습니다.

**해결**  
외부 helper를 별도로 실행하고, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, CycloneDDS URI, setup script 경로를 명시적으로 전달하는 patch를 추가했습니다.

**결과**  
Isaac Sim 내부 로직과 ROS 2 외부 서비스 연동을 분리해 디버깅 가능성을 높였습니다.

---

## 배운 점
- 시뮬레이션에서도 센서 위치, 물체 방향, 충돌체, 판정 조건이 시스템 안정성에 직접적인 영향을 준다.
- QR 인식처럼 외부 조건에 민감한 기능은 실제 decode와 fallback을 분리해 실험 전체가 멈추지 않게 해야 한다.
- 다중 로봇 시스템은 로봇별 역할, 트리거 조건, 예외 분기를 명확히 나눠야 한다.
- 최종 성공 판정은 알고리즘상 편한 기준보다 실제 시스템 동작에 가까운 기준으로 잡아야 한다.

---

## 직무 연결 포인트
이 프로젝트는 Embedded SW QA 관점에서 다음 역량과 연결됩니다.

- 센서 입력값과 제어 동작 결과의 일치 여부 검증
- 단계별 통합 과정에서 오류 원인을 분리하는 디버깅 경험
- fallback, debug image, 로그 출력 등 재현 가능한 검증 구조 설계
- 실제 요구사항을 판정 조건으로 구체화하는 품질 관점의 사고
