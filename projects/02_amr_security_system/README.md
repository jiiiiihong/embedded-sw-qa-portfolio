# AMR Security System

## 프로젝트 개요
박물관 환경을 가정하여 AMR이 순찰을 수행하고, 도난 상황이 발생하면 도둑 좌표를 수신해 추적 모드로 전환하는 시스템을 구현했습니다.

업로드 자료 기준 핵심 구현 파일은 `real_final.py`이며, ROS 2 / TurtleBot4 / Nav2 기반으로 순찰, 교대 순찰, 도착 신호 발행, 추적 goal 갱신을 통합했습니다.

---

## 사용 기술
- Python
- ROS 2 `rclpy`
- TurtleBot4 Navigator
- Nav2 `NavigateToPose` Action
- AMCL pose
- ROS Topic pub/sub
- MultiThreadedExecutor
- ReentrantCallbackGroup
- RGB / Depth / LiDAR 기반 탐지 시스템과 연동

---

## 시스템 구조

```text
순찰 시작
→ AMCL initial pose 발행
→ 지정 waypoint 순찰
→ 특정 위치 도착 시 arrive_position 발행
→ 순찰 완료 후 동료 AMR에 start_patrol_signal 반복 발행
→ 본 로봇 dock

도난 감지 시
→ thief_position 수신
→ 현재 순찰 goal cancel
→ chase_mode 전환
→ AMCL 현재 위치와 thief_position 비교
→ 도둑 30cm 앞 지점으로 Nav2 goal 지속 갱신
→ situation_end 수신 시 추적 종료
```

---

## 주요 구현 기능

### 1. 교대 순찰 흐름
순찰 완료 후 다른 AMR이 이어서 순찰하도록 `start_patrol_signal`을 발행했습니다. 한 번만 보내면 메시지 유실 가능성이 있어 20회 반복 발행하도록 구성했습니다.

```text
robot8 순찰 완료
→ /robot2/start_patrol_signal True 반복 발행
→ robot8 dock
→ robot2 순찰 시작
```

### 2. 도착 위치 이벤트 발행
지정된 waypoint에 도착하면 `/robot8/arrive_position` 토픽으로 위치 ID를 발행했습니다.

```text
(2.05, 2.04)  → data=1  # pot
(-2.25, 0.84) → data=2  # ball
```

이 구조를 통해 AMR 순찰 결과를 외부 도난 감지/상태 시스템과 연결할 수 있게 했습니다.

### 3. 순찰 중 추적 요청 처리
순찰 도중 도둑 좌표가 들어오면 기존 Nav2 goal을 취소하고 추적 모드로 넘어가도록 했습니다.

- `chase_requested`: 추적 요청 감지 플래그
- `chase_mode`: 실제 추적 모드 상태
- `chase_transitioning`: 중복 전환 방지 플래그

### 4. 도둑 위치 기반 goal 갱신
`KeepChaseActionNode`에서 AMCL 현재 위치와 도둑 좌표를 비교하여 도둑 바로 위가 아니라 **도둑 30cm 앞 지점**을 goal로 생성했습니다.

```text
robot position = (rx, ry)
thief position = (tx, ty)
goal = thief_position - unit_vector(robot→thief) * 0.30m
```

이를 통해 로봇이 목표물에 과도하게 충돌하지 않고, 일정 거리에서 추적하도록 했습니다.

### 5. goal update throttle
도둑 위치가 거의 변하지 않았을 때도 goal을 계속 보내면 Nav2 action이 불필요하게 갱신될 수 있습니다. 따라서 직전 goal과 새 goal의 이동량이 `0.05m` 미만이면 갱신하지 않도록 했습니다.

---

## 트러블슈팅

### 문제 1. 초기 위치 설정이 안정적으로 들어가지 않음
**원인**  
AMCL이 활성화되기 전에 initial pose를 발행하거나, subscriber가 준비되지 않은 상태에서 1회 발행하면 초기 위치가 누락될 수 있었습니다.

**해결**  
`TRANSIENT_LOCAL + RELIABLE` QoS를 사용하고, AMCL 활성화 및 subscriber count를 확인한 뒤 initial pose를 발행했습니다.

**결과**  
초기 위치 입력 실패 가능성을 줄이고 순찰 시작 전 localization 안정성을 확보했습니다.

---

### 문제 2. 순찰 중 도난 상황 발생 시 기존 goal이 남음
**원인**  
순찰 waypoint 이동 중 thief_position이 들어오면 기존 NavigateToPose goal이 계속 진행되어 추적 전환이 늦어질 수 있었습니다.

**해결**  
`wait_nav_complete()` 내부에서 `chase_requested`를 감시하고, 추적 요청이 감지되면 `cancelTask()`로 현재 순찰 goal을 취소했습니다.

**결과**  
순찰 루프와 추적 모드가 충돌하지 않고, 도난 상황을 우선 처리할 수 있게 되었습니다.

---

### 문제 3. 추적 모드 중 goal이 너무 자주 갱신됨
**원인**  
도둑 좌표가 거의 동일해도 주기적으로 action goal을 다시 보내면 Nav2가 불안정해지거나 불필요한 명령이 누적될 수 있었습니다.

**해결**  
직전 goal과 새 goal 간 거리 차이가 `MIN_GOAL_SHIFT_M = 0.05m` 미만이면 goal을 보내지 않도록 했습니다.

**결과**  
추적 동작의 흔들림을 줄이고 action server 부하를 낮췄습니다.

---

### 문제 4. 동료 로봇 순찰 시작 신호 유실 가능성
**원인**  
토픽 메시지를 1회만 발행하면 상대 노드가 아직 준비되지 않았거나 통신 타이밍이 어긋났을 때 신호를 놓칠 수 있었습니다.

**해결**  
순찰 완료 후 `start_patrol_signal=True`를 20회 반복 발행했습니다.

**결과**  
교대 순찰 시작 신뢰성을 높였습니다.

---

### 문제 5. 여러 callback과 순찰 루프가 동시에 동작
**원인**  
도둑 위치 수신, AMCL pose 갱신, 순찰 루프, Nav2 action goal 갱신이 동시에 발생하기 때문에 단일 스레드 executor에서는 응답성이 떨어질 수 있었습니다.

**해결**  
`MultiThreadedExecutor`와 별도 node 구조를 사용해 순찰, 신호 수신, 추적 goal 갱신을 분리했습니다.

**결과**  
상태 이벤트와 navigation action을 동시에 처리할 수 있는 구조로 개선했습니다.

---

## 배운 점
- 로봇 시스템은 이동 알고리즘만으로 완성되지 않고, 상태 플래그, 토픽 신뢰성, action cancel 조건이 함께 설계되어야 한다.
- 실제 AMR에서는 정상 순찰보다 예외 상황 전환 로직이 더 중요할 수 있다.
- 다중 로봇 협업은 한 번의 명령보다 메시지 유실, 준비 타이밍, 중복 실행 방지까지 고려해야 안정화된다.

---

## 직무 연결 포인트
Embedded SW QA 관점에서 이 프로젝트는 다음 역량과 연결됩니다.

- 센서 입력 및 외부 이벤트에 따른 상태 전환 검증
- 비동기 메시지 기반 시스템의 race condition 대응
- 정상 순찰과 예외 추적 시나리오 분리
- 반복 발행, goal throttle, cancel 조건 등 안정성 보완 경험
