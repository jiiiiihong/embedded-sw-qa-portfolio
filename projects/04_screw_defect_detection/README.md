# Screw Defect Detection System

## 프로젝트 개요
로봇팔과 RealSense 카메라를 활용하여 나사 체결 상태를 인식하고, 불량 여부를 판단한 뒤 웹 대시보드에서 검사 결과를 확인하는 시스템을 구현했습니다.

업로드 자료 기준 웹 대시보드 구현 파일은 `joingo_modern_tab_dashboard_v6_ko_vr.html`입니다. 해당 파일 상단 주석 기준, **본인 담당 영역은 Web GUI**이며 WebXR/VR 영역은 팀원 담당으로 분리되어 있습니다.

---

## 사용 기술
- HTML / CSS / JavaScript
- Firebase Realtime Database
- Plotly 3D Visualization
- Three.js / WebXR 협업 구조
- RGB stream URL 표시
- 검사 결과 modal UI
- 예외 상황 panel / modal
- 작업대별 데이터 선택 UI

---

## 시스템 흐름

```text
RealSense / 검사 시스템에서 나사 위치와 상태 데이터 생성
→ Firebase Realtime Database에 작업대별 검사 결과 업로드
→ Web GUI가 live_scan/workstations 구독
→ 작업대 선택 dropdown 갱신
→ 나사 좌표와 상태를 3D Plotly 화면에 표시
→ 불량 클릭 시 상세 modal 표시
→ 조치 완료 후 DB status를 normal로 업데이트
→ RGB stream URL이 있으면 영상 탭에서 실시간 화면 표시
→ 예외 상황 발생 시 예외 panel 및 modal 표시
```

---

## 주요 구현 기능

### 1. Firebase 기반 실시간 검사 결과 구독
`live_scan/workstations` 경로를 구독하여 작업대별 나사 검사 데이터를 실시간으로 표시했습니다.

데이터가 들어오면 다음 흐름으로 화면을 갱신합니다.

- 작업대 key 정렬
- dropdown 갱신
- 현재 작업대 선택 유지
- screw data normalize
- home summary / 3D summary / screw list 갱신
- 3D 화면일 경우 render schedule 실행

### 2. 다양한 status 값 정규화
검사 시스템에서 상태값이 `normal`, `ok`, `pass`, `defect`, `ng`, `error` 등 다양한 표현으로 들어올 수 있기 때문에, UI 내부에서는 `normal`과 `defect`로 정규화했습니다.

```text
normal 계열: normal, ok, good, pass, passed, resolved
불량 계열: defect, defective, failed, fail, ng, abnormal, error
```

이 구조를 통해 백엔드/검사 코드의 표현이 조금 달라도 UI가 안정적으로 표시되도록 했습니다.

### 3. Plotly 기반 3D 검사 결과 시각화
나사 좌표를 3D scatter plot으로 표시하고, 정상은 초록색, 불량은 빨간색으로 구분했습니다.

- 배경 point cloud 표시
- 나사 marker 표시
- 좌표 hover 정보 제공
- marker click 시 상세 modal 표시
- point cloud 개수 limit 설정으로 렌더링 부하 제어

### 4. 불량 상세 modal 및 조치 완료 기능
불량 나사를 클릭하면 나사 ID, 좌표, frame, 원본 JSON 로그를 modal로 표시했습니다. 조치 완료 버튼을 누르면 Firebase DB의 해당 나사 status를 `normal`로 업데이트하도록 구성했습니다.

### 5. RGB 영상 확인 탭
`live_scan/rgb_stream/url` 경로에 MJPEG 또는 영상 스트림 URL이 들어오면 별도 탭에서 실시간 영상을 표시하도록 했습니다.

### 6. 예외 상황 UI
`exceptionstatus/current` 경로를 구독하여 비상정지, 일시정지, 안전정지와 같은 예외 상황을 panel과 modal로 표시했습니다.

---

## 트러블슈팅

### 문제 1. 검사 데이터 구조가 고정되어 있지 않음
**원인**  
검사 시스템에서 `screws`, `markers`, 배열형 데이터 등 서로 다른 형태로 결과가 들어올 수 있었습니다.

**해결**  
`normalizeScrews()`와 `normalizeScrew()`를 두어 배열/객체 입력을 모두 동일한 내부 구조로 변환했습니다.

**결과**  
데이터 구조 차이로 UI가 깨지는 문제를 줄이고, 검사 시스템 변경에 유연하게 대응할 수 있게 되었습니다.

---

### 문제 2. status 표현이 제각각이라 정상/불량 표시가 흔들림
**원인**  
`ok`, `pass`, `normal`, `ng`, `error`처럼 시스템별 표현이 달라 UI에서 상태 판단이 불안정했습니다.

**해결**  
`normalizeStatusValue()`에서 다양한 표현을 `normal` 또는 `defect`로 정규화했습니다.

**결과**  
UI에서는 일관된 정상/불량 기준으로 표시할 수 있게 되었습니다.

---

### 문제 3. Point Cloud 렌더링이 무거움
**원인**  
배경 point cloud 전체를 그대로 표시하면 브라우저 렌더링 부하가 커질 수 있었습니다.

**해결**  
`sampleBackground()`에서 point 개수 제한값에 맞춰 step sampling을 적용했습니다. 사용자는 point limit을 조절하여 품질과 성능을 선택할 수 있습니다.

**결과**  
작업대 배경을 유지하면서도 브라우저 렌더링 부담을 줄였습니다.

---

### 문제 4. DB에 아직 데이터가 없을 때 빈 화면처럼 보임
**원인**  
`live_scan/workstations`에 데이터가 없으면 사용자는 연결 실패인지 데이터 대기 상태인지 구분하기 어려웠습니다.

**해결**  
overlay와 placeholder 메시지를 표시하여 “작업대 데이터 없음” 상태를 명확히 보여주도록 했습니다.

**결과**  
DB 연결 상태와 데이터 부재 상태를 구분할 수 있게 되었습니다.

---

### 문제 5. 공개 저장소 업로드 시 Firebase 설정값 노출 위험
**원인**  
원본 HTML에는 Firebase 연결 설정값이 포함되어 있습니다. 공개 GitHub 저장소에 그대로 업로드하면 불필요한 노출 위험이 있습니다.

**해결**  
포트폴리오 저장소에는 원본 HTML 전체를 그대로 올리지 않고, 구조와 구현 내용 중심으로 정리했습니다. 실제 업로드가 필요할 경우 Firebase 설정은 환경변수 또는 별도 config 파일로 분리하고, 예시값으로 치환해야 합니다.

**결과**  
채용용 포트폴리오에서 구현 역량은 보여주되, 공개 저장소 보안 위험은 줄이는 방향으로 관리했습니다.

---

## 배운 점
- 검사 시스템의 UI는 예쁜 화면보다 데이터 구조 변화와 예외 상황을 견디는 것이 중요하다.
- 실시간 DB 기반 화면은 “데이터 없음”, “연결 실패”, “렌더링 실패”를 구분해서 보여줘야 디버깅이 쉬워진다.
- Point Cloud 같은 무거운 시각화는 sampling과 cache 전략이 필요하다.
- 공개 저장소에 올릴 자료는 동작 코드뿐 아니라 API key, DB URL, 개인 정보 노출 여부까지 검토해야 한다.

---

## 직무 연결 포인트
Embedded SW QA 관점에서 이 프로젝트는 다음 역량과 연결됩니다.

- 검사 결과 데이터를 사용자에게 검증 가능한 형태로 시각화한 경험
- 정상/불량 판정 결과를 DB와 UI에 일관되게 반영한 경험
- 예외 상황, RGB 영상, 3D 좌표 등 다양한 입력을 하나의 대시보드로 통합한 경험
- 공개 포트폴리오 작성 시 보안/민감정보 제거를 고려한 경험
