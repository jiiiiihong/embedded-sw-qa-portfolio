# Braille Robot Validation

## 프로젝트 개요
로봇팔을 활용하여 맞춤형 점자를 제작하고, 제작된 점자를 웹캠과 OpenCV 기반 영상 처리로 검증하는 프로젝트를 수행했습니다.

업로드 자료 기준 핵심 구현 파일은 `dot_validation_final_generalized_cells_v3_slopefix.py`이며, 2,380라인 규모의 Python 코드로 웹캠 캡처, ROI 선택, 점 검출, 줄 분리, 점자 칸 분리, 한글 점자 해석, 목표 라벨과의 점형 비교를 통합했습니다.

---

## 사용 기술
- Python
- OpenCV
- NumPy
- Webcam Capture
- ROI Selection
- Image Thresholding
- Braille Pattern Recognition
- Hangul Braille Decomposition
- Debug Visualization / JSON Result Export

---

## 시스템 흐름

```text
웹캠 영상 캡처
→ 사용자가 점자 영역 ROI 선택
→ ROI 전처리 및 점 후보 검출
→ y축 분포 기반 줄 분리
→ x좌표 기반 세로 column 생성
→ column gap으로 점자 칸 분리
→ 각 칸의 6점 점형 mask 생성
→ 한글 점자 후보 해석
→ 목표 라벨을 점형으로 변환
→ 실제 인식 점형과 비교하여 일치/불일치 판정
```

---

## 주요 구현 기능

### 1. 웹캠 캡처 및 가이드라인
촬영 시 점자 줄이 기울어지거나 ROI가 흔들리면 검출 정확도가 낮아집니다. 이를 줄이기 위해 캡처 화면에 가이드라인을 표시했습니다.

- `SPACE`: 현재 프레임 캡처
- `ESC`: 종료
- `--capture-guide-lines`: 예상 줄 수에 맞춘 가이드라인 표시
- `--guide-width-ratio`, `--guide-height-ratio`: 촬영 가이드 박스 크기 조정

### 2. ROI 기반 점 검출
사용자가 마우스로 점자 영역을 선택하면 해당 ROI에 대해서만 검증을 수행했습니다. 전체 화면이 아니라 ROI를 기준으로 처리하여 배경 노이즈를 줄였습니다.

전처리 단계는 다음을 포함합니다.

- grayscale 변환
- median blur
- local peak 강조
- percentile / std 기반 threshold
- morphology close 선택 적용
- 작은 dot 후보 병합

### 3. 줄 단위 분석
여러 줄 점자를 처리하기 위해 전체 ROI에서 먼저 점 후보를 찾고, 점들의 y분포를 기준으로 줄을 분리했습니다.

중요한 개선점은 줄별로 다시 threshold를 계산하지 않고, 전체 ROI에서 만든 `base_binary`를 줄별로 crop해서 사용하는 방식입니다.

```text
문제: 줄별 threshold를 다시 계산하면 약한 점이 사라짐
해결: 전체 ROI 기준 base_binary를 만든 뒤 line box로 crop
```

이 방식으로 줄 분리와 줄별 분석에 쓰이는 점 후보의 일관성을 유지했습니다.

### 4. column gap 기반 점자 칸 분리
처음부터 글자를 해석하지 않고, 점들을 먼저 x좌표 기준 column으로 묶었습니다.

- 가까운 x좌표끼리 같은 세로 column으로 병합
- column 간격이 좁으면 같은 점자 칸
- column 간격이 넓으면 다른 점자 칸
- 각 칸의 왼쪽/오른쪽 column과 row 위치를 기반으로 6점 mask 생성

이를 통해 글자 해석 전에 실제 제작된 점형 자체를 추출할 수 있도록 했습니다.

### 5. 의미 해석보다 점형 판정 우선
한글 점자는 동일한 점형에서 여러 해석 후보가 나올 수 있고, 약자/약어 처리에 따라 문자열 해석이 달라질 수 있습니다.

따라서 최종 판정은 문자열 해석 결과가 아니라 **목표 라벨을 점형으로 변환한 결과와 영상에서 인식한 점형이 일치하는지**를 기준으로 했습니다.

```text
의미 해석: 참고용
최종 판정: expected_cells == actual_cells
```

### 6. 디버그 정보 저장
검출 결과를 사람이 확인할 수 있도록 이미지와 JSON 로그를 저장하는 기능을 구성했습니다.

- 원본 frame
- ROI image
- line boxes
- base enhanced image
- base binary image
- line별 binary/result image
- result JSON
- expected_cells / actual_cells / matched_cells

---

## 트러블슈팅

### 문제 1. 점 하나가 여러 조각으로 쪼개짐
**원인**  
threshold가 너무 높거나 local peak 강조가 강하면 하나의 점이 여러 개의 작은 contour로 분리될 수 있었습니다.

**해결**  
`merge_scale`, `min_merge_dist`를 두어 가까운 점 후보를 병합했습니다. 또한 median filter와 morphology close를 조절할 수 있도록 argument로 노출했습니다.

**결과**  
촬영 조건이 달라져도 점 후보를 병합/분리하며 검출 민감도를 조정할 수 있게 되었습니다.

---

### 문제 2. 붙어 있는 6점이 하나의 큰 blob으로 잡힘
**원인**  
점자 표면의 조명 반사나 emboss 영역이 넓게 밝아지면 여러 점이 하나의 blob으로 붙어 검출되었습니다.

**해결**  
넓게 밝은 영역을 그대로 쓰지 않고, `peak_sigma`, `peak_percentile`, `peak_kernel`을 이용해 작은 local peak 중심으로 점 후보를 남기도록 했습니다.

**결과**  
점자 칸 내부의 개별 점을 더 안정적으로 분리할 수 있었습니다.

---

### 문제 3. 여러 줄 점자에서 첫 줄의 약한 점이 사라짐
**원인**  
줄별 ROI마다 threshold를 다시 계산하면, 상대적으로 약한 점이 있는 줄에서는 기준값이 달라져 점 후보가 사라질 수 있었습니다.

**해결**  
전체 ROI에서 만든 `base_binary`를 line box로 crop하여 각 줄 분석에 사용했습니다.

**결과**  
줄 분리와 줄별 점 검출 기준을 일치시켜 약한 점이 누락되는 문제를 줄였습니다.

---

### 문제 4. 기울기 보정이 오히려 오판을 만듦
**원인**  
촬영 가이드라인을 맞춘 상태에서는 큰 row slope 보정이 실제 점 위치를 왜곡해 행 번호 판단을 잘못하게 만들 수 있었습니다.

**해결**  
`row_slope_max` 기본값을 낮게 두고, 큰 기울기 보정보다 촬영 단계에서 가이드라인을 맞추는 방향으로 조정했습니다.

**결과**  
행 모델이 과하게 기울어져 점형 판단을 망치는 문제를 줄였습니다.

---

### 문제 5. 문자열 해석 후보는 맞는데 실제 점형은 다름
**원인**  
한글 점자는 약자, 받침, 조합 규칙 때문에 문자열 후보가 여러 개 나올 수 있습니다. 문자열만 기준으로 하면 실제 제작된 점형 오류를 놓칠 수 있었습니다.

**해결**  
최종 판정을 문자열 비교가 아니라 cell code sequence 비교로 변경했습니다.

**결과**  
“해석이 그럴듯한가”가 아니라 “로봇이 목표 점형을 실제로 찍었는가”를 검증할 수 있게 되었습니다.

---

## 배운 점
- 검증 시스템에서는 결과 해석보다 판정 기준을 먼저 명확히 해야 한다.
- 실제 영상 기반 검증은 조명, 촬영 각도, 점 간격, 표면 반사에 민감하므로 파라미터를 조정 가능하게 설계해야 한다.
- AI나 복잡한 모델 없이도 문제 구조를 잘 정의하면 rule-based vision pipeline으로 충분히 검증 가능한 영역이 있다.
- 디버그 이미지와 JSON 로그는 검증 결과의 신뢰도를 높이는 핵심 자료가 된다.

---

## 직무 연결 포인트
Embedded SW QA 관점에서 이 프로젝트는 다음 역량과 연결됩니다.

- 출력 결과를 실제 센서 입력으로 검증하는 closed-loop 사고
- 판정 기준을 문자열이 아닌 점형 단위로 재정의한 검증 설계 경험
- threshold, line split, slope correction 등 오류 원인을 파라미터화한 디버깅 경험
- pass/fail 결과뿐 아니라 근거 이미지와 JSON 로그를 남기는 품질 검증 구조 경험
