# 국립순천대학교 딸기 레이저 커팅 수확 기술 개발

## 📌 Project Overview

- **프로젝트명**: 국립순천대학교 딸기 레이저 커팅 수확 기술 개발
- **개발 기간**: 2026.03.15 ~ 2026.04.13
- **Robot**: ROBO003 6-DOF Manipulator
- **Robot Controller**: 다인큐브 제어기
- **Camera**: Intel RealSense D435
- **Framework**: ROS1
- **Vision**: YOLO26 Pose
- **Robot Model / Visualization**: URDF, SRDF, RViz
- **Communication**: ROS Topic, ROS Service, TCP/IP
- **Harvesting Method**: Laser Stem Cutting

기존 딸기 수확 시스템은 딸기 몸체를 객체 검출한 뒤 그리퍼로 파지하여 수확하는 방식이었습니다.

본 프로젝트에서는 수확 방식을 변경하여 **YOLO26 Pose 모델을 이용해 딸기 줄기의 절단 지점을 Keypoint로 검출하고, 해당 위치를 기준으로 ROBO003 매니퓰레이터가 접근한 뒤 레이저를 조사하여 줄기를 절단하는 방식**으로 시스템을 고도화했습니다.

줄기는 매우 얇아 Depth 값을 직접 획득할 경우 값이 불안정해질 수 있기 때문에, 딸기 몸체 위치에서 안정적인 Depth 값을 취득하고 해당 Depth를 줄기 Keypoint에 적용하여 3D 줄기 위치를 계산하는 방식을 적용했습니다.

또한 사용한 PC의 연산 성능이 높지 않은 환경에서도 시스템이 안정적으로 동작하도록, 비전 인식과 로봇 동작을 동시에 계속 수행하지 않고 **Flag 기반 순차 제어 구조**를 적용했습니다.

딸기 줄기 위치가 확정되면 비전 인식을 일시 중지하고, 로봇이 접근 및 레이저 커팅 동작을 모두 완료한 뒤 완료 신호를 보내면 다시 다음 대상을 인식하도록 구성했습니다.

---

## 🎥 Demo

### 1



https://github.com/user-attachments/assets/8b2609c5-006b-4ebd-8c01-e567263b237f



### 2



https://github.com/user-attachments/assets/2961282e-e345-4636-ac29-4e6e07d97cf6



### End-to-End Harvesting

줄기 인식부터 3D 위치 추정, 좌표변환, TCP 통신, 로봇 접근, 레이저 커팅, 완료 신호 및 다음 대상 재인식까지 전체 자동 수확 과정입니다.



https://github.com/user-attachments/assets/66f1ffc7-2702-4ff6-af51-53b5ec586749



---

## 👁️ YOLO26 Pose 기반 딸기 줄기 인식

기존 시스템에서는 딸기 몸체의 Bounding Box 중심을 이용하여 수확 위치를 결정했습니다.

본 프로젝트에서는 줄기 자체를 절단해야 하기 때문에 단순 Object Detection 방식이 아닌 **Pose Estimation 방식**을 적용했습니다.

학습된 YOLO26 Pose 모델에서 딸기 몸체와 줄기 위치를 각각 Keypoint로 검출하도록 구성했습니다.

```python
bx, by = map(int, obj[1])  # Strawberry Body
sx, sy = map(int, obj[3])  # Stem
```

- `Body Keypoint`: Depth 획득용
- `Stem Keypoint`: 실제 레이저 커팅 목표 위치 계산용

### 기존 방식

```text
Strawberry Detection
        ↓
Bounding Box
        ↓
Center Point
        ↓
3D Position
        ↓
Gripper Harvesting
```

### 개선 방식

```text
Strawberry Detection
        ↓
YOLO26 Pose Estimation
        ↓
Stem Keypoint
        ↓
3D Stem Position
        ↓
Manipulator Approach
        ↓
Laser Cutting
```

---

## 📐 Body Depth 기반 Stem 3D Position 계산

딸기 줄기는 매우 얇기 때문에 줄기 픽셀 위치에서 직접 Depth 값을 읽을 경우 Depth Noise 또는 Invalid Value가 발생할 수 있습니다.

이를 해결하기 위해 **Depth는 딸기 몸체 Keypoint 위치에서 획득하고, 줄기 Keypoint의 Pixel Coordinate와 결합하여 3D 줄기 좌표를 계산**했습니다.

### Depth Processing

딸기 몸체 주변의 일정 영역에서 유효 Depth 값을 수집한 뒤 다음 처리를 적용했습니다.

- 주변 Pixel Depth Sampling
- 0값 및 Invalid Depth 제거
- 20% ~ 80% Percentile 기반 Outlier 제거
- Median Depth 계산
- mm → m 단위 변환

```text
Body Keypoint
      ↓
Nearby Depth Sampling
      ↓
Invalid Value Removal
      ↓
20 ~ 80 Percentile Filtering
      ↓
Median Depth
```

### Temporal Smoothing

프레임별 Depth 값의 급격한 변화를 줄이기 위해 이전 Depth 값을 이용한 Temporal Smoothing도 적용했습니다.

```text
depth_smooth =
0.7 × previous_depth
+
0.3 × current_depth
```

이를 통해 Depth 값이 순간적으로 튀는 현상을 줄이고 보다 안정적인 3D Target Position을 생성했습니다.

---

## 📐 Stem 3D Projection

몸체에서 얻은 Depth 값을 이용하고, 실제 X/Y 위치 계산에는 줄기 Keypoint를 사용했습니다.

```text
Stem Pixel (sx, sy)
        +
Body Depth
        ↓
Camera Intrinsic
        ↓
3D Stem Position
```

투영 방식은 다음과 같습니다.

```text
X = (sx - cx) × Depth / fx
Y = (sy - cy) × Depth / fy
Z = Depth
```

이를 통해 실제 줄기 픽셀을 기준으로 Camera Coordinate Frame상의 3D Target Position을 생성했습니다.

---

## 🔄 ROS Vision Pipeline

Vision Node에서는 계산된 3D 줄기 위치를 `PoseStamped` 형태로 발행했습니다.

```text
YOLO26 Pose
      ↓
Body / Stem Keypoint
      ↓
Body Depth
      ↓
Stem 3D Position
      ↓
/target_strawberry_pose
```

사용 Topic:

```text
/target_strawberry_pose
```

Frame:

```text
camera_link1
```

---

## 🔁 Camera → Robot Coordinate Transformation

Vision Node에서 생성된 줄기 3D 좌표는 카메라 기준 좌표이므로 실제 ROBO003가 사용할 수 있도록 Robot Base Frame으로 변환했습니다.

```text
Camera Coordinate
        ↓
ROS TF
camera_link1 → base
        ↓
Robot Base Coordinate
```

ROS TF를 이용하여 카메라 기준 위치를 실제 로봇 작업 좌표계로 변환했습니다.

이를 통해 카메라에서 검출한 줄기 위치를 실제 로봇이 사용할 수 있는 Target Position으로 전달했습니다.

---

## 🦾 Robot Control Architecture

본 프로젝트에서는 비전 시스템에서 계산한 딸기 줄기 위치를 ROS TF를 통해 실제 로봇 기준 좌표계로 변환한 뒤, 해당 좌표를 ROS Service와 TCP/IP 통신을 통해 다인큐브 제어기로 전달했습니다.

URDF/SRDF와 RViz는 ROBO003 로봇 모델의 자세와 좌표계를 확인하고 검증하는 용도로 활용했습니다.

실제 ROBO003의 움직임은 **PC와 다인큐브 제어기 간 TCP/IP 통신**을 통해 수행했습니다.

### 실제 Robot Control Flow

```text
YOLO26 Pose
      ↓
Stem Keypoint
      ↓
3D Stem Position
      ↓
ROS TF
Camera → Robot
      ↓
ROS Service Call
      ↓
PC TCP Client
      ↓
다인큐브 제어기 TCP Server
      ↓
ROBO003 Actual Motion
```

PC는 TCP Client로 동작하고,  
다인큐브 제어기 측 프로그램이 TCP Server 역할을 수행하도록 구성했습니다.

---

## 🔗 ROS Service 기반 Robot Command

좌표변환이 완료된 Target Position은 ROS Service를 통해 로봇 제어 Node로 전달했습니다.

### 주요 기능

- 변환된 Target Position 전달
- Robot Motion Request
- TCP/IP Command 생성
- 다인큐브 제어기로 좌표 전송
- 로봇 동작 완료 응답 수신
- 다음 Vision Sequence 제어

이를 통해 Vision Node와 실제 ROBO003 제어 시스템을 ROS Service와 TCP/IP 통신으로 연결했습니다.

---

## 🌐 TCP Client / Server Communication

실제 ROBO003 제어는 TCP/IP 기반 Client-Server 구조를 사용했습니다.

### PC

```text
TCP Client
```

### 다인큐브 제어기

```text
TCP Server
```

전체 흐름:

```text
ROS Service
    ↓
PC TCP Client
    ↓
Target Coordinate
    ↓
다인큐브 제어기 TCP Server
    ↓
ROBO003 Motion
    ↓
Motion Complete Response
```

ROBO003가 작업을 완료하면 다인큐브 제어기 측 TCP Server에서 완료 응답을 반환하고, 해당 신호를 기준으로 다음 Vision Cycle을 수행하도록 구성했습니다.

---

## 🔥 Laser Cutting System

기존 프로젝트에서는 그리퍼를 이용하여 딸기를 직접 파지하는 방식으로 수확했습니다.

본 프로젝트에서는 그리퍼 대신 **레이저를 이용하여 딸기 줄기를 절단하는 방식**으로 변경했습니다.

매니퓰레이터는 딸기 몸체를 직접 파지하지 않고 비전에서 검출한 줄기 Keypoint 위치를 기준으로 접근하며, 레이저가 줄기에 조사될 수 있도록 위치를 정렬한 뒤 커팅을 수행합니다.

### Laser Cutting Sequence

```text
Stem Detection
      ↓
Stem Position Calculation
      ↓
Robot Approach
      ↓
Laser Alignment
      ↓
Laser ON
      ↓
Stem Cutting
      ↓
Laser OFF
      ↓
Robot Return
```

### 주요 구현 내용

- 줄기 Keypoint 기반 커팅 위치 결정
- ROBO003 목표 위치 접근
- 레이저 조사 위치 정렬
- Laser ON / OFF 제어
- 줄기 절단 수행
- 작업 완료 후 안전 위치 복귀

---

## 🔄 Flag 기반 Vision - Robot 동기화

사용한 PC의 연산 성능이 높지 않은 환경에서도 안정적으로 동작할 수 있도록 비전 추론을 항상 실행하지 않고 **필요한 시점에만 수행하는 Flag 기반 상태 제어 방식**을 적용했습니다.

딸기 줄기 Target이 결정되면 Vision 동작을 중지하고, 이후 로봇이 좌표 이동 및 레이저 커팅을 모두 완료한 뒤 완료 신호를 전달하면 다시 Vision을 활성화합니다.

### 동작 과정

```text
Vision Enable
      ↓
YOLO26 Pose Inference
      ↓
Valid Stem Target Found
      ↓
Vision Disable
      ↓
3D Coordinate Processing
      ↓
TF Transformation
      ↓
ROS Service Call
      ↓
TCP Client
      ↓
다인큐브 제어기
      ↓
ROBO003 Motion
      ↓
Laser Cutting
      ↓
Robot Motion Complete
      ↓
Completion Signal
      ↓
Vision Enable
      ↓
Next Target Detection
```

### 적용 효과

- 로봇 동작 중 불필요한 YOLO 추론 중지
- CPU / GPU 연산 부하 감소
- 같은 Target의 반복 검출 방지
- 중복 Robot Command 방지
- Vision과 Robot Motion 간 명확한 동기화
- 저사양 PC에서도 안정적인 반복 작업 가능

---

## 🔄 End-to-End Harvesting Sequence

전체 자동 수확 시스템은 다음 순서로 동작하도록 구성했습니다.

```text
Initial Pose
    ↓
Vision Start
    ↓
YOLO26 Pose Detection
    ↓
Body / Stem Keypoint Extraction
    ↓
Body Depth Measurement
    ↓
Stem 3D Position Calculation
    ↓
Vision Stop
    ↓
Camera → Robot TF
    ↓
ROS Service Call
    ↓
PC TCP Client
    ↓
다인큐브 제어기 TCP Server
    ↓
ROBO003 Approach
    ↓
Laser Alignment
    ↓
Laser Stem Cutting
    ↓
Robot Return
    ↓
Motion Complete Signal
    ↓
Vision Restart
    ↓
Next Strawberry
```

---

## 🛠 Tech Stack

| Category | Technology |
| --- | --- |
| Programming | Python |
| Framework | ROS1 |
| Vision | YOLO26 Pose, OpenCV |
| Camera | Intel RealSense D435 |
| Depth Processing | Aligned Depth Image, Median Filtering, Temporal Smoothing |
| Coordinate | Camera Intrinsic, ROS TF |
| Robot | ROBO003 6-DOF Manipulator |
| Robot Controller | 다인큐브 제어기 |
| Robot Model | URDF, SRDF |
| Visualization | RViz |
| Communication | ROS Topic, ROS Service, TCP/IP |
| Harvesting | Laser Stem Cutting |

---

## ✅ 기존 시스템 대비 개선점

| 기존 딸기 수확 시스템 | 레이저 커팅 수확 시스템 |
| --- | --- |
| YOLO Object Detection | YOLO26 Pose Estimation |
| 딸기 몸체 Bounding Box 중심 사용 | 줄기 Keypoint 사용 |
| Body Position 기준 접근 | Stem Position 기준 접근 |
| Gripper 기반 파지 수확 | Laser 기반 줄기 절단 |
| 줄기 위치 직접 사용하지 않음 | 줄기 절단 위치 직접 추정 |
| 일반적인 Depth 기반 위치 추정 | Body Depth + Stem Pixel 방식 |
| 연속 Vision Inference | Flag 기반 필요 시점 Vision Inference |
| Robot 동작 중에도 Vision 수행 가능 | Robot 동작 중 Vision 중지 |
| 상대적으로 높은 연산 부하 | 저사양 PC에서도 안정적 동작 |

---

## ✅ Development Results

- YOLO26 Pose 기반 딸기 줄기 Keypoint 검출
- Strawberry Body / Stem Keypoint 분리
- Body Depth 기반 Stem 3D Position 계산
- Depth Outlier Filtering 적용
- Median Depth 적용
- Temporal Depth Smoothing 적용
- Camera → Robot TF Coordinate Transformation 구현
- `/target_strawberry_pose` 기반 Target Pose 전달
- ROS Service 기반 Robot Motion Request 구현
- PC TCP Client / 다인큐브 제어기 TCP Server 통신 구현
- 다인큐브 제어기를 통한 실제 ROBO003 동작 연동
- 줄기 위치 기반 Laser Cutting 구현
- Robot Completion Signal 기반 Vision 재시작
- Flag 기반 Vision / Robot State Synchronization 구현
- 저사양 PC 환경에서 반복 자동 수확 동작 구현

---

## 📂 Source Code

현재 Repository는 국립순천대학교 딸기 레이저 커팅 수확 기술 개발 과정과 동작 결과를 정리하기 위한 포트폴리오 Repository입니다.

소스코드는 협력기관 및 프로젝트 공개 가능 범위를 검토한 후 공개 가능한 부분에 한하여 순차적으로 업데이트할 예정입니다.
