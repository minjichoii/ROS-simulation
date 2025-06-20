# ROS-simulation

### 🔸2025-05-28
- elevator_door_detector.py 노드 구현 ( 엘리베이터 상태 감지하여 ros topic으로 퍼블리시 )
- 현재 YOLO 모델 성능이 좋진 않음 ( 엘리베이터 유무 정도만 확인이 되고 열림(2) 완벽히 열리지 않음(1)이 구분이 안됨 )


### 🔸2025-06-02
- elevator_door_detector.py 수정 ( YOLO 모델의 문제가 아니라 모델 추출해오는 과정이 문제였던 걸로 추정됨. 그러나 middle 예측이 분명하진 않은 것 같아서 일단 open, close 두 클래스로 학습하여 이용)
- DetectMultiBackend 이용, letterBox로 전처리
- (전) raw YOLO 이용 -> (후) NMS(non_max_suppression)이용

### 🔸2025-06-20
- **시스템 구조 설계 및 코드 호환성 수정**
- **상태**: 🚧 시뮬레이션 연결 전 - 각 컴포넌트별 독립 구현 완료

#### 1. elevator_door_detector.py (YOLO 비전 시스템)
- **역할**: YOLOv9 기반 엘리베이터 문 상태 실시간 인식
- **Subscribe**: `/camera/image_raw` (카메라 이미지)
- **Publish**: `/elevator_vision/door_status` (String: "open" or "close")
- **핵심 기능**: DetectMultiBackend, NMS, confidence 기반 최적 detection 선택

#### 2. gazebo_elevator_controller.cpp (메인 제어기)
- **역할**: 상태 머신 기반 엘리베이터 자동화 로직
- **상태**: 9단계 상태 머신 설계 (IDLE_AT_1F → DOOR_OPENING_1F → ... → RETURNING_TO_1F)
- **Subscribe**: 
 - `/elevator/robot_inside`, `/elevator/current_floor`
 - `/elevator/door_status`, `/elevator_vision/door_status`
- **Publish**: `/elevator/door_control`, `/elevator/move_command`
- **핵심 기능**: 자동 문 제어 로직, 로봇 탑승/하차 감지, 미션 통계, 에러 복구

#### 3. elevator_gazebo_actuator.cpp (물리 제어기)
- **역할**: Gazebo 시뮬레이션 환경 제어 준비
- **설계**: Joint Controller(우선) + Model State(백업) 이중 제어
- **Subscribe**: `/elevator/door_control`, `/elevator/move_command`, `/joint_states`, `/gazebo/model_states`
- **Publish**: 
 - Joint 제어: `/elevator/door_joint_position_controller/command`, `/elevator/lift_position_controller/command`
 - 피드백: `/elevator/robot_inside`, `/elevator/current_floor`, `/elevator/door_status`
- **핵심 기능**: 로봇 위치 추적 로직, 정밀한 층간 이동(3.075m), World 파일 호환성

#### 4. robot_elevator_controller.cpp (로봇 제어기)
- **역할**: 로봇의 엘리베이터 탑승/하차 자동 제어
- **조건 기반 행동 로직**:
 - 진입: `1층 + 밖 + 문열림 + 정지상태`
 - 하차: `2층 + 안 + 문열림 + 정지상태`
- **Subscribe**: `/elevator_vision/door_status`, `/elevator/current_floor`, `/elevator/robot_inside`, `/elevator/door_status`
- **Publish**: `/cmd_vel` (로봇 이동 명령)
- **핵심 기능**: 중복 행동 방지(2초 쿨다운), 안전 속도 제어, 타이밍 기반 자동 정지

## 🎯 현재 상태 (2025-06-20)

### ✅ 완료된 작업
- **4개 주요 컴포넌트 구현 완료**
- **ROS 토픽 인터페이스 설계 및 호환성 확보**
- **이중 제어 시스템 설계 (Joint/Model State)**

### 🚧 진행 중인 작업
- **Gazebo 시뮬레이션 환경 연결**
- **World 파일과 컴포넌트 간 호환성 테스트**
- **전체 시스템 통합 테스트**

### 📋 다음 단계
1. Gazebo World 파일과 액추에이터 연결
2. YOLO 카메라와 시뮬레이션 환경 동기화
3. 전체 파이프라인 테스트 및 디버깅
4. 성능 최적화 및 안정성 검증
