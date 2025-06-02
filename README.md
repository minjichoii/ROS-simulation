# ROS-simulation

### 🔸2025-05-28
- elevator_door_detector.py 노드 구현 ( 엘리베이터 상태 감지하여 ros topic으로 퍼블리시 )
- 현재 YOLO 모델 성능이 좋진 않음 ( 엘리베이터 유무 정도만 확인이 되고 열림(2) 완벽히 열리지 않음(1)이 구분이 안됨 )


### 🔸2025-06-02
- elevator_door_detector.py 수정 ( YOLO 모델의 문제가 아니라 모델 추출해오는 과정이 문제였던 걸로 추정됨. 그러나 middle 예측이 분명하진 않은 것 같아서 일단 open, close 두 클래스로 학습하여 이용)
- DetectMultiBackend 이용, letterBox로 전처리
- (전) raw YOLO 이용 -> (후) NMS(non_max_suppression)이용
