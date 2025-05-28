#!/usr/bin/env python3

# 학습된 모델로 엘리베이터 문 상태(close, middle, open) 인식
# ROS 노드(elevator_door_detector.py)에서 해당 상태 토픽으로 발행
# elevator_controller_sm.py에서 토픽 구독해 동작 제어
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import yaml
import os

CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')

with open(CONFIG_PATH, 'r') as file:
    config = yaml.safe_load(file)

# 개별 변수로 꺼내 쓰기
model_path = config.get('model_path')
cfg_path = config.get('cfg_path')
image_topic = config.get('image_topic')
status_topic = config.get('status_topic')

from models.yolo import Model
import torch

class ElevatorDoorDetector:
    def __init__(self):
        rospy.init_node('elevator_door_detector')

        # YOLOv9 model load
        
        rospy.loginfo(f"[YOLO] Loading model from: {model_path}")
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.model = Model(cfg_path).to(self.device)

        checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)

        if 'model' in checkpoint:
            # checkpoint['model']이 state_dict인 경우
            if isinstance(checkpoint['model'"/camera/color/image_raw"], dict):
                self.model.load_state_dict(checkpoint['model'])
            else:
                # checkpoint['model']이 nn.Module인 경우 (rare)
                self.model = checkpoint['model']
        else:
            # 만약 checkpoint가 state_dict일 경우
            self.model.load_state_dict(checkpoint)

        self.model.eval()

        # set ROS
        self.bridge = CvBridge() # ROS image -> OpenCV image 브릿지 생성
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, Int32, queue_size=10) # 엘리베이터 문 상태 퍼블리시

        rospy.loginfo("✅ ElevatorDoorDetector initialized and ready.")

    # 이미지 들어올 때마다 자동 호출되는 콜백 함수
    # ROS image -> OpenCV image (bgr8)
    def image_callback(self, msg): 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[Bridge] Error converting image: {e}")
            return
        
        # OpenCV image -> torch tensor 전처리
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))  # 모델 입력 크기에 맞게 조정 필요
        img = img.astype('float32') / 255.0  # 정규화
        img = torch.from_numpy(img).permute(2,0,1).unsqueeze(0).to(self.device)  # (1,3,640,640)
        img = img.half() # 입력이미지 float32 -> float16

        with torch.no_grad():
            outputs = self.model(img)[0]  # 모델 출력 (tensor)

        door_status = self.extract_door_status(outputs)

        if door_status is not None:
            self.status_pub.publish(Int32(door_status))
            rospy.loginfo(f"[YOLO] Detected elevator door status: {door_status}")
        else:
            rospy.loginfo("[YOLO] No elevator door detected")
    
    def extract_door_status(self, result):
        # YOLO 추론 결과에서 가장 많이 등장한 class를 대표 상태로 반환함
        # 0: close, 1: middle, 2: open

        """
        가장 많이 등장한 클래스를 선택하는 이유
        : YOLO는 여러개의 박스를 동시 검출함 -> 따라서 가장 자주 등장하는 클래스들 대표값으로 삼음
        -> 의미있는 결과를 안정적으로 도출할 수 있음
        """

        if result is None or len(result) == 0:
            rospy.loginfo("[Debug] result is None or empty")
            return None
        
        # result가 리스트 형태이면 첫 번째 요소를 사용
        if isinstance(result, list):
            if len(result) == 0:
                return None
            result = result[0]  # tensor 형태여야 함

        rospy.loginfo(f"[Debug] result shape: {result.shape}") #[Debug] result shape: torch.Size([1, 7, 8400])

        # result: (1, 7, 8400) -> (1, 8400, 7)으로 차원 바꾸기
        result = result.permute(0, 2, 1)  # (batch, num_preds, attributes)
        result = result.squeeze(0)        # (8400, 7)

        class_ids = torch.argmax(result[:, 5:], dim=1)
        rospy.loginfo(f"[YOLO] Class IDS: {class_ids}")

        conf_threshold = 0.5
        object_conf = result[:, 4]

        mask = object_conf > conf_threshold

        class_ids = class_ids[mask]
        
        # 클래스 인덱스만 뽑기 (마지막 컬럼이 클래스 인덱스라 가정)
        classes = class_ids.cpu().numpy().tolist()
        # rospy.loginfo(f"[Debug] classes: {classes}") [[]]
        # classes = classes[0]

        if len(classes) == 0:
            return None


        # 가장 많이 나온 class 추출하고 정수형으로 반환
        majority_class = max(set(classes), key=classes.count)
        return int(majority_class)
    
if __name__ == '__main__':
    try:
        detector = ElevatorDoorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
