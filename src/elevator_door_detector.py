#!/usr/bin/env python3

# 학습된 모델로 엘리베이터 문 상태(close, open) 인식
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

sys.path.append(config.get('sys_path'))

# 개별 변수로 꺼내 쓰기
model_path = config.get('model_path')
cfg_path = config.get('cfg_path')
image_topic = config.get('image_topic')
status_topic = config.get('status_topic')

from models.common import DetectMultiBackend
from utils.general import non_max_suppression, check_img_size
from utils.torch_utils import select_device
import torch
import numpy as np

class ElevatorDoorDetector:
    def __init__(self):
        rospy.init_node('elevator_door_detector')

        # YOLOv9 model load
        rospy.loginfo(f"[YOLO] Loading model from: {model_path}")

        self.device = select_device(0 if torch.cuda.is_available() else 'cpu')
        
        # DetectMultiBackend 사용
        data_yaml = config.get('classes_yaml_path', 'data/data.yaml')
        self.model = DetectMultiBackend(model_path, device=self.device, dnn=False, data=data_yaml, fp16=False)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size((640, 640), s=self.stride)
        
        # 모델 워밍업
        self.model.warmup(imgsz=(1, 3, *self.imgsz))
        
        # set ROS
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, Int32, queue_size=10)

        rospy.loginfo("✅ ElevatorDoorDetector initialized and ready.")
        rospy.loginfo(f"📝 Class names: {self.names}")

    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        """이미지 리사이즈 및 패딩"""
        shape = im.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)

        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return im, ratio, (dw, dh)

    def image_callback(self, msg): 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[Bridge] Error converting image: {e}")
            return
        
        # 이미지 전처리
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img, ratio, pad = self.letterbox(img, self.imgsz, stride=self.stride)
        
        # numpy array to tensor
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255.0
        if len(img.shape) == 3:
            img = img[None]

        with torch.no_grad():
            # 모델 추론
            pred = self.model(img, augment=False, visualize=False)
            
            # 실제 prediction tensor 찾기
            if isinstance(pred, (list, tuple)):
                if hasattr(pred[0], 'shape'):
                    pred = pred[0]
                elif isinstance(pred[0], (list, tuple)) and len(pred[0]) > 0 and hasattr(pred[0][0], 'shape'):
                    pred = pred[0][0]
                elif len(pred) > 1 and hasattr(pred[1], 'shape'):
                    pred = pred[1]
            
            # NMS 적용
            conf_thres = 0.5   # confidence threshold를 0.5로 상향 조정
            iou_thres = 0.45   # NMS IOU threshold
            max_det = 1000     # maximum detections per image
            
            pred = non_max_suppression(pred, conf_thres, iou_thres, None, False, max_det=max_det)

        door_status = self.extract_door_status(pred)

        if door_status is not None:
            self.status_pub.publish(Int32(door_status))
            status_name = self.names[door_status] if door_status < len(self.names) else f"class_{door_status}"
            rospy.loginfo(f"🚪 Elevator door: {status_name} (class={door_status})")
        else:
            rospy.logwarn("⚠️ No elevator door detected")
    
    def extract_door_status(self, pred):
        """NMS 후 결과에서 클래스 추출 (0: close, 1: open)"""
        if pred is None or len(pred) == 0:
            return None
        
        det = pred[0]  # 첫 번째 이미지의 detection 결과
        
        if len(det) == 0:
            return None
        
        # 가장 높은 confidence를 가진 detection의 클래스 선택
        confidences = det[:, 4].cpu().numpy().tolist()
        classes = det[:, 5].cpu().numpy().astype(int).tolist()
        
        if len(classes) == 0:
            return None

        max_conf_idx = confidences.index(max(confidences))
        best_class = classes[max_conf_idx]
        best_conf = confidences[max_conf_idx]
        
        # NMS에서 이미 confidence 필터링이 되었으므로 추가 체크 불필요
        return int(best_class)
    
if __name__ == '__main__':
    try:
        detector = ElevatorDoorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass