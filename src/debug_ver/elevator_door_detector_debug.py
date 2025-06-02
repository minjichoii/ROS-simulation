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

from models.common import DetectMultiBackend  # 바운딩박스 코드와 동일한 모델 로더 사용
from utils.general import non_max_suppression, check_img_size  # NMS 함수 추가
from utils.torch_utils import select_device
import torch
import numpy as np

class ElevatorDoorDetector:
    def __init__(self):
        rospy.init_node('elevator_door_detector')

        # YOLOv9 model load (바운딩박스 코드와 동일한 방식)
        rospy.loginfo(f"[YOLO] Loading model from: {model_path}")
        
        # 바운딩박스 코드와 동일한 모델 로딩 방식
        self.device = select_device(0 if torch.cuda.is_available() else 'cpu')
        
        # DetectMultiBackend 사용 (바운딩박스 코드와 동일)
        data_yaml = config.get('classes_yaml_path', 'data/data.yaml')  # data.yaml 경로
        self.model = DetectMultiBackend(model_path, device=self.device, dnn=False, data=data_yaml, fp16=False)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size((640, 640), s=self.stride)  # check image size
        
        rospy.loginfo(f"[YOLO] Model loaded successfully")
        rospy.loginfo(f"[YOLO] Class names: {self.names}")
        rospy.loginfo(f"[YOLO] Stride: {self.stride}")
        
        # 모델 워밍업
        self.model.warmup(imgsz=(1, 3, *self.imgsz))

        # set ROS
        self.bridge = CvBridge() # ROS image -> OpenCV image 브릿지 생성
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, Int32, queue_size=10) # 엘리베이터 문 상태 퍼블리시

        rospy.loginfo("✅ ElevatorDoorDetector initialized and ready.")

    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        """이미지 리사이즈 및 패딩 (바운딩박스 코드와 동일)"""
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, ratio, (dw, dh)

    # 이미지 들어올 때마다 자동 호출되는 콜백 함수
    # ROS image -> OpenCV image (bgr8)
    def image_callback(self, msg): 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[Bridge] Error converting image: {e}")
            return
        
        # 바운딩박스 코드와 동일한 전처리
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # letterbox 전처리 적용
        img, ratio, pad = self.letterbox(img, self.imgsz, stride=self.stride)
        
        # numpy array to tensor
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        with torch.no_grad():
            # 모델 추론
            pred = self.model(img, augment=False, visualize=False)
            
            rospy.loginfo(f"[Debug] Raw prediction type: {type(pred)}")
            if isinstance(pred, (list, tuple)):
                rospy.loginfo(f"[Debug] Raw prediction length: {len(pred)}")
                for i, item in enumerate(pred):
                    rospy.loginfo(f"[Debug] pred[{i}] type: {type(item)}")
                    if hasattr(item, 'shape'):
                        rospy.loginfo(f"[Debug] pred[{i}] shape: {item.shape}")
                    elif isinstance(item, (list, tuple)):
                        rospy.loginfo(f"[Debug] pred[{i}] length: {len(item)}")
                        if len(item) > 0:
                            rospy.loginfo(f"[Debug] pred[{i}][0] type: {type(item[0])}")
                            if hasattr(item[0], 'shape'):
                                rospy.loginfo(f"[Debug] pred[{i}][0] shape: {item[0].shape}")
                
                # 실제 prediction tensor 찾기
                pred_tensor = None
                if hasattr(pred[0], 'shape'):
                    pred_tensor = pred[0]
                elif isinstance(pred[0], (list, tuple)) and len(pred[0]) > 0 and hasattr(pred[0][0], 'shape'):
                    pred_tensor = pred[0][0]
                elif len(pred) > 1 and hasattr(pred[1], 'shape'):
                    pred_tensor = pred[1]
                
                if pred_tensor is not None:
                    rospy.loginfo(f"[Debug] Found prediction tensor shape: {pred_tensor.shape}")
                    pred = pred_tensor
                else:
                    rospy.logerr("[Debug] Could not find prediction tensor")
                    return
            
            rospy.loginfo(f"[Debug] Prediction shape before NMS: {pred.shape}")
            
            # NMS 적용 (바운딩박스 코드와 동일)
            conf_thres = 0.25  # confidence threshold
            iou_thres = 0.45   # NMS IOU threshold
            max_det = 1000     # maximum detections per image
            
            pred = non_max_suppression(pred, conf_thres, iou_thres, None, False, max_det=max_det)
            
            rospy.loginfo(f"[Debug] After NMS type: {type(pred)}")
            rospy.loginfo(f"[Debug] After NMS length: {len(pred)}")
            if len(pred) > 0:
                rospy.loginfo(f"[Debug] First detection batch shape: {pred[0].shape}")

        door_status = self.extract_door_status(pred)

        if door_status is not None:
            self.status_pub.publish(Int32(door_status))
            rospy.loginfo(f"[YOLO] Detected elevator door status: {door_status}")
        else:
            rospy.loginfo("[YOLO] No elevator door detected")
    
    def extract_door_status(self, pred):
        # NMS 후 결과에서 클래스 추출 (바운딩박스 코드와 동일한 방식)
        # 0: close, 1: open

        if pred is None or len(pred) == 0:
            rospy.loginfo("[Debug] No predictions")
            return None
        
        # 첫 번째 이미지의 detection 결과
        det = pred[0]  # (N, 6) format: [x1, y1, x2, y2, conf, cls]
        
        rospy.loginfo(f"[Debug] Detection shape: {det.shape}")
        rospy.loginfo(f"[Debug] Number of detections: {len(det)}")
        
        if len(det) == 0:
            rospy.loginfo("[Debug] No detections above confidence threshold")
            return None
        
        # 모든 detection 정보 출력
        for i, detection in enumerate(det):
            x1, y1, x2, y2, conf, cls = detection
            rospy.loginfo(f"[Debug] Detection {i}: class={int(cls)}, conf={conf:.3f}")
        
        # 클래스 추출 (6번째 컬럼, 인덱스 5)
        classes = det[:, 5].cpu().numpy().astype(int).tolist()
        confidences = det[:, 4].cpu().numpy().tolist()
        
        rospy.loginfo(f"[Debug] Classes: {classes}")
        rospy.loginfo(f"[Debug] Confidences: {confidences}")
        
        if len(classes) == 0:
            return None

        # 가장 높은 confidence를 가진 detection의 클래스 선택
        max_conf_idx = confidences.index(max(confidences))
        best_class = classes[max_conf_idx]
        best_conf = confidences[max_conf_idx]
        
        rospy.loginfo(f"[Debug] Best detection: class={best_class}, conf={best_conf:.3f}")
        
        return int(best_class)
    
if __name__ == '__main__':
    try:
        detector = ElevatorDoorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass