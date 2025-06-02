#!/usr/bin/env python3

# 엘리베이터 상태 관리 및 제어 노드
# 엘리베이터 이용 전체 과정을 상태 머신으로 관리함!! 
# move_base에 이동 목표 전송, 문 상태 인식 결과 수신, 엘리베이터 제어 인터페이스 호출 등

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped # 목표 자세 설정용?

# 목표 좌표 정의 (추후에는 params.yaml 로드하는 것이 좋음)
ELEVATOR_ENTRANCE_POSE = PoseStamped()
# ELEVATOR_ENTRANCE_POSE.header.frame_id = "map"
# ELEVATOR_ENTRANCE_POSE.pose.position.x = 1.0
# ELEVATOR_ENTRANCE_POSE.pose.position.y = 2.0
# ELEVATOR_ENTRANCE_POSE.pose.orientation.w = 1.0


ELEVATOR_INSIDE_POSE = PoseStamped()
# ELEVATOR_INSIDE_POSE.header.frame_id = "map" # 또는 "elevator_link"

class MoveToState(smach.State):
    def __init__(self, target_pose, pose_name="target"):
        # succeedded: 목표 도달 성공, aborted: 이동 실패(장애물, 경로 없음 등), preempted: 외부에서 중단됨
        smach.State.__init__(self, outcomes=['succeedded', 'aborted', 'preempted'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(f"Connectiong to move_base server for {pose_name}...")
        self.client.wait_for_server()
        rospy.loginfo(f"Connected to mave_base server for {pose_name}.")
        self.target_pose = target_pose
        self.pose_name = pose_name
        self.outcome = None

    def execute(self, userdata):
        rospy.loginfo(f'🚀 Moving to {self.pose_name.upper()}')
        goal = MoveBaseGoal()
        goal.target_pose = self.target_pose
        goal.target_pose.header.stamp = rospy.Time.now() # 목표 전송 시 타임스탬프 설정

        self.client.send_goal(goal, done_cb=self.done_cb)

        # preempt_requested()를 주기적으로 확인하여 선점 요청 처리
        while self.outcome is None:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
        return self.outcome
    
    def done_cb(self, status, result):
        if status == 3: # SUCCEEDED
            rospy.loginfo(f"Goal reached for {self.pose_name}")
            self.outcome = 'succeeded'
        else:
            rospy.logwarn(f"Failed to reach goal for {self.pose_name}. Status: {status}")
            self.outcome = 'aborted'
    
    def request_preempt(self): # SMACH가 호출
        smach.State.request_preempt(self)
        rospy.logwarn(f"Preempt requested for MOVE_TO_{self.pose_name.upper()}")
        self.client.cancel_goal()

class WaitForDoorState(smach.State):
    def __init__(self, target_door_state="open", timeout_duration=30.0):
        smach.State.__init__(self, outcomes=['succeedeed', 'failed', 'preempted'])
        self.target_door_state = target_door_state
        self.current_door_state = ""
        self.timeout = rospy.Duration(timeout_duration)
        self.door_status_sub = None
    
    def door_status_callback(self, msg):
        self.current_door_state = msg.data.lower()
        rospy.loginfo(f'WaitForDoorState: Door status received: {self.current_door_state}')

    def execute(self, userdata):
        rospy.loginfo(f'Executing state WAIT_FOR_DOOR_{self.target_door_state.upper()}')
        self.door_status_sub = rospy.Subscriber('/elevator_vision/door_status', String, self.door_status_callback)
        self.current_door_state = "" # 콜백이 오기 전까지 초기화

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < self.timeout:
            if self.preempt_requested():
                self.service_preempt()
                if self.door_status_sub: self.door_status_sub.unregister()
                return 'preempted'
            
            if self.current_door_state == self.target_door_state:
                rospy.loginfo(f"Door is now {self.target_door_state}!")
                if self.door_status_sub: self.door_status_sub.unregister()
                return 'succeeded'
            rospy.sleep(0.5)

        rospy.logwarn(f"Timeout waiting for door to be {self.target_door_state}.")
        if self.door_status_sub: self.door_status_sub.unregister()
        return 'failed'
    
    def request_preempt(self): # SMACH가 호출
        smach.State.request_preempt(self)
        rospy.logwarn(f"Preempt requested for WAIT_FOR_DOOR_{self.target_door_state.upper()}")

class ControlElevatorState(smach.State):
    def __init__(self, action, target_floor=None): # action: "open_door", "close_door", "goto_floor"
        smach.State.__init__(self, outcomes=['succeedded', 'failed', 'preempted'])
        self.action = action
        self.target_floor = target_floor
        # 실제 서비스 클라이언트 생성 필요
        # 예: self.open_door_client = rospy.ServiceProxy('/elevator_control/open_door', Trigger)
        # 예: self.goto_floor_client = rospy.ServiceProxy('/elevator_control/goto_floor', GoToFloor)
    
    def execute(self, userdata):
        rospy.loginfo(f'Executing state CONTROL_ELEVATOR: {self.action}')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        try:
            # --- 실제 서비스 호출 로직 구현 필요 ---
            # if self.action == "open_door":
            #     rospy.wait_for_service('/elevator_control/open_door', timeout=5.0)
            #     resp = self.open_door_client()
            #     if resp.success: return 'succeeded'
            # elif self.action == "goto_floor" and self.target_floor is not None:
            #     rospy.wait_for_service('/elevator_control/goto_floor', timeout=5.0)
            #     resp = self.goto_floor_client(floor=self.target_floor)
            #     if resp.success: return 'succeeded'
            # else:
            #     return 'failed' # 알 수 없는 액션
            
            rospy.loginfo(f"ControlElevatorState: {self.action} (구현 필요) -> 임시로 성공 처리")
            rospy.sleep(2) # 가상 작업 시간
            return 'succeeded' # 임시 반환
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for {self.action}: {e}")
            return 'failed'
        except rospy.ROSException as e: # wait_for_service 타임아웃 등
            rospy.logerr(f"ROSException for {self.action}: {e}")
            return 'failed'

    def request_preempt(self): # SMACH가 호출
        smach.State.request_preempt(self)
        rospy.logwarn(f"Preempt requested for CONTROL_ELEVATOR: {self.action}")

    def main():
        rospy.init_node('elevator_controller_smach')

        # --- 목표 좌표 로드 (params.yaml 또는 직접 설정) ---
        # 예시: 실제 사용 시에는 파라미터 서버에서 로드하는 것이 좋습니다.
        global ELEVATOR_ENTRANCE_POSE, ELEVATOR_INSIDE_POSE
        ELEVATOR_ENTRANCE_POSE.header.frame_id = "map"
        ELEVATOR_ENTRANCE_POSE.pose.position.x = rospy.get_param("~elevator_entrance_x", 1.0)
        ELEVATOR_ENTRANCE_POSE.pose.position.y = rospy.get_param("~elevator_entrance_y", 2.0)
        ELEVATOR_ENTRANCE_POSE.pose.orientation.w = rospy.get_param("~elevator_entrance_ow", 1.0)
        # ... ELEVATOR_INSIDE_POSE 등 다른 좌표도 설정 ...


        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'mission_preempted'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('IDLE', smach_ros.MonitorState("/start_elevator_mission", String, lambda ud, msg: False), # 외부 토픽으로 시작
                                transitions={'invalid':'MOVING_TO_ELEVATOR_ENTRANCE', 'valid':'MOVING_TO_ELEVATOR_ENTRANCE', 'preempted':'mission_preempted'})

            smach.StateMachine.add('MOVING_TO_ELEVATOR_ENTRANCE',
                                MoveToState(ELEVATOR_ENTRANCE_POSE, "elevator_entrance"),
                                transitions={'succeeded':'REQUESTING_ELEVATOR_CALL', 
                                                'aborted':'mission_failed', 
                                                'preempted':'mission_preempted'})
            
            smach.StateMachine.add('REQUESTING_ELEVATOR_CALL', # 엘리베이터 호출 (현재 층으로 와달라는 의미)
                                ControlElevatorState(action="call_elevator"), # 'call_elevator' 서비스 구현 필요
                                transitions={'succeeded':'WAITING_FOR_DOOR_OPEN',
                                                'failed':'mission_failed',
                                                'preempted':'mission_preempted'})

            smach.StateMachine.add('WAITING_FOR_DOOR_OPEN',
                                WaitForDoorState(target_door_state="open", timeout_duration=60.0), # 문 열림 60초 대기
                                transitions={'succeeded':'ENTERING_ELEVATOR', 
                                                'failed':'mission_failed', # 또는 재시도 로직
                                                'preempted':'mission_preempted'})
            
            # --- 여기에 추가 상태들 구현 ---
            # ENTERING_ELEVATOR (MoveToState(ELEVATOR_INSIDE_POSE))
            # REQUESTING_TARGET_FLOOR (ControlElevatorState(action="goto_floor", target_floor=2))
            # WAITING_FOR_ARRIVAL_AND_DOOR_OPEN (WaitForDoorState, /elevator_sim/current_floor 등 추가 조건 확인)
            # EXITING_ELEVATOR 
            # ...

            # 임시로 간단한 흐름만 구성
            smach.StateMachine.add('ENTERING_ELEVATOR',
                                MoveToState(ELEVATOR_INSIDE_POSE, "elevator_inside"), # 임시로 내부 좌표 사용
                                transitions={'succeeded':'mission_complete', # 실제로는 층 선택, 이동 등으로 이어져야 함
                                                'aborted':'mission_failed',
                                                'preempted':'mission_preempted'})


        # Create and start the introspection server for smach_viewer
        sis = smach_ros.IntrospectionServer('elevator_sm_introspection', sm, '/SM_ROOT_ELEVATOR')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.loginfo(f"SMACH outcome: {outcome}")

        sis.stop()

if __name__ == '__main__':
    main()
