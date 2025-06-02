#!/usr/bin/env python3

import rospy
import smach # 상태머신(state machine) 라이브러리
import smach_ros
from std_msgs.msg import Int32

class FakeMoveState(smach.State):
    """테스트용 가짜 이동 상태 - move_base 없이도 동작"""
    
    def __init__(self, pose_name="target", duration=3.0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.pose_name = pose_name # 이동할 장소 이름
        self.duration = rospy.Duration(duration) # 가짜 이동에 걸릴 시간

    def execute(self, userdata):
        # userdata: 상태 간 데이터 전달용 (테스트 코드에선 사용x)
        rospy.loginfo(f'🚀 [TEST] 가상 이동 시작: {self.pose_name.upper()}')
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < self.duration:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
        
        rospy.loginfo(f'✅ [TEST] 가상 이동 완료: {self.pose_name.upper()}')
        return 'succeeded'

class WaitForDoorState(smach.State):
    """엘리베이터 문 상태 대기 - 실제 로직"""
    # 문 상태 1=열림을 기다림
    def __init__(self, target_door_state=1, timeout_duration=30.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

        self.target_door_state = target_door_state # 기다릴 문 상태
        self.current_door_state = None # 현재 감지된 문 상태
        self.timeout = rospy.Duration(timeout_duration) 
        self.door_status_sub = None # 토픽 구독자
    
    # 엘리베이터 문 상태 토픽을 받을 때마다 호출되는 콜백 함수
    def door_status_callback(self, msg):
        self.current_door_state = msg.data
        door_name = "OPEN" if msg.data == 1 else "CLOSE"
        rospy.loginfo(f'🚪 문 상태 수신: {door_name} (값={msg.data})')

    def execute(self, userdata):
        target_name = "OPEN" if self.target_door_state == 1 else "CLOSE"
        rospy.loginfo(f'⏳ 문 상태 대기 중: {target_name}')
        rospy.loginfo(f'📡 토픽 구독: /elevator_door_status')
        
        # 엘리베이터 문 상태 토픽 구독
        # 구독할 토픽: /elevator_door_status
        self.door_status_sub = rospy.Subscriber('/elevator_door_status', Int32, self.door_status_callback)
        self.current_door_state = None

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < self.timeout:
            if self.preempt_requested():
                self.service_preempt()
                if self.door_status_sub: 
                    self.door_status_sub.unregister()
                return 'preempted'
            
            if self.current_door_state == self.target_door_state:
                rospy.loginfo(f"✅ 문 상태 확인됨: {target_name}!")
                if self.door_status_sub: 
                    self.door_status_sub.unregister()
                return 'succeeded'
            rospy.sleep(0.5)

        rospy.logwarn(f"⏰ 타임아웃: {target_name} 상태를 {self.timeout.to_sec()}초 내에 감지하지 못함")
        if self.door_status_sub: 
            self.door_status_sub.unregister()
        return 'failed'

class SimpleDelayState(smach.State):
    """단순 시간 지연 상태"""
    
    def __init__(self, delay_seconds, description=""):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.delay = rospy.Duration(delay_seconds)
        self.description = description

    def execute(self, userdata):
        rospy.loginfo(f'⏳ {self.description} ({self.delay.to_sec()}초)')
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time) < self.delay:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
            
        rospy.loginfo(f'✅ {self.description} 완료')
        return 'succeeded'

def main():
    rospy.init_node('test_elevator_controller_smach')
    rospy.loginfo("🎯 [TEST] 엘리베이터 상태머신 테스트 모드 시작!")

    # 테스트용 상태머신 생성
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'mission_preempted'])

    # 상태머신에 상태 추가
    with sm:
        # === 1단계: 엘리베이터 앞으로 이동 (가상) ===
        smach.StateMachine.add(
            'MOVING_TO_ELEVATOR_ENTRANCE',
            FakeMoveState("elevator_entrance", 3.0),  # 3초 가상 이동
            transitions={
                'succeeded': 'REQUESTING_ELEVATOR_CALL',
                'preempted': 'mission_preempted'
            }
        )
        
        # === 2단계: 엘리베이터 호출 (시뮬레이션) ===
        smach.StateMachine.add(
            'REQUESTING_ELEVATOR_CALL',
            SimpleDelayState(2.0, "엘리베이터 호출 중..."),
            transitions={
                'succeeded': 'WAITING_FOR_DOOR_OPEN',
                'preempted': 'mission_preempted'
            }
        )

        # === 3단계: 문 열림 대기 (실제 로직) ===
        smach.StateMachine.add(
            'WAITING_FOR_DOOR_OPEN',
            WaitForDoorState(target_door_state=1, timeout_duration=60.0),
            transitions={
                'succeeded': 'ENTERING_ELEVATOR',
                'failed': 'mission_failed',
                'preempted': 'mission_preempted'
            }
        )
        
        # === 4단계: 엘리베이터 탑승 (가상) ===
        smach.StateMachine.add(
            'ENTERING_ELEVATOR',
            FakeMoveState("elevator_inside", 2.0),  # 2초 가상 탑승
            transitions={
                'succeeded': 'SIMULATING_ELEVATOR_TRAVEL',
                'preempted': 'mission_preempted'
            }
        )
        
        # === 5단계: 엘리베이터 이동 (시뮬레이션) ===
        smach.StateMachine.add(
            'SIMULATING_ELEVATOR_TRAVEL',
            SimpleDelayState(5.0, "엘리베이터 2층으로 이동 중..."),
            transitions={
                'succeeded': 'WAITING_FOR_DOOR_OPEN_2F',
                'preempted': 'mission_preempted'
            }
        )
        
        # === 6단계: 2층에서 문 열림 대기 (실제 로직) ===
        smach.StateMachine.add(
            'WAITING_FOR_DOOR_OPEN_2F',
            WaitForDoorState(target_door_state=1, timeout_duration=60.0),
            transitions={
                'succeeded': 'mission_complete',
                'failed': 'mission_failed',
                'preempted': 'mission_preempted'
            }
        )

    # SMACH viewer용 서버
    sis = smach_ros.IntrospectionServer('test_elevator_sm', sm, '/SM_ROOT_TEST')
    sis.start()

    rospy.loginfo("🎮 테스트 시작! 다음과 같이 테스트하세요:")
    rospy.loginfo("📝 터미널에서: rostopic pub /elevator_door_status std_msgs/Int32 'data: 1'")
    rospy.loginfo("🔍 상태 확인: rostopic echo /elevator_door_status")
    rospy.loginfo("👀 시각화: rosrun smach_viewer smach_viewer.py")
    
    # 상태머신 실행
    outcome = sm.execute()
    rospy.loginfo(f"🏁 테스트 결과: {outcome}")

    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 테스트 종료")
    except Exception as e:
        rospy.logerr(f"❌ 테스트 오류: {e}")