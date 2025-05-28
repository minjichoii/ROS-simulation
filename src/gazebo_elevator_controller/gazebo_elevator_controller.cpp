/**
    위치: elevator_navigation/src/gazebo_elevator_controller/gazebo_elevator_controller.cpp (플러그인인 경우) 또는 elevator_navigation/src/gazebo_elevator_controller_node.cpp (ROS 노드인 경우)

    역할: 시뮬레이션 환경(Gazebo) 내의 엘리베이터 모델을 실제로 제어하고, 엘리베이터의 상태 정보를 ROS 토픽이나 서비스로 제공합니다.

    언어: C++ (Gazebo API와 직접 상호작용하려면 C++ 플러그인이 일반적) 또는 Python (만약 Gazebo 모델 상태를 변경하는 ROS 서비스가 이미 있다면)

    주요 기능:

        ROS 서비스 서버 구현:

            /elevator_control/open_door

            /elevator_control/close_door

            /elevator_control/goto_floor (층 번호 입력)

            /elevator_control/call_elevator (호출 층 입력)

        Gazebo 모델 제어: 서비스 요청에 따라 Gazebo API를 사용하여 엘리베이터 모델의 조인트(문), 링크(엘리베이터 본체)를 움직여 문을 열고 닫고, 엘리베이터를 위아래로 이동시킵니다.

        상태 토픽 발행:

            /elevator_sim/current_floor (현재 엘리베이터 층)

            /elevator_sim/door_state (문 상태: "OPENED", "CLOSED", "MOVING")

            /elevator_sim/is_moving (엘리베이터 이동 중 여부)

    구현 복잡도: 이 부분은 Gazebo 플러그인 개발 경험이 필요하며, 엘리베이터 모델의 SDF 구조에 따라 제어 방식이 달라집니다. 간단히 시작하려면, Python ROS 노드가 Gazebo의 /gazebo/set_model_state 나 /gazebo/set_joint_positions 와 같은 내장 서비스를 사용하여 엘리베이터를 "순간이동" 시키거나 문을 여닫는 형태로 시작해볼 수도 있습니다 (애니메이션은 덜 자연스러울 수 있음).

 */

 /*
 전체적인 연결 및 흐름:

    런치 파일 실행: roslaunch elevator_navigation elevator_navigation.launch 명령으로 모든 관련 노드가 실행되고 파라미터가 로드됩니다.

    elevator_controller_sm (상태 머신):

        초기 상태(예: IDLE 또는 외부 명령 대기)에서 시작합니다.

        move_base에 목표 전송: 특정 지점(예: 엘리베이터 앞)으로 이동하기 위해 move_base 액션 서버에 목표를 보냅니다. move_base는 로봇의 이동을 담당합니다.

        문 상태 수신: elevator_door_detector가 발행하는 /elevator_vision/door_status 토픽을 구독하여 현재 문의 시각적 상태를 파악합니다.

        엘리베이터 제어: gazebo_elevator_controller가 제공하는 ROS 서비스(예: /elevator_control/open_door)를 호출하여 시뮬레이션 엘리베이터를 작동시킵니다.

        엘리베이터 상태 수신: gazebo_elevator_controller가 발행하는 토픽(예: /elevator_sim/current_floor)을 구독하여 엘리베이터의 실제 상태를 확인합니다.

    elevator_door_detector: 카메라 이미지를 받아 YOLO 추론을 수행하고, 감지된 엘리베이터 문의 상태를 /elevator_vision/door_status 토픽으로 계속 발행합니다.

    gazebo_elevator_controller: elevator_controller_sm으로부터 서비스 요청을 받으면 Gazebo 내 엘리베이터 모델을 움직이고, 그 결과를 토픽으로 발행합니다.

  */