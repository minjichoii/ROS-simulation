#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

class ElevatorGazeboActuator {
    public:
        ElevatorGazeboActuator() {
            initializeParameters();
            initializePublishers();
            initializeSubscribers();
            initializeServiceClients();
            
            // 상태 업데이트 타이머
            status_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                        &ElevatorGazeboActuator::statusUpdateCallback, this);
            
            // 초기 위치 설정 타이머 (3초 후)
            setup_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                        &ElevatorGazeboActuator::initialSetupCallback, this, true);
            
            ROS_INFO("🚀 Elevator Gazebo Actuator initialized!");
            ROS_INFO("🔧 Supporting both Joint and Model control");
        }

    private:
        ros::NodeHandle nh_;

        // Publishers - Gazebo 제어
        ros::Publisher door_joint_pub_; // Joint 제어 (우선순위)
        ros::Publisher lift_joint_pub_; // Joint 제어 (우선순위)

        // Publishers - Controller에게 피드백
        ros::Publisher robot_inside_pub_; // /elevator/robot_inside
        ros::Publisher current_floor_pub_; // /elevator/current_floor
        ros::Publisher door_status_pub_; // /elevator/door_status
        ros::Publisher elevator_status_pub_; // /elevator/status
        ros::Publisher robot_approach_pub_; // /elevator/robot_approach

        // Subscribers - 명령 받기
        ros::Subscriber door_control_sub_; // /elevator/door_control
        ros::Subscriber move_command_sub_; // /elevator/move_command

        // Subscribers - Gazebo 상태 받기
        ros::Subscriber joint_states_sub_; // /joint_states
        ros::Subscriber model_states_sub_; // /gazebo/mode_states

         // 🎯 Service Clients
        ros::ServiceClient set_model_state_client_;
        ros::ServiceClient get_model_state_client_;

        // Timers
        ros::Timer status_timer_;
        ros::Timer setup_timer_;
        ros::Timer door_timer_;
        ros::Timer lift_timer_;

         // 🎯 현재 상태
        double current_lift_position_;         // 현재 엘리베이터 높이 (m)
        double current_door_position_;         // 현재 문 위치 (0~1)
        double target_lift_position_;          // 목표 엘리베이터 높이
        double target_door_position_;          // 목표 문 위치
        
        bool door_moving_;                     // 문 움직임 중
        bool lift_moving_;                     // 엘리베이터 움직임 중
        bool robot_inside_elevator_;           // 로봇 탑승 상태
        bool robot_near_elevator_;             // 로봇 접근 상태
        bool use_joint_control_;               // Joint 제어 사용 여부
        bool gazebo_ready_;                    // Gazebo 준비 상태
        
        // 🎯 설정 매개변수
        double floor_height_;                  // 층간 높이
        double door_animation_duration_;       // 문 애니메이션 시간
        double lift_animation_duration_;       // 엘리베이터 이동 시간
        double detection_radius_;              // 로봇 감지 반경
        double approach_radius_;               // 로봇 접근 감지 반경
        
        std::string elevator_model_name_;      // 엘리베이터 모델명
        std::string robot_model_name_;         // 로봇 모델명
        
        // 🎯 World 파일 기반 위치들
        geometry_msgs::Pose elevator_floor0_pose_;  // 1층 위치
        geometry_msgs::Pose elevator_floor1_pose_;  // 2층 위치
        geometry_msgs::Pose door_close_pose_;       // 문 닫힘 위치
        geometry_msgs::Pose door_open_pose_;        // 문 열림 위치

    void initializeParameters() {
        // World 파일 기반 실제 값들
        nh_.param("floor_height", floor_height_, 3.075); // world 파일 정확한 값
        nh_.param("door_animation_duration", door_animation_duration_, 3.0);
        nh_.param("lift_animation_duration", lift_animation_duration_, 5.0);
        nh_.param("detection_radius", detection_radius_, 1.2);
        nh_.param("approach_radius", approach_radius_, 3.0);
        nh_.param("elevator_model_name", elevator_model_name_, std::string("elevator"));
        nh_.param("robot_model_name", robot_model_name_, std::string("turtlebot3"));
        nh_.param("use_joint_control", use_joint_control_, true);

        // 초기 상태 설정 (World 파일 기준)
        current_lift_position_ = 0.075; // 1층
        current_door_position_ = 0.0;
        target_lift_position_ = 0.075;
        target_door_position_ = 0.0;

        door_moving_ = false;
        lift_moving_ = false;
        robot_inside_elevator_ = false;
        robot_near_elevator_ = false;
        gazebo_ready_ =false;

        // World 파일 기반 위치 설정
        setupWorldPositions();
        
        ROS_INFO("📋 Parameters initialized:");
        ROS_INFO("   Floor height: %.3fm", floor_height_);
        ROS_INFO("   Detection radius: %.1fm", detection_radius_);
        ROS_INFO("   Use joint control: %s", use_joint_control_ ? "Yes" : "No");
    }

    void setupWorldPositions() {
        // World 파일에서 추출한 실제 위치들
        
        // 1층 엘리베이터 위치 (World 파일 기준)
        elevator_floor0_pose_.position.x = 0.0;
        elevator_floor0_pose_.position.y = 0.0;
        elevator_floor0_pose_.position.z = 0.075;
        elevator_floor0_pose_.orientation.w = 1.0;

        // 2층 위치 (1층 + floor_height)
        elevator_floor1_pose_ = elevator_floor0_pose_;
        elevator_floor1_pose_.position.z = 0.075 + floor_height_;

        // 문 닫힘 위치 (World 파일 기준)
        door_close_pose_.position.x = 0.0;
        door_close_pose_.position.y = 0.0;
        door_close_pose_.position.z = 0.075;
        door_close_pose_.orientation.w = 1.0;

        // 문 열림 위치 ( Y축으로 1.5m 이동 )
        door_open_pose_ = door_close_pose_;
        door_open_pose_.position.y = 1.5;

        ROS_INFO("🖲️ World positions configured");
    }

    void initializePublishers() {
        // Joint 제어 Publishers (우선순위)
        door_joint_pub_ = nh_.advertise<std_msgs::Float64>("/elevator/door_joint_position_controller/command", 1);
        lift_joint_pub_ = nh_.advertise<std_msgs::Float64>("/elevator/lift_position_controller/command", 1);

        // Controller에게 피드백
        robot_inside_pub_ = nh_.advertise<std_msgs::Bool>("/elevator/robot_inside", 1);
        current_floor_pub_ = nh_.advertise<std_msgs::Int32>("/elevator/current_floor", 1);
        door_status_pub_ = nh_.advertise<std_msgs::String>("/elevator/door_status", 1);
        elevator_status_pub_ = nh_.advertise<std_msgs::String>("/elevator/status", 1);
        robot_approach_pub_ = nh_.advertise<std_msgs::String>("/elevator/robot_approach", 1);

        ROS_INFO("📡 Publishers initialized");
    }

    void initializeSubscribers() {
        // 명령 구독
        door_control_sub_ = nh_.subscribe("/elevator/door_control", 1, 
                                        &ElevatorGazeboActuator::doorControlCallback, this);
        move_command_sub_ = nh_.subscribe("/elevator/move_command", 1, 
                                        &ElevatorGazeboActuator::moveCommandCallback, this);
        
        // Gazebo 상태 구독
        joint_states_sub_ = nh_.subscribe("/joint_states", 1, 
                                        &ElevatorGazeboActuator::jointStatesCallback, this);
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, 
                                        &ElevatorGazeboActuator::modelStatesCallback, this);

        ROS_INFO("📡 Subscribers initialized");
    }

    void initializeServiceClients() {
        set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        
        ROS_INFO("🔗 Service clients initialized");
    }

    // 초기 설정 콜백
    void initialSetupCallback(const ros::TimerEvent&) {
        ROS_INFO("🔧 Performing initial setup...");

        // Gazebo 서비스 확인
        if (set_model_state_client_.waitForExistence(ros::Duration(2.0))) {
            gazebo_ready_ = true;
            ROS_INFO("✅ Gazebo services ready");

            // 초기 위치로 설정
            resetToInitialPosition();
        } else {
            ROS_WARN("⚠️ Gazebo services not available, using fallback mode");
            gazebo_ready_ = false;
        }
    }

    void resetToInitialPosition() {
        // 엘리베이터를 1층, 문을 닫힌 상태로 설정
        if (use_joint_control_) {
            // Joint 제어 시도
            std_msgs::Float64 lift_cmd, door_cmd;
            lift_cmd.data = 0.075; // 1층 높이
            door_cmd.data = 0.0; // 문 닫힘

            lift_joint_pub_.publish(lift_cmd);
            door_joint_pub_.publish(door_cmd);

            ROS_INFO("🔧 Initial position set via Joint control");
        } else {
            // Model state 제어 사용
            setElevatorModelState(elevator_floor0_pose_);
            setDoorModelState(door_close_pose_);

            ROS_INFO("🔧 Initial position set via Model State");
        }
    }

    // 명령 콜백 함수들
    void doorControlCallback(const std_msgs::String::ConstPtr& msg) {
        if (door_moving_) {
            ROS_WARN("🚫 Door is already moving, ignoring command: %s", msg->data.c_str());
            return;
        }

        ROS_INFO("🚪 Received door command: %s", msg->data.c_str());

        if (msg->data == "open") {
            controlDoor(1.0); // 완전히 열기
        } else if (msg->data == "close") {
            controlDoor(0.0); // 완전히 닫기
        } else {
            ROS_WARN("❌ Unknown door command: %s", msg->data.c_str());
        }
    }

    void moveCommandCallback(const std_msgs::Int32::ConstPtr& msg) {
        if (lift_moving_) {
            ROS_WARN("🚫 Elevator is already moving, ignoring command: %d", msg->data);
            return;
        }

        int target_floor = msg->data;
        ROS_INFO("🚪 Received door command: %s", target_floor);

        if (target_floor < 0 || target_floor > 1) {
            ROS_WARN("❌ Invalid floor: %d (valid: 0-1)", target_floor);
            return;
        }

        controlElevator(target_floor);
    }

    // Gazebo 상태 콜백들
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i =0; i < msg->name.size(); i++) {
            // World 파일의 실제 Joint 이름 확인
            if (msg->name[i] == "lift" || msg->name[i] == "elevator::lift" || 
                msg->name[i] == "elevator" || msg->name[i].find("lift") != std::string::npos) {
                    current_lift_position_ = msg->position[i];
            } else if (msg->name[i] == "door_joint" || msg->name[i] == "elevator::door_joint" ||
            msg->name[i] == "elevator_door" || msg->name[i].find("door") != std::string::npos) {
                current_door_position_ = msg->position[i];
            }
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        geometry_msgs::Pose robot_pose;
        geometry_msgs::Pose elevator_pose;
        bool robot_found = false;
        bool elevator_found = false;

        // 로봇과 엘리베이터 찾기
        for (size_t i=0; i < msg->name.size(); i++) {
            std::string name = msg->name[i];

            // 로봇 모델 찾기 (다양한 이름 지원)
            if (name.find("robot") != std::string::npos || name.find("turtlebot") != std::string::npos ||
                name.find("tb3") != std::string::npos || name.find("waffle") != std::string::npos) {
                robot_pose = msg->pose[i];
                robot_found = true;
            } 
            // 엘리베이터 모델 찾기 
            else if (name.find("elevator") != std::string::npos) {
                elevator_pose = msg->pose[i];
                elevator_found = true;

                // Joint 제어가 없을 때는 Model State로 현재 위치 업데이트
                if (!use_joint_control_) {
                    current_lift_position_ = elevator_pose.position.z;
                }
            }
        }

        if (robot_found && elevator_found) {
            updateRobotDetection(robot_pose, elevator_pose);
        }
    } 

    // 로봇 감지 로직
    void updateRobotDetection(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& elevator_pose) {
        // 2D 거리 계산
        double dx = robot_pose.position.x - elevator_pose.position.x;
        double dy = robot_pose.position.y - elevator_pose.position.y;
        double dz = robot_pose.position.z - elevator_pose.position.z;
        double distance_2d = sqrt(dx*dx + dy*dy);

        // 높이 차이 확인 (같은 층인지) 
        bool same_height = (abs(dz) < 1.0);

        // 로봇 상태 업데이트
        bool prev_inside = robot_inside_elevator_;
        bool prev_near = robot_near_elevator_;

        robot_inside_elevator_ = (distance_2d < detection_radius_) && same_height;
        robot_near_elevator_ = (distance_2d < approach_radius_) && same_height && !robot_inside_elevator_;

        // 상태 변화 로깅
        if (robot_inside_elevator_ != prev_inside) {
            ROS_INFO("🤖 Robot %s elevator (distance: %.2fm, height_diff: %.2fm)",
                robot_inside_elevator_ ? "ENTERED" : "EXITED", distance_2d, fabs(dz));
        }

        if (robot_near_elevator_ != prev_near) {
            ROS_INFO("🤖 Robot %s elevator area (distance: %.2fm)",
                robot_near_elevator_ ? "APPROACHING" : "LEFT", distance_2d);
        }
    }

    // 실제 제어 함수들
    void controlDoor(double target_position) {
        target_door_position_ = target_position;
        door_moving_ = true;

        ROS_INFO("🚪 Controlling door to position: %.1f", target_position);

        if (use_joint_control_) {
            // Joint 제어 우선
            std_msgs::Float64 cmd;
            cmd.data = target_position;
            door_joint_pub_.publish(cmd);
        } else if (gazebo_ready_) {
            // Model State 제어 백업
            geometry_msgs::Pose target_pose = (target_position > 0.5) ? door_open_pose_ : door_close_pose_;
            setDoorModelState(target_pose);
        }

        door_timer_ = nh_.createTimer(ros::Duration(door_animation_duration_), 
                                    &ElevatorGazeboActuator::doorAnimationComplete, this, true);
    }

    void controlElevator(int target_floor) {
        target_lift_position_ = 0.075 + target_floor * floor_height_;
        lift_moving_ = true;

        ROS_INFO("🛗 Controlling elevator to floor %d (height: %.3fm)", target_floor, target_lift_position_);

        if (use_joint_control_) {
            // Joint 제어 우선
            std_msgs::Float64 cmd;
            cmd.data = target_lift_position_;
            lift_joint_pub_.publish(cmd);
        } else if (gazebo_ready_) {
            // Model State 제어 백업
            geometry_msgs::Pose target_pose = (target_floor == 0) ? elevator_floor0_pose_ : elevator_floor1_pose_;
            setElevatorModelState(target_pose);
        }

        // 이동 완료 타이머
        lift_timer_ = nh_.createTimer(ros::Duration(lift_animation_duration_),
                                    &ElevatorGazeboActuator::liftAnimationComplete, this, true);
    }

    // Model State 제어 (백업용임)
    void setElevatorModelState(const geometry_msgs::Pose& pose) {
        gazebo_msgs::SetModelState srv;
        srv.request.model_state.model_name = elevator_model_name_;
        srv.request.model_state.pose = pose;
        srv.request.model_state.reference_frame = "world";

        if (set_model_state_client_.call(srv)) {
            ROS_INFO("🛗 Elevator model state updated");
        } else {
            ROS_WARN("⚠️ Failed to update elevator model state");
        }
    }

    void setDoorModelState(const geometry_msgs::Pose& pose) {
        gazebo_msgs::SetModelState srv;
        srv.request.model_state.model_name = elevator_model_name_ + "::door";
        srv.request.model_state.pose = pose;
        srv.request.model_state.reference_frame = "world";

        if (set_model_state_client_.call(srv)) {
            ROS_INFO("🚪 Door model state updated");
        } else {
            ROS_WARN("⚠️ Failed to update door model state");
        }
    }

    void doorAnimationComplete(const ros::TimerEvent&) {
        door_moving_ = false;
        ROS_INFO("✅ Door animation completed - Position: %.1f", current_door_position_);
    }

    void liftAnimationComplete(const ros::TimerEvent&) {
        lift_moving_ = false;
        ROS_INFO("✅ Elevator movement completed - Position: %.3fm", current_lift_position_);
    }

    // 상태 피드백 발행
    void statusUpdateCallback(const ros::TimerEvent&) {
        publishRobotInside();
        publishCurrentFloor();
        publishDoorStatus();
        publishElevatorStatus();
        publishRobotApproach();
    }

    void publishRobotInside() {
        std_msgs::Bool msg;
        msg.data = robot_inside_elevator_;
        robot_inside_pub_.publish(msg);
    }

    void publishCurrentFloor() {
        // 현제 높이를 기반으로 층 계산
        int current_floor = (int)round((current_lift_position_ - 0.075) / floor_height_);
        current_floor = std::max(0, std::min(1, current_floor)); // 0-1 범위 제한 

        std_msgs::Int32 msg;
        msg.data = current_floor;
        current_floor_pub_.publish(msg);
    }

    void publishDoorStatus() {
        std_msgs::String msg;

        if (door_moving_) {
            msg.data = "moving";
        } else if (current_door_position_ > 0.8) {
            msg.data = "open";
        } else if (current_door_position_ > 0.2) {
            msg.data = "close";
        } else {
            msg.data = "partial";
        }

        door_status_pub_.publish(msg);
    }

    void publishElevatorStatus() {
        std_msgs::String msg;
        msg.data = lift_moving_ ? "moving" : "stopped";
        elevator_status_pub_.publish(msg);
    }

    void publishRobotApproach() {
        std_msgs::String msg;
        msg.data = robot_near_elevator_;
        robot_approach_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "elevator_gazebo_actuator");

    ElevatorGazeboActuator actuator;

    ROS_INFO("🎯 Elevator Gazebo Actuator running...");
    ROS_INFO("🔧 Ready to control Gazebo elevator with real feedback!");
    ros::spin();
    
    return 0;
}
    
        