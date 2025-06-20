#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>                                              

class GazeboElevatorController {
    public:
        // 자동화 상태
        enum State {
            IDLE_AT_1F,                 // 1층 대기 (문닫힌 상태)
            DOOR_OPENING_1F,            // 1층에서 문 여는 중
            WAITING_FOR_BOARDING,       // 로봇 탑승 대기 (문열린 상태)
            DOOR_CLOSING_1F,            // 1층에서 문 닫는 중 (로봇 탑승 후)
            MOVING_TO_2F,               // 2층으로 이동중
            DOOR_OPENING_2F,            // 2층에서 문 여는 중
            WAITING_FOR_EXIT,           // 로봇 하차 대기 (문열린 상태)
            DOOR_CLOSING_2F,            // 2층에서 문 닫는 중 (로봇 하차 후)
            RETURNING_TO_1F,             // 1층으로 복귀중
            ERROR_STATE                 // 에러 상태
        };

        GazeboElevatorController() : current_state_(IDLE_AT_1F) {
            initializePublishers();
            initializeSubscribers();
            initializeParameters();

            // 상태 업데이트 타이머
            state_timer_ = nh_.createTimer(ros::Duration(0.1), &GazeboElevatorController::stateUpdateCallback, this);

            // 통계 타이머
            statistics_timer_ = nh_.createTimer(ros::Duration(10.0), &GazeboElevatorController::printStatisticsCallback, this);
        
            // 통계 변수 초기화
            mission_count_ = 0;
            total_wait_time_ = 0.0;
            mission_start_time_ = ros::Time::now();

            ROS_INFO("🚀 Gazebo Elevator Controller initialized!");
            ROS_INFO("🏢 Current state: %s", stateToString(current_state_).c_str());
        }

    private:
        ros::NodeHandle nh_;
        
        // 🎯 Publishers - 액추에이터에게 명령 전송
        ros::Publisher door_control_pub_;      // /elevator/door_control
        ros::Publisher elevator_control_pub_;  // /elevator/move_command
        
        // 🎯 Subscribers - 실제 피드백 받기
        ros::Subscriber robot_inside_sub_;     // /elevator/robot_inside
        ros::Subscriber current_floor_sub_;    // /elevator/current_floor
        ros::Subscriber door_status_sub_;      // /elevator/door_status
        ros::Subscriber elevator_status_sub_;  // /elevator/status
        ros::Subscriber robot_approach_sub_;   // /elevator/robot_approach

        ros::Subscriber door_vision_sub_; // /elevator_vision/door_status (YOLO)
        ros::Subscriber model_states_sub_; // /gazebo/model_states (백업용정의)
        
        // 🎯 Timers
        ros::Timer state_timer_;               // 상태 업데이트
        ros::Timer action_timer_;              // 액션 타임아웃
        ros::Timer statistics_timer_;          // 통계 출력
        
        // 🎯 상태 변수들 (실제 피드백 기반)
        State current_state_;
        bool robot_inside_;                    // 실제 로봇 탑승 상태
        bool robot_approaching_;               // 로봇 접근 상태
        int current_floor_;                    // 실제 현재 층 (0=1층, 1=2층)
        std::string door_status_;              // 실제 문 상태 ("open", "close", "moving")
        std::string elevator_status_;          // 실제 엘리베이터 상태 ("moving", "stopped")
        
        // 🎯 백업 상태 변수
        std::string door_state_; // YOLO 문 상태
        bool robot_near_;
        bool is_moving_;

        // 🎯 타이밍 및 통계
        ros::Time state_change_time_;
        ros::Time mission_start_time_;
        double action_timeout_;
        int mission_count_;
        double total_wait_time_;

        // 매개변수들
        double inside_threshold_; // 탑승 감지 거리
        double approach_threshold_; // 접근 감지 거리
        double boarding_timeout_;  // 탑승 대기 시간
        double floor_height_; // 층간 높이

        void initializeParameters() {
            nh_.param("action_timeout", action_timeout_, 30.0);
            nh_.param("inside_threshold", inside_threshold_, 1.2);
            nh_.param("approach_threshold", approach_threshold_, 3.0);
            nh_.param("boarding_timeout", boarding_timeout_, 15.0);
            nh_.param("floor_height", floor_height_, 3.075);

            // 초기 상태 설정
            robot_inside_ = false;
            robot_approaching_ = false;
            current_floor_ = 0;  // 1층에서 시작
            door_status_ = "close";
            elevator_status_ = "stopped";
            
            // 기존 호환 변수들
            door_state_ = "close";
            robot_near_ = false;
            is_moving_ = false;
            
            state_change_time_ = ros::Time::now();
            
            ROS_INFO("📋 Parameters initialized:");
            ROS_INFO("   Floor height: %.3fm", floor_height_);
            ROS_INFO("   Inside threshold: %.1fm", inside_threshold_);
            ROS_INFO("   Approach threshold: %.1fm", approach_threshold_);
        }

        void initializePublishers() {
            door_control_pub_ = nh_.advertise<std_msgs::String>("/elevator/door_control", 1);
            elevator_control_pub_ = nh_.advertise<std_msgs::Int32>("/elevator/move_command", 1);

            ROS_INFO("📡 Publishers initialized");
        }

        void initializeSubscribers() {
            // 새로운 피드백 시스템
            robot_inside_sub_ = nh_.subscribe("/elevator/robot_inside", 1, 
                                            &GazeboElevatorController::robotInsideCallback, this);
            current_floor_sub_ = nh_.subscribe("/elevator/current_floor", 1, 
                                            &GazeboElevatorController::currentFloorCallback, this);
            door_status_sub_ = nh_.subscribe("/elevator/door_status", 1, 
                                        &GazeboElevatorController::doorStatusCallback, this);
            elevator_status_sub_ = nh_.subscribe("/elevator/status", 1, 
                                            &GazeboElevatorController::elevatorStatusCallback, this);
            robot_approach_sub_ = nh_.subscribe("/elevator/robot_approach", 1, 
                                            &GazeboElevatorController::robotApproachCallback, this);
            
            // 기존 시스템과의 호환성
            door_vision_sub_ = nh_.subscribe("/elevator_vision/door_status", 1, 
                                        &GazeboElevatorController::doorVisionCallback, this);
            model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, 
                                            &GazeboElevatorController::modelStatesCallback, this);
            
            ROS_INFO("📡 Subscribers initialized");
        }

        void robotInsideCallback(const std_msgs::Bool::ConstPtr& msg) {
            if (robot_inside_ != msg->data) {
                robot_inside_ = msg->data;
                ROS_INFO("🤖 Robot %s elevator", robot_inside_ ? "ENTERED" : "EXITED");
            }
        }

        void currentFloorCallback(const std_msgs::Int32::ConstPtr& msg) {
            if (current_floor_ != msg->data) {
                current_floor_ = msg->data;
                ROS_INFO("🏢 Floor: %d (%s)", current_floor_, 
                   current_floor_ == 0 ? "1층" : "2층");
            }
        }

        void doorStatusCallback(const std_msgs::String::ConstPtr& msg) {
            if (door_status_ != msg->data) {
                door_status_ = msg->data;
                door_state_ = msg->data;
                ROS_INFO("🚪 Door status: %s", door_status_.c_str());
            }
        }

        void elevatorStatusCallback(const std_msgs::String::ConstPtr& msg) {
            if (elevator_status_ != msg->data) {
                elevator_status_ = msg->data;
                is_moving_ = (elevator_status_ == "moving");
                ROS_INFO("🛗 Elevator status: %s", elevator_status_.c_str());
            }
        }

        void robotApproachCallback(const std_msgs::Bool::ConstPtr& msg) {
            if (robot_approaching_ != msg->data) {
                robot_approaching_ = msg->data;
                robot_near_ = msg->data; // 기존 변수랑 동기화
                ROS_INFO("🤖 Robot %s elevator area", robot_approaching_ ? "APPROACHING" : "LEFT");
            }
        }

        void doorVisionCallback(const std_msgs::String::ConstPtr& msg) {
            if (door_state_ != msg->data) {
                door_state_ = msg->data;
                door_status_ = msg->data;
                ROS_INFO("🎥 YOLO Door: %s", door_state_.c_str());
            }
        }

        void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
            // 백업 시스템 - 액추에이터가 없을 때 사용
            static bool actuator_available = true;
            static int no_feedback_count = 0;
            
            // 피드백이 오지 않으면 직접 계산
            if (++no_feedback_count > 50) {  // 5초간 피드백 없음
                actuator_available = false;
                calculateStatesFromGazebo(msg);
            }
        }

        void calculateStatesFromGazebo(const gazebo_msgs::ModelStates::ConstPtr& msg) {
            geometry_msgs::Pose robot_pose, elevator_pose;
            bool robot_found = false, elevator_found = false;

            for (size_t i = 0; i < msg->name.size(); i++) {
                if (msg->name[i].find("robot") != std::string::npos || 
                msg->name[i].find("turtlebot") != std::string::npos) {
                    robot_pose = msg->pose[i];
                    robot_found = true;
                }
                else if (msg->name[i].find("elevator") != std::string::npos) {
                    elevator_pose = msg->pose[i];
                    elevator_found = true;
                }
            }

            if (robot_found && elevator_found) {
                double dx = robot_pose.position.x - elevator_pose.position.x;
                double dy = robot_pose.position.y - elevator_pose.position.y;
                double distance = sqrt(dx*dx + dy*dy);

                // 층 계산
                int new_floor = (elevator_pose.position.z < floor_height_ / 2) ? 0 : 1;
                if (current_floor_ != new_floor) {
                    current_floor_ = new_floor;
                    ROS_INFO("🏢 Floor (backup): %d", current_floor_);
                }

                // 로봇 상태 계산
                bool new_inside = (distance < inside_threshold_);
                bool new_approaching = (distance < approach_threshold_ && !new_inside);

                if (robot_inside_ != new_inside) {
                    robot_inside_ = new_inside;
                    ROS_INFO("🤖 Robot %s (backup)", robot_inside_ ? "ENTERED" : "EXITED");
                }

                if (robot_approaching_ != new_approaching) {
                    robot_approaching_ = new_approaching;
                    robot_near_ = new_approaching; // 기존 변수와 동기화함
                    ROS_INFO("🤖 Robot %s (backup)", robot_approaching_ ? "APPROACHING" : "LEFT");
                }
            }
        }

        void stateUpdateCallback(const ros::TimerEvent&) {
            switch (current_state_) {
                case IDLE_AT_1F:
                    handleIdleAt1F();
                    break;
                case DOOR_OPENING_1F:
                    handleDoorOpening1F();
                    break;
                case WAITING_FOR_BOARDING:
                    handleWaitingForBoarding();
                    break;
                case DOOR_CLOSING_1F:
                    handleDoorClosing1F();
                    break;
                case MOVING_TO_2F:
                    handleMovingTo2F();
                    break;
                case DOOR_OPENING_2F:
                    handleDoorOpening2F();
                    break;
                case WAITING_FOR_EXIT:
                    handleWaitingForExit();
                    break;
                case DOOR_CLOSING_2F:
                    handleDoorClosing2F();
                    break;
                case RETURNING_TO_1F:
                    handleReturningTo1F();
                    break;
                case ERROR_STATE:
                    handleErrorState();
                    break;
            }
        }

        // 상태별 핸들러들
        void handleIdleAt1F() {
            // 1층에서 로봇 접근 감지 시 자동으로 문 "open"
            if (robot_near_ && current_floor_ == 0 && door_state_ == "close") {
                ROS_INFO("🤖 Robot approaching! Opening door...");
                sendDoorCommand("open");
                changeState(DOOR_OPENING_1F);
                mission_start_time_ = ros::Time::now();
            }
        }

        void handleDoorOpening1F() {
            // YOLO가 문 열림을 확인할 때까지 대기함
            if (door_state_ == "open") {
                ROS_INFO("✅ Door opened! Waiting for robot boarding...");
                changeState(WAITING_FOR_BOARDING);
            }
        }

        void handleWaitingForBoarding() {
            // 로봇 탑승 감지 시 자동으로 문 닫고 출발
            if (robot_inside_) {
                ROS_INFO("🛗 Robot boarded! Closing door...");
                action_timer_.stop();
                sendDoorCommand("close");
                changeState(DOOR_CLOSING_1F);
            }
        }

        void handleDoorClosing1F() {
            // 문이 완전히 닫힌 후 2층으로 올라감
            if (door_state_ == "close") {
                ROS_INFO("🔺 Door closed! Moving to 2F...");
                moveElevatorTo(1);
                changeState(MOVING_TO_2F);
            }
        }

        void handleMovingTo2F() {
            // 2층 도착 확인
            if (current_floor_ == 1 && !is_moving_) {
                ROS_INFO("🔝 Arrived at 2F! Opening door...");
                sendDoorCommand("open");
                changeState(DOOR_OPENING_2F);
            }
        }

        void handleDoorOpening2F() {
            // 2층에서 문 열림 확인
            if (door_state_ ==  "open") {
                ROS_INFO("⬅️ Door opened at 2F! Waiting for robot exit...");
                changeState(WAITING_FOR_EXIT);
            }
        }

        void handleWaitingForExit() {
            // 로봇 하차 감지 후 자동으로 문 "close"
            if (!robot_inside_) {
                ROS_INFO("🔄 Robot exited! Closing door...");
                sendDoorCommand("close");
                changeState(DOOR_CLOSING_2F);
            }
            // 타임아웃 체크 추가함 - 너무 오래 기다릴때는 문 닫기
            else {
                double wait_time = (ros::Time::now() - state_change_time_).toSec();
                if (wait_time > boarding_timeout_) {
                    ROS_WARN("⏰ Exit timeout! Closing door...");
                    sendDoorCommand("close");
                    changeState(DOOR_CLOSING_2F);
                }
            }
        }

        void handleDoorClosing2F() {
            // 문이 완전히 닫힌 후 1층으로 복귀
            if (door_state_ == "close") {
                ROS_INFO("🔻 Door closed! Returning to 1F...");
                moveElevatorTo(0);
                changeState(RETURNING_TO_1F);
            }
        }

        void handleReturningTo1F() {
            // 1층 복귀 완료 확인
            if (current_floor_ == 0 && !is_moving_) {
                ROS_INFO("✅ Back to 1F! Mission completed!");
                changeState(IDLE_AT_1F);
                logMissionComplete();
            }
        }

        void handleErrorState() {
            ROS_ERROR("🚨 Error state! Attempting Recovery...");

            // 간단한 복구 로직
            if (current_floor_ == 0 ) {
                changeState(IDLE_AT_1F);
            } else {
                moveElevatorTo(0);
                changeState(RETURNING_TO_1F);
            }
        }

        // 유틸리티 함수
        void moveElevatorTo(int floor) {
            std_msgs::Int32 floor_cmd;
            floor_cmd.data = floor;
            elevator_control_pub_.publish(floor_cmd);
            
            ROS_INFO("🛗 Elevator move command sent: Floor %d", floor);
        }

        void sendDoorCommand(const std::string& command) {
            std_msgs::String door_cmd;
            door_cmd.data = command;
            door_control_pub_.publish(door_cmd);
            
            ROS_INFO("🚪 Door command sent: %s", command.c_str());
        }

        void changeState(State new_state) {
            if (current_state_ != new_state) {
                ROS_INFO("🔄 State change: %s → %s", 
                    stateToString(current_state_).c_str(), 
                    stateToString(new_state).c_str());
                
                current_state_ = new_state;
                state_change_time_ = ros::Time::now();
                
                // 타임아웃 타이머 재시작
                action_timer_.stop();
                action_timer_ = nh_.createTimer(ros::Duration(action_timeout_), 
                                            &GazeboElevatorController::timeoutCallback, this, true);
            }
        }

        std::string stateToString(State state) {
            switch (state) {
                case IDLE_AT_1F: return "IDLE_AT_1F";
                case DOOR_OPENING_1F: return "DOOR_OPENING_1F";
                case WAITING_FOR_BOARDING: return "WAITING_FOR_BOARDING";
                case DOOR_CLOSING_1F: return "DOOR_CLOSING_1F";
                case MOVING_TO_2F: return "MOVING_TO_2F";
                case DOOR_OPENING_2F: return "DOOR_OPENING_2F";
                case WAITING_FOR_EXIT: return "WAITING_FOR_EXIT";
                case DOOR_CLOSING_2F: return "DOOR_CLOSING_2F";
                case RETURNING_TO_1F: return "RETURNING_TO_1F";
                case ERROR_STATE: return "ERROR_STATE";
                default: return "UNKNOWN";
            }
        }

        // 🎯 타임아웃 및 에러 처리
        void checkTimeout() {
            double elapsed = (ros::Time::now() - state_change_time_).toSec();
            if (elapsed > action_timeout_) {
                ROS_WARN("⏰ State timeout! Current state: %s (%.1fs)", 
                    stateToString(current_state_).c_str(), elapsed);
                changeState(ERROR_STATE);
            }
        }

        void timeoutCallback(const ros::TimerEvent&) {
            ROS_WARN("⏰ Action timeout in state: %s", stateToString(current_state_).c_str());
            changeState(ERROR_STATE);
        }


        // 🎯 통계 및 로깅
        void logMissionComplete() {
            mission_count_++;
            double mission_time = (ros::Time::now() - mission_start_time_).toSec();
            total_wait_time_ += mission_time;
            
            ROS_INFO("📊 Mission #%d completed in %.1f seconds", mission_count_, mission_time);
            
            // 다음 미션 준비
            mission_start_time_ = ros::Time::now();
        }

        void printStatisticsCallback(const ros::TimerEvent&) {
            double avg_time = (mission_count_ > 0) ? (total_wait_time_ / mission_count_) : 0.0;
            
            ROS_INFO("📈 Statistics: %d missions, avg time: %.1fs, state: %s", 
                mission_count_, avg_time, stateToString(current_state_).c_str());
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_elevator_controller");
    
    GazeboElevatorController controller;
    
    ROS_INFO("🛗 Gazebo Elevator Controller running...");
    ROS_INFO("🎯 Compatible with both new actuator and legacy systems");
    ros::spin();
    
    return 0;
}
