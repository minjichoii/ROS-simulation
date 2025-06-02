#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class GazeboElevatorController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers - 상태 정보 발행
    ros::Publisher current_floor_pub_;
    ros::Publisher door_state_pub_;
    ros::Publisher is_moving_pub_;
    ros::Publisher robot_inside_pub_;
    
    // Publisher - Gazebo 엘리베이터 제어 (핵심!)
    ros::Publisher elevator_cmd_pub_;
    
    // Subscribers
    ros::Subscriber model_states_sub_;
    
    // Service servers
    ros::ServiceServer call_elevator_service_;
    ros::ServiceServer goto_floor_service_;
    
    // Timer
    ros::Timer control_timer_;
    ros::Timer door_timer_;
    
    // 실제 환경 기반 파라미터
    double floor_height_;           // 3.075 (실제 측정값)
    double elevator_x_;             // 엘리베이터 X 위치
    double elevator_y_;             // 엘리베이터 Y 위치
    double inside_threshold_;       // 탑승 판단 거리
    double approach_threshold_;     // 접근 판단 거리
    
    // State variables
    int current_floor_;             // 0=1층, 1=2층, -1=이동중
    std::string door_state_;        // "OPENED", "CLOSED", "OPENING", "CLOSING"
    bool is_moving_;
    bool robot_inside_;
    bool robot_near_elevator_;
    
    // 위치 정보
    geometry_msgs::Pose elevator_pose_;
    geometry_msgs::Pose robot_pose_;
    geometry_msgs::Twist elevator_velocity_;
    
    // 자동화 상태머신
    enum ElevatorState {
        IDLE,                       // 대기 상태
        WAITING_FOR_ROBOT,          // 로봇 접근 대기  
        DOOR_OPENING,               // 문 여는 중
        WAITING_FOR_BOARDING,       // 탑승 대기
        DOOR_CLOSING,               // 문 닫는 중
        MOVING_TO_FLOOR,            // 층간 이동
        ARRIVED_AT_DESTINATION      // 도착 완료
    };
    ElevatorState elevator_state_;

public:
    GazeboElevatorController() : private_nh_("~"), current_floor_(0), is_moving_(false), 
                                robot_inside_(false), robot_near_elevator_(false),
                                door_state_("CLOSED"), elevator_state_(IDLE)
    {
        initializeParameters();
        initializePublishers();
        initializeSubscribers();
        initializeServices();
        
        // 제어 루프 타이머 (10Hz)
        control_timer_ = nh_.createTimer(ros::Duration(0.1), &GazeboElevatorController::controlLoop, this);
        
        ROS_INFO("Gazebo Elevator Controller initialized for actual world");
    }
    
private:
    void initializeParameters()
    {
        // 실제 월드 파일 기반 파라미터
        private_nh_.param<double>("floor_height", floor_height_, 3.075);
        private_nh_.param<double>("elevator_x", elevator_x_, 0.0);
        private_nh_.param<double>("elevator_y", elevator_y_, 0.0);
        private_nh_.param<double>("inside_threshold", inside_threshold_, 1.125);  // 엘리베이터 내부 크기의 절반
        private_nh_.param<double>("approach_threshold", approach_threshold_, 2.0);
        
        ROS_INFO("Floor height: %.3f", floor_height_);
        ROS_INFO("Elevator position: (%.1f, %.1f)", elevator_x_, elevator_y_);
    }
    
    void initializePublishers()
    {
        // 상태 정보 발행
        current_floor_pub_ = nh_.advertise<std_msgs::Int32>("/elevator_sim/current_floor", 1);
        door_state_pub_ = nh_.advertise<std_msgs::String>("/elevator_sim/door_state", 1);
        is_moving_pub_ = nh_.advertise<std_msgs::Bool>("/elevator_sim/is_moving", 1);
        robot_inside_pub_ = nh_.advertise<std_msgs::Bool>("/elevator_sim/robot_inside", 1);
        
        // 💥 핵심! Gazebo 플러그인 제어용
        elevator_cmd_pub_ = nh_.advertise<std_msgs::Int32>("/gazebo/default/elevator", 1);
    }
    
    void initializeSubscribers()
    {
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, 
                                        &GazeboElevatorController::modelStatesCallback, this);
    }
    
    void initializeServices()
    {
        call_elevator_service_ = nh_.advertiseService("/elevator_control/call_elevator", 
                                                    &GazeboElevatorController::callElevatorService, this);
        goto_floor_service_ = nh_.advertiseService("/elevator_control/goto_floor", 
                                                 &GazeboElevatorController::gotoFloorService, this);
    }
    
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // 엘리베이터와 로봇 모델 찾기
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            // 💥 실제 모델명에 맞춰 수정 필요
            if (msg->name[i] == "elevator" || msg->name[i].find("elevator") != std::string::npos)
            {
                elevator_pose_ = msg->pose[i];
                if (i < msg->twist.size()) {
                    elevator_velocity_ = msg->twist[i];
                }
            }
            // 로봇 모델명 (실제 환경에 맞춰 수정)
            else if (msg->name[i].find("turtlebot3") != std::string::npos || 
                     msg->name[i].find("robot") != std::string::npos ||
                     msg->name[i].find("trixy") != std::string::npos)
            {
                robot_pose_ = msg->pose[i];
            }
        }
        
        updateElevatorStates();
        publishStates();
    }
    
    void updateElevatorStates()
    {
        // 1. 현재 층 계산 (실제 높이 기반)
        double z_pos = elevator_pose_.position.z;
        current_floor_ = calculateCurrentFloor(z_pos);
        
        // 2. 이동 상태 판단
        double velocity_threshold = 0.05;
        is_moving_ = (fabs(elevator_velocity_.linear.z) > velocity_threshold);
        
        // 3. 로봇 위치 계산
        robot_inside_ = calculateRobotInside();
        robot_near_elevator_ = calculateRobotNearElevator();
    }
    
    int calculateCurrentFloor(double z_position)
    {
        // 실제 층 높이 기반 계산
        if (z_position < floor_height_ / 2) {
            return 0;  // 1층
        }
        else if (z_position > floor_height_ * 1.5) {
            return 1;  // 2층
        }
        else {
            return -1; // 이동 중
        }
    }
    
    bool calculateRobotInside()
    {
        double dx = robot_pose_.position.x - elevator_pose_.position.x;
        double dy = robot_pose_.position.y - elevator_pose_.position.y;
        double dz = robot_pose_.position.z - elevator_pose_.position.z;
        
        // 엘리베이터 내부 영역 체크 (2.25 x 2.25 크기)
        return (fabs(dx) < inside_threshold_ && fabs(dy) < inside_threshold_ && fabs(dz) < 1.0);
    }
    
    bool calculateRobotNearElevator()
    {
        // 1층에서만 접근 감지
        if (current_floor_ != 0) return false;
        
        double dx = robot_pose_.position.x - elevator_pose_.position.x;
        double dy = robot_pose_.position.y - elevator_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        return (distance < approach_threshold_ && !robot_inside_);
    }
    
    void controlLoop(const ros::TimerEvent& event)
    {
        // 🎯 자동화 상태머신 실행
        switch (elevator_state_)
        {
            case IDLE:
                if (robot_near_elevator_ && current_floor_ == 0) {
                    ROS_INFO("🤖 Robot detected! Opening door...");
                    openDoor();
                    elevator_state_ = DOOR_OPENING;
                }
                break;
                
            case DOOR_OPENING:
                if (door_state_ == "OPENED") {
                    ROS_INFO("🚪 Door opened! Waiting for boarding...");
                    elevator_state_ = WAITING_FOR_BOARDING;
                    startBoardingTimer();
                }
                break;
                
            case WAITING_FOR_BOARDING:
                if (robot_inside_) {
                    ROS_INFO("🛗 Robot boarded! Closing door...");
                    closeDoor();
                    elevator_state_ = DOOR_CLOSING;
                    stopBoardingTimer();
                }
                break;
                
            case DOOR_CLOSING:
                if (door_state_ == "CLOSED") {
                    ROS_INFO("⬆️ Moving to 2nd floor...");
                    gotoFloor(1);  // 2층으로 이동
                    elevator_state_ = MOVING_TO_FLOOR;
                }
                break;
                
            case MOVING_TO_FLOOR:
                if (!is_moving_ && current_floor_ == 1) {
                    ROS_INFO("🎯 Arrived at 2nd floor! Opening door...");
                    openDoor();
                    elevator_state_ = ARRIVED_AT_DESTINATION;
                }
                break;
                
            case ARRIVED_AT_DESTINATION:
                if (!robot_inside_ && door_state_ == "OPENED") {
                    ROS_INFO("👋 Robot exited! Returning to idle...");
                    closeDoor();
                    // 3초 후 IDLE로 복귀
                    door_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                                &GazeboElevatorController::returnToIdleCallback, this, true);
                }
                break;
        }
    }
    
    void openDoor()
    {
        door_state_ = "OPENING";
        // 2초 후 문 열림 완료
        door_timer_ = nh_.createTimer(ros::Duration(2.0), 
                                    &GazeboElevatorController::doorOpenedCallback, this, true);
    }
    
    void closeDoor()
    {
        door_state_ = "CLOSING";
        // 2초 후 문 닫힘 완료
        door_timer_ = nh_.createTimer(ros::Duration(2.0), 
                                    &GazeboElevatorController::doorClosedCallback, this, true);
    }
    
    void gotoFloor(int floor)
    {
        std_msgs::Int32 cmd;
        cmd.data = floor;
        elevator_cmd_pub_.publish(cmd);
        ROS_INFO("💨 Elevator command sent: floor %d", floor);
    }
    
    void startBoardingTimer()
    {
        // 5초 탑승 타임아웃
        door_timer_ = nh_.createTimer(ros::Duration(5.0), 
                                    &GazeboElevatorController::boardingTimeoutCallback, this, true);
    }
    
    void stopBoardingTimer()
    {
        door_timer_.stop();
    }
    
    // 타이머 콜백들
    void doorOpenedCallback(const ros::TimerEvent& event)
    {
        door_state_ = "OPENED";
        ROS_INFO("✅ Door fully opened");
    }
    
    void doorClosedCallback(const ros::TimerEvent& event)
    {
        door_state_ = "CLOSED";
        ROS_INFO("✅ Door fully closed");
    }
    
    void boardingTimeoutCallback(const ros::TimerEvent& event)
    {
        if (elevator_state_ == WAITING_FOR_BOARDING) {
            ROS_WARN("⏰ Boarding timeout! Closing door...");
            closeDoor();
            elevator_state_ = DOOR_CLOSING;
        }
    }
    
    void returnToIdleCallback(const ros::TimerEvent& event)
    {
        elevator_state_ = IDLE;
        ROS_INFO("💤 Elevator returned to IDLE state");
    }
    
    void publishStates()
    {
        // 현재 층
        std_msgs::Int32 floor_msg;
        floor_msg.data = current_floor_;
        current_floor_pub_.publish(floor_msg);
        
        // 문 상태
        std_msgs::String door_msg;
        door_msg.data = door_state_;
        door_state_pub_.publish(door_msg);
        
        // 이동 상태
        std_msgs::Bool moving_msg;
        moving_msg.data = is_moving_;
        is_moving_pub_.publish(moving_msg);
        
        // 로봇 탑승 상태
        std_msgs::Bool inside_msg;
        inside_msg.data = robot_inside_;
        robot_inside_pub_.publish(inside_msg);
    }
    
    bool callElevatorService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("📞 Elevator called via service");
        gotoFloor(0);  // 1층 호출
        return true;
    }
    
    bool gotoFloorService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("🎯 Go to floor service called");
        gotoFloor(1);  // 2층 이동
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_elevator_controller");
    
    try
    {
        GazeboElevatorController controller;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("❌ Exception in elevator controller: %s", e.what());
        return 1;
    }
    
    return 0;
}