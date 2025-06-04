#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>


class RobotElevatorController {
    public:
        ros::NodeHandle nh_; // ros 노드 핸들 (python의 rospy.init_node)
    private:
        // subscriber (토픽 받는 애들)
        ros::Subscriber door_state_sub_; // YOLO 문 상태
        ros::Subscriber current_floor_sub_; // 현재 층
        ros::Subscriber robot_inside_sub_; // 로봇이 내부에 있는지 여부

        // publisher (토픽 보내는 애)
        ros::Publisher cmd_vel_pub_; // 로봇 제어 명령 발행

        // member variables
        std::string door_state_; 
        int current_floor_; 
        bool robot_inside_;
        bool is_moving_;

        // 중복 방지 위한 변수
        ros::Time last_action_time_; // 마지막 액션 시간

        // 타이머
        ros::Timer stop_timer_;

    public:
        RobotElevatorController() {
            door_state_ = "close";
            current_floor_ = 0;
            robot_inside_ = false;
            is_moving_ = false;
            last_action_time_ = ros::Time::now();

            // 토픽 구독 설정
            door_state_sub_ = nh_.subscribe(
                "/elevator_door_state",
                1, // 큐 크기
                &RobotElevatorController::doorCallback,
                this
            );

            current_floor_sub_ = nh_.subscribe(
                "/elevator_sim/current_floor",
                1,
                &RobotElevatorController::floorCallback,
                this
            );

            robot_inside_sub_ = nh_.subscribe(
                "/elevator_sim/robot_inside",
                1,
                &RobotElevatorController::insideCallback,
                this
            );

            // 토픽 발행 설정
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            ROS_INFO("🤖 Robot Elevator Controller started!");
            ROS_INFO("🔍️ YOLO detects: 'open' (fully open) / 'close' (fully close)");
        }

        void doorCallback(const std_msgs::String::ConstPtr& msg) {
            std::string prev_door_state = door_state_;
            door_state_ = msg->data; // msg의 data 필드 접근
            
            // 문 상태 변화 시에만 로그 출력
            if (prev_door_state != door_state_) {
                ROS_INFO("Door state: %s", door_state_.c_str());
            }

            if (door_state_ == "open" && !is_moving_) {
                decideAction();
            } else if (door_state_ == "open" && is_moving_) {
                ROS_INFO("Door opened but robot is moving - ignore!");
            }
        }

        void floorCallback(const std_msgs::Int32::ConstPtr& msg) {
            if (current_floor_ != msg->data) {
                current_floor_ = msg->data;
                ROS_INFO("🏢 Floor: %d (%s)", current_floor_, 
                    current_floor_ == 0 ? "1층" : "2층");

                if (door_state_ == "open" && !is_moving_) {
                    ROS_INFO("🔄 Floor changed while door open - rechecking conditions");
                    decideAction();
                }
            }
        }

        void insideCallback(const std_msgs::Bool::ConstPtr& msg) {
            if (robot_inside_ != msg->data) {
                robot_inside_ = msg->data;
                ROS_INFO("📍 Robot: %s", robot_inside_ ? "INSIDE" : "OUTSIDE");
                
                // 로봇 위치가 바뀌었고 문이 열려있고 이동중이 아니면 조건 체크
                if (door_state_ == "open" && !is_moving_) {
                    ROS_INFO("🔄 Robot position changed while door open - rechecking conditions");
                    decideAction();
                }
            }
        }


        // 사용자 정의 함수들

        // 액션 결정
        void decideAction() {
            ROS_INFO("🔍️ Door is open! Checking conditions...");
            printCurrentConditions();

            if (!isBasicSafetyOK()) {
                return;
            }

            // 진입 조건: 1층 + 밖 + 문열림
            if (shouldEnterElevator()) {
                ROS_INFO("✅ ENTERING elevator");
                enterElevator();
            }
            // 하차 조건: 2층 + 안 + 문열림
            else if (shouldExitElevator()) {
                ROS_INFO("✅ EXITING elevator");
                exitElevator();
            }
            else {
                ROS_INFO("❌ No action needed");
                explainCurrentSituation();
            }
        }

        // 기본 안전 조건 정의
        bool isBasicSafetyOK() {
            // 너무 자주 액션하지 않게함
            double time_since_last = (ros::Time::now() = last_action_time_).toSec();
            if (time_since_last < 2.0) {
                ROS_INFO("⚠️ Too soon after last action ($.1fs ago)", time_since_last);
                return false;
            }

            return true;
        }

        // 엘리베이터 진입 가능 조건 
        bool shouldEnterElevator() {
            bool floor_ok = (current_floor_ == 0); // 1층
            bool position_ok = (!robot_inside_); // 밖
            bool door_ok = (door_state_ == "open"); // 문 활짝 열림
            bool moving_ok = (!is_moving_); // 이동중 아님

            ROS_INFO("🔍 Enter check: Floor(1F):%s, Outside:%s, DoorOpen:%s, NotMoving:%s",
                floor_ok ? "✅" : "❌",
                position_ok ? "✅" : "❌", 
                door_ok ? "✅" : "❌",
                moving_ok ? "✅" : "❌");

            return floor_ok && position_ok && door_ok && moving_ok;
        }

        // 엘리베이터 퇴장 조건
        bool shouldExitElevator() {
            bool floor_ok = (current_floor_ == 1); // 2층
            bool position_ok = (robot_inside_); // 안
            bool door_ok = (door_state_ == "open");
            bool moving_ok = (!is_moving_); 

            ROS_INFO("🔍 Exit check: Floor(2F):%s, Inside:%s, DoorOpen:%s, NotMoving:%s",
                floor_ok ? "✅" : "❌",
                position_ok ? "✅" : "❌",
                door_ok ? "✅" : "❌",
                moving_ok ? "✅" : "❌");
        
        return floor_ok && position_ok && door_ok && moving_ok;
        }

        void explainCurrentSituation() {
            if (current_floor_ == 0 && robot_inside_) {
                ROS_INFO("🤔 Robot is inside elevator at 1F (unusual state)");
            }
            else if (current_floor_ == 1 && !robot_inside_) {
                ROS_INFO("🤔 Robot is outside elevator at 2F (unusual state)");
            }
            else if (current_floor_ == 0 && !robot_inside_) {
                ROS_INFO("😎 Robot ready to enter at 1F (waiting for right moment)");
            }
            else if (current_floor_ == 1 && robot_inside_) {
                ROS_INFO("😎 Robot ready to exit at 2F (waiting for right moment)");
            }
        }

        // 엘리베이터 진입
        void enterElevator() {
            ROS_INFO("➡️ ENTERING elevator...");
            is_moving_ = true;
            last_action_time_ = ros::Time::now();
        
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.2;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = 0.0;
            
            cmd_vel_pub_.publish(cmd);
            
            // 1초 후 정지
            stop_timer_ = nh_.createTimer(ros::Duration(1.0), 
                &RobotElevatorController::stopCallback, this, true);
        }

        // 엘리베이터 하차 함수
        void exitElevator() {
            ROS_INFO("🚶⬅️ EXITING elevator...");
            is_moving_ = true;
            last_action_time_ = ros::Time::now();
            
            // 적당한 속도로 전진 (밖으로)
            geometry_msgs::Twist cmd;
            cmd.linear.x = -0.2; // 후진해서 나가기
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = 0.0;
            
            cmd_vel_pub_.publish(cmd);
            
            // 1.5초 후 정지 (더 멀리 나가기)
            stop_timer_ = nh_.createTimer(ros::Duration(1.5), 
                &RobotElevatorController::stopCallback, this, true);
        }

        void stopCallback(const ros::TimerEvent& event) {
             stopRobot();
        
            if (current_floor_ == 0) {
                ROS_INFO("✅ ENTER completed at 1F");
                ROS_INFO("🔮 Elevator should detect robot and close door automatically");
            } else if (current_floor_ == 1) {
                ROS_INFO("✅ EXIT completed at 2F");
                ROS_INFO("🔮 Elevator should detect robot exit and close door automatically");
            }
        }

        void stopRobot() {
            geometry_msgs::Twist stop_cmd;  // 모든 값이 기본적으로 0
            cmd_vel_pub_.publish(stop_cmd);
            is_moving_ = false;
            ROS_INFO("🛑 Robot stopped");
        }

        void printCurrentConditions() {
            ROS_INFO("📊 Current state:");
            ROS_INFO("   🚪 Door: %s", door_state_.c_str());
            ROS_INFO("   🏢 Floor: %d (%s)", current_floor_, current_floor_ == 0 ? "1층" : "2층");
            ROS_INFO("   📍 Robot: %s", robot_inside_ ? "INSIDE" : "OUTSIDE");
            ROS_INFO("   🚶 Moving: %s", is_moving_ ? "YES" : "NO");
        }

        // 디버깅/테스트용 함수들
        void printStatus() {
            ROS_INFO("🔄 Status - Door: %s, Floor: %d, Inside: %s, Moving: %s",
                    door_state_.c_str(), 
                    current_floor_,
                    robot_inside_ ? "YES" : "NO",
                    is_moving_ ? "YES" : "NO"
            );
        }

        // 수동 테스트용
        void manualEnter() {
            ROS_INFO("🧪 Manual test: Enter");
            if (isBasicSafetyOK()) {
                enterElevator();
            }
        }
        
        void manualExit() {
            ROS_INFO("🧪 Manual test: Exit");
            if (isBasicSafetyOK()) {
                exitElevator();
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_elevator_controller");
    
    RobotElevatorController controller;
    
    ROS_INFO("🎉 Robot Elevator Controller ready!");
    ROS_INFO("💡 Complete conditions:");
    ROS_INFO("   ENTER: 1F + OUTSIDE + DOOR_OPEN + NOT_MOVING");
    ROS_INFO("   EXIT:  2F + INSIDE  + DOOR_OPEN + NOT_MOVING");
    ROS_INFO("🤖 Robot will act when YOLO detects 'open' door");
    
    // 15초마다 상태 출력
    ros::Timer status_timer = controller.nh_.createTimer(ros::Duration(15.0), 
        [&controller](const ros::TimerEvent&) {
            controller.printStatus();
        });
    
    ros::spin();
    return 0;
}

