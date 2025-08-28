#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <memory>
#include <cmath>
#include <string>
#include <iostream> 
#include <cstdlib> 
#include <chrono>
#include <iomanip>
#include <Eigen/Geometry>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>


class SportControlNode : public rclcpp::Node
{
public:
    SportControlNode(const rclcpp::NodeOptions &options)
    : Node("sport_control_node", options)
    {
        this->get_parameter_or<std::string>("network_interface", network_interface_, std::string("enxf8e43b808e06"));
        this->get_parameter_or<std::string>("sport_cmd_topic",   sport_cmd_topic_,   std::string("NoYamlRead/SportCmd"));
        this->get_parameter_or<std::string>("guide_topic",       guide_topic_,       std::string("/cmd_vel"));
        this->get_parameter_or<std::string>("map_frame_id",      map_frame_id_,      std::string("map"));

        // 初始化Unitree通道工厂
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

        // 初始化句子
        Last_Operation = "Sport control init";

        // 创建订阅者，订阅/SportCmd话题
        sport_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sport_cmd_topic_, 10, std::bind(&SportControlNode::sport_cmd_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅导航话题
        guide_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            guide_topic_, 10, std::bind(&SportControlNode::guide_callback, this, std::placeholders::_1));

        sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        sport_client->SetTimeout(10.0f);
        sport_client->Init();

        Vui_client = std::make_unique<unitree::robot::go2::VuiClient>();
        Vui_client->SetTimeout(1.0f); 
        Vui_client->Init();

        Avoid_client = std::make_unique<unitree::robot::go2::ObstaclesAvoidClient>();
        Avoid_client->SetTimeout(1.0f); 
        Avoid_client->Init();
        Avoid_client->SwitchSet(1);
        Avoid_client->SwitchGet(ObstaclesAvoidEnable);
        Last_Operation = Last_Operation + "; ObstaclesAvoidEnable: " + std::to_string(ObstaclesAvoidEnable);
        Last_Operation_Time = this->now();

        CurrentErrorCode = sport_client->ClassicWalk(1);
        sport_client->AutoRecoverSet(1);
        sport_client->AutoRecoverGet(AutoRecoverEnable);

        Last_Operation = Last_Operation + "; SpeedLevel 1";
        Last_Operation = Last_Operation + "; ClassicWalk";
        Last_Operation = Last_Operation + "; AutoRecoverEnable: " + std::to_string(AutoRecoverEnable);
        Last_Operation_Time = this->now();

        RCLCPP_INFO(this->get_logger(), "SportControlNode 已启动");
        state_print();
    }

private:

    std::string network_interface_;
    std::string sport_cmd_topic_;
    std::string guide_topic_;
    std::string map_frame_id_;

    std::unique_ptr<unitree::robot::go2::SportClient> sport_client;
    std::unique_ptr<unitree::robot::go2::VuiClient> Vui_client;
    std::unique_ptr<unitree::robot::go2::ObstaclesAvoidClient> Avoid_client;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sport_cmd_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr guide_sub_;
    double guide_x_vel = 0, guide_y_vel = 0, guide_yaw_vel = 0;
    struct Waypoint { double x, y, yaw_deg; };
    std::vector<Waypoint> patrol_points_{{16.3, -0.85, 0.0}, {23.8, 6.81, 1.57}, {27.1, 96.3, 2.2}, {-10.0, 98.0, -1.57}, {27.1, 96.3, -1.57}, {23.8, 6.81, -1.57}}; 
    size_t patrol_idx_  = 0;   // 当前巡逻点索引
    bool   patrol_enable_ = false;  // true 表示正在巡逻

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;

    int32_t CurrentErrorCode = 0;
    int32_t LastErrorCode = 0;

    bool ObstaclesAvoidEnable = 0;
    bool AutoRecoverEnable = 1;
    bool ForwardClimbingEnable = 0;
    bool ContinuousGaitEnable = 0;
    bool MotionFlag = 0;
    float MotionEnable = 0;
    float SpeedScalse = 0.50;
    std::string ModeCheckForm = "unknown";
    std::string ModeCheckMode = "unknown";

    std::string Last_Operation, Last_Motion;
    rclcpp::Time Last_Operation_Time;
    float Last_Operation_Duration_Time;

    // 回调函数，处理SportCmd消息
    void sport_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        int action = static_cast<int>(msg->data[0]);
        Actions(action, msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
    }

    // 回调函数，处理导航消息
    void guide_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        guide_x_vel = msg->linear.x;
        guide_y_vel = msg->linear.y;
        guide_yaw_vel = msg->angular.z;
        Actions(25202123,guide_y_vel,guide_x_vel,guide_yaw_vel,0);
    }
    void sendGoal(double x, double y, double yaw_degs)
    {
        if (!nav_to_pose_client) {
            nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                this->shared_from_this(), "navigate_to_pose");
        }
        
        // 等待服务器就绪
        if (!nav_to_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 1. 准备目标消息d
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = map_frame_id_;
        goal_msg.pose.header.stamp = now();

        // (1) 位置
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        // (2) 朝向：将 yaw(度) 转换成四元数（使用 Eigen）
        double yaw_radians = yaw_degs * M_PI / 180.0; // 若本来就是弧度，则直接用
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_radians, Eigen::Vector3d::UnitZ()));

        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        RCLCPP_INFO(
            this->get_logger(),
            "Sending goal to (%.2f, %.2f) with yaw=%.2f deg",
            x, y, yaw_degs
    );
    // 2. 定义发送目标时的回调选项（可选：结果回调、反馈回调等）
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
            default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        if (patrol_enable_) this->loopGoal();
        };
    // 3. 异步发送目标
    nav_to_pose_client->async_send_goal(goal_msg, send_goal_options);
    }

    void startPatrol()
    {
        if (patrol_enable_) return;           // 已在巡逻，无需重复启动
        patrol_enable_ = true;
        patrol_idx_    = 0;                   // 从第 0 个点开始
        RCLCPP_INFO(get_logger(), "Patrol START -> 共 %zu 个点", patrol_points_.size());
        loopGoal();                           // 立刻发送首个 Goal
    }

    void stopPatrol()
    {
        patrol_enable_ = false;
        RCLCPP_INFO(get_logger(), "Patrol STOP");
    }

    // ===== 核心：loopGoal() 递推发送下一目标 =====
    void loopGoal()
    {
        if (!patrol_enable_ || patrol_points_.empty()) return;

        const auto & wp = patrol_points_[patrol_idx_];
        patrol_idx_ = (patrol_idx_ + 1) % patrol_points_.size();
        sendGoal(wp.x, wp.y, wp.yaw_deg);
    }

    void state_print()
    {
        // 打印时间差及上次动作
        rclcpp::Duration elapsed_time = this->get_clock()->now() - Last_Operation_Time;
        Last_Operation_Duration_Time = elapsed_time.seconds();
        std::cout << "Current Error Code: " << CurrentErrorCode << "; Last Error Code: " << LastErrorCode << std::endl;
        std::cout << "Last Operation: " << Last_Operation << "   " << Last_Operation_Duration_Time << "s passed." << std::endl;
        std::cout << "Last Motion: " << Last_Motion << "   " << Last_Operation_Duration_Time << "s passed." << std::endl;
        
        if (CurrentErrorCode)
            LastErrorCode = CurrentErrorCode;
    }

    void Actions(int Action, double Value1, double Value2, double Value3, double Value4)
    {
        // Action命名规则： 00 00 00 00 00，代表最多同时按下的5个键
        // 每两位AB，A 1/2表示按键/摇杆，B 0～7表示不同的按键名/摇杆名
        switch (Action) {
            case 16000000:
                Last_Operation = "Pause. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->Damp();
                break;
            case 16170000:
                Last_Operation = "Stop Move. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->StopMove();
                break;
            case 25202123:
                if(Value1 || Value2 || Value3 || Value4 ||MotionFlag)
                {
                    float Leftward_Speed = 1 * SpeedScalse * Value1;
                    float Forward_Speed = 2.5 * SpeedScalse * Value2 + 2.5 * SpeedScalse * Value4;
                    float Turning_Speed = 4 * SpeedScalse * Value3;
        
                    if(Forward_Speed>=0)
                        Forward_Speed = Forward_Speed / 2.5 * 3.8;
        
                    Last_Motion = "Moving to Forward: " + std::to_string(Forward_Speed) 
                    + "; Leftward: " + std::to_string(Leftward_Speed)
                    + "; Turning: " + std::to_string(Turning_Speed);
        
                    Last_Operation_Time = this->get_clock()->now();
        
                    std::cout << "Moving... " << std::endl;
        
                    CurrentErrorCode = sport_client->Move(Forward_Speed, Leftward_Speed, Turning_Speed);

                    if(Value1||Value2||Value3)
                        MotionFlag = 1;
                    else
                        MotionFlag = 0;
                }
                break;
            case 25262700:
                CurrentErrorCode = sport_client->SpeedLevel(1);
                if(Value2==-1)
                {
                    Last_Operation = "Speed Scale 25%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.25;
                }
                else if(Value1==1)
                {
                    Last_Operation = "Speed Scale 50%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.5;
                }
                else if(Value1==-1)
                {
                    Last_Operation = "Speed Scale 75%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.75;
                }
                else if(Value2==1)
                {
                    Last_Operation = "Speed Scale 100%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 1.0;
                }
                break;
            case 25100000:
                Last_Operation = "Stand up. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->RecoveryStand();
                break;
            case 25110000:
                Last_Operation = "Sit Down. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->StandDown();
                break;
            case 25120000:
                Last_Operation = "Classic Walk and Obstacle Avoid Start. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->ClassicWalk(1);
                Avoid_client->SwitchSet(1);
                break;
            case 25130000:
                Last_Operation = "Agile Walk and Obstacle Avoid Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                CurrentErrorCode = sport_client->ClassicWalk(0);
                Avoid_client->SwitchSet(0);
                break;
            case 25140000:
                Last_Operation = "Reset Estimator Position. ";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 25170000:
                Last_Operation = "Vacant command. ";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22202100:
                {
                    Last_Motion = "Gimbal Yaw Vel: " + std::to_string(Value1)
                        + "; Gimbal Pitch Vel: " + std::to_string(Value2);
                    Last_Operation_Time = this->get_clock()->now();
                    break;
                }
            case 22232400:
                if(Value1 || Value2)
                {
                    float YawAngle = 0.6 * Value1;
                    float PitchAngle = -0.75 * Value2;
        
                    Last_Motion = "PitchAngle: " + std::to_string(PitchAngle)
                    + "; YawAngle: " + std::to_string(YawAngle);
        
                    Last_Operation_Time = this->get_clock()->now();
                    CurrentErrorCode = sport_client->Euler(0.0, PitchAngle, YawAngle);
                }
                break;
            case 22100000:
                Last_Operation = "IMU Follow Mode Enable.";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22110000:
                Last_Operation = "Auto Track Mode Enable.";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22120000:
                Last_Operation = "Auto Motion Mode Enable.";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22130000:
                Last_Operation = "All Stop and Reset. ";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22170000:
                Last_Operation = "Talking. ";
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 22262700:
                if(Value1==1)
                {
                    Last_Operation = "Start Patrol. ";
                    CurrentErrorCode = Vui_client->SetBrightness(3);
                    startPatrol();
                }
                else if(Value1==-1)
                {
                    Last_Operation = "Go to Warehouse. ";
                    CurrentErrorCode = Vui_client->SetBrightness(0);
                    sendGoal(20, 1, 0);
                }
                else if(Value2==1)
                {
                    Last_Operation = "Stop Patrol. ";
                    CurrentErrorCode = Vui_client->SetBrightness(0);
                    stopPatrol();
                }
                else if(Value2==-1)
                {
                    Last_Operation = "Go To Start Point. ";
                    CurrentErrorCode = Vui_client->SetBrightness(0);
                    sendGoal(16.3, -0.85, 0);
                }
                Last_Operation_Time = this->get_clock()->now();
                break;
            default:
                Last_Operation = "Unknown Command. ";
                break;
        }
        state_print();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .arguments({
        "--ros-args",
        "--params-file", "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Go2Base/config.yaml"
        });
    auto node = std::make_shared<SportControlNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}