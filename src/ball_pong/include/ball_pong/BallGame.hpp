#include "rclcpp/rclcpp.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include "turtlesim/msg/pose.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "bits/stdc++.h"

using Kill = turtlesim::srv::Kill;
using Spawn = turtlesim::srv::Spawn;
using SetPen = turtlesim::srv::SetPen;
using Pose = turtlesim::msg::Pose;
using TeleportAbs = turtlesim::srv::TeleportAbsolute;

using namespace std::chrono_literals;

constexpr double deg2rad(double angle)
{
    return angle * M_PI / 180.0;
}

enum class Dir
{
    STOP = 0,
    UP_LEFT,
    DOWN_LEFT,
    UP_RIGHT,
    DOWN_RIGHT
};

class BallGame : public rclcpp::Node
{
private:
    // 初始化调用服务
    rclcpp::Client<Kill>::SharedPtr kill_turtle1_;
    rclcpp::Client<Spawn>::SharedPtr init_turtle_;
    rclcpp::Client<SetPen>::SharedPtr set_pen_left;
    rclcpp::Client<SetPen>::SharedPtr set_pen_right;
    rclcpp::Client<SetPen>::SharedPtr set_pen_ball;


    // 获取球拍与球的位置信息
    rclcpp::Subscription<Pose>::SharedPtr pose_left;
    rclcpp::Subscription<Pose>::SharedPtr pose_right;
    rclcpp::Subscription<Pose>::SharedPtr pose_ball;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    rclcpp::Client<TeleportAbs>::SharedPtr tele_abs_;

    // 定时运行函数，让球动起来
    rclcpp::TimerBase::SharedPtr ball_move_;

    // 存储球的位置信息
    Pose::SharedPtr pose_ball_info;
    Pose::SharedPtr pose_left_info;
    Pose::SharedPtr pose_right_info;

    // 球拍半宽半高，用于判断是否碰撞
    const double delta_x{0.2};
    const double delta_y{0.5};

    // 最大反弹角
    const double max_bounce_angle{deg2rad(70.0)}; // 70°

    // 方向，六向制
    Dir direction_{Dir::STOP};

public:
    BallGame() : Node("ball_game")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "start init");
        kill_turtle1_ = this->create_client<Kill>("/kill");
        init_turtle_ = this->create_client<Spawn>("/spawn");
        set_pen_left = this->create_client<SetPen>("/left/set_pen");
        set_pen_right = this->create_client<SetPen>("/right/set_pen");
        set_pen_ball = this->create_client<SetPen>("/ball/set_pen");


        send_init_srv();
        rclcpp::sleep_for(500ms);
        spawn_two_pong();
        rclcpp::sleep_for(500ms);
        clean_pen();
        rclcpp::sleep_for(500ms);

        // 初始化必要订阅者与客户端
        pose_ball = this->create_subscription<Pose>("/ball/pose", 10, std::bind(&BallGame::ball_callback, this, std::placeholders::_1));
        pose_left = this->create_subscription<Pose>("/left/pose", 10, std::bind(&BallGame::left_callback, this, std::placeholders::_1));
        pose_right = this->create_subscription<Pose>("/right/pose", 10, std::bind(&BallGame::right_callback, this, std::placeholders::_1));
        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/ball/cmd_vel", 10);
        tele_abs_ = this->create_client<TeleportAbs>("/ball/teleport_absolute");

        // 最后，定时循环move_ball函数，检测球的状态
        ball_move_ = this->create_wall_timer(100ms, std::bind(&BallGame::move_ball, this));
    }

    void send_init_srv()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "clean initial turtle");
        while (!kill_turtle1_->wait_for_service(3s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "interrupted");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting");
        }
        auto kill_msg = std::make_shared<Kill::Request>();
        kill_msg->name = "turtle1";
        kill_turtle1_->async_send_request(kill_msg);
    }

    void ball_callback(Pose::SharedPtr msg)
    {
        pose_ball_info = msg;
    }

    void left_callback(Pose::SharedPtr msg)
    {
        pose_left_info = msg;
    }

    void right_callback(Pose::SharedPtr msg)
    {
        pose_right_info = msg;
    }

    void spawn_two_pong()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "spawn new pong");
        while (!init_turtle_->wait_for_service(3s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "interrupted");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting");
        }
        auto srv_msg = std::make_shared<Spawn::Request>();
        srv_msg->name = "left";
        srv_msg->x = 1.0;
        srv_msg->y = 5.0;
        srv_msg->theta = 3.14159 / 2;
        init_turtle_->async_send_request(srv_msg);
        srv_msg->name = "right";
        srv_msg->x = 10.0;
        srv_msg->y = 5.0;
        srv_msg->theta = 3.14159 / 2;
        init_turtle_->async_send_request(srv_msg);
        srv_msg->name = "ball";
        srv_msg->x = 5.0;
        srv_msg->y = 5.0;
        srv_msg->theta = 0.0;
        init_turtle_->async_send_request(srv_msg);
    }

    void clean_pen()
    {
        auto pen_msg = std::make_shared<SetPen::Request>();
        pen_msg->off = 1;
        set_pen_left->async_send_request(pen_msg);
        set_pen_right->async_send_request(pen_msg);
        set_pen_ball->async_send_request(pen_msg);
    }

    void set_pose_abs(double x, double y, double theta)
    {
        auto pose_abs = std::make_shared<TeleportAbs::Request>();
        pose_abs->x = x;
        pose_abs->y = y;
        pose_abs->theta = theta;
        RCLCPP_DEBUG_STREAM(this->get_logger(), "set abs:" << x << " " << y << " " << theta);
        tele_abs_->async_send_request(pose_abs);
    }

    void set_val(double x, double z)
    {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = x;
        msg->angular.z = z;
        pub_vel->publish(*msg);
    }

    void update_dir()
    {
        double theta = pose_ball_info->theta;
        if (0.0 <= theta && theta < M_PI_2)
        {
            direction_ = Dir::UP_RIGHT;
        }
        else if (M_PI_2 < theta && theta <= M_PI)
        {
            direction_ = Dir::UP_LEFT;
        }
        else if (-M_PI < theta && theta < -M_PI_2)
        {
            direction_ = Dir::DOWN_LEFT;
        }
        else if (-M_PI_2 < theta && theta < 0.0)
        {
            direction_ = Dir::DOWN_RIGHT;
        }
    }

    void check_if_collision()
    {
        double ball_x = pose_ball_info->x;
        double ball_y = pose_ball_info->y;
        double ball_theta = pose_ball_info->theta;

        double left_x = pose_left_info->x;
        double left_y = pose_left_info->y;

        double right_x = pose_right_info->x;
        double right_y = pose_right_info->y;

        double pong_size = delta_y * 2;

        // 如果与左拍碰撞
        if (abs(left_x - ball_x) < delta_x)
        {
            if (abs(left_y - ball_y) < delta_y)
            {
                double intersect_y = (left_y + delta_y) - ball_y;
                double ratio = intersect_y / pong_size;
                double bounce_angle = ratio * max_bounce_angle;
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Bounce angle: " << bounce_angle);
                set_pose_abs(ball_x, ball_y, bounce_angle);
            }
        }

        // 如果与右拍碰撞
        if (abs(right_x - ball_x) < delta_x)
        {
            if (abs(right_y - ball_y) < delta_y)
            {
                double intersect_y = (right_y + delta_y) - ball_y;
                double ratio = intersect_y / pong_size;
                double bounce_angle = ratio * max_bounce_angle;

                if (bounce_angle > 0)
                    bounce_angle += M_PI_2;
                else
                    bounce_angle -= M_PI_2;

                RCLCPP_DEBUG_STREAM(this->get_logger(), "Bounce angle: " << bounce_angle);
                set_pose_abs(ball_x, ball_y, bounce_angle);
            }
        }
    }

    void check_if_collision_with_wall(double x, double y, double theta)
    {

        double new_theta = 0.0;
        if (11.0 < pose_ball_info->y) // hit top wall
        {
            if (direction_ == Dir::UP_LEFT)
            {
                new_theta = 2.0 * M_PI - theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = DOWN_LEFT;
            }
            if (direction_ == Dir::UP_RIGHT)
            {
                new_theta = 2.0 * M_PI - theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = DOWN_RIGHT;
            }
        }

        if (1e-3 > pose_ball_info->y) // hit bottom wall
        {
            if (direction_ == Dir::DOWN_LEFT)
            {
                new_theta = -theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = UP_LEFT;
            }
            if (direction_ == Dir::DOWN_RIGHT)
            {
                new_theta = -theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = UP_RIGHT;
            }
        }

        if (11.0 < pose_ball_info->x) // hit right wall
        {
            if (direction_ == Dir::UP_RIGHT)
            {
                new_theta = M_PI - theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = UP_LEFT;
            }
            if (direction_ == Dir::DOWN_RIGHT)
            {
                new_theta = -M_PI - theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = DOWN_LEFT;
            }
        }

        if (1e-3 > pose_ball_info->x) // hit left wall
        {
            if (direction_ == Dir::UP_LEFT)
            {
                new_theta = M_PI - theta;
                set_pose_abs(x, y, new_theta);
                // direction_ = UP_RIGHT;
            }
            if (direction_ == Dir::DOWN_LEFT)
            {
                new_theta = -(M_PI + theta);
                set_pose_abs(x, y, new_theta);
                // direction_ = DOWN_RIGHT;
            }
        }
    }

    void move_ball()
    {
        // 如果没收到位置信息就暂缓
        if (pose_ball == nullptr || pose_left == nullptr || pose_right == nullptr)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "pose info not received");
            return;
        }

        set_val(2.0, 0.0); // 给球一个线速度，让球继续动

        // 获取球姿态信息
        double x = pose_ball_info->x;
        double y = pose_ball_info->y;
        double theta = pose_ball_info->theta;

        // 更新方向信息
        update_dir();
        // 检查是否与玩家碰撞
        check_if_collision();
        // 检查是否与墙碰撞
        check_if_collision_with_wall(x, y, theta);
    }

    ~BallGame() = default;
};