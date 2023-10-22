#include "teleop_key/TeleopKeyNode.hpp"

#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
