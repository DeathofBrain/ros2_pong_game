#include "ball_pong/BallGame.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BallGame>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
