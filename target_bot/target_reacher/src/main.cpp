#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include "bot_controller/bot_controller.h"

/**
 * @brief This main function sets up the robot to reach multiple targets.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto bot_controller = std::make_shared<BotController>("bot_controller_robot", "robot1");
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<TargetReacher>(bot_controller);
    exec.add_node(node);
    exec.add_node(bot_controller);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}