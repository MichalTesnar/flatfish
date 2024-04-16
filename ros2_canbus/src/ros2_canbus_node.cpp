#include "Ros2Canbus.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Ros2CanbusNode>("canbus");

    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    delete node;
    return 0;
}