#include <homework/homework_node.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Homework>());
    rclcpp::shutdown();
}
