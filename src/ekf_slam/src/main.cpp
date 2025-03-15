#include "rclcpp/rclcpp.hpp"
#include "EKF.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ekf_node = std::make_shared<EKF>();
    rclcpp::spin(ekf_node);
    rclcpp::shutdown();
    return 0;
}
