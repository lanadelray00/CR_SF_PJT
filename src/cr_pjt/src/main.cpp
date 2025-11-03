#include <iostream>
#include <rclcpp/rclcpp.hpp>   // ROS2 핵심 헤더
#include "my_header.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    std::cout << "ROS2 header include success!" << std::endl;
    rclcpp::shutdown();
    
    std::cout << say_hello() << std::endl;
    return 0;
}
