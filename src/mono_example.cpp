/**
 * @file main.cpp
 * @brief Entry point for the ros2_orb_slam3 monocular tracking node.
 * * Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
 */

#include "ros2_orb_slam3/common.hpp"

/**
 * @brief Main function initializing the ROS 2 runtime environment and spinning the monocular node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Execution exit status code.
 */
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MonocularMode>(); 
    
    // Use MultiThreadedExecutor to allow timestep, IMU, and image callbacks to run in parallel
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}