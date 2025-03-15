/* ekf_slam.hpp */
#ifndef EKF_HPP
#define EKF_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <eigen3/Eigen/Dense>
#include <vector>

class EKF : public rclcpp::Node {
public:
    EKF();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void prediction();
    void correction();
    void data_association(const Eigen::MatrixXd &S, double laser_range, double laser_theta, int &associated_index);
    void map_update(const Eigen::MatrixXd &H, const Eigen::VectorXd &y, const Eigen::MatrixXd &K, int associated_index);
    void publish_pose();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmark_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Time last_time;

    double x_predict, y_predict, theta_predict;
    double x_, y_, z_; 
    double vx_, vy_, vz_;
    double wx_, wy_, wz_;
    double theta_; 
    double dx_, dy_, dtheta_;
    double range;
    double dt;

    std::vector<Eigen::VectorXd> landmarks_;
    Eigen::MatrixXd P_;
    std::vector<Eigen::VectorXd> currently_visible_landmarks_;
    std::vector<Eigen::MatrixXd> P_landmarks_;
    std::vector<std::pair<double, double>> laser_data_;
};

#endif // EKF_SLAM_HPP
