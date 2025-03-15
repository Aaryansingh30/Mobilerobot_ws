#include "EKF.hpp"
#include "DBScan.hpp"


EKF::EKF() : Node("ekf_slam"){

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&EKF::odom_callback, this, std::placeholders::_1));
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&EKF::scan_callback, this, std::placeholders::_1));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_robot_pose", 10);
    landmark_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/landmark_positions", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/landmark_distances", 10);

    RCLCPP_INFO(this->get_logger(), "EKF Localization Node Initialized");

    P_ = Eigen::MatrixXd::Identity(3, 3);
    P_(0,0)=0.1;
    P_(1,1)=0.1;
    P_(2,2)=0.05;
}

void EKF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto current_time = rclcpp::Clock().now();
    if (last_time.nanoseconds() != 0) { 
        dt = (current_time - last_time).seconds();
        prediction();
        correction();
        publish_pose();  
    }
    last_time = current_time;

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
    
    vx_ = msg->twist.twist.linear.x;   
    vy_ = msg->twist.twist.linear.y;
    vz_ = msg->twist.twist.linear.z;
    wz_ = msg->twist.twist.angular.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta_);
}

void EKF::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<Point> scan_points;
    
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double range = msg->ranges[i];
        double angle = msg->angle_min + i * msg->angle_increment;
        if (std::isfinite(range) && range > 0.1) {  // Ignore invalid or too close points

            if (angle < -1.0 || angle > 1.0) continue;  // Adjust threshold based on LiDAR position

            double x_local = range * cos(angle);
            double y_local = range * sin(angle);
            scan_points.push_back({x_local, y_local});
        }
    }

    // Run DBSCAN clustering
    double eps = 0.5;  
    int min_pts = 5;   
    DBSCAN dbscan(eps, min_pts);
    dbscan.fit(scan_points);
    auto clusters = dbscan.getClusters(scan_points);

    // Clear previous landmarks
    currently_visible_landmarks_.clear();  // Clear at the start of every scan


    for (const auto& cluster : clusters) {
        if (cluster.size() < 3) continue;  // A circle needs at least 3 points

        Circle circle = dbscan.fitCircle(cluster);
        if (circle.r > 0) {  // Valid circle check
            // Transform circle center from local (robot) frame to global frame
            double global_x = x_ + circle.x * cos(theta_) - circle.y * sin(theta_);
            double global_y = y_ + circle.x * sin(theta_) + circle.y * cos(theta_);

            Eigen::Vector2d global_center(global_x, global_y);

            // Check if the landmark already exists to avoid duplicates
            bool exists = false;
            for (const auto& landmark : landmarks_) {
                if ((landmark - global_center).norm() < 0.3) { // 0.3m threshold for duplicate landmarks
                    exists = true;
                    break;
                }
            }

            if (!exists) {
                landmarks_.push_back(global_center);
            }
            currently_visible_landmarks_.push_back(global_center);
        }
    }
}

void EKF::prediction() {
    if (vx_ == 0.0 && wz_ == 0.0) return;
    
    double R = (fabs(wz_) > 1e-6) ? vx_ / wz_ : 0.0;
    dx_ = (fabs(wz_) > 1e-6) ? R * (sin(theta_ + wz_ * dt) - sin(theta_)) : vx_ * cos(theta_) * dt;
    dy_ = (fabs(wz_) > 1e-6) ? -R * (cos(theta_ + wz_ * dt) - cos(theta_)) : vx_ * sin(theta_) * dt;
    dtheta_ = (fabs(wz_) > 1e-6) ? wz_ * dt : 0.0;

    x_predict = x_ + dx_;
    y_predict = y_ + dy_;
    theta_predict = atan2(sin(theta_ + dtheta_), cos(theta_ + dtheta_));

    Eigen::MatrixXd F_(3, 3);
    F_.setIdentity();
    F_(0, 2) = (fabs(wz_) > 1e-6) ? R * (cos(theta_ + wz_ * dt) - cos(theta_)) : -vx_ * sin(theta_) * dt;
    F_(1, 2) = (fabs(wz_) > 1e-6) ? R * (sin(theta_ + wz_ * dt) - sin(theta_)) : vx_ * cos(theta_) * dt;

    double sigma_x = 0.5, sigma_y = 0.5, sigma_theta = 0.25;
    Eigen::MatrixXd Q_(3, 3);
    Q_ << sigma_x * sigma_x, 0, 0,
          0, sigma_y * sigma_y, 0,
          0, 0, sigma_theta * sigma_theta;

    P_ = F_ * P_ * F_.transpose() + Q_;}

void EKF::correction() {
    if (laser_data_.empty()) return;
    
    Eigen::MatrixXd P_temp = P_;
    Eigen::VectorXd accumulated_update(3);
    accumulated_update.setZero();

    for (const auto& data : laser_data_) {
        double laser_range = data.second;
        double laser_theta = data.first;
        double measured_x = x_predict + laser_range * cos(theta_predict + laser_theta);
        double measured_y = y_predict + laser_range * sin(theta_predict + laser_theta);

        Eigen::VectorXd z(2);
        z << laser_range, laser_theta;

        Eigen::VectorXd h_x(2);
        h_x << sqrt(pow(measured_x - x_predict, 2) + pow(measured_y - y_predict, 2)), 
               atan2(measured_y - y_predict, measured_x - x_predict) - theta_predict;
        h_x(1) = atan2(sin(h_x(1)), cos(h_x(1)));

        Eigen::MatrixXd H(2, 3);
        double range_sq = pow(h_x(0), 2);
        H << (measured_x - x_predict) / h_x(0), (measured_y - y_predict) / h_x(0), 0,
             (measured_y - y_predict) / range_sq, -(measured_x - x_predict) / range_sq, -1;

        Eigen::MatrixXd R(2, 2);
        R << 0.2, 0,
             0, 0.2;

        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        Eigen::VectorXd y = z - h_x;
        y(1) = atan2(sin(y(1)), cos(y(1)));

        // std::vector<int> associated_index;  
        int associated_index = -1;        
         data_association(S, laser_range, laser_theta, associated_index);

         if (associated_index != -1) {
            map_update( H, y, K, associated_index);
        } 
        Eigen::VectorXd state_update = K * y;
        accumulated_update += state_update;
        P_temp = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P_temp;
    } 

    x_predict += accumulated_update(0);
    y_predict += accumulated_update(1);
    theta_predict = atan2(sin(theta_predict + accumulated_update(2)), cos(theta_predict + accumulated_update(2)));
    P_ = P_temp;}

void EKF::data_association(const Eigen::MatrixXd &S, double laser_range, double laser_theta, int &associated_index) {

    double threshold_distance = std::max(5 * std::abs(S.determinant()), 0.1);
    double min_distance = std::numeric_limits<double>::max();
    
    Eigen::VectorXd z(2);
    z << laser_range, laser_theta;
    
    if (!landmarks_.empty()) {
        for (size_t i = 0; i < landmarks_.size(); ++i) {
            double dx = landmarks_[i](0, 0) - x_predict;
            double dy = landmarks_[i](1, 0) - y_predict;
            double range = sqrt(dx * dx + dy * dy);
            double bearing = atan2(dy, dx) - theta_predict;
            bearing = atan2(sin(bearing), cos(bearing));
            
            Eigen::VectorXd h_x(2);
            h_x << range, bearing;
            
            Eigen::VectorXd y = z - h_x;
            y(1) = atan2(sin(y(1)), cos(y(1)));
            double distance = y.transpose() * S.inverse() * y;
            
            if (distance < min_distance && distance < threshold_distance) {
                min_distance = distance;
                associated_index = i;
            }
        }
    }
    
    if (associated_index == -1 && laser_range < 0.5) {
        Eigen::VectorXd new_landmark(2);
        new_landmark << x_predict + laser_range * cos(theta_predict + laser_theta), 
                        y_predict + laser_range * sin(theta_predict + laser_theta);

        landmarks_.push_back(new_landmark);
        currently_visible_landmarks_.push_back(new_landmark);
        P_landmarks_.emplace_back(Eigen::MatrixXd::Identity(2, 2) * 0.2);
    }
}

void EKF::map_update(const Eigen::MatrixXd &H, const Eigen::VectorXd &y, const Eigen::MatrixXd &K, int associated_index) {
        if (associated_index != -1) {
            landmarks_[associated_index] += K.topRows(2) * y;
            currently_visible_landmarks_[associated_index] += K.topRows(2) * y;

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2, 2);
            P_landmarks_[associated_index] = (Eigen::MatrixXd::Identity(2, 2) - K.topRows(2) * H) * P_landmarks_[associated_index];
        }
}

void EKF::publish_pose() {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = x_predict;
    pose_msg.pose.position.y = y_predict;
    pose_msg.pose.orientation.z = sin(theta_predict / 2);
    pose_msg.pose.orientation.w = cos(theta_predict / 2);
    pose_publisher_->publish(pose_msg);

    geometry_msgs::msg::PoseArray landmark_msg;
    landmark_msg.header.stamp = this->get_clock()->now();
    landmark_msg.header.frame_id = "odom";
    for (const auto &landmark : landmarks_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = landmark(0);
        pose.position.y = landmark(1);
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        landmark_msg.poses.push_back(pose);
    }

    landmark_publisher_->publish(landmark_msg);
    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = "odom";
    line_list.header.stamp = this->get_clock()->now();
    line_list.ns = "landmark_distances";
    line_list.id = 0;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.scale.x = 0.02;  
    line_list.color.a = 1.0;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;  
    line_list.color.b = 0.0;
    
    geometry_msgs::msg::Point robot_position;
    robot_position.x = x_predict;
    robot_position.y = y_predict;
    robot_position.z = 0.0;
    
    for (const auto &landmark : currently_visible_landmarks_) {
        geometry_msgs::msg::Point landmark_position;
        landmark_position.x = landmark(0);
        landmark_position.y = landmark(1);
        landmark_position.z = 0.0;
        line_list.points.push_back(robot_position);
        line_list.points.push_back(landmark_position);
    }
    marker_publisher_->publish(line_list);
}

