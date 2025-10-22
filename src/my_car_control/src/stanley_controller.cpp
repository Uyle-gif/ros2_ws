#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class StanleyController : public rclcpp::Node {
public:
  StanleyController() : Node("stanley_controller") {
    // Declare parameters
    this->declare_parameter("K", 1.0);
    this->declare_parameter("L", 0.5);
    this->declare_parameter("desired_speed", 1.0);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("path_file", "path.csv");

    // Load parameters
    K_ = this->get_parameter("K").as_double();
    L_ = this->get_parameter("L").as_double(); 
    desired_speed_ = this->get_parameter("desired_speed").as_double();
    max_steer_ = this->get_parameter("max_steer_deg").as_double() * M_PI / 180.0;
    path_file_ = this->get_parameter("path_file").as_string();

    // Load path file
    loadPathFromCSV();

    // Subscribers & Publishers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&StanleyController::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Stanley Controller initialized. Path size: %zu", path_points_.size());
  }

private:
  // Parameters
  double K_, L_, desired_speed_, max_steer_;
  std::string path_file_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::vector<std::pair<double, double>> path_points_;

  // ================================
  // Load CSV Path File
  // ================================
  void loadPathFromCSV() {
    // Lấy đường dẫn tuyệt đối tới file path.csv trong share/
    std::string package_name = "my_car_control";
    std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
    std::string csv_path = share_dir + "/" + path_file_;

    std::ifstream file(csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), " Could not open path file: %s", csv_path.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      double px, py; char comma;
      if (ss >> px >> comma >> py)
        path_points_.emplace_back(px, py);
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), " Loaded %zu path points from %s", path_points_.size(), csv_path.c_str());
  }

  // ================================
  // Callback: Process /odom
  // ================================
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (path_points_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path loaded yet!");
      return;
    }

    // Current pose
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    // Find closest point
    size_t closest_index = 0;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_points_.size(); ++i) {
      double dx = x - path_points_[i].first;
      double dy = y - path_points_[i].second;
      double dist_sq = dx*dx + dy*dy;        //di​=sqrt((x−xi​)^2+(y−yi​)^2)
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_index = i;
      }
    }

    // Compute heading of path
    double path_yaw;
    if (closest_index < path_points_.size() - 1) {
      double next_x = path_points_[closest_index + 1].first;
      double next_y = path_points_[closest_index + 1].second;
      path_yaw = std::atan2(next_y - path_points_[closest_index].second,
                            next_x - path_points_[closest_index].first);
    } else {
      path_yaw = yaw;
    }

    // Heading error
    double heading_error = path_yaw - yaw;
    if (heading_error > M_PI) heading_error -= 2*M_PI;
    if (heading_error < -M_PI) heading_error += 2*M_PI;

    // Cross-track error
    double dx = x - path_points_[closest_index].first;
    double dy = y - path_points_[closest_index].second;
    double cross_error = (-std::sin(path_yaw) * dx) + (std::cos(path_yaw) * dy);

    // Stanley steering angle
    double steer_angle = heading_error + std::atan2(K_ * cross_error, desired_speed_);
    if (steer_angle > max_steer_) steer_angle = max_steer_;
    if (steer_angle < -max_steer_) steer_angle = -max_steer_;

    // Convert to yaw rate
    double yaw_rate = desired_speed_ * std::tan(steer_angle) / L_;

    // Publish Twist
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = desired_speed_;
    cmd_msg.angular.z = yaw_rate;
    cmd_pub_->publish(cmd_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "cte=%.2f, heading_error=%.2f°, steer=%.2f°, yaw_rate=%.2f rad/s",
      cross_error, heading_error*180/M_PI, steer_angle*180/M_PI, yaw_rate);
  }
};

// Main
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StanleyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}





// #include <cmath>
// #include <string>
// #include <vector>
// #include <fstream>
// #include <sstream>
// #include <limits>

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "tf2/utils.h"
// #include "ament_index_cpp/get_package_share_directory.hpp"


// class StanleyController : public rclcpp::Node {
// public:
//   StanleyController() : Node("stanley_controller") {
//     // Subscriber /odom
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10, std::bind(&StanleyController::odomCallback, this, std::placeholders::_1));
//     // Publisher /cmd_vel
//     cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
//     // Đọc đường dẫn từ file CSV
//     loadPathFromCSV();
//     RCLCPP_INFO(this->get_logger(), "StanleyController READY.");
//   }

// private:
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//   std::vector<std::pair<double, double>> path_points_;

//   void loadPathFromCSV() {
//     std::string package_name = "my_car_control";  // Đổi thành tên package của bạn
//     std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
//     std::string csv_path = share_dir + "/path.csv";
//     std::ifstream file(csv_path);
//     if (!file.is_open()) {
//       RCLCPP_ERROR(this->get_logger(), "cant open path file %s", csv_path.c_str());
//       return;
//     }
//     std::string line;
//     while (std::getline(file, line)) {
//       if (line.empty()) continue;
//       std::stringstream ss(line);
//       double px, py;
//       char comma;
//       if (!(ss >> px >> comma >> py)) {
//         continue; // bỏ qua nếu parse không thành công
//       }
//       path_points_.emplace_back(px, py);
//     }
//     file.close();
//     RCLCPP_INFO(this->get_logger(), "Loaded %zu path points from %s", path_points_.size(), csv_path.c_str());
//   }

//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     if (path_points_.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No path points to follow.");
//       return;
//     }
//     // Lấy vị trí hiện tại của robot
//     double x = msg->pose.pose.position.x;
//     double y = msg->pose.pose.position.y;
//     // Tính góc hướng hiện tại (yaw) từ quaternion
//     double yaw = tf2::getYaw(msg->pose.pose.orientation);

//     // Tìm điểm đường dẫn gần robot nhất
//     size_t closest_index = 0;
//     double min_dist_sq = std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i < path_points_.size(); ++i) {
//       double dx = x - path_points_[i].first;
//       double dy = y - path_points_[i].second;
//       double dist_sq = dx*dx + dy*dy;
//       if (dist_sq < min_dist_sq) {
//         min_dist_sq = dist_sq;
//         closest_index = i;
//       }
//     }
//     double closest_x = path_points_[closest_index].first;
//     double closest_y = path_points_[closest_index].second;

//     // Tính hướng của đường tại điểm gần nhất
//     double path_yaw;
//     if (closest_index < path_points_.size() - 1) {
//       double next_x = path_points_[closest_index + 1].first;
//       double next_y = path_points_[closest_index + 1].second;
//       path_yaw = std::atan2(next_y - closest_y, next_x - closest_x);
//     } else if (closest_index > 0) {
//       double prev_x = path_points_[closest_index - 1].first;
//       double prev_y = path_points_[closest_index - 1].second;
//       path_yaw = std::atan2(closest_y - prev_y, closest_x - prev_x);
//     } else {
//       path_yaw = yaw;
//     }

//     // Sai số góc hướng (theta_e)
//     double heading_error = path_yaw - yaw;
//     // Chuẩn hóa về [-pi, pi]
//     if (heading_error > M_PI) heading_error -= 2*M_PI;
//     if (heading_error < -M_PI) heading_error += 2*M_PI;

//     // Sai số crosstrack (e_cte)
//     double dx = x - closest_x;
//     double dy = y - closest_y;
//     double cross_error = (-std::sin(path_yaw) * dx) + (std::cos(path_yaw) * dy);

//     // Thuật toán Stanley: tính góc lái
//     double vx = msg->twist.twist.linear.x;
//     double vy = msg->twist.twist.linear.y;
//     double speed = std::sqrt(vx*vx + vy*vy);
//     double k = 1.0;
//     double epsilon = 1e-3;
//     double steer_angle = heading_error + std::atan2(k * cross_error, speed + epsilon);

//     // Chuyển góc lái thành vận tốc góc (yaw_rate)
//     double desired_speed = 1.0;  // m/s (có thể điều chỉnh/tuning)
//     double L = 1.0;  // giả sử khoảng cách trục xe 1m
//     double yaw_rate = desired_speed * std::tan(steer_angle) / L;

//     // Tạo Twist và publish
//     geometry_msgs::msg::Twist cmd_msg;
//     cmd_msg.linear.x = desired_speed;
//     cmd_msg.linear.y = 0.0;
//     cmd_msg.linear.z = 0.0;
//     cmd_msg.angular.x = 0.0;
//     cmd_msg.angular.y = 0.0;
//     cmd_msg.angular.z = yaw_rate;
//     cmd_pub_->publish(cmd_msg);
//   }
// };

// // Hàm main
// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<StanleyController>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
