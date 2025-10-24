#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class SmoothCmdVel : public rclcpp::Node
{
public:
    SmoothCmdVel()
    : Node("smooth_cmd_vel"),
      decel_linear_(0.5),       // m/s² – tốc độ giảm dần tuyến tính
      decel_angular_(5.0),      // rad/s² – tốc độ giảm dần quay
      dt_(0.02),                // 50 Hz
      deadband_(0.01)           // vùng chết để tránh rung
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel/manual_raw", 10,
            std::bind(&SmoothCmdVel::cmd_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/manual", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&SmoothCmdVel::timer_callback, this));

        current_.linear.x = 0.0;
        current_.angular.z = 0.0;
        target_.linear.x = 0.0;
        target_.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "✅ smooth_cmd_vel active (dt=%.2f s, decel=%.2f m/s²)",
                    dt_, decel_linear_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_ = *msg;
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist out;

        // ===== Tuyến tính =====
        double diff_lin = target_.linear.x - current_.linear.x;
        double max_step_lin = decel_linear_ * dt_;

        if (std::fabs(diff_lin) < deadband_)
        {
            // Đã gần bằng nhau
            current_.linear.x = target_.linear.x;
        }
        else if (std::fabs(target_.linear.x) < std::fabs(current_.linear.x))
        {
            // Đang giảm tốc (kể cả về 0, hoặc giảm độ âm)
            diff_lin = std::clamp(diff_lin, -max_step_lin, max_step_lin);
            current_.linear.x += diff_lin;
        }
        else
        {
            // Tăng tốc hoặc đổi chiều → nhận ngay
            current_.linear.x = target_.linear.x;
        }

        // ===== Góc quay =====
        double diff_ang = target_.angular.z - current_.angular.z;
        double max_step_ang = decel_angular_ * dt_;

        if (std::fabs(diff_ang) < deadband_)
        {
            current_.angular.z = target_.angular.z;
        }
        else if (std::fabs(target_.angular.z) < std::fabs(current_.angular.z))
        {
            diff_ang = std::clamp(diff_ang, -max_step_ang, max_step_ang);
            current_.angular.z += diff_ang;
        }
        else
        {
            current_.angular.z = target_.angular.z;
        }

        // Xuất lệnh mượt
        out.linear.x  = current_.linear.x;
        out.angular.z = current_.angular.z;
        pub_->publish(out);
    }

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Trạng thái
    geometry_msgs::msg::Twist target_;
    geometry_msgs::msg::Twist current_;

    // Tham số
    double decel_linear_;
    double decel_angular_;
    double dt_;
    double deadband_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothCmdVel>());
    rclcpp::shutdown();
    return 0;
}
