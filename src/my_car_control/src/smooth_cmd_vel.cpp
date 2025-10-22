#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

using namespace std::chrono_literals;

class SmoothCmdVel : public rclcpp::Node
{
public:
    SmoothCmdVel()
    : Node("smooth_cmd_vel"),
      accel_linear_(1),        // m/s² - giới hạn tăng tốc
      decel_linear_(1),        // m/s² - giới hạn giảm tốc
      accel_angular_(5.0),       // rad/s² - quay mượt
      decel_angular_(8.0),
      dt_(0.02)                  // 50 Hz
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

        RCLCPP_INFO(this->get_logger(), " smooth_cmd_vel node started");
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_ = *msg;
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist new_cmd;

        // === Mượt tuyến tính ===
        double diff_lin = target_.linear.x - current_.linear.x;
        double limit_lin = (diff_lin > 0 ? accel_linear_ : decel_linear_) * dt_;

        if (std::abs(diff_lin) > limit_lin)
            current_.linear.x += limit_lin * (diff_lin > 0 ? 1.0 : -1.0);
        else
            current_.linear.x = target_.linear.x;

        // === Mượt góc quay ===
        double diff_ang = target_.angular.z - current_.angular.z;
        double limit_ang = (diff_ang > 0 ? accel_angular_ : decel_angular_) * dt_;

        if (std::abs(diff_ang) > limit_ang)
            current_.angular.z += limit_ang * (diff_ang > 0 ? 1.0 : -1.0);
        else
            current_.angular.z = target_.angular.z;

        // Xuất lệnh ra topic /cmd_vel/manual
        new_cmd.linear.x = current_.linear.x;
        new_cmd.angular.z = current_.angular.z;
        pub_->publish(new_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_;
    geometry_msgs::msg::Twist target_;

    double accel_linear_, decel_linear_;
    double accel_angular_, decel_angular_;
    double dt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothCmdVel>());
    rclcpp::shutdown();
    return 0;
}
