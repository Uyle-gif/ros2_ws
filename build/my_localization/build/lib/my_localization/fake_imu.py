import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import math

class FakeIMUPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.t = 0.0
        self.get_logger().info(' Fake IMU started – publishing /imu/data')

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        yaw = 0.05 * math.sin(self.t / 5.0)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.z = 0.05 * math.cos(self.t / 5.0)
        msg.linear_acceleration.z = 9.81
        self.pub.publish(msg)
        self.t += 0.02

def main():
    rclpy.init()
    node = FakeIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

