import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class FakeGPSPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps')
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.t = 0.0
        self.get_logger().info(' Fake GPS started – publishing /fix (moving path)')

        # toạ độ gốc (ví dụ Bách Khoa HCMUT)
        self.lat0 = 10.762622
        self.lon0 = 106.660172
        self.alt0 = 5.0

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        # ---- tạo quỹ đạo hình số 8 (biên độ ~30 m) ----
        # 1e-4 độ ~ 11 m, nên 3e-4 ~ 33 m
        msg.latitude  = self.lat0 + 3e-4 * math.sin(self.t / 5.0)
        msg.longitude = self.lon0 + 3e-4 * math.sin(self.t / 10.0) * math.cos(self.t / 5.0)
        msg.altitude  = self.alt0 + 1.0 * math.sin(self.t / 3.0)

        msg.position_covariance_type = 2  # COVARIANCE_TYPE_APPROXIMATED
        self.pub.publish(msg)

        self.t += 0.1

def main():
    rclpy.init()
    node = FakeGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
