import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class ImuYawNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_node')

        self.sub = self.create_subscription(
            Imu,
            '/robotis/open_cr/imu',
            self.callback,
            10
        )
        self.kalman = KalmanFilter()

    def callback(self, msg: Imu):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        atas = 2 * (w * z + x * y)
        bawah = 1 - 2 * (y * y + z * z)

        yaw = math.atan2(atas, bawah)

        # Kalman filter
        # yaw_filtered = self.kalman.update(yaw)

        yaw_deg = yaw * 180 / math.pi
        # yaw_filtered_deg = yaw_filtered * 180 / math.pi
        self.get_logger().info(
            f"Raw: {yaw_deg} deg"
        )

        # if abs(yaw_filtered) > 0.01:
        #     self.get_logger().info(
        #         f"Raw: {yaw_deg:.2f} deg | Filtered: {yaw_filtered_deg:.2f} deg"
        #     )
    
class KalmanFilter:
    def __init__(self):
        self.q = 0.001   # process noise (semakin kecil → lebih smooth tapi lambat)
        self.r = 0.05    # measurement noise (semakin besar → lebih percaya model)
        self.x = 0.0     # state (yaw)
        self.p = 1.0     # estimation error

    def update(self, measurement):
        # Prediction
        self.p = self.p + self.q

        # Kalman Gain
        k = self.p / (self.p + self.r)

        # Update state
        self.x = self.x + k * (measurement - self.x)

        # Update error
        self.p = (1 - k) * self.p

        return self.x

def main():
    rclpy.init()
    node = ImuYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()