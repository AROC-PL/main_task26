import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')

        self.subscription = self.create_subscription(
            Imu,
            '/robotis/open_cr/imu',   # ganti sesuai topic kamu
            self.listener_callback,
            10
        )

        # buka file CSV
        self.file = open('imu_data_belakang.csv', 'w', newline='')
        self.writer = csv.writer(self.file)

        # header
        self.writer.writerow(['x', 'y', 'z', 'w'])

    def listener_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # simpan ke CSV
        self.writer.writerow([x, y, z, w])

        self.get_logger().info(f"x:{x}, y:{y}, z:{z}, w:{w}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    rclpy.spin(node)

    node.file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()