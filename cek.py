import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

KANAN = 0.2
KIRI = -0.2

class HeadListener(Node):
    def __init__(self):
        super().__init__('head_listener')

        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('Menunggu data head...')
        self.derajat_kamera = 0.0 
        self.head_tilt = 0.0

    def joint_callback(self, msg: JointState):
        head_pan_found = False
        head_tilt_found = False

        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]
                head_pan_found = True

            elif name == 'head_tilt':
                self.head_tilt = msg.position[i]
                head_tilt_found = True

        # Print kalau dua-duanya ketemu
        if head_pan_found or head_tilt_found:
            self.get_logger().info(
                f'Head Pan = {self.derajat_kamera:.3f} | Head Tilt = {self.head_tilt:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HeadListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()