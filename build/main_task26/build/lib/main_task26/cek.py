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

    def joint_callback(self, msg: JointState):
        # Ambil nilai head_pan dari JointState
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]
                self.get_logger().info(
                    f'Head Pan = {self.derajat_kamera:.3f}'
                )

        # Panggil logika jalan
        self.walk_parameter()

    def walk_parameter(self):
        if self.derajat_kamera > KANAN:
            self.get_logger().info('Kanan → jalan miring kanan')
            self.get_logger().info(
                f"derajat kamera : {self.derajat_kamera:.3f}"
            )

        elif self.derajat_kamera < KIRI:
            self.get_logger().info('Kiri → jalan miring kiri')
            self.get_logger().info(
                f"derajat kamera : {self.derajat_kamera:.3f}"
            )

        else:
            self.get_logger().info('Tengah → berhenti')
            self.get_logger().info(
                f"derajat kamera : {self.derajat_kamera:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = HeadListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()