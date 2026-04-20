import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import JointState
from .main_action import FallRecoveryFull


KANAN = 0.1
KIRI = -0.1


class KickDecision(Node):
    
    def __init__(self):
        super().__init__('kick_decision')

        self.fall_recovery = FallRecoveryFull()
        
        self.module_pub = self.create_publisher(
            String,
            '/robotis/enable_ctrl_module',
            10
        )

        self.motion_pub = self.create_publisher(
            Int32,
            '/robotis/action/page_num',
            10
        )

        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/vision/jarak_kamera',
            self.jarak_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String, '/robot_state', 10)
        
        self.acc_buffer = []
        self.is_running = False
        self._reset_timer = None

        self.derajat_kamera = None
        # self.kick_executed = False
        self.jarak_bola = None

        self.get_logger().info('Kick Decision Siap')

    def joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]

    
    def enable_action_module(self):
        msg = String()
        msg.data = 'action_module'
        self.module_pub.publish(msg)

    def jarak_callback(self, msg):
        self.jarak_bola = msg.data
        
    def kick(self):
        if self.is_running:
            return
        
        if self.derajat_kamera is None or self.jarak_bola is None:
            self.get_logger().info("Menunggu data...")
            return
        
        if self.jarak_bola > 29:
            self.get_logger().info(f"Bola masih jauh: {self.jarak_bola:.2f} cm")
            return

        if self.derajat_kamera > KANAN:
            self.play_motion("right")
        elif self.derajat_kamera < KIRI:
            self.play_motion("left")
        else:
            self.get_logger().info("Bola belum di posisi ideal")
            return

        # self.kick_executed = True
        # time.sleep(2.5)
        # self.kick_executed = False

    def play_motion(self, direction):
        # msg = String()
        # msg.data = "action_module"
        # self.module_pub.publish(msg)

        if direction == "right":
            # msg = Int32()
            # msg.data = 150 
            # self.motion_pub.publish(msg)
            self.fall_recovery.start_recovery(150)
            self.get_logger().info("KICK KANAN")
            
        else:
            # msg = Int32()
            # msg.data = 151 
            # self.motion_pub.publish(msg)
            self.fall_recovery.start_recovery(151)
            self.get_logger().info("KICK KIRI")
            
        time.sleep(3)
        
    

# def main():
#     rclpy.init()
#     node = KickDecision()

#     node.enable_action_module()
#     time.sleep(1.0)

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#             node.kick()
            
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()