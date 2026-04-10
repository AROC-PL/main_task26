import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from op3_walking_module_msgs.msg import WalkingParam

KANAN = 0.2
KIRI = -0.2


class GaitController(Node):
    """
    Base class untuk kontrol gait (walking parameter) robot OP3.
    Menyediakan logika create_param, send_param, dan walk_parameter
    berdasarkan posisi head pan kamera.
    """

    def __init__(self, node_name='gait_controller'):
        super().__init__(node_name)

        self.walking_param_pub = self.create_publisher(
            WalkingParam,
            '/robotis/walking/set_params',
            10
        )
        self.module_pub = self.create_publisher(
            String,
            '/robotis/enable_ctrl_module',
            10
        )
        self.walking_command_pub = self.create_publisher(
            String,
            '/robotis/walking/command',
            10
        )

        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )

        self.derajat_kamera = 0.0
        self.get_logger().info(f'[{node_name}] GaitController siap.')

    def joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]
                self.get_logger().info(f'Head pan: {self.derajat_kamera:.3f}')

    def create_param(self, x=0.0, y=0.0, yaw_deg=0.0):
        param = WalkingParam()
        param.init_x_offset = 0.010
        param.init_y_offset = 0.015
        param.init_z_offset = 0.035
        param.hip_pitch_offset = math.radians(9.0)
        param.period_time = 0.95
        param.dsp_ratio = 0.2
        param.step_fb_ratio = 0.28
        param.x_move_amplitude = x
        param.y_move_amplitude = y
        param.z_move_amplitude = 0.060
        param.angle_move_amplitude = math.radians(yaw_deg)
        param.balance_enable = True
        param.balance_hip_roll_gain = 0.35
        param.balance_knee_gain = 0.3
        param.balance_ankle_roll_gain = 0.7
        param.balance_ankle_pitch_gain = 0.9
        param.y_swap_amplitude = 0.028
        param.z_swap_amplitude = 0.006
        param.arm_swing_gain = 0.2
        param.p_gain = 0
        return param

    def send_param(self, x=0.0, y=0.0, yaw=0.0):
        param = self.create_param(x, y, yaw)
        self.walking_param_pub.publish(param)
        self.get_logger().info('Param dikirim')

    def enable_walking_module(self):
        msg = String()
        msg.data = 'walking_module'
        self.module_pub.publish(msg)

    def start_walking(self):
        cmd = String()
        cmd.data = 'start'
        self.walking_command_pub.publish(cmd)

    def walk_parameter(self):
        if self.derajat_kamera > KANAN:
            self.send_param(0.020, 0.016, 0.0)
            self.get_logger().info('Kanan → jalan miring kanan')
        elif self.derajat_kamera < KIRI:
            self.send_param(0.030, -0.016, 0.0)
            self.get_logger().info('Kiri → jalan miring kiri')
        else:
            self.send_param(0.0, 0.0, 0.0)
            self.get_logger().info('Tengah → berhenti')


def main():
    """Entry point standalone jika crab_walk dijalankan langsung."""
    rclpy.init()
    node = GaitController(node_name='gait_param_sender')
    node.enable_walking_module()
    time.sleep(0.2)
    node.start_walking()
    try:
        while rclpy.ok():
            node.walk_parameter()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()