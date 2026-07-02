import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

ACTION_PAGE_FRONT = 142
ACTION_PAGE_BACK  = 143
THRESHOLD         = 9
FILTER_SIZE       = 5
ACTION_DURATION   = 1.0
KANAN             = 0.1
KIRI              = -0.1



class ActionPublisParam ():
    def __init__(self):                                                                                                              
        super().__init__(node_name='button_soccer_node')

        self.walking_vel_pub = self.create_publisher(
            Twist, '/robotis/walking/velocity', 10)
        
        self.action_pub = self.create_publisher(
            Int32,'/robotis/action/page_num', 10)
        self.walking_param_pub = self.create_publisher(
            WalkingParam,
            '/robotis/walking/set_params',
            10
        )
        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.derajat_kamera = 0.0
        self.get_logger().info('button_soccer_node GaitController siap.')

    def imu_callback(self, msg: Imu):
        acc_x = msg.linear_acceleration.x

        self.acc_buffer.append(acc_x)
        if len(self.acc_buffer) > FILTER_SIZE:
            self.acc_buffer.pop(0)

        avg_acc_x = sum(self.acc_buffer) / len(self.acc_buffer)

        if self.is_running:
            return

        if acc_x < -THRESHOLD or acc_x > THRESHOLD:
            self.is_running = True
            self.get_logger().warn("Jatuh terdeteksi, memulai recovery...")
            self.action_param("stop")
            if avg_acc_x < -THRESHOLD:
                self.get_logger().warn("Jatuh DEPAN")
                self.start_recovery(ACTION_PAGE_FRONT)
            elif avg_acc_x > THRESHOLD:
                self.get_logger().warn("Jatuh BELAKANG")
                self.start_recovery(ACTION_PAGE_BACK)

    def joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'head_pan':
                self.derajat_kamera = msg.position[i]

    def create_param(self, x=0.0, y=0.0, yaw_deg=0.0):
        param = WalkingParam()
        param.init_x_offset = 0.010
        param.init_y_offset = 0.015
        param.init_z_offset = 0.035
        param.hip_pitch_offset = math.radians(12.0)
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

    def walking_param(self, Param):
        cmd = String()
        cmd.data = Param
        self.walking_command_pub.publish(cmd)

    def action_param(self, Param):
        cmd = String()
        cmd.data = Param
        self.action_command_pub.publish(cmd)

    def send_param(self, x=0.0, y=0.0, yaw=0.0):
        param = self.create_param(x, y, yaw)
        self.walking_param_pub.publish(param)
        self.get_logger().info('Param dikirim')

    def state_callback(self, msg):
        self.robot_state = msg.data
        if self.robot_state == "RECOVER" or self.robot_state == "KICK":
            self.action_param("stop")
        elif self.robot_state == "NORMAL":
            self.start_all()

    def init_pose(self, action_page):
        self.get_logger().info(f'Memulai pose inisialisasi: {action_page}')
        page = Int32()
        page.data = action_page
        self.action_pub.publish(page)
        time.sleep(ACTION_DURATION)
        
