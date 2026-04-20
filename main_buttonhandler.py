import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

from main_task26.crab_walk import GaitController
from .main_action import FallRecoveryFull
from .main_head_control import HeadControl


class ButtonSoccerNode(GaitController):
    """
    Node kontrol tombol robot OP3.
    Mewarisi GaitController dari crab_walk.py untuk logika walking parameter.
    Tombol:
      - user  : Jalan diagonal (berdasarkan head pan)
      - start : Jalan lurus biasa
      - mode  : Toggle soccer / walking module
    """

    def __init__(self):
        super().__init__(node_name='button_soccer_node')

        self.create_subscription(
            String, '/robotis/open_cr/button', self.button_callback, 10)

        self.walking_vel_pub = self.create_publisher(
            Twist, '/robotis/walking/velocity', 10)
        
        self.action_pub = self.create_publisher(
            Int32,'/robotis/action/page_num', 10)
        
        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.is_walking = False
        self.current_mode = 'walking'
        self.diagonal_timer = None

        self.get_logger().info('Button Controller Ready')

        self.last_press_time = 0
        self.double_click = 0.5
        self.current_mode = None

    # ------------------------------------------------------------------ #
    #  Module helpers                                                       #
    # ------------------------------------------------------------------ #

    def enable_soccer_mode(self):
        msg = String()
        msg.data = 'soccer_module'
        self.module_pub.publish(msg)

    def enable_walking_mode(self):
        # Reuse parent enable_walking_module
        self.enable_walking_module()

    # ------------------------------------------------------------------ #
    #  Walking start / stop                                                #
    # ------------------------------------------------------------------ #

    def start_walking(self):
        super().start_walking()
        twist = Twist()
        twist.linear.x = 0.03
        self.walking_vel_pub.publish(twist)
        self.is_walking = True

    def state_callback(self, msg):
        self.robot_state = msg.data
        if self.robot_state == "RECOVER" or "KICK":
            self.stop_all()

        elif self.robot_state == "NORMAL":
            self.start_all()

    def stop_walking(self):
        if not self.is_walking:
            return
        cmd = String()
        cmd.data = 'stop'
        self.walking_command_pub.publish(cmd)
        self.is_walking = False

    # ------------------------------------------------------------------ #
    #  Diagonal mode                                                       #
    # ------------------------------------------------------------------ #

    def start_diagonal_mode(self):
        if self.diagonal_timer is not None:
            return
        self.stop_walking()
        self.enable_walking_module()
        time.sleep(0.2)
        self.start_walking()
        self.diagonal_timer = self.create_timer(0.1, self.walk_parameter)

    def stop_diagonal_mode(self):
        if self.diagonal_timer is not None:
            self.diagonal_timer.cancel()
            self.diagonal_timer = None

    # ------------------------------------------------------------------ #
    #  Button callback                                                     #
    # ------------------------------------------------------------------ #

    def button_callback(self, msg: String):
        button = msg.data
        now = time.time()

        if button == 'start':
            if now - self.last_press_time < self.double_click:
                self.stop_all()
                self.current_mode = None
                self.last_press_time = 0
                return
            
            self.last_press_time = now
            self.get_logger().info('START → Jalan Diagonal')
        
            self.stop_all()
            self.start_all()
                

        elif button == 'user':
            self.stop_all()
            self.get_logger().info("USER pressed → Init Pose")
            if self.is_walking:
                self.stop_walking()

            module_msg = String()
            module_msg.data = "action_module"
            self.module_pub.publish(module_msg)

            time.sleep(0.5)  # kasih waktu module aktif

            page = Int32()
            page.data = 15  # page init pose
            self.action_pub.publish(page)

            self.is_walking = False


        elif button == 'mode':
            self.stop_all()

            self.current_mode = 'walking'
            
            self.executor.add_node(HeadControl())
            self.get_logger().info('MODE → Walking Mode')
            self.enable_walking_mode()

    def stop_all(self):
        self.stop_diagonal_mode()
        self.stop_walking()

    def start_all(self):
        self.current_mode = 'walking'
        self.start_diagonal_mode()
        self.executor.add_node(GaitController())  
        self.executor.add_node(FallRecoveryFull())

def main(args=None):
    rclpy.init(args=args)
    node = ButtonSoccerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()