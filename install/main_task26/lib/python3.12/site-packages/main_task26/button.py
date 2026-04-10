import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

from main_task26.crab_walk import GaitController


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

        self.is_walking = False
        self.current_mode = 'walking'
        self.diagonal_timer = None

        self.get_logger().info('Button Controller Ready')

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

        if button == 'user':
            self.get_logger().info('USER → Jalan Diagonal')
            self.current_mode = 'walking'
            self.stop_all()
            self.start_diagonal_mode()

        elif button == 'start':
            self.get_logger().info('START → Jalan Biasa')
            self.stop_diagonal_mode()
            if not self.is_walking:
                self.enable_walking_module()
                time.sleep(0.5)
                self.start_walking()

        elif button == 'mode':
            self.stop_all()
            if self.current_mode == 'walking':
                self.current_mode = 'soccer'
                self.get_logger().info('MODE → Soccer Mode')
                self.enable_soccer_mode()
            else:
                self.current_mode = 'walking'
                self.get_logger().info('MODE → Walking Mode')
                self.enable_walking_mode()

    def stop_all(self):
        self.stop_diagonal_mode()
        self.stop_walking()


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