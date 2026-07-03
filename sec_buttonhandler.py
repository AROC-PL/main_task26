import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

from .main_head_control import HeadControl
from .sec_action import ActionPublisParam

class SecButtonSoccerNode(Node):
    def __init__(self):
        super().__init__('Sec_button_soccer_node')

        self.action = ActionPublisParam()

        self.create_subscription(
            String, '/robotis/open_cr/button', self.button_callback, 10)
        
        self.action_pub = self.create_publisher(
            Int32,'/robotis/action/page_num', 10)
        
        self.get_logger().info('Secondary Button Controller Ready')
        self.is_walking = False
        self.is_posisi = False
        self.current_mode = 'walking'
        self.double_click = 0.5
        self.last_press_time = 0

    def button_callback(self, msg: String):
        button = msg.data
        now = time.time()

        if button == 'mode':
            self.current_mode = 'walking'
            self.executor.add_node(HeadControl())
            self.get_logger().info('MODE → Walking Mode')
            # self.enable_walking_mode() 
             
        elif button == 'start':
            if self.is_walking == True:
                self.get_logger().info('START → Stop Walking')
                self.action.stop_walking()
                self.is_walking = False
                self.action.set_walking_state("Stop_jalan")   
            else:      
                self.last_press_time = now
                self.get_logger().info('START → Jalan Diagonal')
                self.action.action_param("walking_module")
                time.sleep(0.5)  # kasih waktu module aktif
                self.action.start_walking()
                self.action.set_walking_state("Jalan")
                self.is_walking = True
                self.action.start_recovery()

        elif button == 'user':
            if self.is_posisi == False:
                self.get_logger().info("USER pressed → Init Pose")
                self.action.action_param("action_module")
                time.sleep(0.5)  # kasih waktu module aktif
                self.action.init_pose(9)  # page init pose
                self.is_posisi = False
            else:
                self.get_logger().info("USER pressed → bringup pose")
                self.action.action_param("action_module")
                time.sleep(0.5)  
                self.action.init_pose(15)  # 
                self.is_posisi = True