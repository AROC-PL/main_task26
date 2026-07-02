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

        self.action = ActionPublisParam(self)

        self.create_subscription(
            String, '/robotis/open_cr/button', self.button_callback, 10)
        
        self.action_pub = self.create_publisher(
            Int32,'/robotis/action/page_num', 10)
        
        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.is_walking = False

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
            self.action.action_param("action_module")

            time.sleep(0.5)  # kasih waktu module aktif
            self.action.init_pose(9)  # page init pose

            self.is_walking = False
        elif button == 'mode':
            self.current_mode = 'walking'
            self.executor.add_node(HeadControl())
            self.get_logger().info('MODE → Walking Mode')
            self.enable_walking_mode()  
    
    
       