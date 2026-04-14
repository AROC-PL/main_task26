import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from main_task26.bangun import FallRecoveryFull
from main_task26.button import ButtonSoccerNode
from .main_head_control import HeadControl
from .main_jarak import JarakCalculation
# from .main_vision import VisionYolo


class MainTask(Node):
    def __init__(self):
        super().__init__('main_task_node')

        self.create_subscription(
            String,
            '/obj_detect',
            self.obj_callback,
            10
        )

        self.get_logger().info('MainTask Node Started')

    def obj_callback(self, msg: String):
        self.get_logger().info(f'Object detected: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    #vision main vision jangan taruh sini 
    
    main_node = MainTask()
    bangun_node = FallRecoveryFull()
    button_node = ButtonSoccerNode()
    head_control = HeadControl()
    jarak = JarakCalculation()
    
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(head_control)
    executor.add_node(jarak)
    executor.add_node(bangun_node)
    executor.add_node(button_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        button_node.stop_all()
        executor.shutdown()   
        main_node.destroy_node()
        head_control.destroy_node()
        jarak.destroy_node()
        bangun_node.destroy_node()
        button_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()