import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class JarakCalculation(Node):
    def __init__(self):
        super().__init__('jarak_calculation')

        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )
        

        self.tinggi_robot = 45
        self.sudut_kamera = 0.0
        self.sudut_mat = 1.575
        self.kalibrasi_jarak =60    #cm 

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "head_tilt":
                self.sudut_kamera = msg.position[i]
                self.get_logger().info(f"Head tilt: {self.sudut_kamera}")
                self.hitung_jarak()

    def hitung_jarak(self):
        if abs(math.tan(self.sudut_kamera)) < 1e-6:
            return
        
        sudut_rad = self.sudut_kamera-self.sudut_mat
        # radian = math.radians(sudut_rad)
        jarak = (self.tinggi_robot*math.tan(sudut_rad))

        print(f"sudutrad = {sudut_rad}")
        # print(f"radian = {radian}")
        jarak_final = jarak-self.kalibrasi_jarak 
        print(f"jarak|| = {jarak_final}")

def main(args=None):
    rclpy.init(args=args)
    node = JarakCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()