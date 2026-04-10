import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math

tinggiRobot = 50
sudutKamera = 0.0
sudutMati = 1.575
kalibrasiJarak = 30 

class JarakCalculation(Node):
    def __init__(self):
        super().__init__('jarak_calculation')

        self.create_subscription(
            JointState,
            '/robotis/goal_joint_states',
            self.joint_callback,
            10
        )
        self.jarak_pub = self.create_publisher(
            Float32,
            '/jarak_bola',
            10
        )

        self.tinggi_robot = tinggiRobot
        self.sudut_kamera = sudutKamera
        self.sudut_mat = sudutMati
        self.kalibrasi_jarak =kalibrasiJarak  

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "head_tilt":
                self.sudut_kamera = msg.position[i]
                # self.get_logger().info(f"Head tilt: {self.sudut_kamera}")
                self.hitung_jarak()

    def hitung_jarak(self):
        if abs(math.tan(self.sudut_kamera)) < 1e-6:
            return
        
        sudut_rad = self.sudut_kamera-self.sudut_mat
        # radian = math.radians(sudut_rad)
        jarak = (self.tinggi_robot*math.tan(sudut_rad))

        # print(f"sudutrad = {sudut_rad}")
        # print(f"radian = {radian}")
        jarak_final = jarak-self.kalibrasi_jarak 
        # print(f"jarak = {jarak_final}")
        msg = Float32()
        msg.data = jarak_final
        self.jarak_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JarakCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
