import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from .main_PID import PIDControl

HEAD_PAN = 0
HEAD_TILT = 1


class HeadControl(Node):

    def __init__(self):
        super().__init__('button_stand_scan_node')

        # ==============================
        # STATE GLOBAL
        # ==============================
        self.robot_state = "NORMAL"

        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        # ==============================
        # Subscriber
        # ==============================
        self.create_subscription(
            String,
            '/obj_detect',
            self.obj_callback,
            10
        )

        # ==============================
        # Publisher
        # ==============================
        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10
        )

        self.page_pub = self.create_publisher(
            Int32, '/robotis/action/page_num', 10
        )

        self.head_pub = self.create_publisher(
            JointState,
            '/robotis/head_control/set_joint_states',
            10
        )

        # ==============================
        # STATE INTERNAL
        # ==============================
        self.state = 2
        self.enable_action_module()

        self.head_enabled = False
        self.head_mode = "scan"

        self.get_logger().info("AUTO START: Stand + Tracking Aktif")

        # ==============================
        # PID SETUP
        # ==============================
        self.pid = [PIDControl() for _ in range(2)]

        # TILT
        self.pid[HEAD_TILT].setConstant(Kp=1.0, Ki=0.0, Kd=0.2)
        self.pid[HEAD_TILT].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_TILT].setRange(0, 480, -1.2, 0.0)
        self.pid[HEAD_TILT].setSetPoints(240)

        # PAN
        self.pid[HEAD_PAN].setConstant(Kp=1.0, Ki=0.0, Kd=0.07)
        self.pid[HEAD_PAN].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_PAN].setRange(0, 640, -1.2, 1.2)
        self.pid[HEAD_PAN].setSetPoints(320)

        for i in range(2):
            self.pid[i].Init()
            self.pid[i].setEnableWindUpLimit()
            self.pid[i].setEnableWindUpCrossing()

        # ==============================
        # SCAN PARAMETER
        # ==============================
        self.pan = 0.0
        self.tilt = -0.3
        self.scan_dir = 1

        self.last_detection_time = self.get_clock().now()
        self.scan_timeout = 1.0

        # ==============================
        # MAIN LOOP
        # ==============================
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Head Tracking + Auto Scan Initialized")

    # ==============================
    # STATE CALLBACK
    # ==============================
    def state_callback(self, msg):
        self.robot_state = msg.data
        if self.robot_state == "RECOVER":
            self.head_enabled = False
    def is_recovering(self):
        return self.robot_state == "RECOVER"

    # ==========================================================
    # CONTROL LOOP
    # ==========================================================
    def control_loop(self):

        if self.is_recovering():
            return

        if self.state != 2:
            return

        # pastikan head module aktif
        if not self.head_enabled:
            self.enable_head_module()
            return   # ⛔ penting: tunggu next loop

        now = self.get_clock().now()
        dt = (now - self.last_detection_time).nanoseconds / 1e9

        if dt > self.scan_timeout:
            if self.head_mode != "scan":
                self.enter_scan_mode()
            self.scan_head()
        else:
            if self.head_mode != "track":
                self.enter_track_mode()
            self.track_head()
    # ==========================================================
    # OBJECT CALLBACK
    # ==========================================================
    def obj_callback(self, msg):

        # 🔥 STOP TRACKING SAAT RECOVERY
        if self.is_recovering():
            return

        lines = msg.data.strip().split('\n')

        for line in lines:
            parts = line.strip().split(',')

            if len(parts) == 2:
                try:
                    cx = int(parts[0])
                    cy = int(parts[1])

                    center_x = 450 // 2
                    center_y = 337 // 2

                    deadband = 8

                    if cx > 0 and cy > 0:
                        self.last_detection_time = self.get_clock().now()

                        error_x = cx - center_x
                        error_y = cy - center_y

                        if abs(error_x) < deadband:
                            pass
                        else:
                            self.pid[HEAD_PAN].setSetPoints(center_x)
                            self.pid[HEAD_PAN].calculate(cx)
                            self.pan = self.pid[HEAD_PAN].getOutput()

                        if abs(error_y) < deadband:
                            pass
                        else:
                            self.pid[HEAD_TILT].setSetPoints(center_y)
                            self.pid[HEAD_TILT].calculate(cy)
                            self.tilt = self.pid[HEAD_TILT].getOutput() - 0.3

                except:
                    self.get_logger().warn(f"Gagal parse: {line}")

    # ==========================================================
    # MODE TRANSITION
    # ==========================================================
    def enter_scan_mode(self):
        self.head_mode = "scan"

        for i in range(2):
            self.pid[i].Init()

        self.pan = 0.0
        self.tilt = -0.3

        self.get_logger().info("ENTER SCAN MODE")

    def enter_track_mode(self):
        self.head_mode = "track"

        for i in range(2):
            self.pid[i].Init()

        self.get_logger().info("ENTER TRACK MODE")

    # ==========================================================
    # TRACK MODE
    # ==========================================================
    def track_head(self):
        self.publish_servo(self.pan, self.tilt)

    # ==========================================================
    # SCAN MODE
    # ==========================================================
    def scan_head(self):

        self.pan += 0.05 * self.scan_dir

        if self.pan >= 1.2:
            self.pan = 1.2
            self.scan_dir = -1
        elif self.pan <= -1.2:
            self.pan = -1.2
            self.scan_dir = 1

        self.publish_servo(self.pan, -0.4)

    # ==========================================================
    # SERVO PUBLISH
    # ==========================================================
    def publish_servo(self, pan, tilt):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]

        self.head_pub.publish(msg)

    # ==========================================================
    # UTIL
    # ==========================================================
    def enable_action_module(self):
        mode = String()
        mode.data = "action_module"
        self.module_pub.publish(mode)
        self.get_logger().info("Action module enabled")
        

    def enable_head_module(self):
        mode = String()
        mode.data = "head_control_module"
        self.module_pub.publish(mode)
        self.get_logger().info("Head control module enabled")
        self.head_enabled = True  

    def play_page(self, num):
        cmd = Int32()
        cmd.data = num
        self.page_pub.publish(cmd)


def main():
    rclpy.init()
    node = HeadControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()