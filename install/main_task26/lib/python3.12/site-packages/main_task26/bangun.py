import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, String

ACTION_PAGE_FRONT = 142
ACTION_PAGE_BACK  = 143
THRESHOLD         = 9
FILTER_SIZE       = 5
ACTION_DURATION   = 5.0


class FallRecoveryFull(Node):
    def __init__(self):
        super().__init__('fall_recovery_full')

        # ───────── PUB & SUB ─────────
        self.sub_imu = self.create_subscription(
            Imu, '/robotis/open_cr/imu',
            self.imu_callback, 10)

        self.action_pub = self.create_publisher(
            Int32, '/robotis/action/page_num', 10)

        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)

        # 🔥 GLOBAL STATE
        self.state_pub = self.create_publisher(
            String, '/robot_state', 10)

        # ───────── INTERNAL ─────────
        self.acc_buffer = []
        self.is_running = False
        self._reset_timer = None

        self.get_logger().info("Fall Recovery READY (no more chaos)")

    # ───────── IMU CALLBACK ─────────
    def imu_callback(self, msg: Imu):
        acc_x = msg.linear_acceleration.x

        self.acc_buffer.append(acc_x)
        if len(self.acc_buffer) > FILTER_SIZE:
            self.acc_buffer.pop(0)

        avg_acc_x = sum(self.acc_buffer) / len(self.acc_buffer)

        if self.is_running:
            return

        if avg_acc_x < -THRESHOLD:
            self.get_logger().warn("Jatuh DEPAN")
            self.start_recovery(ACTION_PAGE_FRONT)

        elif avg_acc_x > THRESHOLD:
            self.get_logger().warn("Jatuh BELAKANG")
            self.start_recovery(ACTION_PAGE_BACK)

    # ───────── START RECOVERY ─────────
    def start_recovery(self, page):
        self.is_running = True

        # 🔥 publish state
        self.publish_state("RECOVER")

        # aktifkan action module
        self.set_action_module()

        # kirim action
        self.do_action(page)

        # setelah sedikit delay → balik ke head module
        self.create_timer(0.5, self.enable_head_once)

        # timer selesai recovery
        if self._reset_timer:
            self._reset_timer.cancel()

        self._reset_timer = self.create_timer(
            ACTION_DURATION,
            self.finish_recovery
        )

    # ───────── FINISH RECOVERY ─────────
    def finish_recovery(self):
        self.get_logger().info("Recovery selesai → balik NORMAL")

        self.is_running = False
        self.publish_state("NORMAL")

        if self._reset_timer:
            self._reset_timer.cancel()
            self._reset_timer = None

    # ───────── MODULE CONTROL ─────────
    def set_action_module(self):
        msg = String()
        msg.data = 'action_module'
        self.module_pub.publish(msg)

    def enable_head_once(self):
        msg = String()
        msg.data = "head_control_module"
        self.module_pub.publish(msg)
        self.get_logger().info("Head module ON")

    def do_action(self, page):
        msg = Int32()
        msg.data = page
        self.action_pub.publish(msg)

    # ───────── STATE PUB ─────────
    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

