import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from .main_PID import PIDControl
import json

HEAD_PAN  = 0
HEAD_TILT = 1


class HeadControl(Node):
    """
    Head tracking + scan node.

    PRINSIP ANTI-TABRAKAN:
    ─────────────────────
    • Node ini TIDAK pernah publish ke /robotis/enable_ctrl_module sendiri
      kecuali saat state kembali ke NORMAL.
    • HeadControl hanya bergerak saat state == "NORMAL".
    • Saat state == "RECOVER" atau "KICK", semua output servo berhenti.
    """

    def __init__(self):
        super().__init__('head_control_node')

        # ── State dari sistem ────────────────────────────────────────────
        self.robot_state  = "NORMAL"
        self.head_mode    = "scan"
        self.head_enabled = False   # akan di-set True oleh enable_head_module()

        # ── Subscriber ──────────────────────────────────────────────────
        self.create_subscription(
            String, '/communication/robot_state',
            self.state_callback, 10)

        self.create_subscription(
            String, '/obj_detect',
            self.obj_callback, 10)

        self.create_subscription(
            String, '/pid_params',
            self._pid_params_callback, 10)

        # ── Publisher ────────────────────────────────────────────────────
        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)

        self._page_pub = self.create_publisher(
            Int32, '/robotis/action/page_num', 10)

        self.head_pub = self.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', 10)

        # ── PID ──────────────────────────────────────────────────────────
        self.pid = [PIDControl() for _ in range(2)]

        self.pid[HEAD_TILT].setConstant(Kp=1.5, Ki=0.09, Kd=0.19)
        self.pid[HEAD_TILT].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_TILT].setRange(0, 337, -1.5, -0.2)
        self.pid[HEAD_TILT].setSetPoints(168.5)

        self.pid[HEAD_PAN].setConstant(Kp=0.7, Ki=0.15, Kd=0.2)
        self.pid[HEAD_PAN].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_PAN].setRange(0, 450, -1.5, 1.2)
        self.pid[HEAD_PAN].setSetPoints(225.0)

        for i in range(2):
            self.pid[i].Init()
            self.pid[i].setEnableWindUpLimit()
            self.pid[i].setEnableWindUpCrossing()

        # ── Scan parameter ───────────────────────────────────────────────
        self.pan       = 0.0
        self.tilt      = -0.3
        self.scan_dir  = 1

        self.last_detection_time = self.get_clock().now()
        self.scan_timeout        = 1.0

        # ── Aktifkan head module SETELAH semua inisialisasi selesai ──────
        # Pakai timer sekali jalan (0.3 s) agar publisher sudah terdaftar
        self.create_timer(0.3, self._delayed_enable)

        # ── Main loop ────────────────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("HeadControl READY")

    # ════════════════════════════════════════════════════════════════════
    #  DELAYED ENABLE  — publish modul setelah node benar-benar siap
    # ════════════════════════════════════════════════════════════════════

    def _delayed_enable(self):
        """Dipanggil sekali ~0.3 s setelah __init__ selesai."""
        self.enable_head_module()
        # Batalkan timer ini agar hanya berjalan sekali
        # (ROS2 tidak punya one-shot timer bawaan, jadi kita flag-kan)
        self._delayed_enable_done = True

    # ════════════════════════════════════════════════════════════════════
    #  STATE CALLBACK  (satu definisi, lengkap)
    # ════════════════════════════════════════════════════════════════════

    def state_callback(self, msg: String):
        prev = self.robot_state
        self.robot_state = msg.data

        if self.robot_state in ("RECOVER", "KICK") and prev == "NORMAL":
            # Reset PID supaya tidak ada integral error sisa
            for i in range(2):
                self.pid[i].Init()
            self.head_enabled = False
            self.get_logger().info(f"HeadControl: pause karena state={self.robot_state}")

        elif self.robot_state == "NORMAL" and prev in ("RECOVER", "KICK"):
            # Aktifkan kembali modul kepala
            self.enable_head_module()
            # Mulai dari scan supaya kepala tidak loncat
            self._enter_scan_mode()
            self.get_logger().info("HeadControl: resume → SCAN MODE")

    def _is_active(self) -> bool:
        return self.robot_state == "NORMAL" and self.head_enabled

    # ════════════════════════════════════════════════════════════════════
    #  CONTROL LOOP  (0.05 s)
    # ════════════════════════════════════════════════════════════════════

    def control_loop(self):
        if not self._is_active():
            return

        now = self.get_clock().now()
        dt  = (now - self.last_detection_time).nanoseconds / 1e9

        if dt > self.scan_timeout:
            if self.head_mode != "scan":
                self._enter_scan_mode()
            self._scan_head()
        else:
            if self.head_mode != "track":
                self._enter_track_mode()
            self._track_head()

    # ════════════════════════════════════════════════════════════════════
    #  OBJECT CALLBACK
    # ════════════════════════════════════════════════════════════════════

    def obj_callback(self, msg: String):
        if not self._is_active():
            return

        lines = msg.data.strip().split('\n')
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) != 2:
                continue
            try:
                cx = int(parts[0])
                cy = int(parts[1])
            except ValueError:
                self.get_logger().warn(f"Gagal parse obj_detect: '{line}'")
                continue

            if cx <= 0 or cy <= 0:
                continue

            self.last_detection_time = self.get_clock().now()

            CENTER_X, CENTER_Y = 225, 168
            DEADBAND = 8

            if abs(cx - CENTER_X) >= DEADBAND:
                self.pid[HEAD_PAN].setSetPoints(CENTER_X)
                self.pid[HEAD_PAN].calculate(cx)
                self.pan = self.pid[HEAD_PAN].getOutput()

            if abs(cy - CENTER_Y) >= DEADBAND:
                self.pid[HEAD_TILT].setSetPoints(CENTER_Y)
                self.pid[HEAD_TILT].calculate(cy)
                self.tilt = self.pid[HEAD_TILT].getOutput() - 0.3

    # ════════════════════════════════════════════════════════════════════
    #  PID TUNER CALLBACK
    # ════════════════════════════════════════════════════════════════════

    def _pid_params_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            pan  = data.get("pan",  {})
            tilt = data.get("tilt", {})

            if pan:
                self.pid[HEAD_PAN].setConstant(
                    Kp=float(pan.get("kp", self.pid[HEAD_PAN].Kp)),
                    Ki=float(pan.get("ki", self.pid[HEAD_PAN].Ki)),
                    Kd=float(pan.get("kd", self.pid[HEAD_PAN].Kd)),
                )
                self.pid[HEAD_PAN].Init()
                self.get_logger().info(f"[PID PAN]  {pan}")

            if tilt:
                self.pid[HEAD_TILT].setConstant(
                    Kp=float(tilt.get("kp", self.pid[HEAD_TILT].Kp)),
                    Ki=float(tilt.get("ki", self.pid[HEAD_TILT].Ki)),
                    Kd=float(tilt.get("kd", self.pid[HEAD_TILT].Kd)),
                )
                self.pid[HEAD_TILT].Init()
                self.get_logger().info(f"[PID TILT] {tilt}")

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f"[PID Tuner] Gagal parse: {e}")

    # ════════════════════════════════════════════════════════════════════
    #  MODE TRANSITION
    # ════════════════════════════════════════════════════════════════════

    def _enter_scan_mode(self):
        self.head_mode = "scan"
        for i in range(2):
            self.pid[i].Init()
        self.pan  = 0.0
        self.tilt = -0.3
        self.get_logger().info("HEAD → SCAN MODE")

    def _enter_track_mode(self):
        self.head_mode = "track"
        for i in range(2):
            self.pid[i].Init()
        self.get_logger().info("HEAD → TRACK MODE")

    # ════════════════════════════════════════════════════════════════════
    #  TRACK / SCAN
    # ════════════════════════════════════════════════════════════════════

    def _track_head(self):
        self._publish_servo(self.pan, self.tilt)

    def _scan_head(self):
        self.pan += 0.05 * self.scan_dir

        if self.pan >= 1.2:
            self.pan      = 1.2
            self.scan_dir = -1
        elif self.pan <= -1.2:
            self.pan      = -1.2
            self.scan_dir = 1

        self._publish_servo(self.pan, -0.4)

    # ════════════════════════════════════════════════════════════════════
    #  MODULE CONTROL
    # ════════════════════════════════════════════════════════════════════

    def enable_action_module(self):
        mode = String()
        mode.data = "action_module"
        self.module_pub.publish(mode)
        # self.get_logger().info("Action module enabled")

    def enable_head_module(self):
        mode = String()
        mode.data = "head_control_module"
        self.module_pub.publish(mode)
        self.head_enabled = True
        # self.get_logger().info("Head control module enabled")

    def play_page(self, num):
        cmd = Int32()
        cmd.data = num
        self._page_pub.publish(cmd)   # ← diperbaiki dari self.page_pub

    # ════════════════════════════════════════════════════════════════════
    #  SERVO PUBLISH
    # ════════════════════════════════════════════════════════════════════

    def _publish_servo(self, pan: float, tilt: float):
        if not self.head_enabled:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]
        self.head_pub.publish(msg)


def main():
    rclpy.init()
    node = HeadControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()