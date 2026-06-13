import math
import copy

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import String

from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam


class AdaptiveWalking(Node):
    """
    Adaptive walking node untuk ROBOTIS OP3.

    Versi awal ini masih rule-based agar aman untuk robot asli.
    Bagian choose_mode_rule_based() nanti bisa diganti dengan model deep learning.
    """

    def __init__(self):
        super().__init__('adaptive_walking_node')

        self.create_subscription(
            Imu,
            '/robotis/open_cr/imu',
            self.imu_callback,
            10
        )

        self.walking_param_pub = self.create_publisher(
            WalkingParam,
            '/robotis/walking/set_params',
            10
        )

        self.walking_cmd_pub = self.create_publisher(
            String,
            '/robotis/walking/command',
            10
        )

        self.get_param_client = self.create_client(
            GetWalkingParam,
            '/robotis/walking/get_params'
        )

        self.latest_imu = None
        self.current_param = None
        self.param_future = None
        self.last_mode = None
        self.enabled = True

        # Update parameter tidak perlu terlalu cepat.
        # Walking module OP3 tetap berjalan pada control loop internalnya sendiri.
        self.timer = self.create_timer(0.3, self.control_loop)

        self.get_logger().info('AdaptiveWalking READY')

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def estimate_roll_pitch(self, imu_msg: Imu):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt((ay * ay) + (az * az)))

        return roll, pitch

    def request_param_once(self):
        if self.current_param is not None:
            return

        if not self.get_param_client.service_is_ready():
            self.get_logger().warn('/robotis/walking/get_params belum ready')
            return

        if self.param_future is None:
            req = GetWalkingParam.Request()
            self.param_future = self.get_param_client.call_async(req)
            self.get_logger().info('Request walking parameter...')
            return

        if self.param_future.done():
            result = self.param_future.result()
            self.param_future = None

            if result is None:
                self.get_logger().warn('Gagal mengambil walking parameter')
                return

            self.current_param = result.parameters
            self.get_logger().info('Walking parameter diterima')

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def safe_set(self, param, field_name, value):
        if hasattr(param, field_name):
            setattr(param, field_name, value)
        else:
            self.get_logger().warn(f'Field {field_name} tidak ada di WalkingParam')

    def publish_walking_command(self, command):
        msg = String()
        msg.data = command
        self.walking_cmd_pub.publish(msg)

    def choose_mode_rule_based(self, roll, pitch):
        """
        Placeholder untuk deep learning.

        Nanti fungsi ini diganti menjadi:
        features -> model inference -> mode walking.
        """
        roll_abs = abs(roll)
        pitch_abs = abs(pitch)

        # Sekitar 20 derajat. Kalau lebih dari ini, lebih aman stop.
        if roll_abs > 0.35 or pitch_abs > 0.35:
            return 'stop'

        # Sekitar 9 derajat.
        if pitch_abs > 0.16:
            return 'pitch_balance'

        if roll_abs > 0.16:
            return 'roll_balance'

        return 'normal'

    def apply_mode_to_param(self, base_param, mode):
        param = copy.deepcopy(base_param)

        if mode == 'normal':
            self.safe_set(param, 'x_move_amplitude', 0.010)
            self.safe_set(param, 'period_time', 0.600)
            self.safe_set(param, 'balance_ankle_pitch_gain', 0.9)
            self.safe_set(param, 'balance_ankle_roll_gain', 1.0)

        elif mode == 'pitch_balance':
            # Robot mulai miring depan/belakang.
            # Langkah diperkecil dan ankle pitch gain dinaikkan.
            self.safe_set(param, 'x_move_amplitude', 0.006)
            self.safe_set(param, 'period_time', 0.680)
            self.safe_set(param, 'balance_ankle_pitch_gain', 1.15)
            self.safe_set(param, 'balance_ankle_roll_gain', 1.0)

        elif mode == 'roll_balance':
            # Robot mulai miring kiri/kanan.
            # Langkah diperkecil dan ankle roll gain dinaikkan.
            self.safe_set(param, 'x_move_amplitude', 0.006)
            self.safe_set(param, 'period_time', 0.680)
            self.safe_set(param, 'balance_ankle_pitch_gain', 0.9)
            self.safe_set(param, 'balance_ankle_roll_gain', 1.25)

        elif mode == 'stop':
            self.publish_walking_command('stop')
            return None

        # Safety limiter. Jangan percaya output adaptif tanpa pagar.
        if hasattr(param, 'x_move_amplitude'):
            param.x_move_amplitude = self.clamp(param.x_move_amplitude, 0.000, 0.020)

        if hasattr(param, 'period_time'):
            param.period_time = self.clamp(param.period_time, 0.550, 0.800)

        if hasattr(param, 'balance_ankle_pitch_gain'):
            param.balance_ankle_pitch_gain = self.clamp(param.balance_ankle_pitch_gain, 0.5, 1.5)

        if hasattr(param, 'balance_ankle_roll_gain'):
            param.balance_ankle_roll_gain = self.clamp(param.balance_ankle_roll_gain, 0.5, 1.5)

        return param

    def control_loop(self):
        if not self.enabled:
            return

        self.request_param_once()

        if self.latest_imu is None:
            self.get_logger().info('Menunggu data IMU...')
            return

        if self.current_param is None:
            return

        roll, pitch = self.estimate_roll_pitch(self.latest_imu)
        mode = self.choose_mode_rule_based(roll, pitch)

        new_param = self.apply_mode_to_param(self.current_param, mode)

        if new_param is not None:
            self.walking_param_pub.publish(new_param)

        if self.last_mode != mode:
            log_msg = f'MODE={mode} | roll={roll:.3f} rad | pitch={pitch:.3f} rad'
            if mode == 'stop':
                self.get_logger().warn(log_msg)
            else:
                self.get_logger().info(log_msg)

        self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveWalking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
