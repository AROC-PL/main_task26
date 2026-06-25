#!/usr/bin/env python3
"""
imu_yaw_node.py  –  ROS2 Node
─────────────────────────────
• Subscribe  : /robotis/open_cr/imu
• Hitung     : Yaw dari quaternion
• Kirim      : Raw yaw (radian) ke GUI via TCP socket (port 5005)
• Format     : satu baris per data  →  "1.2345\n"

Jalankan:
    python3 imu_yaw_node.py
"""

import math
import socket
import threading
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

HOST = "127.0.0.1"
PORT = 5005


class YawSocketServer:
    """
    TCP server sederhana.
    GUI connect sebagai client; node kirim yaw terus menerus.
    Mendukung banyak client sekaligus.
    """

    def __init__(self, host=HOST, port=PORT):
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()

        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(5)
        self._srv.setblocking(False)

        t = threading.Thread(target=self._accept_loop, daemon=True)
        t.start()
        print(f"[SocketServer] Listening on {host}:{port}")

    # ── Accept loop ───────────────────────────────────────────────────────────
    def _accept_loop(self):
        import select
        while True:
            readable, _, _ = select.select([self._srv], [], [], 1.0)
            if readable:
                conn, addr = self._srv.accept()
                with self._lock:
                    self._clients.append(conn)
                print(f"[SocketServer] GUI connected from {addr}")

    # ── Broadcast ke semua client ─────────────────────────────────────────────
    def send(self, yaw_rad: float):
        msg = f"{yaw_rad:.6f}\n".encode()
        dead = []
        with self._lock:
            for c in self._clients:
                try:
                    c.sendall(msg)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                print("[SocketServer] Client disconnected")

    def close(self):
        with self._lock:
            for c in self._clients:
                c.close()
        self._srv.close()


# ═════════════════════════════════════════════════════════════════════════════
class ImuYawNode(Node):
    def __init__(self, server: YawSocketServer):
        super().__init__("imu_yaw_node")
        self._server = server
        self.sub = self.create_subscription(
            Imu,
            "/robotis/open_cr/imu",
            self._callback,
            10,
        )
        self.get_logger().info("ImuYawNode started – sending data to GUI via socket")

    def _callback(self, msg: Imu):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        atas  = 2.0 * (w * z + x * y)
        bawah = 1.0 - 2.0 * (y * y + z * z)
        yaw   = math.atan2(atas, bawah)   # radian

        self._server.send(yaw)

        self.get_logger().info(f"Yaw: {math.degrees(yaw)}°  →  sent to GUI")
        time.sleep(1)

# ═════════════════════════════════════════════════════════════════════════════
def main():
    server = YawSocketServer()

    rclpy.init()
    node = ImuYawNode(server)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Node] Shutting down…")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.close()


if __name__ == "__main__":
    main()