import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO

SET_CONF = 0.6
IMG_WIDTH = 450
IMG_HEIGHT = 337


class VisionYolo(Node):
    def __init__(self):
        super().__init__('usb_cam_yolo')

        self.bridge = CvBridge()

        #MODEL HARUS DI CONVERT DI PC YANG AKAN DI GUNAKAN!!!!! WAJIB !!!!

        self.model = YOLO(
            "/home/aroc/model/best_openvino_model",
            task="detect"
        )

        # FPS
        self.prev_time = 0.0
        self.fps = 0.0

        # Publisher hasil deteksi
        self.obj_pub = self.create_publisher(
            String,
            '/obj_detect',
            10
        )

        # Subscriber kamera
        self.create_subscription(
            Image,
            '/usb_cam_node/image_raw',
            self.image_callback,
            1
        )

        self.get_logger().info("✅ YOLO OpenVINO ROS2 Node Jalan")

    def image_callback(self, msg):
        try:
            # =========================
            # IMAGE CONVERT
            # =========================
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )

            frame = cv2.resize(frame, (IMG_WIDTH, IMG_HEIGHT))

            # =========================
            # YOLO INFERENCE
            # =========================
            results = self.model(
                frame,
                imgsz=960,
                conf=SET_CONF,
                device="cpu",
                verbose=False
            )

            r = results[0]
            annotated_frame = r.plot()

            best_conf = 0.0
            best_cx = None
            best_cy = None

            # =========================
            # AMBIL DETEKSI TERBAIK (BOLA)
            # =========================
            if r.boxes is not None and len(r.boxes) > 0:

                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = r.names[cls_id]

                    if class_name != "bola":
                        continue

                    if conf > best_conf:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        best_conf = conf
                        best_cx = cx
                        best_cy = cy

            # =========================
            # PUBLISH
            # =========================
            if best_cx is not None:
                self.get_logger().info(
                    f"BEST DETECTION ||||| conf={best_conf:.2f} | center=({best_cx},{best_cy})"
                )

                msg_out = String()
                msg_out.data = f"{best_cx},{best_cy}"
                self.obj_pub.publish(msg_out)

            # =========================
            # HITUNG FPS
            # =========================
            curr_time = time.time()
            if self.prev_time != 0:
                self.fps = 1.0 / (curr_time - self.prev_time)
            self.prev_time = curr_time

            cv2.putText(
                annotated_frame,
                f"FPS: {self.fps:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # =========================
            # DISPLAY
            # =========================
            cv2.imshow("YOLO OpenVINO ROS2", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = VisionYolo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()