import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detection_node')

        self.bridge = CvBridge()
        #self.model = YOLO("/home/krishm701/Desktop/aritra_files/ros2_ws/src/yolo_detection_nv2/runs2/runs/detect/train/weights/best.pt")
        self.confidence_threshold = 0.85
        self.uv_image = np.zeros((720, 1280, 3), dtype=np.uint8)  # Assuming 720p resolution

        # Subscribers using message_filters
        #self.image_sub = Subscriber(self, Image, "/wamv/sensors/cameras/front_left_camera_sensor/image_raw")
        self.image_sub = Subscriber(self, Image, "/zed2i/left/image_rect_color")
        self.uv_sub = Subscriber(self, PoseArray, "/uv_xyz")

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.uv_sub],
            queue_size=10,
            slop=0.1  # tolerance in seconds
        )
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg, uv_msg):
        self.update_uv_image(uv_msg)

        try:
            cam_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            return

        # Blend UV overlay
        overlay = cv2.addWeighted(cam_image, 1.0, self.uv_image, 0.5, 0)

        # Run YOLO detection
        results = self.model(overlay)
        self.classify_buoy(overlay, results)

        # Display result
        cv2.imshow("YOLO + UV Overlay", overlay)
        cv2.waitKey(1)

    def update_uv_image(self, msg):
        self.uv_image = np.zeros((720, 1280, 3), dtype=np.uint8)

        if not msg.poses:
            return

        # Extract and validate UV coordinates
        uvs = np.array([[int(p.orientation.x), int(p.orientation.y)] for p in msg.poses])
        valid_mask = (uvs[:, 0] >= 0) & (uvs[:, 0] < 1280) & \
                     (uvs[:, 1] >= 0) & (uvs[:, 1] < 720)
        uvs = uvs[valid_mask]

        # Draw red circles for UV points
        radius = 3
        for x, y in uvs:
            cv2.circle(self.uv_image, (x, y), radius, (0, 0, 255), -1)

    def classify_buoy(self, image, results):
        for result in results:
            for box in result.boxes:
                conf = box.conf.item() if hasattr(box.conf, 'item') else float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                u_center = (x1 + x2) // 2
                v_center = (y1 + y2) // 2

                buoy_roi = image[y1:y2, x1:x2]
                if buoy_roi.size == 0:
                    continue

                hsv = cv2.cvtColor(buoy_roi, cv2.COLOR_BGR2HSV)
                red_mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) + \
                           cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
                green_mask = cv2.inRange(hsv, (36, 100, 100), (86, 255, 255))

                red_pixels = cv2.countNonZero(red_mask)
                green_pixels = cv2.countNonZero(green_mask)

                if red_pixels > green_pixels:
                    color = "Red"
                    color_code = (0, 0, 255)
                else:
                    color = "Green"
                    color_code = (0, 255, 0)

                cv2.rectangle(image, (x1, y1), (x2, y2), color_code, 2)
                label = f"{color} ({conf*100:.1f}%)"
                cv2.putText(image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_code, 2)
                cv2.circle(image, (u_center, v_center), radius=4, color=(255, 255, 0), thickness=-1)
                cv2.putText(image, f"({u_center}, {v_center})", (u_center + 5, v_center - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()