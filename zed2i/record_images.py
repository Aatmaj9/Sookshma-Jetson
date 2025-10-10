#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys, termios, tty
from datetime import datetime
from pathlib import Path

SAVE_DIR = Path("/bags/images")  # or any folder you mounted
SAVE_DIR.mkdir(parents=True, exist_ok=True)

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class ImageSaver(Node):
    def __init__(self):
        super().__init__('zed_image_saver')
        self.bridge = CvBridge()
        self.latest = None
        self.sub = self.create_subscription(
            Image, "/zed/zed_node/image_rect_color", self.cb, 10)
        self.get_logger().info("Press SPACE to save current image, Ctrl+C to exit.")
        self.run()

    def cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == ' ' and self.latest is not None:
                name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".png"
                path = SAVE_DIR / name
                cv2.imwrite(str(path), self.latest)
                print(f"Saved {path}")
            elif key in ('\x03', '\x04'):  # Ctrl+C / Ctrl+D
                break

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
