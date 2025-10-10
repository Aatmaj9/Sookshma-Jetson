#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, sys, termios, tty
from datetime import datetime
from pathlib import Path

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
    def __init__(self, save_dir):
        super().__init__('zed_image_saver')
        self.bridge = CvBridge()
        self.latest = None
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.sub = self.create_subscription(
            Image, "/zed/zed_node/image_rect_color", self.cb, 10)
        print("Press SPACE to capture image, Ctrl+C to exit.")

    def cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def listen(self):
        while rclpy.ok():
            key = get_key()
            if key == ' ' and self.latest is not None:
                name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".png"
                path = self.save_dir / name
                cv2.imwrite(str(path), self.latest)
                print(f"Saved {path}")
            elif key in ('\x03', '\x04'):
                break

def main():
    if len(sys.argv) < 2:
        print("Usage: capture_image.py <save_directory>")
        sys.exit(1)

    save_dir = sys.argv[1]
    rclpy.init()
    node = ImageSaver(save_dir)
    try:
        node.listen()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
