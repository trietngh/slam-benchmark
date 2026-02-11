import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_ = self.create_publisher(Image, '/test/topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.i = 0

    def timer_callback(self):
        # Create a dummy black image
        cv_image = np.zeros((100, 100, 3), dtype=np.uint8)
        # Put some text
        cv2.putText(cv_image, str(self.i), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing image {self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = DummyPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
