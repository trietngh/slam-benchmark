import os
import sys
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
COLOR_WARN = '\033[93m'
COLOR_ERROR = '\033[91m'
COLOR_RESET = '\033[0m'

class FrameSaverNode(Node):
    def __init__(self):
        super().__init__('frame_saver_node')

        # Parameters
        self.declare_parameter('topics', [''], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter('save_rate', 0.0)
        self.declare_parameter('save_dir', '.')
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('mode', 'async')
        self.declare_parameter('timeout', 0.1)

        self.topics = [t for t in self.get_parameter('topics').value if t]
        self.save_rate = self.get_parameter('save_rate').value
        self.save_dir = self.get_parameter('save_dir').value
        self.encoding = self.get_parameter('encoding').value
        self.mode = self.get_parameter('mode').value
        self.timeout = self.get_parameter('timeout').value

        if not self.topics:
            self.log_warn("No topics specified to save.")
            return

        # Setup save directory
        try:
            self.run_dir = self._setup_run_directory()
            self.topic_dirs = self._setup_topic_directories()
        except OSError as e:
            self.log_error(f"Failed to setup save directories: {e}")
            sys.exit(1)

        self.bridge = CvBridge()
        self.subs = []

        # Sync mode buffer
        self.latest_msgs = {}

        # Async mode rate limiting
        self.last_save_time = {}

        if self.mode == 'async':
            if self.save_rate <= 0:
                self.get_logger().info(f"Starting in ASYNC mode")
            else:
                self.get_logger().info(f"Starting in ASYNC mode with rate {self.save_rate} Hz")
            for i, topic in enumerate(self.topics):
                # Use a lambda to capture topic index
                callback = lambda msg, index=i+1: self.async_callback(msg, index)
                sub = self.create_subscription(Image, topic, callback, 10)
                self.subs.append(sub)
                self.last_save_time[i+1] = 0.0
        elif self.mode == 'sync':
            self.get_logger().info(f"Starting in SYNC mode with rate {self.save_rate} Hz")
            if self.save_rate <= 0:
                self.log_error("Save rate must be > 0 for sync mode. Defaulting to 1.0 Hz.")
                self.save_rate = 1.0
            
            for i, topic in enumerate(self.topics):
                 # Use a lambda to capture topic index
                callback = lambda msg, index=i+1: self.sync_buffer_callback(msg, index)
                # Ensure the buffer size is enough
                sub = self.create_subscription(Image, topic, callback, 10)
                self.subs.append(sub)
            
            self.timer = self.create_timer(1.0 / self.save_rate, self.sync_timer_callback)
        else:
             self.log_error(f"Unknown mode: {self.mode}. Use 'async' or 'sync'.")

    def log_warn(self, msg):
        self.get_logger().warn(f"{COLOR_WARN}{msg}{COLOR_RESET}")

    def log_error(self, msg):
        self.get_logger().error(f"{COLOR_ERROR}{msg}{COLOR_RESET}")

    def _setup_run_directory(self):
        # Determine run directory name: YYMMDD_NNN
        date_str = datetime.now().strftime("%y%m%d")
        
        if not os.path.exists(self.save_dir):
            try:
                os.makedirs(self.save_dir, exist_ok=True)
            except OSError as e:
                 raise OSError(f"Could not create base directory '{self.save_dir}': {e}")
        
        if not os.access(self.save_dir, os.W_OK):
             raise OSError(f"Base directory '{self.save_dir}' is not writable.")

        # Find next available index
        index = 1
        while True:
            run_name = f"{date_str}_{index:03d}"
            path = os.path.join(self.save_dir, run_name)
            if not os.path.exists(path):
                break
            index += 1
        
        try:
            os.makedirs(path)
        except OSError as e:
            raise OSError(f"Could not create run directory '{path}': {e}")

        self.get_logger().info(f"Saving images to: {path}")
        return path

    def _setup_topic_directories(self):
        dirs = {}
        for i, topic in enumerate(self.topics):
            topic_index = i + 1
            path = os.path.join(self.run_dir, str(topic_index))
            try:
                os.makedirs(path, exist_ok=True)
            except OSError as e:
                raise OSError(f"Could not create topic directory '{path}': {e}")
            dirs[topic_index] = path
            self.get_logger().info(f"Topic {topic} -> {path}")
        return dirs

    def async_callback(self, msg, topic_index):
        # Use ROS time for consistency with simulation
        now_ros = self.get_clock().now().nanoseconds / 1e9

        if self.save_rate > 0:
            if (now_ros - self.last_save_time.get(topic_index, 0.0)) < (1.0 / self.save_rate):
                return
        
        self.last_save_time[topic_index] = now_ros
        self.save_image(msg, topic_index)

    def sync_buffer_callback(self, msg, topic_index):
        self.latest_msgs[topic_index] = msg

    def sync_timer_callback(self):
        now = self.get_clock().now()
        
        for i, topic in enumerate(self.topics):
            topic_index = i + 1
            msg = self.latest_msgs.get(topic_index)
            
            if msg:
                # Check timeout
                msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
                age = (now - msg_time).nanoseconds / 1e9
                
                if age < self.timeout:
                    self.save_image(msg, topic_index)
                else:
                    # Message too old
                    self.log_warn(f"Message from topic {topic_index} is too old")
                    self.log_warn(f"Message age: {age}")
                    self.log_warn(f"Now: {now.nanoseconds / 1e9}")
                    self.log_warn(f"Msg time: {msg_time.nanoseconds / 1e9}")
                    pass

    def save_image(self, msg, topic_index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except CvBridgeError as e:
            self.log_error(f"CvBridge Error: {e}")
            return

        timestamp = msg.header.stamp
        # Filename: timestamp.png
        # Use valid chars. ROS2 timestamp is sec.nanosec
        filename = f"{timestamp.sec}.{timestamp.nanosec:09d}.png"
        path = os.path.join(self.topic_dirs[topic_index], filename)
        
        try:
            cv2.imwrite(path, cv_image)
        except OSError as e:
            self.log_error(f"Failed to save image to '{path}': {e}")
        # self.get_logger().debug(f"Saved {path}")

def main(args=None):
    rclpy.init(args=args)
    
    # Check for errors during init
    try:
        node = FrameSaverNode()
        rclpy.spin(node)
        node.destroy_node()
    except SystemExit:
        pass
    except Exception as e:
        print(f"{COLOR_ERROR}Fatal error: {e}{COLOR_RESET}")
    finally:
        # Attempt silent shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
