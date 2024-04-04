#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense_testing_msgs.msg import CameraInfoStats
import time

class CameraInfoNode(Node):

    def __init__(self):
        super().__init__('camera_info_node')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        # declare class variables
        self.last_time = time.time()
        self.frame_count = 0
        self.info_pub = self.create_publisher(CameraInfoStats, '/camera_info_stats', 10)
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10) # size of message queue

    def image_callback(self, msg):
        current_time = time.time()
        self.frame_count += 1

        # calculate FPS every 10 frames
        if self.frame_count == 10:
            fps = self.frame_count / (current_time - self.last_time)
            self.get_logger().info(f'Received image - Resolution: {msg.width}x{msg.height} @ {fps:.2f} FPS')
            info_msg = CameraInfoStats()
            info_msg.width = msg.width
            info_msg.height = msg.height
            info_msg.fps = fps
            self.info_pub.publish(info_msg)
            self.frame_count = 0
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    camera_info_node = CameraInfoNode()
    rclpy.spin(camera_info_node)

    camera_info_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
