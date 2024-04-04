import rclpy
from rclpy.node import Node
from realsense_testing_msgs.msg import Example 

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Example, 'example_topic', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Example()
        msg.message = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.message}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
