import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess

class KeywordDirectionManager(Node):
    def __init__(self):
        super().__init__('keyword_direction_manager')

        self.latest_sound_direction = None
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        

        self.keyword_subscriber = self.create_subscription(
            String,
            '/keyword_detection',
            self.keyword_callback,
            10)

        self.direction_subscriber = self.create_subscription(
            Int32,
            '/sound_direction',
            self.direction_callback,
            10)

        self.rotation_complete_subscriber = self.create_subscription(
            String,
            '/rotation_complete',
            self.rotation_complete_callback,
            10)

        self.keyword_direction_publisher = self.create_publisher(
            Int32,
            '/keyword_direction',
            10)

    def keyword_callback(self, msg):
        self.get_logger().debug('Received keyword: "%s"' % msg.data)
        if self.latest_sound_direction is not None:
            self.publish_keyword_direction()

    def direction_callback(self, msg):
        self.latest_sound_direction = msg.data

    def rotation_complete_callback(self, msg):
        if msg.data == "done":
            self.get_logger().debug('Received rotation complete message: "%s"' % msg.data)
            # Start the homepp_follower yolofollower package
            subprocess.run(["ros2", "run", "homepp_follower", "yoloFollower"])
            self.get_logger().debug('Started homepp_follower yoloFollower package')

    def publish_keyword_direction(self):
        msg = Int32()
        msg.data = self.latest_sound_direction
        self.keyword_direction_publisher.publish(msg)
        self.get_logger().debug('Published keyword direction: %d' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = KeywordDirectionManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
