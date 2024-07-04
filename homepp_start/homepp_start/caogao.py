import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from launch import LaunchDescription
from launch_ros.actions import Node

class KeywordDirectionManager(Node):
    def __init__(self):
        super().__init__()

        self.latest_sound_direction = None
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.keyword_subscriber = self.create_subscription(
            String,
            '/keyword_detection',
            self.keyword_callback,
            10)
        self.get_logger().debug('Subscribed to /keyword_detection topic')

        self.direction_subscriber = self.create_subscription(
            Int32,
            '/sound_direction',
            self.direction_callback,
            10)
        self.get_logger().debug('Subscribed to /sound_direction topic')

        self.keyword_direction_publisher = self.create_publisher(
            Int32,
            '/keyword_direction',
            10)
        self.get_logger().debug('Publishing to /keyword_direction topic')

        self.rotation_complete_subscriber = self.create_subscription(
            String,
            '/rotation_complete',
            self.rotation_complete_callback,
            10)
        self.get_logger().debug('Subscribed to /rotation_complete topic')

    def keyword_callback(self, msg):
        self.get_logger().debug('Received keyword: "%s"' % msg.data)
        if self.latest_sound_direction is not None:
            self.publish_keyword_direction()

    def direction_callback(self, msg):
        self.latest_sound_direction = msg.data
        self.get_logger().debug('Received sound direction: %d' % self.latest_sound_direction)

    def publish_keyword_direction(self):
        msg = Int32()
        msg.data = self.latest_sound_direction
        self.keyword_direction_publisher.publish(msg)
        self.get_logger().debug('Published keyword direction: %d' % msg.data)

    def rotation_complete_callback(self, msg):
        self.get_logger().info("received")
        if msg.data == "done":
            self.get_logger().info("Received rotation complete message: %s" % msg.data)
            self.start_yolo_follower()

    def start_yolo_follower(self):
        self.get_logger().info("Starting yoloFollower node...")
        
        # Replace 'yoloFollower' with your actual node name and adjust parameters accordingly
        node = Node(
            package='homepp_follower',
            executable='yolofollower',
            namespace='yolofollower',
            output='screen',
            emulate_tty=True,
        )
        self.get_logger().debug('Launching yoloFollower node...')

        # Launch the node
        executor = self.create_executable_context()
        self.get_logger().info("Starting yoloFollower node...")
        rclpy.get_global_executor().add_node(node)
        future = node.execute()

def main(args=None):
    rclpy.init(args=args)

    node = KeywordDirectionManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
