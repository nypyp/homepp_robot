import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class LookupTransformNode(Node):

    def __init__(self):
        super().__init__('lookup_transform_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self):
        try:
            timeout = rclpy.time.Duration(seconds=1.0)  # 设置超时时间
            while not self.tf_buffer.can_transform('camera_link', 'base_link', rclpy.time.Time(), timeout):
                self.get_logger().info('Waiting for transform...')    
            transform = self.tf_buffer.lookup_transform('camera_link', 'base_link', rclpy.time.Time())
            self.get_logger().info('Transform: %s' % transform)
            return transform
        except Exception as e:
            self.get_logger().error('Failed to lookup transform: %s' % str(e))
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LookupTransformNode()
    transform = node.lookup_transform()
    rclpy.shutdown()

if __name__ == '__main__':
    main()