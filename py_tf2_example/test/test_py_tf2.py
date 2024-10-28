import unittest
import rclpy
from py_tf2_example.py_tf2_node import LookupTransformNode

class TestLookupTransformNode(unittest.TestCase):

    def test_lookup_transform(self):
        rclpy.init()
        node = LookupTransformNode()
        transform = node.lookup_transform()
        self.assertIsNotNone(transform)

if __name__ == '__main__':
    unittest.main()