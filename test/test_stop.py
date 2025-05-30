import unittest
from unittest.mock import MagicMock
from sensor_msgs.msg import LaserScan
from robot_safety_controller.state_machine import StopMotion
import rclpy
from rclpy.node import Node

class TestStopMotion(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_node', automatically_declare_parameters_from_overrides=True)
        cls.node.declare_parameter('collision.clearance_distance', 0.6)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.state = StopMotion(self.node)

    def test_collision_not_cleared(self):
        scan = LaserScan()
        scan.ranges = [0.2, 0.3]
        self.state.scan_cb(scan)
        self.assertFalse(self.state.collision_cleared)

    def test_collision_cleared_transition(self):
        scan = LaserScan()
        scan.ranges = [1.0, 0.7, 0.8]  # greater than clearance_distance
        self.state.scan_cb(scan)
        self.assertTrue(self.state.collision_cleared)
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'manually_cleared')

if __name__ == '__main__':
    unittest.main()
