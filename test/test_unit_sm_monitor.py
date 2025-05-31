import unittest
from unittest.mock import MagicMock
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from robot_safety_controller.state_machine import Monitor
import rclpy
from rclpy.node import Node

class TestMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_node', automatically_declare_parameters_from_overrides=True)
        cls.node.declare_parameter('battery.max_voltage', 100.0)
        cls.node.declare_parameter('battery.min_voltage', 0.0)
        cls.node.declare_parameter('battery.low_threshold', 30.0)
        cls.node.declare_parameter('collision.stop_distance', 0.4)
        cls.node.declare_parameter('cpu.max_temperature_threshold', 60.0)
        cls.node.declare_parameter('cpu.normal_temperature_threshold', 20.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.state = Monitor(self.node)

    def test_low_battery_triggers_transition(self):
        self.state.battery_cb(Float32(data=25.0))
        self.assertEqual(self.state.battery_level, 25.0)
        self.state.collision_detected = False
        # simulate the return value from execute
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'low_battery')

    def test_high_temperature_triggers_transition(self):
        self.state.cpu_cb(Float32(data=70.0))
        self.assertEqual(self.state.cpu_temperature, 70.0)
        self.state.collision_detected = False
        # simulate the return value from execute
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'cpu_overheat')

    def test_collision_detected_triggers_transition(self):
        self.state.battery_cb(Float32(data=50.0))
        scan = LaserScan()
        scan.ranges = [0.3,0.6,0.7]  
        self.state.scan_cb(scan)
        self.assertTrue(self.state.collision_detected)
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'collision_detected')

    def test_normal_operation_returns_normal(self):
        self.state.battery_cb(Float32(data=80.0))
        self.state.cpu_cb(Float32(data=40.0))
        scan = LaserScan()
        scan.ranges = [0.5, 0.6, 0.7]
        self.state.scan_cb(scan)
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'normal')

if __name__ == '__main__':
    unittest.main()
