import unittest
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from robot_safety_controller.state_machine import Monitor, RotateBattery, RotateCpu, StopMotion
import rclpy
from rclpy.node import Node

class TestSafetyIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("integration_test_node", automatically_declare_parameters_from_overrides=True)
        cls.node.declare_parameter('battery.max_voltage', 100.0)
        cls.node.declare_parameter('battery.low_threshold', 30.0)
        cls.node.declare_parameter('collision.stop_distance', 0.4)
        cls.node.declare_parameter('collision.clearance_distance', 0.6)
        cls.node.declare_parameter('recovery.rotation_speed', 0.5)
        cls.node.declare_parameter('cpu.max_temperature_threshold', 60.0)
        cls.node.declare_parameter('cpu.normal_temperature_threshold', 20.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_monitor_to_rotate_battery(self):
        monitor = Monitor(self.node)
        rotate = RotateBattery(self.node)

        # Simulate low battery -> transition to low_battery
        monitor.battery_cb(Float32(data=20.0))
        monitor.cpu_cb(Float32(data=40.0))
        monitor.scan_cb(LaserScan(ranges=[1.0]))
        outcome = monitor.execute(userdata={})
        self.assertEqual(outcome, 'low_battery')

        # Rotate until battery recovered
        rotate.battery_cb(Float32(data=100.0))
        outcome = rotate.execute(userdata={})
        self.assertEqual(outcome, 'battery_ok')

    def test_monitor_to_rotate_cpu(self):
        monitor = Monitor(self.node)
        rotate = RotateCpu(self.node)

        # Simulate high temperature -> transition to high temperature
        monitor.battery_cb(Float32(data=80.0))
        monitor.cpu_cb(Float32(data=70.0))
        monitor.scan_cb(LaserScan(ranges=[1.0]))
        outcome = monitor.execute(userdata={})
        self.assertEqual(outcome, 'cpu_overheat')

        # Rotate until battery recovered
        rotate.cpu_cb(Float32(data=40.0))
        outcome = rotate.execute(userdata={})
        self.assertEqual(outcome, 'cpu_ok')        

    def test_monitor_to_stop(self):
        monitor = Monitor(self.node)
        stop = StopMotion(self.node)

        # Simulate collision -> transition to collision_detected
        monitor.battery_cb(Float32(data=80.0))  
        monitor.cpu_cb(Float32(data=40.0))
        monitor.scan_cb(LaserScan(ranges=[0.2])) 
        outcome = monitor.execute(userdata={})
        self.assertEqual(outcome, 'collision_detected')

        # Stop motion should be triggered
        stop.scan_cb(LaserScan(ranges=[0.7])) 
        outcome = stop.execute(userdata={})
        self.assertEqual(outcome, 'manually_cleared')

if __name__ == '__main__':
    unittest.main()
