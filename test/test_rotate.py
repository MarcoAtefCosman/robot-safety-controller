
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from robot_safety_controller.state_machine import RotateBase
import time

class TestRotateBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_node', automatically_declare_parameters_from_overrides=True)
        cls.node.declare_parameter('recovery.rotation_speed', 0.5)
        cls.node.declare_parameter('battery.max_voltage', 100.0)
        cls.node.declare_parameter('battery.low_threshold', 30.0)


    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.state = RotateBase(self.node)

    def test_rotation_stops_when_battery_recovers(self):
        # Simulate recovered battery voltage
        self.state.battery_cb(Float32(data=100.0))
        outcome = self.state.execute(userdata={})
        self.assertEqual(outcome, 'battery_ok')

    def test_twist_message_published(self):
        
        # Store cmd messages
        messages = []

        def fake_publish(msg):
            messages.append(msg)

        self.state.cmd_vel_pub.publish = fake_publish

        # Start with low battery
        self.state.battery_cb(Float32(data=10.0))

        # Schedule a battery recovery after 1s
        def recover_battery():
            self.state.battery_cb(Float32(data=100.0))

        self.node.create_timer(1, recover_battery)

        outcome = self.state.execute(userdata={})

        self.assertEqual(outcome, 'battery_ok')
        self.assertTrue(len(messages) > 0, "No Twist messages were published.")
        self.assertTrue(
            any(abs(msg.angular.z - 0.5) < 1e-3 for msg in messages),
            "No Twist message had angular.z == 0.5"
        )
        

if __name__ == '__main__':
    unittest.main()
