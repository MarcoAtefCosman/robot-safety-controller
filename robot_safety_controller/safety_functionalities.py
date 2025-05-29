import rclpy
import py_trees as pt
from py_trees.blackboard import Blackboard
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate(pt.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_pub = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        return True

    def update(self):
        low = Blackboard.storage.get("/battery_low", None)
        self.node.get_logger().info(f"[Rotate] /battery_low = {low}")
        if low:
            msg = Twist()
            msg.angular.z = 0.5
            self.cmd_pub.publish(msg)
            return pt.common.Status.RUNNING
        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status != pt.common.Status.RUNNING:
            self.cmd_pub.publish(Twist())

class StopMotion(pt.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_pub = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        return True

    def update(self):
        detected = Blackboard.storage.get("/collision_detected", None)
        self.node.get_logger().info(f"[StopMotion] /collision_detected = {detected}")
        if detected:
            self.cmd_pub.publish(Twist())
            return pt.common.Status.RUNNING
        return pt.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status != pt.common.Status.RUNNING:
            self.cmd_pub.publish(Twist())

class BatteryMonitor(pt.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.threshold = 30.0
        self.subscription = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscription = self.node.create_subscription(
            Float32, '/battery_voltage', self.callback, 10
        )
        return True

    def callback(self, msg):
        low = msg.data < self.threshold
        self.blackboard.set("/battery_voltage", msg.data)
        self.blackboard.set("/battery_low", low)
        self.node.get_logger().info(f"[BatteryMonitor] /battery_voltage = {msg.data:.2f} | /battery_low = {low}")

    def update(self):
        return pt.common.Status.RUNNING

class LaserMonitor(pt.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.subscription = None
        self.safe_distance = 0.25

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscription = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        return True

    def callback(self, msg):
        try:
            valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]
            if valid_ranges:
                min_dist = min(valid_ranges)
                detected = min_dist < self.safe_distance
                self.blackboard.set("/collision_detected", detected)
                self.node.get_logger().info(f"[LaserMonitor] dist = {min_dist:.2f} | /collision_detected = {detected}")
        except Exception as e:
            self.node.get_logger().error(f"[LaserMonitor] Error: {str(e)}")

    def update(self):
        return pt.common.Status.RUNNING
