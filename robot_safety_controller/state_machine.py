import rclpy
import smach
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class MonitorBatteryAndCollision(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'collision_detected', 'normal'])
        self.node = node
        
        self.battery_level =  node.get_parameter('battery.max_voltage').value
        self.min_battery_level = node.get_parameter('battery.min_voltage').value
        self.low_battery_threshold = node.get_parameter('battery.low_threshold').value
        
        self.stop_distance = node.get_parameter('collision.stop_distance').value
        
        self.collision_detected = False
        
        # Create subscribers with callback groups
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.battery_sub = node.create_subscription(
            Float32, '/battery_voltage', self.battery_cb, 10,
            callback_group=self.callback_group)
        self.scan_sub = node.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10,
            callback_group=self.callback_group)
        
        self.node.get_logger().info("Monitor state initialized")

    def battery_cb(self, msg):
        self.battery_level = msg.data
        self.node.get_logger().info(f"Battery update: {self.battery_level}")

    def scan_cb(self, msg):
        if not msg.ranges:
            self.node.get_logger().warning("Empty scan received!")
            return
            
        min_distance = min(msg.ranges)
        self.collision_detected = min_distance < self.stop_distance
        self.node.get_logger().info(f"Scan update: {min_distance:.2f}m")

    def execute(self, userdata):
        self.node.get_logger().info("Starting monitoring...")
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                if self.collision_detected:
                    self.node.get_logger().error("COLLISION DETECTED!")
                    return 'collision_detected'
                elif self.battery_level < self.low_battery_threshold:
                    self.node.get_logger().error("LOW BATTERY!")
                    return 'low_battery'
        finally:
            executor.remove_node(self.node)
        
        return 'normal'

class RotateBase(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['battery_ok'])
        self.node = node

        self.battery_level = node.get_parameter('battery.max_voltage').value
        self.low_battery_threshold = node.get_parameter('battery.low_threshold').value
        self.rotation_speed = node.get_parameter('recovery.rotation_speed').value

        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.battery_sub = node.create_subscription(
            Float32, '/battery_voltage', self.battery_cb, 10,
            callback_group=self.callback_group)
        
        self.node.get_logger().info("Rotate state initialized")

    def battery_cb(self, msg):
        self.battery_level = msg.data
        self.node.get_logger().info(f"Battery update: {self.battery_level}")

    def execute(self, userdata):
        self.node.get_logger().info("Starting rotation...")
        rotate_msg = Twist()
        rotate_msg.angular.z = self.rotation_speed
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                self.cmd_vel_pub.publish(rotate_msg)
                
                if self.battery_level >= self.low_battery_threshold:
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
                    self.node.get_logger().info("Battery recovered, stopping rotation")
                    return 'battery_ok'
                    
        finally:
            executor.remove_node(self.node)
            
        return 'battery_ok'

class StopMotion(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['manually_cleared'])
        self.node = node

        self.clearance_distance = node.get_parameter('collision.clearance_distance').value
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.collision_cleared = False

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.scan_sub = node.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10,
            callback_group=self.callback_group)
        
        
        self.node.get_logger().info("Stop state initialized")

    def scan_cb(self, msg):
        if not msg.ranges:
            self.node.get_logger().warning("Empty scan received!")
            return
            
        min_distance = min(msg.ranges)
        self.collision_cleared = min_distance > self.clearance_distance
        self.node.get_logger().info(f"Scan update: {min_distance:.2f}m")

    def execute(self, userdata):
        self.node.get_logger().error("EMERGENCY STOP ACTIVATED!")
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok() and not self.collision_cleared:
                executor.spin_once(timeout_sec=0.1)
                
            self.node.get_logger().info("Obstacle cleared, resuming operation")
            return 'manually_cleared'
            
        finally:
            executor.remove_node(self.node)

def main(args=None):
    rclpy.init(args=args)
    node = Node(
    'safety_controller',
    automatically_declare_parameters_from_overrides=True
    )
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['shutdown'])
    
    with sm:
        smach.StateMachine.add('MONITOR', MonitorBatteryAndCollision(node),
            transitions={
                'low_battery': 'ROTATE',
                'collision_detected': 'STOP',
                'normal': 'MONITOR'
            })
            
        smach.StateMachine.add('ROTATE', RotateBase(node),
            transitions={'battery_ok': 'MONITOR'})
            
        smach.StateMachine.add('STOP', StopMotion(node),
            transitions={'manually_cleared': 'MONITOR'})
    
    # Main executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Starting state machine")
        outcome = sm.execute()
        node.get_logger().info(f"State machine finished with outcome: {outcome}")
    except Exception as e:
        node.get_logger().error(f"Error in state machine: {str(e)}")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()