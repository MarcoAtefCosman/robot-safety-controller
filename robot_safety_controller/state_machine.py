import rclpy
import smach
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class Monitor(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'collision_detected', 'cpu_overheat' ,'normal'])
        self.node = node
        
        self.battery_level =  node.get_parameter('battery.max_voltage').value
        self.low_battery_threshold = node.get_parameter('battery.low_threshold').value
        self.low_battery_detected = False

        self.stop_distance = node.get_parameter('collision.stop_distance').value
        self.collision_detected = False

        self.high_temperature_threshold = node.get_parameter('cpu.max_temperature_threshold').value
        self.cpu_temperature = node.get_parameter('cpu.normal_temperature_threshold').value
        self.high_temperature_detected = False
        
        # Create subscribers with callback groups
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.battery_sub = node.create_subscription(
            Float32, '/battery_voltage', self.battery_cb, 10,
            callback_group=self.callback_group)
        
        self.cpu_sub = node.create_subscription(
            Float32, '/cpu_temperature', self.cpu_cb, 10,
            callback_group=self.callback_group)
        
        self.scan_sub = node.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10,
            callback_group=self.callback_group)
        
        self.node.get_logger().info("Monitor state initialized")

    def battery_cb(self, msg):
        self.battery_level = msg.data
        self.low_battery_detected = self.battery_level <= self.low_battery_threshold
        self.node.get_logger().info(f"Battery update: {self.battery_level}")
    
    def cpu_cb(self, msg):
        self.cpu_temperature = msg.data
        self.high_temperature_detected = self.cpu_temperature >= self.high_temperature_threshold
        self.node.get_logger().info(f"CPU temperature update: {self.cpu_temperature}")

    def scan_cb(self, msg):
        if not msg.ranges:
            self.node.get_logger().warning("Empty scan received!")
            return
        min_distance = min(msg.ranges)
        self.collision_detected = min_distance <= self.stop_distance
        self.node.get_logger().info(f"Scan update: {min_distance:.2f}m")

    def execute(self, userdata):
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                self.node.get_logger().info("Monitoring...")
                executor.spin_once(timeout_sec=0.1)
                if self.collision_detected:
                    self.node.get_logger().error("COLLISION DETECTED!")
                    return 'collision_detected'
                
                elif self.high_temperature_detected:
                    self.node.get_logger().error("CPU OVERHEATED!")
                    return 'cpu_overheat'
                
                elif self.low_battery_detected:
                    self.node.get_logger().error("LOW BATTERY!")
                    return 'low_battery'
                
                return 'normal'
        finally:
            executor.remove_node(self.node)
        

class RotateBattery(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['battery_ok'])
        self.node = node

        self.battery_level =  node.get_parameter('battery.max_voltage').value
        self.low_battery_threshold = node.get_parameter('battery.low_threshold').value
        self.normal_battery_detected = False
        
        self.rotation_speed = node.get_parameter('recovery.rotation_speed').value

        # Create cmd publisher
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscribers with callback groups
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.battery_sub = node.create_subscription(
            Float32, '/battery_voltage', self.battery_cb, 10,
            callback_group=self.callback_group)
        
        self.node.get_logger().info("Rotate state initialized")

    def battery_cb(self, msg):
        self.battery_level = msg.data
        self.normal_battery_detected = self.battery_level > self.low_battery_threshold
        self.node.get_logger().info(f"Battery update: {self.battery_level}")

    def execute(self, userdata):
        rotate_msg = Twist()
        rotate_msg.angular.z = self.rotation_speed
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                self.node.get_logger().error("RECHARGE THE BATTERY!")
                executor.spin_once(timeout_sec=0.1)
                self.cmd_vel_pub.publish(rotate_msg)
                
                if self.normal_battery_detected:
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
                    self.node.get_logger().info("Battery recovered, stopping rotation")
                    return 'battery_ok'
                    
        finally:
            executor.remove_node(self.node)

class RotateCpu(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['cpu_ok'])
        self.node = node

        self.high_temperature_threshold = node.get_parameter('cpu.max_temperature_threshold').value
        self.cpu_temperature = node.get_parameter('cpu.normal_temperature_threshold').value
        self.normal_temperature_detected = False
        
        self.rotation_speed = node.get_parameter('recovery.rotation_speed').value

        # Create cmd publisher
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscribers with callback groups
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.cpu_sub = node.create_subscription(
            Float32, '/cpu_temperature', self.cpu_cb, 10,
            callback_group=self.callback_group)
        
        self.node.get_logger().info("Rotate state initialized")

    def cpu_cb(self, msg):
        self.cpu_temperature = msg.data
        self.normal_temperature_detected = (self.high_temperature_threshold - self.cpu_temperature) >= 15.0
        self.node.get_logger().info(f"CPU temperature update: {self.cpu_temperature}")

    def execute(self, userdata):
        rotate_msg = Twist()
        rotate_msg.angular.z = -1 * self.rotation_speed
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                self.node.get_logger().error("TURN ON THE COOLING SYSTEM!")
                executor.spin_once(timeout_sec=0.1)
                self.cmd_vel_pub.publish(rotate_msg)
                
                if self.normal_temperature_detected:
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
                    self.node.get_logger().info("CPU recovered, stopping rotation")
                    return 'cpu_ok'
                    
        finally:
            executor.remove_node(self.node)

class StopMotion(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['manually_cleared'])
        self.node = node

        self.clearance_distance = node.get_parameter('collision.clearance_distance').value
        self.collision_cleared = False

        # Create cmd publisher
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscribers with callback groups
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
        stop_msg = Twist()
        
        # Create executor for this state
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        try:
            while rclpy.ok():
                self.node.get_logger().error("EMERGENCY STOP ACTIVATED!")
                executor.spin_once(timeout_sec=0.1)
                self.cmd_vel_pub.publish(stop_msg)

                if self.collision_cleared:
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
        smach.StateMachine.add('MONITOR', Monitor(node),
            transitions={
                'low_battery': 'ROTATE_BATTERY',
                'cpu_overheat': 'ROTATE_CPU',
                'collision_detected': 'STOP',
                'normal': 'MONITOR'
            })
            
        smach.StateMachine.add('ROTATE_BATTERY', RotateBattery(node),
            transitions={'battery_ok': 'MONITOR'})
        
        smach.StateMachine.add('ROTATE_CPU', RotateCpu(node),
            transitions={'cpu_ok': 'MONITOR'})
            
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