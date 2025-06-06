import rclpy
import py_trees as pt
import py_trees_ros as ptr
from py_trees.behaviours import Failure
from py_trees.blackboard import Blackboard
import threading
from robot_safety_controller.safety_functionalities import (
    BatteryMonitor, CPUMonitor, LaserMonitor, RotateBattery, RotateCPU, StopMotion
)

class LogCondition(pt.behaviour.Behaviour):
    def __init__(self, name, key, expected_value):
        super().__init__(name)
        self.key = key
        self.expected_value = expected_value

    def update(self):
        actual = Blackboard.storage.get(self.key, None)  
        match = actual == self.expected_value
        print(f"[{self.name}] {self.key} = {actual} (expected: {self.expected_value}) -> {match}")
        return pt.common.Status.SUCCESS if match else pt.common.Status.FAILURE

def create_root():
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    # === Blackboard Topic Monitoring ===
    monitors = pt.composites.Parallel(name="Topics2BB", policy=pt.common.ParallelPolicy.SuccessOnAll())
    monitors.add_children([
        BatteryMonitor("Battery2BB"),
        LaserMonitor("Scan2BB"),
        CPUMonitor("CPU2BB")
    ])

    # === Priority Safety Logic ===
    priorities = pt.composites.Selector(name="Priorities", memory=False)

    # --- Collision Handling ---
    collision_check = pt.composites.Sequence(name="Collision Checking", memory=False)
    is_colliding = LogCondition("Is Colliding?", "/collision_detected", True)
    stop_platform = StopMotion("Stop Platform")
    collision_check.add_children([is_colliding, stop_platform])

    # --- CPU Emergency Handling ---
    cpu_check = pt.composites.Sequence(name="CPU Emergency", memory=False)
    is_cpu_temperature_high = LogCondition("CPU Temperature High?", "/temperature_high", True)
    rotate_cpu_platform = RotateCPU("Rotate CPU Platform")
    cpu_check.add_children([is_cpu_temperature_high, rotate_cpu_platform])

    # --- Battery Emergency Handling ---
    battery_check = pt.composites.Sequence(name="Battery Emergency", memory=False)
    is_battery_low = LogCondition("Battery Low?", "/battery_low", True)
    rotate_battery_platform = RotateBattery("Rotate Battery Platform")
    battery_check.add_children([is_battery_low, rotate_battery_platform])

    # --- Idle fallback
    idle = Failure(name="Idle")

    priorities.add_children([
        collision_check,
        cpu_check,
        battery_check,
        idle
    ])

    root.add_children([monitors, priorities])
    return root


def main():
    rclpy.init()
    node = rclpy.create_node(
        'safety_controller',
        automatically_declare_parameters_from_overrides=True
    )
    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root)
    tree.setup(timeout=30.0, node=node)

    try:
        tree.node.get_logger().info("[Main] Behavior tree is running")
        tree.tick_tock(period_ms=100)
        rclpy.spin(tree.node)

    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()