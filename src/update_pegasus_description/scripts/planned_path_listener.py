#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import time

class PlannedPathListener(Node):
    def __init__(self):
        super().__init__("planned_path_listener")

        # --- Arduino Serial (adjust port if needed) ---
        self.arduino = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
        time.sleep(2)  # wait for Arduino to reset
        self.get_logger().info("✅ Connected to Arduino on /dev/ttyUSB0")

        # --- ROS2 subscriber ---
        self.create_subscription(DisplayTrajectory, "/display_planned_path", self.listener_callback, 10)
        self.traj_count = 0
        self.get_logger().info("Listening on /display_planned_path ...")

    def listener_callback(self, msg: DisplayTrajectory):
        """
        Extract joint names and positions from DisplayTrajectory
        and send them to Arduino in 'T#,val1,val2,...' format.
        """
        for traj in msg.trajectory:
            joint_names = traj.joint_trajectory.joint_names
            points = traj.joint_trajectory.points

            for point in points:
                # Format values
                vals = [int(pos * 1000) for pos in point.positions]  # scale example
                line = f"T{self.traj_count}," + ",".join(map(str, vals))

                # Send to Arduino
                self.arduino.write((line + "\n").encode("utf-8"))
                self.get_logger().info(f"📡 Sent to Arduino: {line}")

            self.traj_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PlannedPathListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

