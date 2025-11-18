#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import time
import threading

class PlannedPathListener(Node):
    def __init__(self):
        super().__init__("planned_path_listener")

        # Trajectory buffer
        self.trajectory_buffer = []
        self.buffer_lock = threading.Lock()
        self.processing = False
        self.last_send_time = 0
        self.min_send_interval = 0.005  # 5ms minimum interval between sends
        
        # --- Arduino Serial (adjust port if needed) ---
        self.port = "/dev/ttyUSB0"  # Or '/tmp/virtual_arduino_sim' for simulation
        try:
            self.arduino = serial.Serial(self.port, 115200, timeout=1, write_timeout=1)
            self.arduino.flushInput()
            self.arduino.flushOutput()
            time.sleep(1)  # Reduced wait time
            if self.arduino.is_open:
                self.get_logger().info(f"✅ Connected to Arduino on {self.port}")
            else:
                raise serial.SerialException("Failed to open serial port")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to Arduino on {self.port}: {e}")
            self.arduino = None
            return

        # --- ROS2 subscriber ---
        self.create_subscription(DisplayTrajectory, "/display_planned_path", self.listener_callback, 10)
        self.traj_count = 0
        self.get_logger().info("Listening on /display_planned_path ...")

    def listener_callback(self, msg: DisplayTrajectory):
        """
        Buffer trajectory points and send them efficiently to Arduino.
        Uses milliradians for precise position control.
        """
        if self.arduino is None or not self.arduino.is_open:
            self.get_logger().warn("Serial connection not available, skipping trajectory")
            return
            
        with self.buffer_lock:
            if self.processing:
                self.get_logger().warn("Already processing a trajectory, skipping new one")
                return
            self.processing = True
            self.trajectory_buffer.clear()

        for traj in msg.trajectory:
            joint_names = traj.joint_trajectory.joint_names
            points = traj.joint_trajectory.points

            if not points:
                self.get_logger().warn("Empty trajectory, skipping")
                continue

            # Log joint names for verification
            self.get_logger().info(f"Trajectory joints: {joint_names}")

            # Buffer all points first
            trajectory_points = []
            total_duration = 0.0
            
            for point in points:
                try:
                    positions = point.positions
                    if len(positions) != 5:
                        self.get_logger().warn(f"Unexpected number of joints: {len(positions)}, expected 5")
                        continue
                        
                    # Scale positions to match Arduino's expectations (milliradians)
                    full_positions = list(positions) + [0.0]  # Add dummy 6th joint
                    vals = [int(pos * 1000) for pos in full_positions]  # Convert to milliradians
                    
                    # Store point timing
                    point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    if point_time > total_duration:
                        total_duration = point_time
                        
                    trajectory_points.append((vals, point_time))
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing point: {e}")
                    continue
            
            if not trajectory_points:
                self.get_logger().warn("No valid points in trajectory")
                self.processing = False
                return
                
            # Send all points as a batch with timing
            try:
                start_time = time.time()
                
                # Send trajectory start marker
                cmd = f"T{self.traj_count}\n"  # Simplified format
                self.arduino.write(cmd.encode())
                self.arduino.flush()
                time.sleep(0.05)  # Increased delay for Arduino to process
                
                # Send all points
                for i, (vals, target_time) in enumerate(trajectory_points):
                    # Format point data
                    point_cmd = f"P{i}," + ",".join(map(str, vals)) + "\n"
                    self.arduino.write(point_cmd.encode())
                    self.arduino.flush()
                    
                    # Calculate and maintain timing
                    elapsed = time.time() - start_time
                    if i < len(trajectory_points) - 1:
                        next_time = trajectory_points[i + 1][1]
                        sleep_time = max(0, next_time - elapsed)
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                
                self.get_logger().info(f"✅ Sent trajectory T{self.traj_count} ({len(trajectory_points)} points, {total_duration:.2f}s)")
                self.traj_count += 1
                
            except Exception as e:
                self.get_logger().error(f"Failed to send trajectory: {e}")
            finally:
                self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = PlannedPathListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received shutdown signal")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        if hasattr(node, 'arduino') and node.arduino is not None:
            try:
                node.arduino.close()
                node.get_logger().info("Serial port closed")
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()