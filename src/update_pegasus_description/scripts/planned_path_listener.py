#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import time
import os
import math

class PlannedPathListener(Node):
    def __init__(self):
        super().__init__("planned_path_listener")
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/trajectory_status', 10)
        
        # Feedback publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states_feedback', 10)
        
        # Arduino serial
        self.port = os.getenv('ROBOT_PORT', '/dev/ttyUSB0')
        self.arduino = None
        self.connect_serial()
        
        # Subscriber
        self.create_subscription(
            DisplayTrajectory, 
            "/display_planned_path", 
            self.listener_callback, 
            10
        )
        
        # Trajectory Execution State
        self.trajectory_queue = [] # Queue of trajectory segments (lists of points)
        self.current_trajectory = None
        self.traj_point_index = 0
        self.traj_start_time = 0.0
        self.last_point_time = 0.0
        self.traj_count = 0
        
        # Execution Timer (20Hz check)
        self.create_timer(0.05, self.execution_loop)
        
        self.get_logger().info("Listening on /display_planned_path (Buffered Mode)...")

    def connect_serial(self):
        try:
            if self.arduino:
                self.arduino.close()
            self.arduino = serial.Serial(self.port, 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"✅ Connected to Arduino on {self.port}")
            self.publish_status(f"Connected to {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect: {e}")
            self.publish_status("Serial Connection Failed")
            self.arduino = None

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        # self.get_logger().info(message) # Reduce log spam
    
    def listener_callback(self, msg: DisplayTrajectory):
        """
        Callback for new plans. 
        Appends all trajectories in the message to the execution queue, ignoring duplicates.
        """
        if not msg.trajectory:
            return

        added_count = 0
        for traj in msg.trajectory:
            points = traj.joint_trajectory.points
            if not points:
                continue
                
            # Check for duplicates
            last_traj = None
            if self.trajectory_queue:
                last_traj = self.trajectory_queue[-1]
            elif self.current_trajectory:
                last_traj = self.current_trajectory
            
            if last_traj and self.are_trajectories_equal(points, last_traj):
                self.get_logger().info("🚫 Ignoring duplicate trajectory")
                continue

            self.trajectory_queue.append(points)
            added_count += 1
        
        if added_count > 0:
            self.publish_status(f"📥 Buffered {added_count} segments (Queue: {len(self.trajectory_queue)})")

    def are_trajectories_equal(self, traj1, traj2):
        if len(traj1) != len(traj2):
            return False
        
        # Check start and end points first for speed
        if not self.are_points_equal(traj1[0], traj2[0]) or not self.are_points_equal(traj1[-1], traj2[-1]):
            return False
            
        # Check all points
        for p1, p2 in zip(traj1, traj2):
            if not self.are_points_equal(p1, p2):
                return False
        return True

    def are_points_equal(self, p1, p2, tol=1e-4):
        if len(p1.positions) != len(p2.positions):
            return False
        for v1, v2 in zip(p1.positions, p2.positions):
            if abs(v1 - v2) > tol:
                return False
        return True

    def execution_loop(self):
        """
        Periodically checks if we need to send the next point.
        """
        if self.arduino is None or not self.arduino.is_open:
            return

        # If no current trajectory, try to pop from queue
        if self.current_trajectory is None:
            if self.trajectory_queue:
                self.current_trajectory = self.trajectory_queue.pop(0)
                self.traj_point_index = 0
                self.last_point_time = 0.0
                self.traj_count += 1
                self.publish_status(f"▶️ Starting T{self.traj_count} ({len(self.current_trajectory)} pts)")
            else:
                # Nothing to do
                return

        if self.traj_point_index >= len(self.current_trajectory):
            # Done with current segment
            self.current_trajectory = None
            self.publish_status(f"✅ T{self.traj_count} Complete")
            return

        # Get current point
        point = self.current_trajectory[self.traj_point_index]
        
        # Calculate timing
        # point.time_from_start is Duration
        target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        
        # Determine duration for this segment
        segment_duration = target_time - self.last_point_time
        
        # Enforce minimums
        if segment_duration < 0.05: segment_duration = 0.05 # 50ms min
        
        # Send to Arduino
        self.send_point(point, segment_duration)
        
        # Advance
        self.last_point_time = target_time
        self.traj_point_index += 1
        
        # Wait for the duration (Non-blocking sleep? No, we are in a timer callback)
        # Ideally, we shouldn't sleep here, but if we want to pace the Arduino, 
        # we either sleep OR we rely on the timer frequency.
        # Since the Arduino moves over 'duration', we should probably wait that long 
        # before sending the NEXT command.
        # BUT, if we sleep, we block the node.
        # Better approach: 
        # The Arduino buffers? No, our simple code doesn't buffer much.
        # We should sleep to pace it. 
        # For small durations (e.g. 50-100ms), a small sleep is acceptable in a 20Hz loop 
        # if the loop period is larger than the sleep? No.
        
        # Compromise: We sleep here. It blocks the node for a fraction of a second, 
        # but since we want to execute THIS point, it's okay. 
        # New callbacks will still be queued or handled after this sleep.
        
        # OPTIMIZATION: Overlap commands slightly (15ms) to ensure continuous motion
        # This prevents the robot from stopping completely between points
        sleep_time = max(0.0, segment_duration - 0.015)
        time.sleep(sleep_time)

    def send_point(self, point, duration_sec):
        try:
            duration_ms = int(duration_sec * 1000)
            positions = point.positions
            
            if len(positions) < 5: return

            # Map MoveIt joints to Arduino IDs
            # MoveIt: [Base, Shoulder, Elbow, Wrist1, Wrist2]
            # Arduino IDs: {0, 1, 5, 3, 4, 2}
            
            base = positions[0]
            shoulder = positions[1]
            elbow = positions[2]
            wrist1 = positions[3]
            wrist2 = positions[4]
            
            # Convert to steps
            val0 = self.rad_to_pos(0, base)
            val1 = self.rad_to_pos(1, shoulder)
            val2 = self.rad_to_pos(5, shoulder) # ID 5 is Shoulder R (Mirror)
            val3 = self.rad_to_pos(3, elbow)
            val4 = self.rad_to_pos(4, wrist1)
            val5 = self.rad_to_pos(2, wrist2)
            
            vals = [val0, val1, val2, val3, val4, val5]
            
            # Send to Arduino: T{id},{duration},{vals}
            line = f"T{self.traj_count},{duration_ms}," + ",".join(map(str, vals))
            self.arduino.write((line + "\n").encode())
            # self.arduino.flush() # flush can be blocking, maybe skip
            
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def rad_to_pos(self, id_val, rad):
        # Calibrated Centers (Steps at 0 rad)
        # ID 1: 2176, ID 5: 731, ID 3: 3498, ID 4: 3828, ID 2: 900, ID 0: 0
        
        if id_val == 0:
            rad = self.normalize_angle(rad) # Enabled: Ensure +/- 180 deg range
            pass

        steps = int(rad * 651.73)
        if id_val == 0: return 0 + steps
        if id_val == 1: return 2176 + steps
        if id_val == 5: return 731 - steps 
        if id_val == 3: return 3498 + steps
        if id_val == 4: return 3828 + steps
        if id_val == 2: return 900 + steps
        return 2048 + steps

def main(args=None):
    rclpy.init(args=args)
    node = PlannedPathListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'arduino') and node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()