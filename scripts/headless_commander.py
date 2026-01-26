#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import os
import threading
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Empty

# Add path to pegasus_commander
current_dir = os.path.dirname(os.path.abspath(__file__))
pegasus_script_path = os.path.abspath(os.path.join(current_dir, "../src/update_pegasus_description/scripts"))
sys.path.append(pegasus_script_path)

from pegasus_commander import PegasusCommander

class HeadlessCommander(Node):
    def __init__(self):
        super().__init__('headless_commander')
        
        self.get_logger().info("Initializing Headless Commander...")
        
        # Initialize the actual commander
        # We don't want it to spin its own node, so we might need to be careful
        # PegasusCommander inherits from Node. We can't have two nodes with same name easily
        # But PegasusCommander is named 'pegasus_commander_internal'
        self.commander = PegasusCommander(wait_for_services=True)
        
        # Subscribers for GUI commands
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/pegasus/command/joint_target',
            self.joint_command_callback,
            10
        )
        
        self.pose_cmd_sub = self.create_subscription(
            PoseStamped,
            '/pegasus/command/pose_target',
            self.pose_command_callback,
            10
        )
        
        self.stop_cmd_sub = self.create_subscription(
            Empty,
            '/pegasus/command/stop',
            self.stop_command_callback,
            10
        )
        
        # Publishers for status
        self.execution_status_pub = self.create_publisher(
            Bool,
            '/pegasus/status/execution',
            10
        )
        
        self.pose_status_pub = self.create_publisher(
            PoseStamped,
            '/pegasus/status/pose',
            10
        )
        
        # Timer for status updates
        self.create_timer(0.1, self.status_timer_callback)
        
        self.get_logger().info("Headless Commander Ready! Waiting for commands...")

    def joint_command_callback(self, msg):
        """Handle joint target commands"""
        self.get_logger().info(f"[DEBUG] Received joint command: {msg.position}")
        # Map incoming joint names/positions to commander's expected order if needed
        # For now assuming msg.position matches [joint1, joint2, joint3, joint4, joint5]
        
        # We need to convert tuple/list to what plan_to_joint_values expects
        try:
            result = self.commander.plan_to_joint_values(list(msg.position))
            if result['success']:
                self.commander.execute_live_trajectory(result)
            else:
                self.get_logger().warn(f"Planning failed: {result['message']}")
        except Exception as e:
            self.get_logger().error(f"Error executing joint command: {e}")

    def pose_command_callback(self, msg):
        """Handle pose target commands"""
        self.get_logger().info(f"[DEBUG] Received pose command: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")
        try:
            # Extract pose
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Convert quaternion to RPY if needed, or if plan_to_cartesian_pose supports quat
            # PegasusCommander doesn't seem to have plan_to_cartesian_pose in the file I read earlier?
            # Let me check PegasusCommander again. It had plan_to_joint_values.
            # I might need to implement plan_to_cartesian_pose in PegasusCommander or here.
            # Wait, I saw plan_to_cartesian_pose in the grep results for GUI_commander!
            # It must be in PegasusCommander but I missed it in the view_file (maybe it was further down).
            
            # Assuming it exists:
            if hasattr(self.commander, 'plan_to_cartesian_pose'):
                # We need to convert quat to RPY for the current API likely
                # Or just pass what it needs.
                # For now, let's assume we need to extract RPY
                from scipy.spatial.transform import Rotation as R
                r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                roll, pitch, yaw = r.as_euler('xyz', degrees=False)
                
                result = self.commander.plan_to_cartesian_pose(x, y, z, roll, pitch, yaw, duration=0.1)
                self.get_logger().info(f"[DEBUG] Plan result: {result['success']}")
                if result['success']:
                    # Pass the FULL result dictionary so it can find 'raw_trajectory'
                    self.get_logger().info(f"[DEBUG] Executing trajectory")
                    self.commander.execute_live_trajectory(result)
            else:
                self.get_logger().error("PegasusCommander missing plan_to_cartesian_pose method")
                
        except Exception as e:
            self.get_logger().error(f"Error executing pose command: {e}")

    def stop_command_callback(self, msg):
        """Handle stop command"""
        self.get_logger().info("Received STOP command")
        if hasattr(self.commander, 'stop_execution'):
            self.commander.stop_execution()
        elif hasattr(self.commander, 'is_executing'):
            self.commander.is_executing = False

    def status_timer_callback(self):
        """Publish status updates"""
        # Execution status
        is_executing = getattr(self.commander, 'is_executing', False)
        self.execution_status_pub.publish(Bool(data=is_executing))
        
        # Current pose
        current_pose = self.commander.get_current_pose() # [x, y, z, r, p, y]
        if current_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.pose.position.x = current_pose[0]
            msg.pose.position.y = current_pose[1]
            msg.pose.position.z = current_pose[2]
            # Convert RPY back to quat
            from scipy.spatial.transform import Rotation as R
            q = R.from_euler('xyz', [current_pose[3], current_pose[4], current_pose[5]], degrees=False).as_quat()
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            
            self.pose_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # We need to spin both nodes. 
    # Since PegasusCommander is a Node, and HeadlessCommander is a Node.
    # We can use a MultiThreadedExecutor.
    
    headless = HeadlessCommander()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(headless)
    executor.add_node(headless.commander)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        headless.destroy_node()
        headless.commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
