#!/usr/bin/env python3
import sys
import os
import yaml
import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock, Event
from functools import partial
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotState, Constraints, PositionIKRequest, DisplayTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory
import logging
import serial
import math
from std_msgs.msg import Bool, String, Empty
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from builtin_interfaces.msg import Duration as ROSDuration

class PegasusCommander(Node):  

    def __init__(self, wait_for_services=True):
        # Note: Added wait_for_services arg to match old signature, though we might ignore it or use it
        print("[PegasusCommander] Calling super().__init__...")
        sys.stdout.flush()
        
        # Initialize node with minimal blocking
        try:
            super().__init__('pegasus_commander_internal') # Changed name to avoid conflict if run as script
            print("[PegasusCommander] super().__init__() complete")
        except Exception as e:
            print(f"[PegasusCommander] Warning: super().__init__() raised: {e}")
        
        sys.stdout.flush()
        self.logger = self.get_logger()
        self.load_joint_limits()
        self.is_executing = False

        # Determine default joint_limits file path
        try:
            pkg_share = get_package_share_directory('pegasus_arm_moveit_config')
            default_joint_limits = os.path.join(pkg_share, 'config', 'joint_limits.yaml')
        except Exception:
            default_joint_limits = os.path.join(os.path.dirname(__file__), 'joint_limits.yaml')

        self.declare_parameter('controller_name', 'pegasus_arm_controller')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('end_effector_frame', 'end_effector_link')
        self.declare_parameter('joint_limits_file', default_joint_limits)

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        self.joint_limits_file = self.get_parameter('joint_limits_file').get_parameter_value().string_value

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]

        # Named poses
        self.named_poses = {
            'home': [0.0, 1.3984, -1.3001, -1.31, 0.0],
            'extended': [-6.2, -0.2676, -0.1221, 0.3017, -6.2]
        }

        # Initialize state flags
        self.controller_manager_available = False
        self.action_server_available = False
        self.controller_available = False
        self.joint_states_available = False
        self.trajectory_topic_available = False
        self.tf_available = False

        self.goal_lock = Lock()
        self.callback_lock = Lock()

        self.current_joint_state = None
        self._current_values = [0.0] * len(self.joint_names)
        self.current_goal_handle = None

        # Track MoveIt state
        self.moveit_enabled = True

        # Arduino serial setup
        self.arduino_serial = None
        self.arduino_port = '/tmp/virtual_arduino_sim' # Default port
        self.arduino_baudrate = 9600
        self._setup_arduino_serial()

        print("[PegasusCommander] Initializing services and topics...")
        sys.stdout.flush()
        # Initialize services and topics first (skip blocking waits)
        self.initialize_services_and_topics()
        print("[PegasusCommander] Services and topics initialized")
        sys.stdout.flush()
        
        print("[PegasusCommander] Checking TF availability...")
        sys.stdout.flush()
        # Now check TF availability (after tf_buffer is created)
        self.check_tf_availability(level_check=True)
        print("[PegasusCommander] TF check complete")
        sys.stdout.flush()
        
        print("[PegasusCommander] Creating timer...")
        sys.stdout.flush()
        # Create timer for periodic checks
        self.create_timer(2.0, self.check_services_and_topics)
        print("[PegasusCommander] Timer created")
        sys.stdout.flush()



    def load_joint_limits(self):
        """Load joint limits from YAML file or use defaults"""
        try:
            if hasattr(self, 'joint_limits_file') and os.path.exists(self.joint_limits_file):
                with open(self.joint_limits_file, 'r') as file:
                    joint_limits_data = yaml.safe_load(file)
                
                self.joint_limits = {}
                for joint_name in self.joint_names:
                    joint_data = joint_limits_data.get('joint_limits', {}).get(joint_name, {})
                    if joint_data.get('has_position_limits', False):
                        min_pos = joint_data.get('min_position', -3.14)
                        max_pos = joint_data.get('max_position', 3.14)
                    else:
                        # If no position limits in YAML, use defaults
                        min_pos, max_pos = -3.14, 3.14
                    self.joint_limits[joint_name] = (min_pos, max_pos)
                
                self.logger.info(f"Loaded joint limits from {self.joint_limits_file}")
            else:
                # Use default limits if file doesn't exist
                self._use_default_joint_limits()
        except Exception as e:
            self.logger.error(f"Failed to load joint limits: {str(e)}")
            self._use_default_joint_limits()

    def _use_default_joint_limits(self):
        """Set default joint limits"""
        self.joint_limits = {
            'joint1': (-6.28, 6.28),
            'joint2': (-0.61, 0.61),
            'joint3': (-1.75, 1.75),
            'joint4': (-1.31, 1.31),
            'joint5': (-6.28, 6.28)
        }
        self.logger.info("Using default joint limits")









    def play_trajectory(self, traj_msg, hz=30.0):
        """Send trajectory with consolidated logging"""
        try:
            if not hasattr(self, 'traj_action_client') or self.traj_action_client is None:
                self.get_logger().error(' No trajectory action client available')
                return
            
            if not self.traj_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(' Action server not available')
                return
            
            self.is_executing = True
            
            from control_msgs.action import FollowJointTrajectory
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = traj_msg
            
            self.get_logger().info(f" Executing trajectory: {len(traj_msg.points)} points")
            
            send_goal_future = self.traj_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._trajectory_feedback_callback
            )
            send_goal_future.add_done_callback(self._trajectory_goal_response_callback)
            
        except Exception as e:
            self.get_logger().error(f' Trajectory execution error: {e}')
            self.is_executing = False








    def _trajectory_feedback_callback(self, feedback_msg):
        """Handle trajectory execution feedback"""
        # Optional: Log progress
        pass



    def _trajectory_goal_response_callback(self, future):
        """Handle trajectory goal response with status logging"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(" Trajectory goal REJECTED")
                self.is_executing = False
                return
            
            self.get_logger().info(" Trajectory goal ACCEPTED")
            
            with self.goal_lock:
                self.current_goal_handle = goal_handle
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._trajectory_result_callback)
            
        except Exception as e:
            self.get_logger().error(f" Goal response error: {str(e)}")
            self.is_executing = False





    def _trajectory_result_callback(self, future):
        """Handle trajectory result with clear status"""
        try:
            result = future.result().result
            status = future.result().status
            
            if status == 4:  # SUCCEEDED
                self.get_logger().info(" Trajectory execution COMPLETED")
            elif status == 5:  # ABORTED
                error_str = result.error_string if hasattr(result, 'error_string') else 'Unknown error'
                self.get_logger().error(f"Trajectory ABORTED: {error_str}")
            elif status == 6:  # PREEMPTED
                self.get_logger().warn("⚠ Trajectory PREEMPTED (cancelled)")
            else:
                self.get_logger().warn(f"⚠ Trajectory ended with status {status}")
            
        except Exception as e:
            self.get_logger().error(f" Result callback error: {str(e)}")
        finally:
            #  ALWAYS clear execution flag
            self.is_executing = False
            if hasattr(self, 'goal_lock'):
                with self.goal_lock:
                    self.current_goal_handle = None







    def diagnose_publishers(self):
        """Check what's publishing to /joint_states"""
        if not self.commander:
            self.log_action("Commander not available")
            return
        
        try:
            # Get publishers on /joint_states
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'info', '/joint_states', '-v'], 
                                capture_output=True, text=True, timeout=5)
            
            self.log_action("=== /joint_states Publishers ===")
            self.log_action(result.stdout)
            
        except Exception as e:
            self.log_action(f"Error diagnosing: {e}")


    def validate_joint_solution(self, joint_positions):
        """Validate that joint positions are within limits."""
        try:
            if len(joint_positions) != len(self.joint_names):
                return False, f"Expected {len(self.joint_names)} joints, got {len(joint_positions)}"
            
            violations = []
            for i, (name, pos) in enumerate(zip(self.joint_names, joint_positions)):
                min_limit, max_limit = self.joint_limits.get(name, (-3.14, 3.14))
                
                if pos < min_limit or pos > max_limit:
                    violations.append(
                        f"{name}: {pos:.3f} (limits: [{min_limit:.3f}, {max_limit:.3f}])"
                    )
            
            if violations:
                return False, "; ".join(violations)
            
            return True, "Valid"
            
        except Exception as e:
            return False, f"Validation error: {str(e)}"






    def solve_ik(self, x, y, z, roll=None, pitch=None, yaw=None):
        """Use MoveIt's IK service to compute joint angles for a given Cartesian pose.
        Falls back to simulated geometric IK if service unavailable.
        """
        # Default orientation to 0 if not specified
        if roll is None:
            roll = 0.0
        if pitch is None:
            pitch = 0.0
        if yaw is None:
            yaw = 0.0

        # Try MoveIt service first
        try:
            if self.ik_client.wait_for_service(timeout_sec=1.0):
                # Build target pose
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.base_frame
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z

                quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
                pose_stamped.pose.orientation.x = quat[0]
                pose_stamped.pose.orientation.y = quat[1]
                pose_stamped.pose.orientation.z = quat[2]
                pose_stamped.pose.orientation.w = quat[3]

                # Build IK request (ROS2 version - no 'attempts' field)
                req = GetPositionIK.Request()
                req.ik_request = PositionIKRequest()
                req.ik_request.group_name = "pegasus_arm"
                req.ik_request.pose_stamped = pose_stamped
                req.ik_request.avoid_collisions = True
                req.ik_request.timeout = rclpy.duration.Duration(seconds=0.5).to_msg()

                # Call service
                future = self.ik_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done():
                    result = future.result()
                    if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                        joint_state = result.solution.joint_state
                        joint_positions = [0.0] * len(self.joint_names)
                        for i, name in enumerate(self.joint_names):
                            if name in joint_state.name:
                                idx = joint_state.name.index(name)
                                if idx < len(joint_state.position):
                                    joint_positions[i] = joint_state.position[idx]
                        self.get_logger().info(f"IK solved successfully: {joint_positions}")
                        return joint_positions
        except Exception as e:
            self.get_logger().warn(f"MoveIt IK service error: {str(e)}")

        # Fallback: Simulated geometric IK
        self.get_logger().info("Using simulated geometric IK (MoveIt unavailable)")
        try:
            import math
            # Link lengths from URDF: upper arm (shoulder to elbow), forearm (elbow to wrist)
            L2 = 0.3  # upper arm length
            L3 = 0.3  # forearm length

            # joint1: base rotation to face target
            joint1 = math.atan2(y, x)

            # Project target into arm plane (radial distance)
            px = math.sqrt(x**2 + y**2)
            pz = z

            # Check reachability and clamp if necessary
            reach = math.sqrt(px**2 + pz**2)
            if reach > L2 + L3:
                self.get_logger().warn("Target out of reach, scaling down")
                scale = (L2 + L3) / reach
                px *= scale
                pz *= scale
            elif reach < abs(L2 - L3):
                self.get_logger().warn("Target unreachable (inside inner workspace), using max extension")
                px = (L2 + L3) * px / reach if reach > 0 else 0
                pz = (L2 + L3) * pz / reach if reach > 0 else 0

            # Compute joint3 (elbow angle) - elbow-down configuration
            cos_theta3 = (px**2 + pz**2 - L2**2 - L3**2) / (2 * L2 * L3)
            cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
            theta3 = math.acos(cos_theta3)
            theta3 = -theta3  # Negative for typical elbow-down pose

            sin_theta3 = math.sin(theta3)

            # Compute joint2 (shoulder angle)
            k1 = L2 + L3 * cos_theta3
            k2 = L3 * sin_theta3
            theta2 = math.atan2(pz, px) - math.atan2(k2, k1)

            # joint4: wrist pitch (simple: 0, or adjust for target pitch if needed)
            joint4 = 0.0  # Could set to pitch for better orientation matching

            # joint5: wrist roll (use target roll)
            joint5 = roll

            joint_positions = [joint1, theta2, theta3, joint4, joint5]

            # Clamp to joint limits
            limits = self.get_joint_limits()
            for i, j in enumerate(joint_positions):
                minv, maxv = limits[self.joint_names[i]]
                joint_positions[i] = np.clip(j, minv, maxv)

            self.get_logger().info(f"Simulated IK solution: {joint_positions}")
            return joint_positions
        except Exception as e:
            self.get_logger().error(f"Simulated IK error: {str(e)}")
            return None



    def solve_fk(self, joint_positions):
        """Compute forward kinematics for given joint positions using MoveIt's FK service."""
        if not self.fk_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("FK service not available (/compute_fk)")
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.end_effector_frame]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = joint_positions

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result and result.error_code.val == result.error_code.SUCCESS:
            pose = result.pose_stamped[0].pose
            self.get_logger().info(
                f"FK Pose: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
            )
            return pose
        else:
            self.get_logger().warn("FK computation failed")
            return None

    def plan_to_cartesian_pose(self, x, y, z, roll=None, pitch=None, yaw=None, show_in_rviz=True, duration=2.0):
        """Plan to a Cartesian pose (X, Y, Z) with optional orientation."""
        try:
            # Set default orientation
            if roll is None:
                roll = 0.0
            if pitch is None:
                pitch = 0.0
            if yaw is None:
                yaw = 0.0
            
            self.get_logger().info(f"Planning to Cartesian pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                                f"r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}")
            
            # Solve IK to get joint positions
            joint_solution = self.solve_ik(x, y, z, roll, pitch, yaw)
            
            if joint_solution is None:
                return {
                    'success': False,
                    'message': "IK solver failed - pose unreachable"
                }
            
            self.get_logger().info(f"IK solution: {[f'{j:.3f}' for j in joint_solution]}")
            
            # Validate joint solution
            is_valid, msg = self.validate_joint_solution(joint_solution)
            if not is_valid:
                self.get_logger().warn(f"Joint solution invalid: {msg}")
                # Clamp to limits
                limits = self.get_joint_limits()
                joint_solution = [
                    np.clip(joint_solution[i], *limits[self.joint_names[i]]) 
                    for i in range(len(joint_solution))
                ]
                self.get_logger().info(f"Clamped solution: {[f'{j:.3f}' for j in joint_solution]}")
            
            # Plan to those joint positions
            result = self.plan_to_joint_values(joint_solution, duration=duration)
            
            if not result['success']:
                return result
            
            # Show in RViz if requested
            if show_in_rviz:
                if 'raw_trajectory' in result and result['raw_trajectory'] is not None:
                     self._publish_single_display_trajectory(result['raw_trajectory'])
                elif 'trajectory' in result:
                    self._publish_single_display_trajectory(result['trajectory'])
                self.get_logger().info("Published Cartesian trajectory to RViz")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error in Cartesian planning: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return {
                'success': False,
                'message': f"Error in Cartesian planning: {str(e)}"
            }


    def stop_execution(self):
        """Stop current live trajectory execution"""
        self.get_logger().info("Stopping trajectory execution...")
        
        if hasattr(self, 'execution_timer') and self.execution_timer:
            self.execution_timer.cancel()
            self.execution_timer = None
        
        if hasattr(self, 'is_executing'):
            self.is_executing = False
        
        # Reset execution state
        if hasattr(self, 'execution_elapsed'):
            self.execution_elapsed = 0.0
        if hasattr(self, 'pid_integral'):
            self.pid_integral = 0.0
        if hasattr(self, 'pid_prev_error'):
            self.pid_prev_error = 0.0
        if hasattr(self, 'pid_last_time'):
            self.pid_last_time = None
        
        self.get_logger().info("Trajectory execution stopped")



    def execute_live_trajectory(self, result_dict):
        """Execute trajectory using action client"""
        try:
            # Stop any existing execution first - REMOVED for smooth streaming
            # if hasattr(self, 'is_executing') and self.is_executing:
            #     self.get_logger().info("Stopping previous execution...")
            #     self.stop_execution()
            #     import time
            #     time.sleep(0.2)
            
            # Extract trajectory message
            raw_traj = None
            
            if isinstance(result_dict, dict):
                if 'raw_trajectory' in result_dict and result_dict['raw_trajectory'] is not None:
                    raw_traj = result_dict['raw_trajectory']
                elif 'trajectory' in result_dict:
                    # Need to reconstruct from trajectory info
                    trajectory_info = result_dict['trajectory']
                    if trajectory_info and 'points' in trajectory_info:
                        raw_traj = self._create_trajectory_msg(trajectory_info)
            
            if raw_traj is None:
                self.get_logger().error("No valid trajectory data to execute")
                return
            
            # Ensure trajectory message is valid
            if not hasattr(raw_traj, 'points') or len(raw_traj.points) == 0:
                self.get_logger().error("Trajectory has no points")
                return
            
            self.get_logger().info(f"Executing trajectory: {len(raw_traj.points)} points, joints: {raw_traj.joint_names}")
            
            #  PUBLISH TO /display_planned_path FOR LISTENER
            from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
            display_traj = DisplayTrajectory()
            display_traj.trajectory.append(RobotTrajectory())
            display_traj.trajectory[0].joint_trajectory = raw_traj
            
            if hasattr(self, 'display_trajectory_pub') and self.display_trajectory_pub:
                self.display_trajectory_pub.publish(display_traj)
                self.get_logger().info(" Published to /display_planned_path for listener")
            
            # Send to action server or direct
            if self.moveit_enabled:
                self.play_trajectory(raw_traj)
            else:
                # Direct Arduino Control: Send the LAST point in the trajectory
                # This assumes the trajectory is a smooth interpolation to the target
                if raw_traj.points:
                    target_joints = raw_traj.points[-1].positions
                    self.send_joints_to_arduino(target_joints)
                    self.get_logger().info(f"Direct streaming to Arduino: {target_joints}")
                else:
                    self.get_logger().warn("No points in trajectory for direct control")
            
        except Exception as e:
            self.get_logger().error(f"Trajectory execution error: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            if hasattr(self, 'is_executing'):
                self.is_executing = False


   

    def _create_trajectory_msg(self, trajectory_info):
        """Create JointTrajectory message from trajectory info dict"""
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration as ROSDuration
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = trajectory_info.get('joint_names', self.joint_names)
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "base_link"
        
        for pt_info in trajectory_info['points']:
            point = JointTrajectoryPoint()
            point.positions = pt_info['positions']
            point.velocities = pt_info.get('velocities', [0.0] * len(point.positions))
            time_sec = pt_info['time_from_start']
            point.time_from_start = ROSDuration(
                sec=int(time_sec),
                nanosec=int((time_sec - int(time_sec)) * 1e9)
            )
            traj_msg.points.append(point)
        
        return traj_msg


    # Add the check_tf_availability method to PegasusCommander
    def check_tf_availability(self, level_check=False):
        """Check if TF transforms are available (non-blocking check only)."""
        if not hasattr(self, 'tf_buffer'):
            self.logger.warn("TF buffer not initialized yet")
            self.tf_available = False
            return
        
        # Just do a quick single check without retries or blocking
        try:
            frames = self.tf_buffer.all_frames_as_string()
            if not frames:
                self.logger.debug("No TF frames available yet")
                self.tf_available = False
                return
            
            if self.base_frame in frames and self.end_effector_frame in frames:
                try:
                    self.tf_buffer.lookup_transform(
                        self.base_frame, self.end_effector_frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1))
                    self.tf_available = True
                    self.logger.info(f"TF transform from {self.base_frame} to {self.end_effector_frame} available")
                    return
                except Exception as e:
                    self.logger.debug(f"TF lookup failed (will retry later): {str(e)}")
            else:
                self.logger.debug(f"Frames not ready yet. Base: {self.base_frame in frames}, EE: {self.end_effector_frame in frames}")
                
        except Exception as e:
            self.logger.debug(f"Quick TF check failed: {str(e)}")
        
        self.tf_available = False



    def get_current_pose(self):
        """Get current end effector pose from TF transform.
        Returns [x, y, z, roll, pitch, yaw] or None if unavailable.
        """
        if not self.tf_available or not hasattr(self, 'tf_buffer'):
            return None
        
        try:
            # Lookup transform from base frame to end effector frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract orientation (quaternion) and convert to RPY
            quat = transform.transform.rotation
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)
            
            return [x, y, z, roll, pitch, yaw]
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get current pose: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error getting pose: {str(e)}")
            return None


    def get_current_joint_values(self):
        """Return current joint values from joint states.
        Returns list of joint positions or None if unavailable.
        """
        with self.goal_lock:
            if self.current_joint_state is None:
                return self._current_values.copy()
            
            # Map joint state positions to our joint order
            joint_positions = [0.0] * len(self.joint_names)
            for i, name in enumerate(self.joint_names):
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    if idx < len(self.current_joint_state.position):
                        joint_positions[i] = self.current_joint_state.position[idx]
            
            return joint_positions


    def send_joints_to_arduino(self, joint_values):
        """Send joint positions to Arduino via serial.
        Returns True if successful, False otherwise.
        """
        if self.arduino_serial is None:
            self.get_logger().warn("Arduino serial not connected")
            return False
        
        try:
            # Format: "J<j1>,<j2>,<j3>,<j4>,<j5>\n"
            command = "J" + ",".join([f"{v:.4f}" for v in joint_values]) + "\n"
            self.arduino_serial.write(command.encode('utf-8'))
            self.get_logger().debug(f"Sent to Arduino: {command.strip()}")
            
            # Optional: wait for acknowledgment
            time.sleep(0.05)  # Small delay for Arduino processing
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send to Arduino: {str(e)}")
            return False


    def move_to_joint_positions(self, joint_positions, velocity_scaling=0.3):
        """Move robot to specified joint positions.
        Returns (success: bool, error_code: int)
        """
        if not self.moveit_enabled:
            # Direct Arduino control mode
            success = self.send_joints_to_arduino(joint_positions)
            if success:
                return True, MoveItErrorCodes.SUCCESS
            else:
                return False, MoveItErrorCodes.CONTROL_FAILED
        
        # MoveIt control mode
        try:
            if not self.controller_available:
                self.get_logger().error(f"Controller {self.controller_name} not available")
                return False, MoveItErrorCodes.CONTROL_FAILED
            
            # Use action client to send trajectory
            return self.move_to_joint_positions_action(joint_positions, velocity_scaling)
            
        except Exception as e:
            self.get_logger().error(f"Error moving to joint positions: {str(e)}")
            return False, MoveItErrorCodes.FAILURE


    def set_moveit_enabled(self, enabled):
        """Enable or disable MoveIt mode.
        When disabled, uses direct Arduino control.
        """
        self.moveit_enabled = enabled
        if not enabled and self.arduino_serial is None:
            self._setup_arduino_serial()
        
        mode = "MoveIt" if enabled else "Direct Arduino"
        self.get_logger().info(f"Control mode: {mode}")




    # Add get_available_frames method to PegasusCommander
    def get_available_frames(self):
        """Get list of available TF frames."""
        try:
            if not hasattr(self, 'tf_buffer'):
                return []
            frames = self.tf_buffer.all_frames_as_string()
            return sorted(frames.split()) if frames else []
        except Exception as e:
            self.logger.error(f"Error getting TF frames: {str(e)}")
            return []
        
    def get_joint_limits(self):
        """Return a copy of the joint limits dictionary."""
        return self.joint_limits.copy()


    def move_to_pose(self, pose, velocity_scaling=0.3):
        """Move to a Cartesian pose using IK and motion planning.
        Follows the same pattern as joint space - continuous real-time planning updates.
        
        Args:
            pose: List of [x, y, z] or [x, y, z, roll, pitch, yaw]
            velocity_scaling: Speed multiplier (default: 0.3)
            
        Returns:
            Tuple of (success: bool, error_code: int) for compatibility with GUI
        """
        try:
            # Extract position and orientation
            x, y, z = pose[0:3]
            roll, pitch, yaw = pose[3:6] if len(pose) >= 6 else (0.0, 0.0, 0.0)
            
            # Plan to the Cartesian pose (with RViz visualization)
            result = self.plan_to_cartesian_pose(x, y, z, roll, pitch, yaw)
            
            if result["success"]:
                return True, MoveItErrorCodes.SUCCESS
            else:
                return False, MoveItErrorCodes.PLANNING_FAILED
                
        except Exception as e:
            self.get_logger().error(f"Error in move_to_pose: {str(e)}")
            return False, MoveItErrorCodes.FAILURE
 
 
 
 
 
    def _setup_arduino_serial(self):
        """Modified version with consolidated logging"""
        max_retries = 3
        retry_delay = 1
        
        for attempt in range(max_retries):
            try:
                if not os.path.exists(self.arduino_port):
                    if attempt == 0:  # Only log first attempt
                        self.get_logger().warn(f"Port {self.arduino_port} not found")
                    time.sleep(retry_delay)
                    continue
                    
                self.arduino_serial = serial.Serial(
                    self.arduino_port, 
                    self.arduino_baudrate, 
                    timeout=1,
                    write_timeout=1
                )
                
                self.get_logger().info(f" Connected to: {self.arduino_port}")
                time.sleep(1)
                
                # Clear buffers
                self.arduino_serial.flushInput()
                self.arduino_serial.flushOutput()
                
                # Test connection
                if "sim" in self.arduino_port:
                    try:
                        self.arduino_serial.write(b"PING\n")
                        response = self.arduino_serial.readline().decode('utf-8').strip()
                        self.get_logger().info(f"Connection verified: {response}")
                    except Exception:
                        pass
                
                return
                        
            except Exception as e:
                if attempt == max_retries - 1:  # Only log final failure
                    self.get_logger().error(f" Failed to connect: {str(e)}")
                else:
                    time.sleep(retry_delay)
        
        self.arduino_serial = None



    def _trajectory_status_callback(self, msg):
        """Forward trajectory status to GUI action log"""
        try:
            # Get the GUI reference if it exists
            if hasattr(self, '_gui_log_callback') and self._gui_log_callback:
                self._gui_log_callback(msg.data)
        except Exception as e:
            self.get_logger().debug(f"Error forwarding status: {e}")




    def initialize_services_and_topics(self):
        # Skip blocking waits - just create clients without waiting
        # The actual availability will be checked asynchronously via the timer
        self.controller_manager_available = False

        # === Action Clients ===
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_action_client = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')


        from std_msgs.msg import String
        self.trajectory_status_sub = self.create_subscription(
            String,
            '/trajectory_status',
            self._trajectory_status_callback,
            10
        )
        self.get_logger().info("Subscribed to /trajectory_status")
        

        # === Publishers ===
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.controller_name}/joint_trajectory',
            10
        )

        from moveit_msgs.msg import DisplayTrajectory
        self.display_trajectory_pub = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10
        )

        # Publish simulated joint_states for RViz visualization when playing trajectories
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # === Service Clients ===
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        # Add IK service client
        from moveit_msgs.srv import GetPositionIK, GetPositionFK
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        # Add FK service client (useful for verification)
        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        # === TF (Transform) ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Joint State Subscription ===
        def _local_joint_state_callback(msg):
            try:
                with self.goal_lock:
                    self.current_joint_state = msg
                    if self.current_joint_state:
                        for i, name in enumerate(self.current_joint_state.name):
                            if name in self.joint_names:
                                idx = self.joint_names.index(name)
                                if idx < len(self._current_values) and i < len(self.current_joint_state.position):
                                    self._current_values[idx] = self.current_joint_state.position[i]
            except Exception as e:
                self.get_logger().error(f"Local joint_state callback error: {e}")

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            _local_joint_state_callback,
            10
        )

        self.get_logger().info("All services, topics, and publishers initialized successfully.")
    






    def wait_for_service(self, service_name, timeout_sec):
        client = self.create_client(ListControllers, service_name)
        return client.wait_for_service(timeout_sec=timeout_sec)

    def activate_controllers(self):
        try:
            if not self.list_controllers_client.wait_for_service(timeout_sec=5.0):
                self.logger.error("Controller manager service unavailable")
                return False

            request = ListControllers.Request()
            future = self.list_controllers_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error("Failed to list controllers")
                return False

            controllers = future.result().controller
            required_controllers = ["joint_state_broadcaster", self.controller_name]
           
            active_controllers = [c.name for c in controllers if c.state == "active"]

            if not all(c in controllers for c in required_controllers):
                self.logger.error(f"Required controllers {required_controllers} not found")
                return False

            if not all(c in active_controllers for c in required_controllers):
                os.system(f"ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --controller-manager-timeout 60 &")
                os.system(f"ros2 run controller_manager spawner {self.controller_name} --controller-manager /controller_manager --controller-manager-timeout 60 &")
                time.sleep(5.0)

                future = self.list_controllers_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None:
                    self.logger.error("Failed to list controllers after spawning")
                    return False

                active_controllers = [c.name for c in future.result().controller if c.state == "active"]
               
                return False

            if not all(c in active_controllers for c in required_controllers):
                os.system(f"ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --controller-manager-timeout 60 &")
                os.system(f"ros2 run controller_manager spawner {self.controller_name} --controller-manager /controller_manager --controller-manager-timeout 60 &")
                time.sleep(5.0)

                future = self.list_controllers_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None:
                    self.logger.error("Failed to list controllers after spawning")
                    return False

                active_controllers = [c.name for c in future.result().controller if c.state == "active"]
                if not all(c in active_controllers for c in required_controllers):
                    self.logger.error("Failed to activate all required controllers")
                    return False

            self.controller_available = True
            self.trajectory_topic_available = True
            return True
        except Exception as e:
            self.logger.error(f"Error activating controllers: {str(e)}")
            return False

    def update_controller_name(self, new_controller):
        self.controller_name = new_controller
        self.traj_action_client = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.controller_name}/joint_trajectory',
            10
        )
        self.check_services_and_topics()



    def check_services_and_topics(self):
        """Refined service checking with reduced logging spam"""
        if not self.controller_manager_available:
            return

        # Check action server (only log changes)
        prev_action = getattr(self, '_prev_action_available', None)
        action_available = self.move_action_client.wait_for_server(timeout_sec=1.0)
        
        if action_available != prev_action:
            if action_available:
                self.get_logger().info("MoveIt action server available")
            else:
                self.get_logger().warn(" MoveIt action server unavailable")
            self._prev_action_available = action_available
        
        self.action_server_available = action_available

        # Check controller manager
        if not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            prev_cm = getattr(self, '_prev_cm_available', True)
            if prev_cm:
                self.get_logger().warn(" Controller manager service unavailable")
                self._prev_cm_available = False
            return

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            controllers = future.result().controller
            controller_names = [c.name for c in controllers]
            
            # Check main controller (only log changes)
            prev_ctrl = getattr(self, '_prev_controller_available', None)
            controller_active = False
            
            if self.controller_name in controller_names:
                for controller in controllers:
                    if controller.name == self.controller_name and controller.state == "active":
                        controller_active = True
                        break
            
            if controller_active != prev_ctrl:
                if controller_active:
                    self.get_logger().info(f" {self.controller_name} active")
                else:
                    self.get_logger().warn(f" {self.controller_name} inactive")
                self._prev_controller_available = controller_active
            
            self.controller_available = controller_active
            
            # Check joint state broadcaster (only log changes)
            prev_jsb = getattr(self, '_prev_jsb_available', None)
            jsb_active = False
            
            if "joint_state_broadcaster" in controller_names:
                for controller in controllers:
                    if controller.name == "joint_state_broadcaster" and controller.state == "active":
                        jsb_active = True
                        break
            
            if jsb_active != prev_jsb:
                if jsb_active:
                    self.get_logger().info(" joint_state_broadcaster active")
                else:
                    self.get_logger().warn(" joint_state_broadcaster inactive")
                self._prev_jsb_available = jsb_active

        # Check topics (only log changes)
        topics = self.get_topic_names_and_types()
        
        prev_js = getattr(self, '_prev_joint_states_available', None)
        js_available = '/joint_states' in [t[0] for t in topics]
        
        if js_available != prev_js:
            if js_available:
                self.get_logger().info(" Joint states topic available")
            else:
                self.get_logger().warn(" Joint states topic unavailable")
            self._prev_joint_states_available = js_available
        
        self.joint_states_available = js_available
        
        trajectory_topic = f'/{self.controller_name}/joint_trajectory'
        prev_traj = getattr(self, '_prev_trajectory_available', None)
        traj_available = trajectory_topic in [t[0] for t in topics]
        
        if traj_available != prev_traj:
            if traj_available:
                self.get_logger().info(f" Trajectory topic available")
            else:
                self.get_logger().warn(f" Trajectory topic unavailable")
            self._prev_trajectory_available = traj_available
        
        self.trajectory_topic_available = traj_available












    def stop_all_background_tasks(self):
        """Stop all timers and background tasks that might be publishing"""
        try:
            # Stop execution
            if hasattr(self, 'is_executing'):
                self.is_executing = False
            
            # Cancel execution timer if exists
            if hasattr(self, 'execution_timer') and self.execution_timer:
                self.execution_timer.cancel()
                self.execution_timer = None
            
            # List all timers and cancel them
            if hasattr(self, '_timers'):
                for timer in self._timers:
                    try:
                        timer.cancel()
                    except Exception:
                        pass
            
            self.get_logger().info("All background tasks stopped")
            
        except Exception as e:
            self.get_logger().error(f"Error stopping background tasks: {e}")  # <-- SINGLE LINE ONLY!


    def move_to_joint_positions_action(self, joint_positions, velocity_scaling=0.3):
        if not self.traj_action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"Action server /{self.controller_name}/follow_joint_trajectory not available")
            return False, MoveItErrorCodes.FAILURE



        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = rclpy.duration.Duration(seconds=1.0 / velocity_scaling).to_msg()
        traj.points = [point]
        traj.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory = traj

        self.logger.info(f"Sending joint trajectory action: {joint_positions}")
        self.traj_action_client.send_goal_async(goal_msg).add_done_callback(self._cancel_callback)
        return True, MoveItErrorCodes.SUCCESS

    def move_cartesian(self, delta, is_rotation, velocity_scaling=0.3):
        """Move in cartesian space by a delta amount.
        
        Args:
            delta: [x, y, z] for translation or [roll, pitch, yaw] for rotation
            is_rotation: True if rotating, False if translating
            velocity_scaling: Speed scaling factor (0.0-1.0)
            
        Returns:
            (success, error_code) tuple
        """
        if not self.moveit_enabled:
            self.logger.error("MoveIt is not enabled. Cannot move cartesian.")
            return False, MoveItErrorCodes.FAILURE

        if not self.controller_available:
            self.logger.error(f"Cannot move: {self.controller_name} not active.")
            return False, MoveItErrorCodes.CONTROL_FAILED

        try:
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.logger.error("Current pose unavailable. Cannot perform Cartesian movement.")
                return False, MoveItErrorCodes.FAILURE

            # Compute new pose
            new_pose = list(current_pose)
            if is_rotation:
                # For rotation, delta is in radians around the respective axis
                if len(delta) != 3:
                    self.logger.error("Invalid delta for rotation. Must be 3D vector for roll/pitch/yaw.")
                    return False, MoveItErrorCodes.FAILURE
                
                # Convert Euler angles (roll, pitch, yaw) to quaternion
                r = R.from_euler('xyz', delta, degrees=False)
                quat = r.as_quat()
                
                # Update only the orientation part
                new_pose[3] += quat[0]
                new_pose[4] += quat[1]
                new_pose[5] += quat[2]
            else:
                # For translation, delta is in meters for x, y, z
                if len(delta) != 3:
                    self.logger.error("Invalid delta for translation. Must be 3D vector for x/y/z.")
                    return False, MoveItErrorCodes.FAILURE
                
                new_pose[0] += delta[0]
                new_pose[1] += delta[1]
                new_pose[2] += delta[2]

            # Send the computed pose to MoveIt
            return self.move_to_pose(new_pose, velocity_scaling)
        except Exception as e:
            self.logger.error(f"Error during Cartesian movement: {str(e)}")
            return False, MoveItErrorCodes.FAILURE



    def _cancel_callback(self, future):
        try:
            if future.result().return_code == 0:
                self.logger.info("Goal cancelled successfully")
            else:
                self.logger.info("Goal cancellation failed")
        except Exception as e:
            self.logger.error(f"Error during goal cancellation: {str(e)}")
       
        finally:
            with self.goal_lock:
                self.current_goal_handle = None



    def plan_to_joint_values(self, joint_values_str, duration=2.0):
        """Plan to joint values and return a trajectory for visualization/execution.
        Returns dict with keys: success (bool), trajectory (info dict), raw_trajectory (JointTrajectory msg), message (str).
        """
        try:
            # Convert to float list
            try:
                target = [float(v) for v in joint_values_str]
            except (ValueError, TypeError):
                return {'success': False, 'trajectory': None, 'raw_trajectory': None, 'message': 'Invalid joint values'}

            if len(target) != len(self.joint_names):
                return {'success': False, 'trajectory': None, 'raw_trajectory': None, 'message': f'Joint count mismatch: expected {len(self.joint_names)}, got {len(target)}'}

            start = self.get_current_joint_values()
            # If no current values, assume zeros
            if start is None or len(start) != len(target):
                start = [0.0] * len(target)

            # Validate target
            is_valid, msg = self.validate_joint_solution(target)
            if not is_valid:
                self.get_logger().warn(f"Joint solution invalid: {msg}")
                # Clamp to limits
                limits = self.get_joint_limits()
                target = [np.clip(target[i], *limits[self.joint_names[i]]) for i in range(len(target))]

            # Build interpolated trajectory (6 points including start and goal)
            num_points = 6
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            total_time = duration  # seconds (adjustable)
            points = []
            for i in range(num_points):
                alpha = i / (num_points - 1)
                pos = [start[j] * (1 - alpha) + target[j] * alpha for j in range(len(target))]
                p = JointTrajectoryPoint()
                p.positions = pos
                p.velocities = [0.1] * len(pos)  # Simple constant velocity
                p.time_from_start = rclpy.duration.Duration(seconds=alpha * total_time).to_msg()
                points.append(p)
            traj_msg.points = points
            traj_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish for immediate visualization if trajectory topic available
            try:
                if hasattr(self, 'trajectory_pub') and self.trajectory_pub is not None:
                    self.trajectory_pub.publish(traj_msg)
                    self.get_logger().info('Published planned trajectory for visualization')
            except Exception:
                pass

            trajectory_info = {'num_points': len(points), 'total_time': total_time}
            return {
                'success': True,
                'trajectory': trajectory_info,
                'raw_trajectory': traj_msg,
                'message': 'Planned successfully (simulated)'
            }
        except Exception as e:
            self.get_logger().error(f'plan_to_joint_values error: {e}')
            return {'success': False, 'trajectory': None, 'raw_trajectory': None, 'message': str(e)}

    def _publish_single_display_trajectory(self, trajectory_data):
        """Publish trajectory for MoveIt visualization in RViz - SINGLE PUBLICATION
        Args:
            trajectory_data: Can be a dict (trajectory_info) or JointTrajectory msg
        """
        try:
            from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
            
            # Create DisplayTrajectory message
            display_traj = DisplayTrajectory()
            display_traj.model_id = "pegasus_arm"
            
            # Create RobotTrajectory
            robot_traj = RobotTrajectory()
            
            # Handle different input types
            if hasattr(trajectory_data, 'points'):
                # It's already a JointTrajectory message
                robot_traj.joint_trajectory = trajectory_data
            elif isinstance(trajectory_data, dict) and 'points' in trajectory_data:
                # It's a trajectory info dict with points
                robot_traj.joint_trajectory = self._create_trajectory_msg(trajectory_data)
            else:
                self.get_logger().warn("Invalid data for display trajectory")
                return
            
            display_traj.trajectory.append(robot_traj)
            
            # Publish
            if hasattr(self, 'display_trajectory_pub'):
                self.display_trajectory_pub.publish(display_traj)
                self.get_logger().info("Published display trajectory to RViz")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing display trajectory: {e}")

    def plan_to_joint_state(self, joint_positions):
        """Plan to a joint configuration using the motion planning service."""
        # This seems to be a wrapper around plan_to_joint_values in the GUI version
        # But in the original file it was different.
        # In GUI version, plan_to_joint_values does the planning (simulated).
        # So we can just call plan_to_joint_values.
        return self.plan_to_joint_values(joint_positions)

if __name__ == '__main__':
    rclpy.init()
    node = PegasusCommander()
    rclpy.spin(node)
