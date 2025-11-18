#!/usr/bin/env python3

"""
Enhanced Pegasus Arm Commander Module
=====================================

A reusable commander module that uses MoveIt2 services to plan trajectories
with single planning execution and live trajectory support.

Usage:
    from pegasus_commander import PegasusCommander
    
    # Initialize ROS2
    rclpy.init()
    commander = PegasusCommander()
    
    # Plan to joint configuration (plans once, returns trajectory)
    joint_values = ['0.883', '0.429', '0.330', '0.894', '1.362']
    result = commander.plan_to_joint_values(joint_values)
    
    # Execute live trajectory if planning successful
    if result['success']:
        commander.execute_live_trajectory(result['trajectory'])
    
    # Clean up
    commander.destroy_node()
    rclpy.shutdown()
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
import math
import time  # ADD THIS LINE
from threading import Lock, Thread  # Update this line
# Message and service imports
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionIKRequest,
    RobotState,
    DisplayTrajectory,
    RobotTrajectory,
    MoveItErrorCodes
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as ROSDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint




class PegasusCommander(Node):
    def __init__(self, wait_for_services=True):
        super().__init__('pegasus_commander_internal')
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5"
        ]

         
    # Add missing joint_limits initialization BEFORE load_joint_limits()
        self.joint_limits = {
            'joint1': (-6.28, 6.28),
            'joint2': (-0.61, 0.61),
            'joint3': (-1.75, 1.75),
            'joint4': (-1.31, 1.31),
            'joint5': (-6.28, 6.28)
        }
    
        self.is_executing = False

        # Service clients
        self.motion_plan_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.display_trajectory_pub = self.create_publisher(DisplayTrajectory, '/move_group/display_planned_path', 10)
        
        self.goal_lock = Lock()
        self.current_joint_state = None
        self._current_values = [0.0] * len(self.joint_names)
        
        # TF buffer
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Configuration
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5"
        ]   

        self.group_name = "pegasus_arm"
        self.end_effector_link = "end_effector_link"
        
        # Predefined poses
        self.named_poses = {
            'home': [0.0, -0.26, 0.0, 0.0, 0.0],
            'extended': [-6.2, -0.2676, -0.1221, 0.3017, -6.2]
        }
        
        # Live trajectory execution state
        self.execution_timer = None
        self.current_trajectory = None
        self.execution_start_time = None
        self.execution_speed = 1.0
        self.is_executing = False
        # Internal elapsed time used for PID-driven progression (seconds)
        self.execution_elapsed = 0.0

        # PID speed controller parameters (tune as needed)
        self.pid_kp = 1.2
        self.pid_ki = 0.0
        self.pid_kd = 0.01
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = None
        # Clamp for speed multiplier applied to progression
        self.min_speed = 0.05
        self.max_speed = 2.0

        # Planning cache to avoid repeated planning for the same joint goal
        self._last_plan_joints = None
        self._last_plan_result = None
        self._plan_tolerance = 1e-6
        
        # Wait for services if requested
        if wait_for_services:
            self._wait_for_services()
        
        self.get_logger().info("Enhanced Pegasus Commander initialized!")
        self.get_logger().info(f"Available joints: {self.joint_names}")
 

    def get_current_pose(self):
        """
        Get current end effector pose from TF transform.
        Returns [x, y, z, roll, pitch, yaw] or None if unavailable.
        """
        if not hasattr(self, 'tf_buffer'):
            self.get_logger().warn("TF buffer not initialized")
            return None
        
        try:
            from scipy.spatial.transform import Rotation as R
            from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
            
            # Lookup transform from base to end effector
            transform = self.tf_buffer.lookup_transform(
                "base_link",  # base frame
                self.end_effector_link,  # end effector frame
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
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error getting current pose: {str(e)}")
            return None


  

    def get_current_joint_values(self):
        """
        Return current joint values from joint states.
        Returns list of joint positions or None if unavailable.
        """
        if not hasattr(self, 'goal_lock'):
            return [0.0] * len(self.joint_names)
        
        with self.goal_lock:
            if not hasattr(self, 'current_joint_state') or self.current_joint_state is None:
                return [0.0] * len(self.joint_names)
            
            # Map joint state positions to our joint order
            joint_positions = [0.0] * len(self.joint_names)
            for i, name in enumerate(self.joint_names):
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    if idx < len(self.current_joint_state.position):
                        joint_positions[i] = self.current_joint_state.position[idx]
            
            return joint_positions


    def _interpolate_trajectory_at_time(self, time_seconds):
        """
        Interpolate joint positions at a specific time in the trajectory.
        
        Args:
            time_seconds: Time from start of trajectory
            
        Returns:
            list: Interpolated joint positions
        """
        if not self.current_trajectory or not self.current_trajectory['points']:
            return None
        
        points = self.current_trajectory['points']
        
        # If before start, return first point
        if time_seconds <= 0:
            return points[0]['positions']
        
        # If after end, return last point
        if time_seconds >= points[-1]['time_from_start']:
            return points[-1]['positions']
        
        # Find the two points to interpolate between
        for i in range(len(points) - 1):
            t0 = points[i]['time_from_start']
            t1 = points[i+1]['time_from_start']
            
            if t0 <= time_seconds <= t1:
                # Linear interpolation
                alpha = (time_seconds - t0) / (t1 - t0) if (t1 - t0) > 0 else 0
                
                p0 = points[i]['positions']
                p1 = points[i+1]['positions']
                
                interpolated = [
                    p0[j] * (1 - alpha) + p1[j] * alpha 
                    for j in range(len(p0))
                ]
                
                return interpolated
        
        return points[-1]['positions']







    def _wait_for_services(self, timeout_sec=10.0):
        """Wait for MoveIt2 services to become available"""
        self.get_logger().info("Waiting for MoveIt2 services...")
        
        services_ready = True
        if not self.motion_plan_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("Motion planning service not available")
            services_ready = False
            
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("IK service not available")
        
        if services_ready:
            self.get_logger().info("Services ready!")
        else:
            self.get_logger().warn("Some services not available - limited functionality")
        
        return services_ready
    
    def plan_to_joint_values(self, joint_values, show_in_rviz=True):
        """
        Main interface function: Plan to joint configuration from a list of values.
        Plans ONCE and returns trajectory data for live execution.
        
        Args:
            joint_values: List of 5 joint values (can be strings or floats)
                         Example: ['0.883', '0.429', '0.330', '0.894', '1.362']
                         or [0.883, 0.429, 0.330, 0.894, 1.362]
            show_in_rviz: Whether to show the planned path in RViz (default: True)
        
        Returns:
            dict: Result with 'success' (bool), 'message' (str), and optionally 'trajectory'
        """
        try:
            # Convert to float if strings are provided
            if isinstance(joint_values, (list, tuple)):
                joint_positions = [float(val) for val in joint_values]
            else:
                return {
                    "success": False,
                    "message": "joint_values must be a list or tuple"
                }
            
            # Validate number of joints
            if len(joint_positions) != len(self.joint_names):
                return {
                    "success": False,
                    "message": f"Expected {len(self.joint_names)} joint values, got {len(joint_positions)}"
                }
            
            self.get_logger().info(f"Planning to joint values: {joint_positions}")
            result = self.plan_to_joint_state(joint_positions)
            
            # Show in RViz only once if requested and planning was successful
            if result['success'] and 'trajectory' in result and show_in_rviz:
                self._publish_single_display_trajectory(result['trajectory'])
                self.get_logger().info("Published single trajectory visualization to RViz")
            
            return result
            
        except ValueError as e:
            return {
                "success": False,
                "message": f"Error converting joint values to float: {str(e)}"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Error in plan_to_joint_values: {str(e)}"
            }


 
    def plan_to_joint_state(self, joint_positions):
        """Plan to a joint configuration using the motion planning service."""
        if not self.motion_plan_client.service_is_ready():
            # Fallback to direct joint state publishing
            return self._create_simple_trajectory(joint_positions)
        
        # Check planning cache
        if self._last_plan_joints and self._last_plan_result:
            if all(
                abs(a - b) < self._plan_tolerance 
                for a, b in zip(joint_positions, self._last_plan_joints)
            ):
                self.get_logger().info("Using cached planning result")
                return self._last_plan_result
        
        try:
            self.get_logger().info(f"Planning to joint state: {joint_positions}")
            
            # Create motion plan request
            request = GetMotionPlan.Request()
            
            # Set up the motion plan request
            motion_plan_request = MotionPlanRequest()
            motion_plan_request.group_name = self.group_name
            motion_plan_request.num_planning_attempts = 3
            motion_plan_request.allowed_planning_time = 5.0
            motion_plan_request.planner_id = "RRTConnectkConfigDefault"
            
            # Set default start state to home position
            default_joint_state = JointState()
            default_joint_state.name = self.joint_names
            default_joint_state.position = self.named_poses['home']
            motion_plan_request.start_state = RobotState()
            motion_plan_request.start_state.joint_state = default_joint_state
            motion_plan_request.start_state.is_diff = False
            
            # Create goal constraints
            goal_constraints = Constraints()
            
            for joint_name, position in zip(self.joint_names, joint_positions):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = float(position)
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                goal_constraints.joint_constraints.append(joint_constraint)
            
            motion_plan_request.goal_constraints.append(goal_constraints)
            
            # Set workspace bounds
            motion_plan_request.workspace_parameters.header.frame_id = "base_link"
            motion_plan_request.workspace_parameters.min_corner.x = -1.0
            motion_plan_request.workspace_parameters.min_corner.y = -1.0  
            motion_plan_request.workspace_parameters.min_corner.z = -1.0
            motion_plan_request.workspace_parameters.max_corner.x = 1.0
            motion_plan_request.workspace_parameters.max_corner.y = 1.0
            motion_plan_request.workspace_parameters.max_corner.z = 1.0
            
            request.motion_plan_request = motion_plan_request
            
            # Call the service ONCE
            self.get_logger().info("Calling motion planning service (single call)...")
            future = self.motion_plan_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                
                if response and response.motion_plan_response.error_code.val == 1:  # SUCCESS
                    self.get_logger().info("Planning successful!")
                    
                    # Extract trajectory ONCE
                    trajectory_info = self._extract_trajectory_info(
                        response.motion_plan_response.trajectory
                    )
                    
                    # Update planning cache
                    self._last_plan_joints = joint_positions
                    self._last_plan_result = {
                        "success": True,
                        "trajectory": trajectory_info,
                        "raw_trajectory": response.motion_plan_response.trajectory,
                        "message": "Planning successful - ready for execution"
                    }
                    
                    return self._last_plan_result
                else:
                    error_code = response.motion_plan_response.error_code.val if response else "No response"
                    self.get_logger().error(f"Planning failed with error code: {error_code}")
                    return {
                        "success": False,
                        "message": f"Planning failed with error code: {error_code}"
                    }
            else:
                self.get_logger().error("Service call timed out")
                return {
                    "success": False,
                    "message": "Service call timed out"
                }
                
        except Exception as e:
            self.get_logger().error(f"Planning error: {str(e)}")
            return {
                "success": False,
                "message": f"Planning error: {str(e)}"
            }


    def _create_simple_trajectory(self, joint_positions):
        """Create a simple trajectory without motion planning service"""
        try:
            start = self.get_current_joint_values()
            if start is None or len(start) != len(self.joint_names):
                start = self.named_poses['home']
            
            # Clamp joint positions to limits
            clamped_positions = []
            for i, (pos, name) in enumerate(zip(joint_positions, self.joint_names)):
                min_limit, max_limit = self.joint_limits.get(name, (-3.14, 3.14))
                clamped_positions.append(float(np.clip(pos, min_limit, max_limit)))
            
            # Create trajectory with proper structure
            num_points = 6
            total_time = 2.0
            
            trajectory_info = {
                'joint_names': self.joint_names,
                'points': [],
                'total_time': total_time,
                'num_points': num_points
            }
            
            for i in range(num_points):
                alpha = i / (num_points - 1)
                pos = [start[j] * (1 - alpha) + clamped_positions[j] * alpha 
                    for j in range(len(clamped_positions))]
                
                point_info = {
                    'positions': pos,
                    'velocities': [0.0] * len(pos),
                    'accelerations': [0.0] * len(pos),
                    'time_from_start': alpha * total_time
                }
                trajectory_info['points'].append(point_info)
            
            # Create proper JointTrajectory message
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration as ROSDuration
            
            joint_traj = JointTrajectory()
            joint_traj.joint_names = self.joint_names  # Make sure this matches
            joint_traj.header.stamp = self.get_clock().now().to_msg()
            joint_traj.header.frame_id = "base_link"
            
            for point_info in trajectory_info['points']:
                point = JointTrajectoryPoint()
                point.positions = point_info['positions']
                point.velocities = point_info['velocities']
                point.accelerations = point_info['accelerations']
                
                time_sec = point_info['time_from_start']
                point.time_from_start = ROSDuration(
                    sec=int(time_sec), 
                    nanosec=int((time_sec - int(time_sec)) * 1e9)
                )
                joint_traj.points.append(point)
            
            self.get_logger().info(f"Created simple trajectory: {len(joint_traj.points)} points")
            
            return {
                "success": True,
                "trajectory": trajectory_info,
                "raw_trajectory": joint_traj,
                "message": "Simple trajectory created"
            }
            
        except Exception as e:
            self.get_logger().error(f"Error creating simple trajectory: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return {
                "success": False,
                "message": f"Failed to create trajectory: {str(e)}"
            }





    def _publish_single_display_trajectory(self, trajectory_info):
        """Publish trajectory for MoveIt visualization in RViz - SINGLE PUBLICATION"""
        try:
            display_trajectory = DisplayTrajectory()
            display_trajectory.model_id = "pegasus"
            
            # Create RobotTrajectory
            robot_trajectory = RobotTrajectory()
            
            # Create joint trajectory
            joint_trajectory = JointTrajectory()
            joint_trajectory.header = Header()
            joint_trajectory.header.stamp = self.get_clock().now().to_msg()
            joint_trajectory.header.frame_id = "base_link"
            joint_trajectory.joint_names = trajectory_info['joint_names']
            
            # Add trajectory points
            for point_info in trajectory_info['points']:
                point = JointTrajectoryPoint()
                point.positions = point_info['positions']
                point.velocities = point_info.get('velocities', [0.0] * len(point_info['positions']))
                point.accelerations = point_info.get('accelerations', [0.0] * len(point_info['positions']))
                
                # Convert time
                time_sec = point_info['time_from_start']
                point.time_from_start = ROSDuration(sec=int(time_sec), nanosec=int((time_sec - int(time_sec)) * 1e9))
                
                joint_trajectory.points.append(point)
            
            # Set up the robot trajectory
            robot_trajectory.joint_trajectory = joint_trajectory
            
            # Set up the display trajectory
            display_trajectory.trajectory.append(robot_trajectory)
            
            # Single publish call
            self.display_trajectory_pub.publish(display_trajectory)
            self.get_logger().info("Published single display trajectory to RViz")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish display trajectory: {e}")
    

    def execute_live_trajectory(self, result_dict):
        """Execute trajectory using action client"""
        try:
            # Stop any existing execution first
            if hasattr(self, 'is_executing') and self.is_executing:
                self.get_logger().info("Stopping previous execution...")
                self.stop_execution()
                import time
                time.sleep(0.2)
            
            # Extract trajectory message
            raw_traj = None
            
            if isinstance(result_dict, dict):
                if 'raw_trajectory' in result_dict:
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
            
            # Send to action server
            self.play_trajectory(raw_traj)
            
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



if __name__ == '__main__':
    print("Enhanced Pegasus Commander - Single Plan with Live Trajectory")
    print("=" * 60)
    print("This module is designed to be imported and used programmatically.")
    print()
    print("Key Features:")
    print("• Single planning execution (no multiple visualizations)")
    print("• Live trajectory execution capabilities")  
    print("• Clean RViz visualization")
    print()
    print("Basic Usage:")
    print("  from pegasus_commander import PegasusCommander")
    print("  rclpy.init()")
    print("  commander = PegasusCommander()")
    print()
    print("  # Plan once")
    print("  result = commander.plan_to_joint_values(['0.883', '0.429', '0.330', '0.894', '1.362'])")
    print()
    print("  # Execute live trajectory")
    print("  if result['success']:")
    print("      commander.execute_live_trajectory(result['trajectory'])")
    print()
    print("  commander.destroy_node()")
    print("  rclpy.shutdown()")
    print()
    print("Advanced Usage:")
    print("  # Get trajectory data for external systems")
    print("  live_data = commander.get_live_trajectory_data(result['trajectory'])")
    print("  # Check execution status")
    print("  progress = commander.get_execution_progress()")
    print("  is_running = commander.is_trajectory_executing()")
