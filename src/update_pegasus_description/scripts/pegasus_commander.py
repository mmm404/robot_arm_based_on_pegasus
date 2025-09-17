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
from rclpy.node import Node
from rclpy.duration import Duration
import math

# Message and service imports
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionIKRequest,
    RobotState,
    DisplayTrajectory,
    RobotTrajectory
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as ROSDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PegasusCommander(Node):
    """
    Enhanced Commander for the Pegasus robotic arm using MoveIt2 services.
    Plans once and provides live trajectory execution capabilities.
    """
    
    def __init__(self, wait_for_services=True):
        super().__init__('pegasus_commander')
        
        # Service clients
        self.motion_plan_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Publishers for live trajectory execution
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publisher for MoveIt visualization (single use)
        self.display_trajectory_pub = self.create_publisher(
            DisplayTrajectory, 
            '/move_group/display_planned_path', 
            10
        )
        
        # Configuration
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
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
        """
        Plan to a joint configuration using the motion planning service.
        This method plans ONCE without any intermediate publications.
        """
        if not self.motion_plan_client.service_is_ready():
            return {
                "success": False, 
                "message": "Motion planning service not available"
            }
        
        # Check planning cache: if the joint positions are the same as the last plan, return the cached result
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
    
    def execute_live_trajectory(self, trajectory_info, speed_factor=1.0):
        """
        Execute the planned trajectory in real-time by publishing joint states.
        This is for live robot control, not visualization.
        
        Args:
            trajectory_info: Trajectory information from successful planning
            speed_factor: Speed multiplier (1.0 = normal speed, 0.5 = half speed, 2.0 = double speed)
        
        Returns:
            dict: Execution status
        """
        if not trajectory_info or not trajectory_info['points']:
            return {
                "success": False,
                "message": "No valid trajectory provided for execution"
            }
        
        if self.is_executing:
            self.stop_execution()
        
        self.current_trajectory = trajectory_info
        # Use an internal elapsed counter rather than relying solely on wall-clock start time,
        # this allows PID to modulate how fast elapsed time progresses.
        self.execution_elapsed = 0.0
        self.execution_start_time = self.get_clock().now()
        self.execution_speed = speed_factor
        self.is_executing = True
        
        # Initialize PID state
        self.pid_integral = 0.0
        # initial error = distance from start to goal
        start_pos = self.current_trajectory['points'][0]['positions']
        goal_pos = self.current_trajectory['points'][-1]['positions']
        self.pid_prev_error = self._compute_distance(start_pos, goal_pos)
        self.pid_last_time = self.get_clock().now()
        
        # Create timer for live execution (50Hz for smooth control)
        self.execution_timer = self.create_timer(0.02, self._execution_callback)
        
        total_time = trajectory_info['total_time'] / max(0.0001, speed_factor)
        self.get_logger().info(f"Starting live trajectory execution (est. {total_time:.2f}s, PID speed control active)")
        
        return {
            "success": True,
            "message": f"Live trajectory execution started (approx duration: {total_time:.2f}s)",
            "estimated_duration": total_time
        }
    
    def _compute_distance(self, a, b):
        """Compute Euclidean distance between two same-length lists of numbers."""
        try:
            return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        except Exception:
            return float('inf')
    
    def _execution_callback(self):
        """Timer callback for live trajectory execution using PID to control progression speed."""
        if not self.current_trajectory or not self.is_executing:
            return
        
        # Compute dt since last PID update
        now = self.get_clock().now()
        if self.pid_last_time is None:
            self.pid_last_time = now
            dt = (now - self.pid_last_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 1e-6
        self.pid_last_time = now
        
        # Get final goal and current interpolated position
        final_point = self.current_trajectory['points'][-1]
        final_positions = final_point['positions']
        current_positions = self._interpolate_trajectory_at_time(self.execution_elapsed)
        
        # PID error: distance to goal (large -> speed up, small -> slow down)
        error = self._compute_distance(current_positions, final_positions)
        self.pid_integral += error * dt
        derivative = (error - self.pid_prev_error) / dt if dt > 0 else 0.0
        self.pid_prev_error = error
        
        # PID output controls the progression speed multiplier
        pid_output = (self.pid_kp * error) + (self.pid_ki * self.pid_integral) + (self.pid_kd * derivative)
        # Combine user-requested speed_factor with PID output and clamp
        speed_command = pid_output * self.execution_speed
        speed_command = max(self.min_speed, min(self.max_speed, speed_command))
        
        # Advance internal elapsed time according to commanded speed
        self.execution_elapsed += dt * speed_command
        
        # If reached or exceeded final time, publish final and stop
        total_time = self.current_trajectory['total_time']
        if self.execution_elapsed >= total_time:
            # Publish final position
            self._publish_joint_command(final_positions)
            self.stop_execution()
            self.get_logger().info("Live trajectory execution complete (PID)")
            return
        
        # Interpolate and publish current positions
        interp_positions = self._interpolate_trajectory_at_time(self.execution_elapsed)
        if interp_positions:
            self._publish_joint_command(interp_positions)
    
    def _publish_joint_command(self, positions):
        """Publish joint command for live robot control and print positions for monitoring."""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = positions
        joint_state.velocity = [0.0] * len(positions)
        joint_state.effort = [0.0] * len(positions)
        
        self.joint_state_pub.publish(joint_state)
        # Print/log positions for monitoring
        try:
            pos_str = ', '.join([f"{p:.4f}" for p in positions])
            self.get_logger().info(f"Joint positions: [{pos_str}]")
        except Exception:
            self.get_logger().info(f"Joint positions: {positions}")
    
    def stop_execution(self):
        """Stop current live trajectory execution"""
        if self.execution_timer:
            self.execution_timer.cancel()
            self.execution_timer = None
        
        self.is_executing = False
        # reset PID/elapsed
        self.execution_elapsed = 0.0
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = None
        self.get_logger().info("Trajectory execution stopped")
    
    def is_trajectory_executing(self):
        """Check if a trajectory is currently being executed"""
        return self.is_executing
    
    def get_execution_progress(self):
        """Get current execution progress as percentage (0-100)"""
        if not self.current_trajectory or self.current_trajectory.get('total_time', 0.0) <= 0:
            return 0.0
        
        progress = min(100.0, (self.execution_elapsed / self.current_trajectory['total_time']) * 100.0)
        return progress
    
    def plan_to_named_pose(self, pose_name, show_in_rviz=True):
        """Plan to a predefined named pose"""
        if pose_name not in self.named_poses:
            return {
                "success": False, 
                "message": f"Unknown pose: {pose_name}. Available poses: {list(self.named_poses.keys())}"
            }
        
        joint_positions = self.named_poses[pose_name]
        self.get_logger().info(f"Planning to named pose '{pose_name}': {joint_positions}")
        
        return self.plan_to_joint_state(joint_positions)
    
    def solve_ik(self, target_pose):
        """Solve inverse kinematics for a Cartesian pose"""
        if not self.ik_client.service_is_ready():
            return {
                "success": False,
                "message": "IK service not available"
            }
        
        try:
            self.get_logger().info(f"Solving IK for pose: {target_pose}")
            
            request = GetPositionIK.Request()
            
            # Set up IK request
            request.ik_request.group_name = self.group_name
            request.ik_request.ik_link_name = self.end_effector_link
            request.ik_request.attempts = 5
            request.ik_request.timeout = ROSDuration(sec=5, nanosec=0)
            
            # Create pose stamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = float(target_pose['position'][0])
            pose_stamped.pose.position.y = float(target_pose['position'][1])
            pose_stamped.pose.position.z = float(target_pose['position'][2])
            
            pose_stamped.pose.orientation.x = float(target_pose['orientation'][0])
            pose_stamped.pose.orientation.y = float(target_pose['orientation'][1])
            pose_stamped.pose.orientation.z = float(target_pose['orientation'][2])
            pose_stamped.pose.orientation.w = float(target_pose['orientation'][3])
            
            request.ik_request.pose_stamped = pose_stamped
            
            # Call IK service
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                
                if response and response.error_code.val == 1:  # SUCCESS
                    # Extract joint positions
                    joint_positions = []
                    solution_state = response.solution.joint_state
                    
                    for joint_name in self.joint_names:
                        if joint_name in solution_state.name:
                            idx = solution_state.name.index(joint_name)
                            joint_positions.append(solution_state.position[idx])
                        else:
                            return {
                                "success": False,
                                "message": f"Joint {joint_name} not found in IK solution"
                            }
                    
                    self.get_logger().info(f"IK solution: {joint_positions}")
                    return {
                        "success": True,
                        "joint_positions": joint_positions,
                        "message": "IK successful"
                    }
                else:
                    error_code = response.error_code.val if response else "No response"
                    return {
                        "success": False,
                        "message": f"IK failed with error code: {error_code}"
                    }
            else:
                return {
                    "success": False,
                    "message": "IK service call timed out"
                }
                
        except Exception as e:
            self.get_logger().error(f"IK error: {str(e)}")
            return {
                "success": False,
                "message": f"IK error: {str(e)}"
            }
    
    def plan_to_pose(self, target_pose, show_in_rviz=True):
        """Plan to a Cartesian pose by first solving IK, then planning to joint state"""
        # First solve IK
        ik_result = self.solve_ik(target_pose)
        
        if not ik_result['success']:
            return ik_result
        
        # Then plan to the joint configuration
        joint_positions = ik_result['joint_positions']
        return self.plan_to_joint_state(joint_positions)
    
    def _extract_trajectory_info(self, robot_trajectory):
        """Extract useful information from a robot trajectory"""
        try:
            joint_trajectory = robot_trajectory.joint_trajectory
            
            trajectory_info = {
                'joint_names': list(joint_trajectory.joint_names),
                'points': [],
                'total_time': 0.0,
                'num_points': len(joint_trajectory.points)
            };
            
            for point in joint_trajectory.points:
                point_info = {
                    'positions': list(point.positions),
                    'velocities': list(point.velocities) if point.velocities else [],
                    'accelerations': list(point.accelerations) if point.accelerations else [],
                    'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                }
                trajectory_info['points'].append(point_info)
            
            if trajectory_info['points']:
                trajectory_info['total_time'] = trajectory_info['points'][-1]['time_from_start']
            
            return trajectory_info
            
        except Exception as e:
            self.get_logger().error(f"Error extracting trajectory: {str(e)}")
            return None
    
    def get_trajectory_summary(self, trajectory_info):
        """Get trajectory summary as a dictionary"""
        if not trajectory_info:
            return {"error": "No trajectory information available"}
            
        summary = {
            "joint_names": trajectory_info['joint_names'],
            "num_waypoints": trajectory_info['num_points'],
            "total_time": trajectory_info['total_time'],
        }
        
        if trajectory_info['points']:
            start_pos = trajectory_info['points'][0]['positions']
            end_pos = trajectory_info['points'][-1]['positions']
            
            summary["start_position"] = {
                name: pos for name, pos in zip(trajectory_info['joint_names'], start_pos)
            }
            summary["end_position"] = {
                name: pos for name, pos in zip(trajectory_info['joint_names'], end_pos)
            }
            
            # Add sample waypoints if available
            if trajectory_info['num_points'] > 4:
                sample_indices = [
                    trajectory_info['num_points'] // 4,
                    trajectory_info['num_points'] // 2,
                    3 * trajectory_info['num_points'] // 4
                ]
                
                summary["sample_waypoints"] = []
                for idx in sample_indices:
                    if idx < len(trajectory_info['points']):
                        point = trajectory_info['points'][idx]
                        summary["sample_waypoints"].append({
                            "time": point['time_from_start'],
                            "positions": point['positions']
                        })
        
        return summary
    
    def get_live_trajectory_data(self, trajectory_info):
        """
        Extract trajectory data suitable for external live execution systems.
        Returns raw trajectory points with timing information.
        
        Args:
            trajectory_info: Trajectory information from successful planning
            
        Returns:
            dict: Live trajectory data with waypoints and timing
        """
        if not trajectory_info or not trajectory_info['points']:
            return {
                "success": False,
                "message": "No valid trajectory data"
            }
        
        # Extract waypoints for live execution
        waypoints = []
        for point in trajectory_info['points']:
            waypoint = {
                'joint_positions': point['positions'],
                'joint_velocities': point.get('velocities', [0.0] * len(point['positions'])),
                'joint_accelerations': point.get('accelerations', [0.0] * len(point['positions'])),
                'time_from_start': point['time_from_start']
            }
            waypoints.append(waypoint)
        
        return {
            "success": True,
            "joint_names": trajectory_info['joint_names'],
            "waypoints": waypoints,
            "total_duration": trajectory_info['total_time'],
            "num_waypoints": len(waypoints),
            "message": "Live trajectory data ready for execution"
        }


# Convenience function for direct use
def plan_joint_values(joint_values, wait_for_services=True, show_in_rviz=True):
    """
    Convenience function to plan to joint values without managing the node yourself.
    
    Args:
        joint_values: List of 5 joint values (strings or floats)
        wait_for_services: Whether to wait for MoveIt services to be ready
        show_in_rviz: Whether to show the planned path in RViz
    
    Returns:
        dict: Planning result with trajectory information
        
    Note: This function assumes ROS2 is already initialized
    """
    commander = PegasusCommander(wait_for_services=wait_for_services)
    
    try:
        result = commander.plan_to_joint_values(joint_values, show_in_rviz=show_in_rviz)
        return result
        
    finally:
        commander.destroy_node()


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
