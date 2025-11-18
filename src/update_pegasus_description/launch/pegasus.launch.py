#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, SolidPrimitive
from moveit_msgs.msg import JointConstraint, MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive as ShapeSolidPrimitive  # Note: Different import for shape_msgs
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import threading
import time

class DigitalTwinController(Node):
    def __init__(self):
        super().__init__('digital_twin_controller')
        
        # Parameters
        self.declare_parameter('target_pose', 'home')
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('planning_pipeline', 'RRTConnectkConfigDefault')  # Changed default to valid planner_id
        
        self.target_pose = self.get_parameter('target_pose').get_parameter_value().string_value
        self.execute_motion = self.get_parameter('execute_motion').get_parameter_value().bool_value
        self.planner_id = self.get_parameter('planning_pipeline').get_parameter_value().string_value  # Renamed for clarity
        
        # Callback group for multi-threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Action client for MoveIt
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/digital_twin/status', 10)
        self.joint_data_pub = self.create_publisher(String, '/digital_twin/joint_data', 10)
        self.trajectory_pub = self.create_publisher(String, '/digital_twin/trajectory', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/digital_twin/command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Internal state
        self.current_joint_states = None
        self.planning_result = None
        self.is_planning = False
        self.last_successful_plan = None
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Digital Twin Controller initialized')
        self.get_logger().info(f'Target pose: {self.target_pose}')
        self.get_logger().info(f'Execute motion: {self.execute_motion}')
        self.get_logger().info(f'Planner ID: {self.planner_id}')
    
    def joint_state_callback(self, msg):
        """Store current joint states for digital twin data"""
        self.current_joint_states = msg
        
        # Publish joint data as JSON for hardware interface
        joint_data = {
            'timestamp': time.time(),
            'joint_names': list(msg.name),
            'joint_positions': list(msg.position),
            'joint_velocities': list(msg.velocity) if msg.velocity else [],
            'joint_efforts': list(msg.effort) if msg.effort else []
        }
        
        joint_msg = String()
        joint_msg.data = json.dumps(joint_data)
        self.joint_data_pub.publish(joint_msg)
    
    def command_callback(self, msg):
        """Handle incoming commands from external interface"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type', '')
            
            if command_type == 'move_to_pose':
                self.handle_move_to_pose_command(command_data)
            elif command_type == 'move_to_joint_state':
                self.handle_move_to_joint_state_command(command_data)
            elif command_type == 'get_current_state':
                self.handle_get_current_state_command()
            elif command_type == 'set_planning_pipeline':
                self.handle_set_planning_pipeline_command(command_data)
            else:
                self.get_logger().warn(f'Unknown command type: {command_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse command JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error handling command: {e}')
    
    def handle_move_to_pose_command(self, command_data):
        """Handle move to Cartesian pose command"""
        pose_data = command_data.get('pose', {})
        
        # Create pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        target_pose.pose.position.x = pose_data.get('x', 0.5)
        target_pose.pose.position.y = pose_data.get('y', 0.0)
        target_pose.pose.position.z = pose_data.get('z', 0.5)
        
        target_pose.pose.orientation.x = pose_data.get('qx', 0.0)
        target_pose.pose.orientation.y = pose_data.get('qy', 0.0)
        target_pose.pose.orientation.z = pose_data.get('qz', 0.0)
        target_pose.pose.orientation.w = pose_data.get('qw', 1.0)
        
        self.plan_to_pose(target_pose)
    
    def handle_move_to_joint_state_command(self, command_data):
        """Handle move to joint state command"""
        joint_targets = command_data.get('joint_targets', {})
        self.plan_to_joint_state(joint_targets)
    
    def handle_get_current_state_command(self):
        """Handle get current state command"""
        if self.current_joint_states:
            state_data = {
                'timestamp': time.time(),
                'joint_names': list(self.current_joint_states.name),
                'joint_positions': list(self.current_joint_states.position),
                'planning_status': 'idle' if not self.is_planning else 'planning'
            }
            
            status_msg = String()
            status_msg.data = json.dumps(state_data)
            self.status_pub.publish(status_msg)
    
    def handle_set_planning_pipeline_command(self, command_data):
        """Handle set planning pipeline command"""
        pipeline = command_data.get('pipeline', 'RRTConnectkConfigDefault')
        self.planner_id = pipeline
        self.get_logger().info(f'Planner ID changed to: {pipeline}')
    
    def plan_to_pose(self, target_pose):
        """Plan motion to target pose"""
        if self.is_planning:
            self.get_logger().warn('Already planning, ignoring request')
            return
        
        self.is_planning = True
        
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            self.is_planning = False
            return
        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'pegasus_arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.planner_id = self.planner_id
        
        # Set start state from current joints if available
        if self.current_joint_states:
            start_state = RobotState()
            start_state.joint_state = self.current_joint_states
            goal.request.start_state = start_state
        
        # Create constraints
        constraints = Constraints()
        
        # Position constraint with tolerance region
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = 'end_effector_link'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        pos_constraint.weight = 1.0
        
        bounding_volume = BoundingVolume()
        primitive = ShapeSolidPrimitive()
        primitive.type = ShapeSolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # Small tolerance radius
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint with tolerance
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = 'end_effector_link'
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Sending planning request...')
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.planning_response_callback)
    
    def plan_to_joint_state(self, joint_targets):
        """Plan motion to target joint state"""
        if self.is_planning:
            self.get_logger().warn('Already planning, ignoring request')
            return
        
        self.is_planning = True
        
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            self.is_planning = False
            return
        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'pegasus_arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.planner_id = self.planner_id
        
        # Set start state from current joints if available
        if self.current_joint_states:
            start_state = RobotState()
            start_state.joint_state = self.current_joint_states
            goal.request.start_state = start_state
        
        # Create single constraints for all joints
        constraints = Constraints()
        for joint_name, target_value in joint_targets.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Sending planning request...')
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.planning_response_callback)
    
    def planning_response_callback(self, future):
        """Handle planning response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Planning request rejected')
                self.is_planning = False
                return
            
            self.get_logger().info('Planning request accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.planning_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Planning request failed: {e}')
            self.is_planning = False
    
    def planning_result_callback(self, future):
        """Handle planning result"""
        try:
            result = future.result().result
            self.planning_result = result
            self.is_planning = False
            
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Planning succeeded!')
                self.last_successful_plan = result.planned_trajectory
                
                # Publish trajectory data
                trajectory_data = {
                    'timestamp': time.time(),
                    'success': True,
                    'planning_time': result.planning_time,
                    'trajectory_points': len(result.planned_trajectory.joint_trajectory.points) if result.planned_trajectory.joint_trajectory else 0,
                    'joint_names': list(result.planned_trajectory.joint_trajectory.joint_names) if result.planned_trajectory.joint_trajectory else []
                }
                
                traj_msg = String()
                traj_msg.data = json.dumps(trajectory_data)
                self.trajectory_pub.publish(traj_msg)
                
                # Execute if requested (Note: Placeholder; implement ExecuteTrajectory if needed)
                if self.execute_motion:
                    self.get_logger().info('Executing motion... (Placeholder: Add execution logic here)')
                    # For real execution, create another ActionClient for /execute_trajectory and send the planned_trajectory
                
            else:
                self.get_logger().error(f'Planning failed with error code: {result.error_code.val}')
                
                # Publish failure data
                trajectory_data = {
                    'timestamp': time.time(),
                    'success': False,
                    'error_code': result.error_code.val
                }
                
                traj_msg = String()
                traj_msg.data = json.dumps(trajectory_data)
                self.trajectory_pub.publish(traj_msg)
                
        except Exception as e:
            self.get_logger().error(f'Planning result handling failed: {e}')
            self.is_planning = False
    
    def publish_status(self):
        """Publish periodic status updates"""
        status_data = {
            'timestamp': time.time(),
            'node_status': 'active',
            'is_planning': self.is_planning,
            'planning_pipeline': self.planner_id,
            'has_current_state': self.current_joint_states is not None,
            'last_plan_successful': self.last_successful_plan is not None
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = DigitalTwinController()
    
    # Use multi-threaded executor for handling multiple callbacks
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()