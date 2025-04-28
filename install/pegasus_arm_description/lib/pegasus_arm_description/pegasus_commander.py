#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
from threading import Thread
from functools import partial
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, LookupException
from scipy.spatial.transform import Rotation as R
import time


class PegasusArmGUI:
    def __init__(self, root, commander_node):
        self.root = root
        self.commander_node = commander_node

        # Set window title and size
        root.title("Pegasus Arm Controller")
        root.geometry("600x500")
        root.resizable(False, False)

        # Create a style for the pressed buttons
        self.style = ttk.Style()
        self.style.configure("Pressed.TButton", background="#ADD8E6")

        # Bind arrow keys to directional movement
        root.bind("<Up>", lambda event: self.handle_key_press("up"))
        root.bind("<Down>", lambda event: self.handle_key_press("down"))
        root.bind("<Left>", lambda event: self.handle_key_press("left"))
        root.bind("<Right>", lambda event: self.handle_key_press("right"))

        # Create main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Display current joint states
        self.joint_frame = ttk.LabelFrame(main_frame, text="Joint States", padding="10")
        self.joint_frame.pack(fill=tk.X, pady=10)

        self.joint_labels = []
        self.joint_values = []

        # Get current joint values and names for display
        joint_values = self.commander_node.get_current_joint_values()
        joint_names = self.commander_node.get_joint_names()
        num_joints = len(joint_values)

        for i in range(num_joints):
            joint_row = ttk.Frame(self.joint_frame)
            joint_row.pack(fill=tk.X, pady=2)

            # Use actual joint name if available, otherwise fallback to Joint X
            joint_name = joint_names[i] if i < len(joint_names) else f"Joint {i+1}"
            joint_label = ttk.Label(joint_row, text=f"{joint_name}:", width=20)
            joint_label.pack(side=tk.LEFT)

            joint_value = ttk.Label(joint_row, text=f"{joint_values[i]:.3f}", width=10)
            joint_value.pack(side=tk.LEFT)

            self.joint_labels.append(joint_label)
            self.joint_values.append(joint_value)

        # Create control frame for direction buttons
        control_frame = ttk.LabelFrame(main_frame, text="Control", padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Create grid for direction buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(pady=20)

        # Create directional buttons
        self.buttons = {}
        self.buttons["up"] = ttk.Button(button_frame, text="Up", width=10, 
                                      command=partial(self.handle_button_press, "up"))
        self.buttons["up"].grid(row=0, column=1, padx=5, pady=5)

        self.buttons["left"] = ttk.Button(button_frame, text="Left", width=10,
                                        command=partial(self.handle_button_press, "left"))
        self.buttons["left"].grid(row=1, column=0, padx=5, pady=5)

        self.buttons["right"] = ttk.Button(button_frame, text="Right", width=10,
                                         command=partial(self.handle_button_press, "right"))
        self.buttons["right"].grid(row=1, column=2, padx=5, pady=5)

        self.buttons["down"] = ttk.Button(button_frame, text="Down", width=10,
                                        command=partial(self.handle_button_press, "down"))
        self.buttons["down"].grid(row=2, column=1, padx=5, pady=5)

        # Joint selector
        joint_control_frame = ttk.Frame(control_frame)
        joint_control_frame.pack(fill=tk.X, pady=10)

        ttk.Label(joint_control_frame, text="Select Joint:").pack(side=tk.LEFT, padx=5)

        # Joint selection dropdown - use actual joint names
        self.selected_joint = tk.StringVar()
        if joint_names:
            self.selected_joint.set(joint_names[0])  # Default to first joint
        else:
            self.selected_joint.set("No joints available")

        joint_dropdown = ttk.Combobox(joint_control_frame, textvariable=self.selected_joint, 
                                    values=joint_names, state="readonly", width=20)
        joint_dropdown.pack(side=tk.LEFT, padx=5)

        # Step size control
        ttk.Label(joint_control_frame, text="Step Size:").pack(side=tk.LEFT, padx=10)

        self.step_size = tk.DoubleVar(value=0.1)
        step_sizes = [0.01, 0.05, 0.1, 0.2, 0.5]

        step_dropdown = ttk.Combobox(joint_control_frame, textvariable=self.step_size,
                                   values=step_sizes, state="readonly", width=5)
        step_dropdown.pack(side=tk.LEFT, padx=5)

        # Positions frame
        positions_frame = ttk.Frame(control_frame)
        positions_frame.pack(fill=tk.X, pady=10)

        # Home position button
        home_button = ttk.Button(positions_frame, text="Go to Home Position", 
                                command=self.go_to_home)
        home_button.pack(side=tk.LEFT, padx=5)

        # Zero position button
        zero_button = ttk.Button(positions_frame, text="Go to Zero Position", 
                                command=self.go_to_zero)
        zero_button.pack(side=tk.LEFT, padx=5)

        # Position display
        pose_frame = ttk.LabelFrame(main_frame, text="End Effector Pose", padding="10")
        pose_frame.pack(fill=tk.X, pady=10)

        # Position display rows
        self.pose_labels = {}
        for coord in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            pose_row = ttk.Frame(pose_frame)
            pose_row.pack(fill=tk.X, pady=2)

            label = ttk.Label(pose_row, text=f"{coord}:", width=10)
            label.pack(side=tk.LEFT)

            value = ttk.Label(pose_row, text="0.000", width=10)
            value.pack(side=tk.LEFT)

            self.pose_labels[coord] = value

        # Status label
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, 
                               relief=tk.SUNKEN, anchor=tk.W)
        status_label.pack(fill=tk.X, side=tk.BOTTOM, pady=5)

        # Update displays periodically
        self.update_displays()

    def get_selected_joint_index(self):
        """Get the currently selected joint index (0-based)"""
        selected = self.selected_joint.get()
        joint_names = self.commander_node.get_joint_names()
        if selected in joint_names:
            return joint_names.index(selected)
        else:
            # Fallback to parsing "Joint X" format if needed
            try:
                return int(selected.split()[1]) - 1
            except:
                return 0

    def move_direction(self, direction):
        """Handle directional button presses"""
        joint_index = self.get_selected_joint_index()
        step = self.step_size.get()

        # Get current joint values
        joint_goal = self.commander_node.get_current_joint_values()

        # Check if joint index is valid
        if joint_index < 0 or joint_index >= len(joint_goal):
            self.status_var.set(f"Invalid joint index: {joint_index}")
            return

        # Adjust the selected joint based on direction
        if direction == "up" or direction == "right":
            joint_goal[joint_index] += step
        elif direction == "down" or direction == "left":
            joint_goal[joint_index] -= step

        # Send command to move the robot
        joint_names = self.commander_node.get_joint_names()
        if joint_index < len(joint_names):
            joint_name = joint_names[joint_index]
            self.status_var.set(f"Moving {joint_name} {direction}...")
        else:
            self.status_var.set(f"Moving Joint {joint_index+1} {direction}...")

        # Execute the move with callback
        future = self.commander_node.move_to_joint_positions(joint_goal)
        future.add_done_callback(self._move_complete_callback)

    def _move_complete_callback(self, future):
        """Callback when move action completes"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._result_callback)
            else:
                self.status_var.set("Movement rejected")
        except Exception as e:
            self.status_var.set(f"Movement error: {str(e)}")

    def _result_callback(self, future):
        """Callback for movement result"""
        try:
            result = future.result().result
            error_codes = {1: "SUCCESS", -1: "PLANNING_FAILED", -2: "INVALID_MOTION_PLAN"}
            error_msg = error_codes.get(result.error_code.val, f"Unknown error {result.error_code.val}")
            self.status_var.set(f"Movement result: {error_msg}")
        except Exception as e:
            self.status_var.set(f"Result error: {str(e)}")

    def update_displays(self):
        """Update the displayed joint values and pose"""
        try:
            # Get current joint values
            joint_values = self.commander_node.get_current_joint_values()

            # Update the displayed joint values
            for i, value_label in enumerate(self.joint_values):
                if i < len(joint_values):
                    value_label.config(text=f"{joint_values[i]:.3f}")

            # Get and update end effector pose
            pose = self.commander_node.get_current_pose()
            if pose:
                self.pose_labels["X"].config(text=f"{pose[0]:.3f}")
                self.pose_labels["Y"].config(text=f"{pose[1]:.3f}")
                self.pose_labels["Z"].config(text=f"{pose[2]:.3f}")
                self.pose_labels["Roll"].config(text=f"{pose[3]:.3f}")
                self.pose_labels["Pitch"].config(text=f"{pose[4]:.3f}")
                self.pose_labels["Yaw"].config(text=f"{pose[5]:.3f}")
        except Exception as e:
            self.status_var.set(f"Update error: {e}")
        
        # Schedule the next update
        self.root.after(200, self.update_displays)  # Reduced to 200ms for smoother updates

    def handle_button_press(self, direction):
        """Handle button press with proper button reset"""
        # Change button appearance temporarily
        button = self.buttons.get(direction)
        if button:
            button.state(['pressed'])
            self.root.update_idletasks()  # Force GUI update
        
        # Execute the move
        self.move_direction(direction)
        
        # Reset button after a brief delay
        self.root.after(100, lambda: button.state(['!pressed']) if button else None)

    def handle_key_press(self, direction):
        """Handle key press with visual button feedback"""
        # Highlight the corresponding button
        button = self.buttons.get(direction)
        if button:
            button.state(['pressed'])
            self.root.update_idletasks()  # Force GUI update
        
        # Execute the move
        self.move_direction(direction)
        
        # Reset button after a brief delay
        self.root.after(100, lambda: button.state(['!pressed']) if button else None)

    def go_to_home(self):
        """Move arm to home position defined in SRDF"""
        self.status_var.set("Moving to home position...")

        # Use named target from SRDF
        success = self.commander_node.move_to_named_target("home")

        # Update status
        if success:
            self.status_var.set("Moved to home position")
        else:
            self.status_var.set("Failed to move to home position")

        # Update displayed values
        self.update_displays()

    def go_to_zero(self):
        """Move arm to zero position (all joints at 0)"""
        self.status_var.set("Moving to zero position...")

        # Set all joints to zero
        zero_position = [0.0] * len(self.commander_node.get_current_joint_values())
        future = self.commander_node.move_to_joint_positions(zero_position)
        future.add_done_callback(self._move_complete_callback)


class PegasusCommander(Node):
    def __init__(self):
        super().__init__('pegasus_commander')
        
        # Initialize the action client for MoveGroup
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Create a subscriber for joint state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Initialize storage for current joint state
        self.current_joint_state = None
        self._joint_names = ["joint1_base", "joint2_shoulder", "joint3_elbow", "joint4", "joint5_wrist"]
        self._current_values = [0.0] * len(self._joint_names)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for action server with retries
        if not self.wait_for_action_server():
            self.get_logger().error("MoveIt action server not available. Continuing without action server.")
            self.action_server_available = False
        else:
            self.action_server_available = True

    def joint_state_callback(self, msg):
        """Callback to process incoming joint state messages"""
        self.current_joint_state = msg
        
        # Update internal joint state representation
        if self.current_joint_state:
            for i, name in enumerate(self.current_joint_state.name):
                if name in self._joint_names:
                    idx = self._joint_names.index(name)
                    if idx < len(self._current_values) and i < len(self.current_joint_state.position):
                        self._current_values[idx] = self.current_joint_state.position[i]

    def get_current_joint_values(self):
        """Get current joint values from the robot."""
        # Return values from joint state if available, otherwise use stored values
        if self.current_joint_state:
            return self._current_values
        return self._current_values

    def get_joint_names(self):
        """Get joint names from the robot description."""
        return self._joint_names

    def get_current_pose(self):
        """Get the current end-effector pose with retries and better error handling."""
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', 'end_effector_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                rpy = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_euler('xyz')
                return [translation.x, translation.y, translation.z, rpy[0], rpy[1], rpy[2]]
            except LookupException:
                self.get_logger().warn(f"Transform attempt {attempt+1}/{max_attempts} failed")
                time.sleep(0.1)
        self.get_logger().error("Failed to get end-effector pose")
        return None

    def _build_joint_constraints(self, joint_positions):
        """Helper function to build joint constraints."""
        joint_constraints = Constraints()
        for i, joint_value in enumerate(joint_positions):
            if i < len(self._joint_names):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = self._joint_names[i]
                joint_constraint.position = joint_value
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                joint_constraints.joint_constraints.append(joint_constraint)
        return joint_constraints

    def move_to_joint_positions(self, joint_positions):
        """Move to the specified joint positions."""
        if not self.action_server_available:
            self.get_logger().error("Cannot move to joint positions: Action server not available.")
            return None
        
        self.get_logger().info(f"Attempting to move to joint positions: {joint_positions}")
        
        # Create a MoveGroup goal message
        goal_msg = MoveGroup.Goal()
        
        # Configure the request
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Set joint constraints
        goal_msg.request.goal_constraints = [self._build_joint_constraints(joint_positions)]
        
        # Configure planning options
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.group_name = "pegasus_arm"  # Should match your MoveIt group name
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Send the goal and return the future
        self.get_logger().info('Sending move goal...')
        return self.move_action_client.send_goal_async(goal_msg)

    def move_to_named_target(self, target_name):
        """Move to a named target defined in SRDF."""
        if not self.action_server_available:
            self.get_logger().error(f"Cannot move to named target '{target_name}': Action server not available.")
            return False
        
        self.get_logger().info(f"Attempting to move to named target: {target_name}")
        
        # Create a MoveGroup goal message
        goal_msg = MoveGroup.Goal()
        
        # Configure the request
        goal_msg.request.group_name = "pegasus_arm"
        goal_msg.request.named_target = target_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Send the goal
        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Named target goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        success = result.error_code.val == MoveGroup.Result.SUCCESS
        if not success:
            self.get_logger().error(f"Failed to move to {target_name}: {result.error_code.val}")
        return success

    def wait_for_action_server(self):
        """Wait for the action server to be available with retries."""
        self.get_logger().info('Waiting for MoveIt action server...')
        max_retries = 10  # Increased retries
        retry_interval = 2.0  # seconds

        for attempt in range(max_retries):
            if self.move_action_client.wait_for_server(timeout_sec=retry_interval):
                self.get_logger().info('MoveIt action server is available.')
                return True
            self.get_logger().warn(f"Attempt {attempt + 1}/{max_retries}: MoveIt action server not available. Retrying...")

        self.get_logger().error("MoveIt action server not available after retries.")
        return False


def ros_spin(node):
    """Function to spin the ROS node in a separate thread"""
    rclpy.spin(node)


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create the ROS node first
    commander_node = PegasusCommander()
    
    # Create a separate thread for the ROS node
    ros_thread = Thread(target=ros_spin, args=(commander_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Create and start the tkinter GUI
        root = tk.Tk()
        app = PegasusArmGUI(root, commander_node)
        
        # Run the tkinter main loop
        root.mainloop()
        
    except Exception as e:
        print(f"Error in GUI: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        commander_node.destroy_node()
        rclpy.shutdown()
        
        # Make sure the ROS thread terminates
        if ros_thread.is_alive():
            ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()