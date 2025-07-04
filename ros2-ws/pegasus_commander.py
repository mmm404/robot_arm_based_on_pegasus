#!/usr/bin/env python3
import sys
import os
import yaml
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Lock, Event
from functools import partial
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
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

# Define MoveIt error codes mapping
MOVEIT_ERROR_CODES = {
    MoveItErrorCodes.SUCCESS: "Success",
    MoveItErrorCodes.FAILURE: "Failure",
    MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "Plan invalidated by environment change",
    MoveItErrorCodes.CONTROL_FAILED: "Control failed",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "Unable to acquire sensor data",
    MoveItErrorCodes.TIMED_OUT: "Operation timed out",
    MoveItErrorCodes.PREEMPTED: "Operation preempted",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "Start state in collision",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "Start state violates path constraints",
    MoveItErrorCodes.GOAL_IN_COLLISION: "Goal in collision",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "Goal violates path constraints",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "Goal constraints violated",
    MoveItErrorCodes.INVALID_GROUP_NAME: "Invalid group name",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "Invalid goal constraints",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "Invalid robot state",
    MoveItErrorCodes.INVALID_LINK_NAME: "Invalid link name",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "Invalid object name",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "Frame transform failure",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "Collision checking unavailable",
    MoveItErrorCodes.ROBOT_STATE_STALE: "Robot state stale",
    MoveItErrorCodes.SENSOR_INFO_STALE: "Sensor info stale",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "Communication failure",
    MoveItErrorCodes.NO_IK_SOLUTION: "No IK solution"
}

class PegasusArmGUI:
    def __init__(self, root, commander_node):
        self.root = root
        self.commander_node = commander_node
        self.movement_in_progress = Event()
        self.last_command_time = 0
        self.command_cooldown = 0.5
        self.lock = Lock()

        # Configure logging
        self.logger = logging.getLogger(__name__)
        self.setup_gui()
        self.setup_keyboard_shortcuts()
        
        self.log_action("Controller started successfully")
        if not self.commander_node.controller_manager_available:
            self.log_action("Controller manager unavailable. Check if controller_manager is running.")
            messagebox.showwarning("Warning", "Controller manager unavailable. Please start the launch file.")
        if not self.commander_node.tf_available:
            self.log_action("TF tree unavailable. End effector pose updates disabled.")
            messagebox.showinfo("Info", "TF unavailable. Try changing Base Frame or check robot_state_publisher.")
        
        self.status_check()
        self.update_displays()
        self.root.after(1000, self.update_tf_status)

    def setup_gui(self):
        self.root.geometry("800x700")
        self.root.resizable(False, False)
        self.root.title("Pegasus Arm Controller")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Configure styles
        self.style = ttk.Style()
        self.style.configure("TButton", font=("Helvetica", 10), padding=10)
        self.style.configure("Rounded.TButton", relief="flat", borderwidth=1, padding=10)
        self.style.map("Rounded.TButton", background=[("active", "#d9d9d9")], relief=[("pressed", "sunken")])
        self.style.configure("Danger.TButton", background="red", foreground="white", font=("Helvetica", 10))
        self.style.configure("Success.TButton", background="green", foreground="white", font=("Helvetica", 10))
        self.style.configure("Status.TLabel", font=("Helvetica", 9))
        self.style.configure("Alert.TLabel", foreground="red", font=("Helvetica", 9, "bold"))

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Status frame
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=2)
        
        self.tf_status_var = tk.StringVar(value="TF Status: Checking...")
        tf_status_style = "Success.TLabel" if self.commander_node.tf_available else "Alert.TLabel"
        self.tf_status_label = ttk.Label(status_frame, textvariable=self.tf_status_var, style=tf_status_style)
        self.tf_status_label.pack(fill=tk.X, pady=2)

        frame_selector_frame = ttk.Frame(status_frame)
        frame_selector_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(frame_selector_frame, text="Base Frame:").pack(side=tk.LEFT, padx=5)
        self.base_frame_var = tk.StringVar(value=self.commander_node.base_frame)
        self.base_frame_dropdown = ttk.Combobox(
            frame_selector_frame,
            textvariable=self.base_frame_var,
            values=["world", "base_link"],
            state="readonly",
            width=15
        )
        self.base_frame_dropdown.pack(side=tk.LEFT, padx=5)
        self.base_frame_dropdown.bind("<<ComboboxSelected>>", lambda e: self.refresh_tf_status())
        
        refresh_button = ttk.Button(
            frame_selector_frame,
            text="Refresh TF",
            command=self.refresh_tf_status,
            style="Rounded.TButton"
        )
        refresh_button.pack(side=tk.LEFT, padx=5)

        self.status_var = tk.StringVar(value="Initializing...")
        status_label = ttk.Label(status_frame, textvariable=self.status_var, style="Status.TLabel")
        status_label.pack(fill=tk.X, pady=5)

        # Notebook for tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Joint Control Tab
        joint_tab = ttk.Frame(self.notebook)
        self.notebook.add(joint_tab, text="Joint Control")
        
        self.joint_frame = ttk.LabelFrame(joint_tab, text="Joint States", padding="10")
        self.joint_frame.pack(fill=tk.X, pady=10)
        self.setup_joint_controls()
        
        control_frame = ttk.LabelFrame(joint_tab, text="Control", padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        self.control_frame = control_frame
        self.setup_control_buttons()
        
        # End Effector Tab
        ee_tab = ttk.Frame(self.notebook)
        self.notebook.add(ee_tab, text="End Effector")
        
        pose_frame = ttk.LabelFrame(ee_tab, text="End Effector Pose", padding="10")
        pose_frame.pack(fill=tk.X, pady=10)
        self.pose_frame = pose_frame
        self.setup_pose_display()
        
        cartesian_frame = ttk.LabelFrame(ee_tab, text="Cartesian Control", padding="10")
        cartesian_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        self.setup_cartesian_controls(cartesian_frame)
        
        # Settings Tab
        settings_tab = ttk.Frame(self.notebook)
        self.notebook.add(settings_tab, text="Settings")
        self.setup_settings_tab(settings_tab)
        
        # Timestamp
        self.timestamp_var = tk.StringVar(value="Timestamp: --:--:--")
        timestamp_label = ttk.Label(main_frame, textvariable=self.timestamp_var, font=("Helvetica", 9, "italic"))
        timestamp_label.pack(fill=tk.X, pady=5)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Action Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        self.log_frame = log_frame
        
        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED, wrap=tk.WORD, font=("Helvetica", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        log_scrollbar = ttk.Scrollbar(self.log_text, orient="vertical", command=self.log_text.yview)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)

    def setup_joint_controls(self):
        self.joint_labels = []
        self.joint_values = []
        self.joint_sliders = []
        self.joint_vars = []

        joint_values = self.commander_node.get_current_joint_values()
        joint_names = self.commander_node.get_joint_names()
        joint_limits = self.commander_node.get_joint_limits()
        num_joints = len(joint_values)

        for i in range(num_joints):
            joint_row = ttk.Frame(self.joint_frame)
            joint_row.pack(fill=tk.X, pady=5)

            joint_name = joint_names[i]
            joint_label = ttk.Label(joint_row, text=f"{joint_name}:", width=20, font=("Helvetica", 10))
            joint_label.pack(side=tk.LEFT)

            joint_value = ttk.Label(joint_row, text=f"{joint_values[i]:.3f}", width=10, font=("Helvetica", 10))
            joint_value.pack(side=tk.LEFT)
            self.joint_labels.append(joint_label)
            self.joint_values.append(joint_value)

            joint_var = tk.DoubleVar(value=joint_values[i])
            self.joint_vars.append(joint_var)
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            slider = ttk.Scale(
                joint_row,
                from_=min_limit,
                to=max_limit,
                variable=joint_var,
                orient=tk.HORIZONTAL,
                command=lambda v, idx=i: self.handle_slider(idx)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.joint_sliders.append(slider)

    def setup_control_buttons(self):
        button_frame = ttk.Frame(self.control_frame)
        button_frame.pack(pady=10)

        self.buttons = {}
        directions = [("up", "Up", 0, 1), ("left", "Left", 1, 0), ("right", "Right", 1, 2), ("down", "Down", 2, 1)]
        for dir_en, dir_label, row, col in directions:
            self.buttons[dir_en] = ttk.Button(
                button_frame,
                text=dir_label,
                width=10,
                style="Rounded.TButton",
                command=partial(self.handle_button_press, dir_en)
            )
            self.buttons[dir_en].grid(row=row, column=col, padx=5, pady=5)

        preset_frame = ttk.Frame(self.control_frame)
        preset_frame.pack(pady=10)

        home_button = ttk.Button(
            preset_frame,
            text="Home",
            style="Rounded.TButton",
            command=self.go_to_home
        )
        home_button.pack(side=tk.LEFT, padx=5)

        zero_button = ttk.Button(
            preset_frame,
            text="Zero",
            style="Rounded.TButton",
            command=self.go_to_zero
        )
        zero_button.pack(side=tk.LEFT, padx=5)

        stop_button = ttk.Button(
            preset_frame,
            text="STOP",
            style="Danger.TButton",
            command=self.emergency_stop
        )
        stop_button.pack(side=tk.LEFT, padx=20)

        retry_controllers_button = ttk.Button(
            preset_frame,
            text="Retry Controllers",
            style="Rounded.TButton",
            command=self.retry_controllers
        )
        retry_controllers_button.pack(side=tk.LEFT, padx=5)

        joint_control_frame = ttk.Frame(self.control_frame)
        joint_control_frame.pack(fill=tk.X, pady=10)

        ttk.Label(joint_control_frame, text="Joint:").pack(side=tk.LEFT, padx=5)
        joint_names = self.commander_node.get_joint_names()
        self.selected_joint = tk.StringVar(value=joint_names[0] if joint_names else "No joints available")
        joint_dropdown = ttk.Combobox(
            joint_control_frame,
            textvariable=self.selected_joint,
            values=joint_names,
            state="readonly",
            width=20,
            font=("Helvetica", 10)
        )
        joint_dropdown.pack(side=tk.LEFT, padx=5)

        ttk.Label(joint_control_frame, text="Step:").pack(side=tk.LEFT, padx=5)
        self.step_size = tk.DoubleVar(value=0.1)
        step_sizes = [0.01, 0.05, 0.1, 0.2, 0.5]
        step_dropdown = ttk.Combobox(
            joint_control_frame,
            textvariable=self.step_size,
            values=step_sizes,
            state="readonly",
            width=5,
            font=("Helvetica", 10)
        )
        step_dropdown.pack(side=tk.LEFT, padx=5)

        ttk.Label(joint_control_frame, text="Vel:").pack(side=tk.LEFT, padx=5)
        self.velocity_scale = tk.DoubleVar(value=0.3)
        velocity_scales = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
        velocity_dropdown = ttk.Combobox(
            joint_control_frame,
            textvariable=self.velocity_scale,
            values=velocity_scales,
            state="readonly",
            width=5,
            font=("Helvetica", 10)
        )
        velocity_dropdown.pack(side=tk.LEFT, padx=5)

    def setup_pose_display(self):
        self.pose_labels = {}
        for coord in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            pose_row = ttk.Frame(self.pose_frame)
            pose_row.pack(fill=tk.X, pady=5)

            label = ttk.Label(pose_row, text=f"{coord}:", width=10, font=("Helvetica", 10))
            label.pack(side=tk.LEFT)

            value = ttk.Label(pose_row, text="N/A" if not self.commander_node.tf_available else "0.000", width=10, font=("Helvetica", 10))
            value.pack(side=tk.LEFT)
            self.pose_labels[coord] = value

    def setup_cartesian_controls(self, parent_frame):
        cartesian_controls = ttk.Frame(parent_frame)
        cartesian_controls.pack(pady=10)
        
        linear_frame = ttk.LabelFrame(cartesian_controls, text="Linear Motion", padding="5")
        linear_frame.pack(side=tk.LEFT, padx=10, fill=tk.Y)
        
        self.cart_buttons = {}
        cartesian_dirs = [
            ("x_pos", "X+", 0, 2), ("x_neg", "X-", 0, 0),
            ("y_pos", "Y+", 1, 2), ("y_neg", "Y-", 1, 0),
            ("z_pos", "Z+", 2, 2), ("z_neg", "Z-", 2, 0)
        ]
        
        for dir_key, label, row, col in cartesian_dirs:
            self.cart_buttons[dir_key] = ttk.Button(
                linear_frame,
                text=label,
                width=5,
                style="Rounded.TButton",
                command=partial(self.move_cartesian, dir_key)
            )
            self.cart_buttons[dir_key].grid(row=row, column=col, padx=5, pady=5)
        
        rotation_frame = ttk.LabelFrame(cartesian_controls, text="Rotation", padding="5")
        rotation_frame.pack(side=tk.LEFT, padx=10, fill=tk.Y)
        
        rotation_dirs = [
            ("roll_pos", "Roll+", 0, 2), ("roll_neg", "Roll-", 0, 0),
            ("pitch_pos", "Pitch+", 1, 2), ("pitch_neg", "Pitch-", 1, 0),
            ("yaw_pos", "Yaw+", 2, 2), ("yaw_neg", "Yaw-", 2, 0)
        ]
        
        for dir_key, label, row, col in rotation_dirs:
            self.cart_buttons[dir_key] = ttk.Button(
                rotation_frame,
                text=label,
                width=7,
                style="Rounded.TButton",
                command=partial(self.move_cartesian, dir_key)
            )
            self.cart_buttons[dir_key].grid(row=row, column=col, padx=5, pady=5)

        step_frame = ttk.Frame(parent_frame)
        step_frame.pack(pady=5)
        
        ttk.Label(step_frame, text="Cartesian Step (m/rad):").pack(side=tk.LEFT, padx=5)
        self.cart_step_size = tk.DoubleVar(value=0.01)
        cart_step_sizes = [0.005, 0.01, 0.02, 0.05, 0.1]
        cart_step_dropdown = ttk.Combobox(
            step_frame,
            textvariable=self.cart_step_size,
            values=cart_step_sizes,
            state="readonly",
            width=5,
            font=("Helvetica", 10)
        )
        cart_step_dropdown.pack(side=tk.LEFT, padx=5)

    def setup_settings_tab(self, parent_frame):
        settings_frame = ttk.Frame(parent_frame, padding="10")
        settings_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(settings_frame, text="Controller Name:").pack(anchor="w", padx=5, pady=5)
        self.controller_name_var = tk.StringVar(value=self.commander_node.controller_name)
        controller_entry = ttk.Entry(settings_frame, textvariable=self.controller_name_var, width=30)
        controller_entry.pack(anchor="w", padx=5)
        
        update_controller_button = ttk.Button(
            settings_frame,
            text="Update Controller",
            style="Rounded.TButton",
            command=self.update_controller_name
        )
        update_controller_button.pack(anchor="w", padx=5, pady=5)
        
        reload_limits_button = ttk.Button(
            settings_frame,
            text="Reload Joint Limits",
            style="Rounded.TButton",
            command=self.commander_node.load_joint_limits
        )
        reload_limits_button.pack(anchor="w", padx=5, pady=5)

    def setup_keyboard_shortcuts(self):
        self.root.bind("<Up>", lambda e: self.handle_key_press("up"))
        self.root.bind("<Down>", lambda e: self.handle_key_press("down"))
        self.root.bind("<Left>", lambda e: self.handle_key_press("left"))
        self.root.bind("<Right>", lambda e: self.handle_key_press("right"))
        self.root.bind("<h>", lambda e: self.go_to_home())
        self.root.bind("<z>", lambda e: self.go_to_zero())
        self.root.bind("<Escape>", lambda e: self.emergency_stop())
        self.root.bind("<F5>", lambda e: self.refresh_tf_status())

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.commander_node.cancel_current_goals()
            self.root.destroy()
            rclpy.shutdown()

    def log_action(self, message):
        self.logger.info(message)
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        line_count = int(self.log_text.index('end-1c').split('.')[0])
        if line_count > 1000:
            self.log_text.delete('1.0', f"{line_count-500}.0")
        self.log_text.config(state=tk.DISABLED)

    def status_check(self):
        status = []
        if self.commander_node.action_server_available:
            status.append("MoveIt OK")
        else:
            status.append("MoveIt Disconnected")
        if self.commander_node.controller_available:
            status.append("Controller OK")
        else:
            status.append("Controller Disconnected")
        if self.commander_node.joint_states_available:
            status.append("Joints OK")
        else:
            status.append("Joints Disconnected")
        if self.commander_node.tf_available:
            status.append("TF OK")
        else:
            status.append("TF Disconnected")
        self.status_var.set(" | ".join(status))
        self.root.after(1000, self.status_check)

    def update_tf_status(self):
        if self.commander_node.tf_available:
            self.tf_status_var.set(f"TF Status: Connected (Base: {self.commander_node.base_frame} → {self.commander_node.end_effector_frame})")
            self.tf_status_label.configure(style="Success.TLabel")
        else:
            self.tf_status_var.set(f"TF Status: Disconnected (Base: {self.commander_node.base_frame} → {self.commander_node.end_effector_frame})")
            self.tf_status_label.configure(style="Alert.TLabel")
        self.root.after(5000, self.update_tf_status)

    def refresh_tf_status(self):
        new_base_frame = self.base_frame_var.get()
        if new_base_frame != self.commander_node.base_frame:
            self.commander_node.base_frame = new_base_frame
            self.log_action(f"Changed base frame to '{new_base_frame}'")
        
        self.commander_node.check_tf_availability(level_check=True)
        self.update_tf_status()
        
        frames = self.commander_node.get_available_frames()
        if frames:
            self.base_frame_dropdown['values'] = frames
            self.log_action(f"Available frames: {', '.join(frames)}")
        else:
            self.log_action("No TF frames available. Check 'robot_state_publisher'.")

    def retry_controllers(self):
        self.log_action("Attempting to activate controllers...")
        success = self.commander_node.activate_controllers()
        if success:
            self.log_action("Controllers activated successfully")
            messagebox.showinfo("Success", "Controllers activated successfully")
        else:
            self.log_action("Failed to activate controllers. Check controller_manager.")
            messagebox.showerror("Error", "Failed to activate controllers. Please check logs.")

    def update_controller_name(self):
        new_controller = self.controller_name_var.get()
        if new_controller != self.commander_node.controller_name:
            self.commander_node.update_controller_name(new_controller)
            self.log_action(f"Updated controller name to '{new_controller}'")
            messagebox.showinfo("Success", f"Controller name updated to '{new_controller}'")

    def get_selected_joint_index(self):
        joint_name = self.selected_joint.get()
        joint_names = self.commander_node.get_joint_names()
        return joint_names.index(joint_name) if joint_name in joint_names else -1

    def move_direction(self, direction):
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            self.log_action("Command too soon, please wait...")
            return

        if self.movement_in_progress.is_set():
            self.log_action("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = current_time

        try:
            joint_index = self.get_selected_joint_index()
            step = self.step_size.get()
            joint_goal = self.commander_node.get_current_joint_values()
            joint_limits = self.commander_node.get_joint_limits()
            joint_name = self.commander_node.get_joint_names()[joint_index]

            if joint_index < 0 or joint_index >= len(joint_goal):
                self.log_action(f"Invalid joint index: {joint_index}")
                messagebox.showerror("Error", "Invalid joint selected")
                return

            if direction in ["up", "right"]:
                joint_goal[joint_index] += step
            elif direction in ["down", "left"]:
                joint_goal[joint_index] -= step

            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)

            self.log_action(f"Moving {joint_name} {direction}...")
            velocity = self.velocity_scale.get()
            success, error_code = self.commander_node.move_to_joint_positions(joint_goal, velocity)
            if success:
                self.joint_vars[joint_index].set(joint_goal[joint_index])
                self.log_action(f"{joint_name} movement completed")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Movement failed: {error_msg}")
                messagebox.showerror("Error", f"Movement failed: {error_msg}")
        except Exception as e:
            self.log_action(f"Error during movement: {str(e)}")
            messagebox.showerror("Error", f"Movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def handle_slider(self, joint_index):
        if self.movement_in_progress.is_set():
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            joint_goal = self.commander_node.get_current_joint_values()
            joint_name = self.commander_node.get_joint_names()[joint_index]
            joint_goal[joint_index] = self.joint_vars[joint_index].get()
            joint_limits = self.commander_node.get_joint_limits()
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)

            self.log_action(f"Moving {joint_name} via slider...")
            velocity = self.velocity_scale.get()
            success, error_code = self.commander_node.move_to_joint_positions(joint_goal, velocity)
            if success:
                self.log_action(f"{joint_name} slider movement completed")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Slider movement failed: {error_msg}")
                messagebox.showerror("Error", f"Slider movement failed: {error_msg}")
        except Exception as e:
            self.log_action(f"Error during slider movement: {str(e)}")
            messagebox.showerror("Error", f"Slider movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def move_cartesian(self, direction):
        if self.movement_in_progress.is_set():
            self.log_action("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            step = self.cart_step_size.get()
            direction_map = {
                "x_pos": [step, 0, 0], "x_neg": [-step, 0, 0],
                "y_pos": [0, step, 0], "y_neg": [0, -step, 0],
                "z_pos": [0, 0, step], "z_neg": [0, 0, -step],
                "roll_pos": [step, 0, 0], "roll_neg": [-step, 0, 0],
                "pitch_pos": [0, step, 0], "pitch_neg": [0, -step, 0],
                "yaw_pos": [0, 0, step], "yaw_neg": [0, 0, -step]
            }
            is_rotation = direction.startswith(("roll", "pitch", "yaw"))
            delta = direction_map.get(direction, [0, 0, 0])

            self.log_action(f"Moving end effector {'rotation' if is_rotation else 'position'} ({direction})...")
            success, error_code = self.commander_node.move_cartesian(delta, is_rotation, self.velocity_scale.get())
            if success:
                self.log_action(f"Cartesian movement ({direction}) completed")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Cartesian movement failed: {error_msg}")
                messagebox.showerror("Error", f"Cartesian movement failed: {error_msg}")
        except Exception as e:
            self.log_action(f"Error during Cartesian movement: {str(e)}")
            messagebox.showerror("Error", f"Cartesian movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def update_displays(self):
        try:
            if not self.movement_in_progress.is_set():
                joint_values = self.commander_node.get_current_joint_values()
                for i, (value_label, var) in enumerate(zip(self.joint_values, self.joint_vars)):
                    if i < len(joint_values):
                        value_label.config(text=f"{joint_values[i]:.3f}")
                        var.set(joint_values[i])

                if self.commander_node.tf_available:
                    pose = self.commander_node.get_current_pose()
                    if pose:
                        for coord, value in zip(["X", "Y", "Z", "Roll", "Pitch", "Yaw"], pose):
                            self.pose_labels[coord].config(text=f"{value:.3f}")
                    else:
                        for coord in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
                            self.pose_labels[coord].config(text="N/A")

            self.timestamp_var.set(f"Timestamp: {time.strftime('%H:%M:%S')}")
        except Exception as e:
            self.log_action(f"Update error: {str(e)}")

        update_interval = 1000 if self.movement_in_progress.is_set() else 500
        self.root.after(update_interval, self.update_displays)

    def handle_button_press(self, direction):
        button = self.buttons.get(direction)
        if button:
            button.state(['pressed'])
            self.root.update_idletasks()
        self.move_direction(direction)
        self.root.after(100, lambda: button.state(['!pressed']) if button else None)

    def handle_key_press(self, direction):
        button = self.buttons.get(direction)
        if button:
            button.state(['pressed'])
            self.root.update_idletasks()
        self.move_direction(direction)
        self.root.after(100, lambda: button.state(['!pressed']) if button else None)

    def go_to_home(self):
        if self.movement_in_progress.is_set():
            self.log_action("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            self.log_action("Moving to home position...")
            success, error_code = self.commander_node.move_to_named_target("home", self.velocity_scale.get())
            if success:
                self.log_action("Moved to home position")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Failed to move to home: {error_msg}")
                messagebox.showerror("Error", f"Failed to move to home: {error_msg}")
        except Exception as e:
            self.log_action(f"Error moving to home: {str(e)}")
            messagebox.showerror("Error", f"Home movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def go_to_zero(self):
        if self.movement_in_progress.is_set():
            self.log_action("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            self.log_action("Moving to zero position...")
            zero_position = [0.0] * len(self.commander_node.get_joint_names())
            velocity = self.velocity_scale.get()
            success, error_code = self.commander_node.move_to_joint_positions(zero_position, velocity)
            if success:
                self.log_action("Moved to zero position")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Failed to move to zero: {error_msg}")
                messagebox.showerror("Error", f"Failed to move to zero: {error_msg}")
        except Exception as e:
            self.log_action(f"Error moving to zero: {str(e)}")
            messagebox.showerror("Error", f"Zero movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def emergency_stop(self):
        self.log_action("EMERGENCY STOP ACTIVATED")
        self.commander_node.cancel_current_goals()
        self.movement_in_progress.clear()
        self.root.after(1000, lambda: self.log_action("Emergency stop completed"))
        messagebox.showinfo("Info", "Emergency stop completed")

class PegasusCommander(Node):
    def __init__(self):
        super().__init__('pegasus_commander')
        self.logger = self.get_logger()

        # Declare parameters
        self.declare_parameter('controller_name', 'pegasus_arm_controller')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('end_effector_frame', 'end_effector_link')
        self.declare_parameter('joint_limits_file', os.path.join(
            get_package_share_directory('pegasus_arm_moveit_config'),
            'config',
            'joint_limits.yaml'
        ))

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        self.joint_limits_file = self.get_parameter('joint_limits_file').get_parameter_value().string_value

        self._joint_names = ["joint1_base", "joint2_shoulder", "joint3_elbow", "joint4", "joint5_wrist"]
        self.load_joint_limits()

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
        self._current_values = [0.0] * len(self._joint_names)
        self.current_goal_handle = None

        self.initialize_services_and_topics()
        self.check_tf_availability(level_check=True)
        self.create_timer(2.0, self.check_services_and_topics)

    def load_joint_limits(self):
        try:
            with open(self.joint_limits_file, 'r') as file:
                joint_limits_data = yaml.safe_load(file)
            
            self.joint_limits = {}
            for joint_name in self._joint_names:
                joint_data = joint_limits_data.get('joint_limits', {}).get(joint_name, {})
                if joint_data.get('has_position_limits', False):
                    min_pos = joint_data.get('min_position', -3.14)
                    max_pos = joint_data.get('max_position', 3.14)
                else:
                    min_pos, max_pos = -3.14, 3.14
                self.joint_limits[joint_name] = (min_pos, max_pos)
            self.logger.info("Loaded joint limits from joint_limits.yaml")
        except Exception as e:
            self.logger.error(f"Failed to load joint_limits.yaml: {str(e)}")
            self.joint_limits = {
                'joint1_base': (-3.14, 3.14),
                'joint2_shoulder': (-1.22173, 0.610865),
                'joint3_elbow': (-2.74533, 1.0),
                'joint4': (-1.309, 0.0),
                'joint5_wrist': (-3.14, 3.14)
            }
            self.logger.info("Using fallback joint limits")

    def initialize_services_and_topics(self):
        max_retries = 15
        timeout_sec = 10.0
        for attempt in range(max_retries):
            if self.wait_for_service('/controller_manager/list_controllers', timeout_sec):
                self.controller_manager_available = True
                self.logger.info("Controller manager service available")
                break
            self.logger.warn(f"Controller manager service not available (attempt {attempt+1}/{max_retries})")
            time.sleep(2.0)
        if not self.controller_manager_available:
            self.logger.error("Failed to connect to controller manager after retries.")

        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_action_client = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.controller_name}/joint_trajectory',
            10
        )

        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_service(self, service_name, timeout_sec):
        client = self.create_client(ListControllers, service_name)
        return client.wait_for_service(timeout_sec=timeout_sec)

    def check_tf_availability(self, level_check=False):
        max_retries = 10
        retry_delay = 2.0
        for attempt in range(max_retries):
            try:
                frames = self.tf_buffer.all_frames_as_string()
                if not frames:
                    self.logger.warn(f"No TF frames available (attempt {attempt+1}/{max_retries})")
                    continue
                if self.base_frame not in frames:
                    self.logger.warn(f"Base frame '{self.base_frame}' not in TF tree (attempt {attempt+1}/{max_retries})")
                elif self.end_effector_frame not in frames:
                    self.logger.warn(f"End effector frame '{self.end_effector_frame}' not in TF tree (attempt {attempt+1}/{max_retries})")
                else:
                    self.tf_buffer.lookup_transform(
                        self.base_frame, self.end_effector_frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=2.0))
                    self.tf_available = True
                    self.logger.info(f"TF transform from {self.base_frame} to {self.end_effector_frame} available")
                    return
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.logger.warn(f"TF lookup failed: {str(e)} (attempt {attempt+1}/{max_retries})")
            except Exception as e:
                self.logger.error(f"Unexpected TF error: {str(e)}")
            time.sleep(retry_delay)
        
        self.tf_available = False
        self.logger.error(f"TF unavailable after {max_retries} retries.")
        if level_check:
            self.logger.info("Try changing the base frame in the GUI or ensuring 'robot_state_publisher' is running.")

    def get_available_frames(self):
        try:
            frames = self.tf_buffer.all_frames_as_string()
            return sorted(frames.split()) if frames else []
        except Exception as e:
            self.logger.error(f"Error getting TF frames: {str(e)}")
            return []

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
        if not self.controller_manager_available:
            self.logger.warn("Controller manager unavailable. Skipping checks.")
            return

        if not self.action_server_available and self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("MoveIt action server available")
            self.action_server_available = True
        elif not self.action_server_available:
            self.logger.warn("MoveIt action server not available")

        if not self.list_controllers_client.wait_for_service(timeout_sec=5.0):
            if self.controller_available:
                self.logger.warn("Controller manager service unavailable")
                self.controller_available = False
            return

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            controllers = future.result().controller
            controller_names = [c.name for c in controllers]
            if self.controller_name in controller_names:
                for controller in controllers:
                    if controller.name == self.controller_name and controller.state == "active":
                        if not self.controller_available:
                            self.logger.info(f"{self.controller_name} active")
                            self.controller_available = True
                        break
                else:
                    if self.controller_available:
                        self.logger.warn(f"{self.controller_name} inactive")
                        self.controller_available = False
            else:
                self.logger.warn(f"{self.controller_name} not found in controller list: {controller_names}")
                self.controller_available = False

            if "joint_state_broadcaster" in controller_names:
                for controller in controllers:
                    if controller.name == "joint_state_broadcaster" and controller.state == "active":
                        self.logger.info("joint_state_broadcaster active")
                        break
                else:
                    self.logger.warn("joint_state_broadcaster inactive")
            else:
                self.logger.warn("joint_state_broadcaster not found in controller list")
        else:
            self.logger.warn("Failed to list controllers")

        topics = self.get_topic_names_and_types()
        if '/joint_states' in [t[0] for t in topics]:
            if not self.joint_states_available:
                self.logger.info("Joint states topic available")
                self.joint_states_available = True
        else:
            self.logger.warn("Joint states topic not available")
            self.joint_states_available = False

        trajectory_topic = f'/{self.controller_name}/joint_trajectory'
        if trajectory_topic in [t[0] for t in topics]:
            if not self.trajectory_topic_available:
                self.logger.info(f"Trajectory topic {trajectory_topic} available")
                self.trajectory_topic_available = True
        else:
            self.logger.warn(f"Trajectory topic {trajectory_topic} not available")
            self.trajectory_topic_available = False

    def joint_state_callback(self, msg):
        with self.goal_lock:
            self.current_joint_state = msg
            if self.current_joint_state:
                for i, name in enumerate(self.current_joint_state.name):
                    if name in self._joint_names:
                        idx = self._joint_names.index(name)
                        if idx < len(self._current_values) and i < len(self.current_joint_state.position):
                            self._current_values[idx] = self.current_joint_state.position[i]

    def get_current_joint_values(self):
        with self.goal_lock:
            return self._current_values.copy()

    def get_joint_names(self):
        return self._joint_names.copy()

    def get_joint_limits(self):
        return self.joint_limits.copy()

    def get_current_pose(self):
        if not self.tf_available:
            return None

        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.end_effector_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                rpy = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_euler('xyz')
                return [translation.x, translation.y, translation.z, rpy[0], rpy[1], rpy[2]]
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.logger.error(f"TF Lookup Error: {str(e)}")
                if attempt == max_attempts - 1:
                    self.logger.error("Failed to get end-effector pose after retries.")
                time.sleep(0.1)
            except Exception as e:
                self.logger.error(f"Unexpected error getting pose: {str(e)}")
                break
        return None

    def move_to_joint_positions(self, joint_positions, velocity_scaling=0.3):
        if not self.controller_available or not self.trajectory_topic_available:
            self.logger.error(f"Cannot move: {self.controller_name} not active or trajectory topic unavailable.")
            return False, MoveItErrorCodes.CONTROL_FAILED.val

        max_wait = 10.0
        start_time = time.time()
        while time.time() - start_time < max_wait:
            if self.controller_available and self.trajectory_topic_available:
                break
            self.logger.warn("Waiting for controller to be ready...")
            time.sleep(0.5)

        with self.goal_lock:
            try:
                traj_msg = JointTrajectory()
                traj_msg.joint_names = self._joint_names
                point = JointTrajectoryPoint()
                point.positions = list(joint_positions)
                point.velocities = [0.0] * len(joint_positions)
                point.time_from_start = rclpy.duration.Duration(seconds=1.0 / velocity_scaling).to_msg()
                traj_msg.points = [point]
                traj_msg.header.stamp = self.get_clock().now().to_msg()

                self.logger.info(f"Sending joint trajectory: {joint_positions}")
                self.trajectory_pub.publish(traj_msg)
                time.sleep(0.5)
                return True, MoveItErrorCodes.SUCCESS.val
            except Exception as e:
                self.logger.error(f"Error sending trajectory: {str(e)}")
                return self.move_to_joint_positions_action(joint_positions, velocity_scaling)

    def move_to_joint_positions_action(self, joint_positions, velocity_scaling=0.3):
        if not self.traj_action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"Action server /{self.controller_name}/follow_joint_trajectory not available")
            return False, MoveItErrorCodes.CONTROL_FAILED.val

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = rclpy.duration.Duration(seconds=1.0 / velocity_scaling).to_msg()
        traj.points = [point]
        goal_msg.trajectory = traj

        self.logger.info(f"Sending joint trajectory action: {joint_positions}")
        future = self.traj_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None or not future.result().accepted:
            self.logger.error("Trajectory goal rejected")
            return False, MoveItErrorCodes.FAILURE.val

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code == 0:
            self.logger.info("Trajectory action completed successfully")
            return True, MoveItErrorCodes.SUCCESS.val
        self.logger.error(f"Trajectory action failed: {result.error_string}")
        return False, MoveItErrorCodes.CONTROL_FAILED.val

    def move_to_named_target(self, target_name, velocity_scaling=0.3):
        if not self.action_server_available:
            self.logger.error(f"Cannot move to named target '{target_name}': Action server not available")
            return False, MoveItErrorCodes.CONTROL_FAILED.val

        max_retries = 5
        for attempt in range(max_retries):
            try:
                goal_msg = MoveGroup.Goal()
                goal_msg.request.group_name = "arm"
                goal_msg.request.named_target = target_name
                goal_msg.request.allowed_planning_time = 10.0
                goal_msg.request.num_planning_attempts = 10
                goal_msg.request.max_velocity_scaling_factor = velocity_scaling * 0.9
                goal_msg.request.max_acceleration_scaling_factor = velocity_scaling * 0.7
                goal_msg.request.planner_id = "RRTConnect"

                self.logger.info(f"Sending named target '{target_name}' (attempt {attempt+1}/{max_retries})...")
                future = self.move_action_client.send_goal_async(goal_msg)
                with self.goal_lock:
                    self.current_goal_handle = None
                future.add_done_callback(self._store_goal_handle)

                rclpy.spin_until_future_complete(self, future)
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.logger.error("Named target goal rejected")
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result
                error_code = result.error_code.val
                if error_code == MoveItErrorCodes.SUCCESS:
                    self.logger.info(f"Successfully moved to {target_name}")
                    return True, error_code
                self.logger.error(f"Failed to move to {target_name}: {MOVEIT_ERROR_CODES.get(error_code, 'Unknown error')}")
                return False, error_code
            except Exception as e:
                self.logger.error(f"Error moving to named target: {str(e)}")
                if attempt == max_retries - 1:
                    return False, MoveItErrorCodes.FAILURE.val
                time.sleep(0.5 * (2 ** attempt))
        return False, MoveItErrorCodes.FAILURE.val

    def move_cartesian(self, delta, is_rotation, velocity_scaling=0.3):
        if not self.action_server_available:
            self.logger.error("Cannot move Cartesian: Action server not available")
            return False, MoveItErrorCodes.CONTROL_FAILED.val

        if not self.tf_available:
            self.logger.error("Cannot move Cartesian: TF unavailable")
            return False, MoveItErrorCodes.FRAME_TRANSFORM_FAILURE.val

        max_retries = 3
        for attempt in range(max_retries):
            try:
                current_pose = self.get_current_pose()
                if current_pose is None:
                    self.logger.error("Failed to get current pose for Cartesian motion")
                    return False, MoveItErrorCodes.FRAME_TRANSFORM_FAILURE.val

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = self.base_frame
                goal_pose.header.stamp = self.get_clock().now().to_msg()

                if is_rotation:
                    current_rpy = current_pose[3:6]
                    new_rpy = [current_rpy[i] + delta[i] for i in range(3)]
                    quat = R.from_euler('xyz', new_rpy).as_quat()
                    goal_pose.pose.orientation.x = quat[0]
                    goal_pose.pose.orientation.y = quat[1]
                    goal_pose.pose.orientation.z = quat[2]
                    goal_pose.pose.orientation.w = quat[3]
                    goal_pose.pose.position.x = current_pose[0]
                    goal_pose.pose.position.y = current_pose[1]
                    goal_pose.pose.position.z = current_pose[2]
                else:
                    goal_pose.pose.position.x = current_pose[0] + delta[0]
                    goal_pose.pose.position.y = current_pose[1] + delta[1]
                    goal_pose.pose.position.z = current_pose[2] + delta[2]
                    current_quat = R.from_euler('xyz', current_pose[3:6]).as_quat()
                    goal_pose.pose.orientation.x = current_quat[0]
                    goal_pose.pose.orientation.y = current_quat[1]
                    goal_pose.pose.orientation.z = current_quat[2]
                    goal_pose.pose.orientation.w = current_quat[3]

                goal_msg = MoveGroup.Goal()
                goal_msg.request.group_name = "arm"
                goal_msg.request.allowed_planning_time = 10.0
                goal_msg.request.max_velocity_scaling_factor = velocity_scaling
                goal_msg.request.max_acceleration_scaling_factor = velocity_scaling
                goal_msg.request.goal_constraints = [PositionConstraint(
                    header=goal_pose.header,
                    link_name=self.end_effector_frame,
                    target_point_offset=goal_pose.pose.position
                )]

                self.logger.info(f"Sending Cartesian goal: {delta} (rotation: {is_rotation})")
                future = self.move_action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None or not future.result().accepted:
                    self.logger.error("Cartesian goal rejected")
                    continue

                result_future = future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result
                error_code = result.error_code.val
                if error_code == MoveItErrorCodes.SUCCESS:
                    self.logger.info("Cartesian motion completed successfully")
                    return True, error_code
                self.logger.error(f"Cartesian motion failed: {MOVEIT_ERROR_CODES.get(error_code, 'Unknown error')}")
                return False, error_code
            except Exception as e:
                self.logger.error(f"Error in Cartesian motion: {str(e)}")
                if attempt == max_retries - 1:
                    return False, MoveItErrorCodes.FAILURE.val
                time.sleep(0.5)
        return False, MoveItErrorCodes.FAILURE.val

    def _store_goal_handle(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                with self.goal_lock:
                    self.current_goal_handle = goal_handle
        except Exception as e:
            self.logger.error(f"Error storing goal handle: {str(e)}")

    def cancel_current_goals(self):
        with self.goal_lock:
            if self.current_goal_handle:
                self.logger.info("Cancelling current movement goal")
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_callback)
            else:
                self.logger.info("No active goal to cancel")

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

def ros_spin(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"ROS spin error: {str(e)}")

def main(args=None):
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    commander_node = PegasusCommander()
    ros_thread = Thread(target=ros_spin, args=(commander_node,))
    ros_thread.daemon = True
    ros_thread.start()

    try:
        root = tk.Tk()
        app = PegasusArmGUI(root, commander_node)
        root.mainloop()
    except Exception as e:
        commander_node.get_logger().error(f"GUI error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        commander_node.destroy_node()
        rclpy.shutdown()
        if ros_thread.is_alive():
            ros_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()
