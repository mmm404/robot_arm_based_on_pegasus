import sys
import os

# --- ROS 1/2 COMPATIBILITY FIX ---
# Remove ROS 1 paths from sys.path to prevent conflicts (e.g. cv_bridge, sensor_msgs)
# This must be done BEFORE importing any ROS packages
sys.path = [p for p in sys.path if "/opt/ros/noetic" not in p and "/opt/ros/melodic" not in p]
# ---------------------------------

print("DEBUG: sys.path at startup:")
for p in sys.path:
    print(f"  {p}")

import yaml
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from threading import Thread, Lock, Event
from functools import partial
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import sensor_msgs
print(f"DEBUG: sensor_msgs location: {sensor_msgs.__file__}")
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R
import time  # ADD THIS LINE
from threading import Lock, Thread  # Update this line
import numpy as np
from ament_index_python.packages import get_package_share_directory
import logging
import serial
import subprocess as sp  # For xdotool commands
import cv2
import math
from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv_bridge
print(f"DEBUG: cv_bridge location: {cv_bridge.__file__}")


from PIL import Image as PILImage, ImageTk 
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose

import serial 

# PegasusCommander import removed - using ROS topics instead

# Import socket server
from pegasus_control.socket_server import SocketServer, RemoteCommand, CommandID

portname = '/tmp/virtual_arduino_sim'

# Workspace limits calculated from URDF (conservative estimates)
WORKSPACE_LIMITS = {
    'x': (0.05, 0.65),
    'y': (-0.40, 0.40),   
    'z': (0.05, 0.60),    
    'roll': (-3.14, 3.14),
    'pitch': (-1.57, 1.57),
    'yaw': (-3.14, 3.14)
}



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

def clamp_to_workspace(x, y, z, roll=None, pitch=None, yaw=None):
    """Clamp Cartesian pose to workspace boundaries.
    Returns (clamped_x, clamped_y, clamped_z, clamped_roll, clamped_pitch, clamped_yaw, was_clamped)
    """
    was_clamped = False
    
    # Clamp position
    x_clamped = np.clip(x, WORKSPACE_LIMITS['x'][0], WORKSPACE_LIMITS['x'][1])
    y_clamped = np.clip(y, WORKSPACE_LIMITS['y'][0], WORKSPACE_LIMITS['y'][1])
    z_clamped = np.clip(z, WORKSPACE_LIMITS['z'][0], WORKSPACE_LIMITS['z'][1])
    
    if x != x_clamped or y != y_clamped or z != z_clamped:
        was_clamped = True
    
    # Clamp orientation if provided
    roll_clamped = pitch_clamped = yaw_clamped = None
    if roll is not None:
        roll_clamped = np.clip(roll, WORKSPACE_LIMITS['roll'][0], WORKSPACE_LIMITS['roll'][1])
        if roll != roll_clamped:
            was_clamped = True
    if pitch is not None:
        pitch_clamped = np.clip(pitch, WORKSPACE_LIMITS['pitch'][0], WORKSPACE_LIMITS['pitch'][1])
        if pitch != pitch_clamped:
            was_clamped = True
    if yaw is not None:
        yaw_clamped = np.clip(yaw, WORKSPACE_LIMITS['yaw'][0], WORKSPACE_LIMITS['yaw'][1])
        if yaw != yaw_clamped:
            was_clamped = True
    
    return x_clamped, y_clamped, z_clamped, roll_clamped, pitch_clamped, yaw_clamped, was_clamped



class HandTrackingDisplay(ttk.LabelFrame):
    """Widget to display real-time hand tracking data in a compact horizontal layout"""
    
    def __init__(self, parent):
        super().__init__(parent, text="Hand Tracking", padding="5")
        
        # Create main horizontal container
        main_container = ttk.Frame(self)
        main_container.pack(fill=tk.X, expand=False)
        
        # Left side: Position display (horizontal layout)
        position_frame = ttk.Frame(main_container)
        position_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        ttk.Label(position_frame, text="Position:", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=(0, 5))
        
        self.x_label = ttk.Label(position_frame, text="X: --", font=("Courier", 8))
        self.x_label.pack(side=tk.LEFT, padx=3)
        
        self.y_label = ttk.Label(position_frame, text="Y: --", font=("Courier", 8))
        self.y_label.pack(side=tk.LEFT, padx=3)
        
        self.z_label = ttk.Label(position_frame, text="Z: --", font=("Courier", 8))
        self.z_label.pack(side=tk.LEFT, padx=3)
        
        # Middle: Gesture display
        gesture_frame = ttk.Frame(main_container)
        gesture_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(gesture_frame, text="Gesture:", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=(0, 5))
        
        self.gesture_var = tk.StringVar(value="None")
        self.gesture_label = ttk.Label(
            gesture_frame, 
            textvariable=self.gesture_var,
            font=("Courier", 10, "bold"),
            foreground="#00AA00"
        )
        self.gesture_label.pack(side=tk.LEFT)
        
        # Right side: Connection status
        status_frame = ttk.Frame(main_container)
        status_frame.pack(side=tk.LEFT, padx=10)
        
        self.status_var = tk.StringVar(value="Disconnected")
        status_label = ttk.Label(status_frame, textvariable=self.status_var, font=("Helvetica", 8))
        status_label.pack(side=tk.LEFT)
        
    def update_position(self, x: float, y: float, z: float):
        """Update position display"""
        self.x_label.config(text=f"X: {x:6.1f}")
        self.y_label.config(text=f"Y: {y:6.1f}")
        self.z_label.config(text=f"Z: {z:6.3f}")
        self.status_var.set("Connected")
    
    def update_gesture(self, gesture: str):
        """Update gesture display"""
        self.gesture_var.set(gesture)
        # Change color based on gesture
        color_map = {
            "Open_Palm": "#0066FF",
            "Closed_Fist": "#FF0000",
            "Thumb_Up": "#00AA00",
            "Thumb_Down": "#FFA500",
            "None": "#888888"
        }
        self.gesture_label.config(foreground=color_map.get(gesture, "#888888"))
    
    def set_disconnected(self):
        """Mark as disconnected"""
        self.status_var.set("⚫ Disconnected")
        self.x_label.config(text="X: --")
        self.y_label.config(text="Y: --")
        self.z_label.config(text="Z: --")
        self.gesture_var.set("None")





class PegasusArmGUI:
    """GUI class (NOT a ROS node to avoid blocking initialization)"""


    def __init__(self, root, node=None, wait_for_services=False):
        self.root = root
        self.node = node  # Store the ROS node
        # Use print for logging since we're not a ROS node
        
        class SimpleLogger:
            def info(self, msg):
                print(f"[GUI] {msg}")
            def error(self, msg):
                print(f"[GUI ERROR] {msg}")
            def warn(self, msg):
                print(f"[GUI WARN] {msg}")
            def debug(self, msg):
                pass  # Silent debug logs
        
        self.logger = SimpleLogger()

        # Initialize status variables
        self.timestamp_var = tk.StringVar(value="Timestamp: --:--:--")
        self.tf_status_var = tk.StringVar(value="TF Status: Checking...")

        # Joint info
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self._current_values = [0.0] * len(self.joint_names)
        
        # Default joint limits (min, max)
        self.joint_limits = {
            'joint1': (-6.28, 6.28),
            'joint2': (-0.61, 0.61),
            'joint3': (-1.75, 1.75),
            'joint4': (-1.31, 1.31),
            'joint5': (-6.28, 6.28)
        }

        # Frames for TF and planning
        self.base_frame = "base_link"
        self.end_effector_link = "tool0"
        self.end_effector_frame = "tool0"
        # Cartesian active trackers (new)
        self.cartesian_slider_active = {}  # e.g., {'X': False, 'Y': False, 'Z': False}
        self.orientation_slider_active = {}  # e.g., {'Roll': False, ...}
        # Status flags
        self.moveit_enabled = tk.BooleanVar(value=False)
        self.controller_manager_available = False
        self.tf_available = False
        self.action_server_available = False
        self.controller_available = False
        self.joint_states_available = False
        self.use_orientation = tk.BooleanVar(value=True)  # Enable RPY sliders by default
        
        # Add control mode (new - was missing)
        self.control_mode = tk.StringVar(value="cartesian")
        self.control_mode.trace('w', self.on_control_mode_changed)
        
        # Cartesian active trackers (updated: init as dicts with defaults)
        self.cartesian_slider_active = {'X': False, 'Y': False, 'Z': False}
        self.orientation_slider_active = {'Roll': False, 'Pitch': False, 'Yaw': False}
        
        # Joint active tracker (new - was missing)
        self.slider_active = [False] * len(self.joint_names)
        
        # Add these missing attributes
        self.movement_in_progress = Event()
        self.lock = Lock()
        self.goal_lock = Lock()  # Added missing lock for thread-safety
        self.last_command_time = 0
        self.command_cooldown = 0.1
        self.error_dialog_active = False
        self.controller_name = "pegasus_arm_controller"
        self.current_joint_positions = None
        self.current_joint_time = 0
        
        # Debouncing for Cartesian sliders
        self.last_cartesian_plan_time = 0
        self.cartesian_plan_cooldown = 0.1  # 100ms between plans
        self.hand_control_enabled = tk.BooleanVar(value=True)
        self.hand_control_active = False
        self.last_hand_command_time = 0
        self.hand_command_cooldown = 0.15  # Seconds between hand commands
        
        # Subscribe to hand control topics (will be created when commander initializes)
        self.hand_cartesian_sub = None
        self.hand_gesture_sub = None
        self.hand_active_sub = None

        # Initialize commander as a separate PegasusCommander instance
        try:
            print("[GUI] Initializing ExternalPegasusCommander...")
            sys.stdout.flush()
            
            # DO NOT INITIALIZE EXTERNAL COMMANDER - IT BLOCKS!
            # We'll skip it and the GUI will work without it
            self.commander = None
            # Hand tracking initialization
            self.hand_tracker = None
            self.last_hand_update = 0
            self.hand_tracking_enabled = tk.BooleanVar(value=False)  # Disabled by default
            
            # Initialize CvBridge for camera feed
            self.bridge = CvBridge()
            
            # RViz positioning vars
            # self.rviz_window_title = "Pegasus Arm RViz"  # Matches launcher env var
            self.auto_position_rviz = tk.BooleanVar(value=True)  # Toggle for auto-snap
            self.rviz_position_timer = None  # For periodic checks
            self.rviz_placeholder = None  # Ref to hide placeholder
            print("[GUI] ℹ ExternalPegasusCommander will be initialized in background...")
            sys.stdout.flush()
                
        except Exception as e:
            print(f"[GUI] ✗ Warning: Failed to initialize ExternalPegasusCommander: {e}")
            import traceback
            traceback.print_exc()
            sys.stdout.flush()
        # Initialize ROS publishers for remote control
        if self.node:
            self.joint_cmd_pub = self.node.create_publisher(JointState, '/pegasus/command/joint_target', 10)
            self.pose_cmd_pub = self.node.create_publisher(PoseStamped, '/pegasus/command/pose_target', 10)
            self.stop_cmd_pub = self.node.create_publisher(Empty, '/pegasus/command/stop', 10) # Need to import Empty
            
            # Subscribe to status
            self.execution_status_sub = self.node.create_subscription(Bool, '/pegasus/status/execution', self.execution_status_callback, 10)
            self.pose_status_sub = self.node.create_subscription(PoseStamped, '/pegasus/status/pose', self.pose_status_callback, 10)
        else:
            print("[GUI] ⚠ No ROS node provided - running in offline mode")
            self.joint_cmd_pub = None
            self.pose_cmd_pub = None
            self.stop_cmd_pub = None
        
        self.commander = None # No direct commander object
        
        # Hand tracking initialization
        self.hand_tracker = None
        self.last_hand_update = 0
        self.hand_tracking_enabled = tk.BooleanVar(value=False)  # Disabled by default

        # Named poses
        self.named_poses = {
            'home': [0.0, -0.26, 0.0, 0.0, 0.0],
            'extended': [-6.2, -0.2676, -0.1221, 0.3017, -6.2]
        }

        # Publishers (moved up for query_robot_positions dependency)
        print("[GUI] Creating joint_state publisher...")
        sys.stdout.flush()
        self.joint_state_pub = None  # Skip - not a ROS node

        # Query initial positions at startup for MoveIt starting state (moved after publishers)
        print("[GUI] Querying robot positions...")
        sys.stdout.flush()
        self.query_robot_positions()

        # Initialize socket server for remote control (do this after GUI setup to avoid blocking)
        print("[GUI] Initializing socket server placeholder...")
        sys.stdout.flush()
        self.socket_server = None
        self.remote_source = None  # Track source of remote commands

        # TF listener
        print("[GUI] Creating TF buffer and listener...")
        sys.stdout.flush()
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = None  # Skip - not a ROS node
            self.tf_available = False
        except Exception as e:
            print(f"[GUI] Warning: TF unavailable at init: {e}")
            sys.stdout.flush()

        # Check controller manager (non-blocking)
        print("[GUI] Creating controller manager client...")
        sys.stdout.flush()
        try:
            self.cm_client = None  # Skip - not a ROS node
            self.controller_manager_available = False
        except Exception:
            print("[GUI] Warning: Controller manager client creation failed")
            sys.stdout.flush()

        # GUI
        print("[GUI] Setting up GUI...")
        sys.stdout.flush()
        self.setup_gui()
        print("[GUI] GUI setup complete")
        sys.stdout.flush()
        
        print("[GUI] Starting display updates...")
        sys.stdout.flush()
        self.update_displays()
        
        # Initialize socket server after GUI is up (in background)
        print("[GUI] Scheduling socket server initialization...")
        sys.stdout.flush()
        self.init_socket_server_async()
        
        # Setup hand control subscriptions
        print("[GUI] Setting up hand control subscriptions...")
        sys.stdout.flush()
        self.setup_hand_control_subscriptions()
        
        print("[GUI] PegasusArmGUI initialization COMPLETE")
        sys.stdout.flush()




    def setup_hand_control_subscriptions(self):
        """Setup ROS2 subscriptions for hand control data from cv_pegasus_bridge."""
        if not self.node:
            self.log_action("⚠ No ROS node available for hand control setup")
            return
        
        try:
            # Subscribe to Cartesian commands from bridge
            self.hand_cartesian_sub = self.node.create_subscription(
                PoseStamped,
                '/hand_control/cartesian_command',
                self.hand_cartesian_callback,
                10
            )
            
            # Gesture subscription REMOVED
            self.hand_gesture_sub = None
            
            # Subscribe to tracking active status
            self.hand_active_sub = self.node.create_subscription(
                Bool,
                '/hand_control/control_active',
                self.hand_active_callback,
                10
            )
            
            self.log_action("✓ Hand control subscriptions created")
            
        except Exception as e:
            self.log_action(f"✗ Hand control setup error: {e}")
            # Retry on error too, just in case
            self.root.after(5000, self.setup_hand_control_subscriptions)



    def execution_status_callback(self, msg):
        """Update execution status from headless commander"""
        is_executing = msg.data
        # Update GUI state if needed
        # For example, disable buttons if executing
        pass

    def pose_status_callback(self, msg):
        """Update current pose from headless commander"""
        # Update internal state
        # msg is PoseStamped
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Convert quat to RPY
        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        
        # Update displays if not dragging sliders
        # This logic was partly in update_displays, but we can do it here too or cache it
        self.current_pose_cache = [x, y, z, roll, pitch, yaw]
        
        # Update TF status to "OK" since we are getting data
        self.tf_status_var.set("TF Status: OK (Remote)")

    def hand_cartesian_callback(self, msg):
        """Handle incoming Cartesian commands from hand tracking."""
        # Check if hand control is enabled in GUI
        if not self.hand_control_enabled.get():
            return
        
        # Check if in Cartesian mode
        if self.control_mode.get() != "cartesian":
            # print(f"[DEBUG] Hand control ignored: Mode is {self.control_mode.get()}")
            return
        
        # Rate limiting
        current_time = time.time()
        if current_time - self.last_hand_command_time < self.hand_command_cooldown:
            return
        
        self.last_hand_command_time = current_time
        
        try:
            # Extract position
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Extract orientation
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            r = R.from_quat(q)
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)
            
            # Update GUI sliders (in main thread)
            def update_sliders():
                if hasattr(self, 'xyz_vars'):
                    self.xyz_vars['X'].set(x)
                    self.xyz_vars['Y'].set(y)
                    self.xyz_vars['Z'].set(z)
                    
                    # Update displays
                    self.xyz_values_display['X'].config(text=f"{x:.3f}")
                    self.xyz_values_display['Y'].config(text=f"{y:.3f}")
                    self.xyz_values_display['Z'].config(text=f"{z:.3f}")
                
                if hasattr(self, 'rpy_vars') and self.use_orientation.get():
                    self.rpy_vars['Roll'].set(roll)
                    self.rpy_vars['Pitch'].set(pitch)
                    self.rpy_vars['Yaw'].set(yaw)
                    
                    # Update displays
                    self.rpy_values_display['Roll'].config(text=f"{roll:.3f}")
                    self.rpy_values_display['Pitch'].config(text=f"{pitch:.3f}")
                    self.rpy_values_display['Yaw'].config(text=f"{yaw:.3f}")
            
            self.root.after(0, update_sliders)

            # Force active if we are receiving data
            self.hand_control_active = True

            # Auto-plan if tracking is active and not executing (moved from gesture callback)
            # We use the same rate limiting as above (self.last_hand_command_time)
            is_executing = False
            if self.commander:
                is_executing = getattr(self.commander, 'is_executing', False)
                
            if self.hand_control_active and not is_executing:
                 # self.log_action(f" Auto-planning from cartesian update")
                 print("[DEBUG] Triggering plan_current_mode from hand callback")
                 self.root.after(0, self.plan_current_mode)
            else:
                 print(f"[DEBUG] Auto-plan skipped: Active={self.hand_control_active}, Executing={is_executing}")
            
        except Exception as e:
            self.logger.debug(f"Hand cartesian callback error: {e}")

    def setup_hand_control_toggle(self):
        """Add hand control enable/disable toggle to GUI."""
        if not hasattr(self, 'control_frame'):
            return
        
        # Create frame for hand control toggle
        hand_control_frame = ttk.LabelFrame(
            self.control_frame, 
            text="Hand Control", 
            padding="5"
        )
        hand_control_frame.pack(fill=tk.X, pady=(5,5))
        
        # Enable/disable checkbox
        hand_toggle = ttk.Checkbutton(
            hand_control_frame,
            text="Enable Hand Tracking Control",
            variable=self.hand_control_enabled,
            command=self.on_hand_control_toggle
        )
        hand_toggle.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.hand_status_label = ttk.Label(
            hand_control_frame,
            text="Status: Enabled" if self.hand_control_enabled.get() else "Status: Disabled",
            font=("Helvetica", 9)
        )
        self.hand_status_label.pack(side=tk.LEFT, padx=10)



    def on_hand_control_toggle(self):
        """Handle hand control enable/disable toggle."""
        if self.hand_control_enabled.get():
            self.log_action(" Hand control ENABLED")
            self.hand_status_label.config(text="Status: Enabled")
            
            # Switch to Cartesian mode if not already
            if self.control_mode.get() != "cartesian":
                self.control_mode.set("cartesian")
                self.on_control_mode_changed()
        else:
            self.log_action("⏸ Hand control DISABLED")
            self.hand_status_label.config(text="Status: Disabled")
            self.hand_control_active = False




    def hand_active_callback(self, msg):
        """Update hand control active status."""
        self.hand_control_active = msg.data



    def position_rviz_window(self):
        """Enhanced RViz window positioning with action log feedback"""
        try:
            if self.robot_view_frame.winfo_width() <= 0 or self.robot_view_frame.winfo_height() <= 0:
                self.log_action(" RViz frame not ready, retrying...")
                return False

            frame_x = self.robot_view_frame.winfo_rootx()
            frame_y = self.robot_view_frame.winfo_rooty()
            frame_width = self.robot_view_frame.winfo_width()
            frame_height = self.robot_view_frame.winfo_height()
            
            # Find RViz window
            find_cmd = ["xdotool", "search", "--class", "rviz"]
            result = sp.run(find_cmd, capture_output=True, text=True)
            win_ids = result.stdout.strip().split('\n')
            
            if not win_ids or win_ids == ['']:
                # Only log every 5 attempts to reduce spam
                if not hasattr(self, '_rviz_search_count'):
                    self._rviz_search_count = 0
                self._rviz_search_count += 1
                if self._rviz_search_count % 5 == 0:
                    self.log_action(f" Searching for RViz window... (attempt {self._rviz_search_count})")
                return False

            # Find main window (largest)
            main_window = None
            max_size = 0
            
            for win_id in win_ids:
                try:
                    geom_result = sp.run(
                        ["xdotool", "getwindowgeometry", win_id],
                        capture_output=True, text=True
                    )
                    
                    if "Geometry:" in geom_result.stdout:
                        geom_line = [l for l in geom_result.stdout.split('\n') if 'Geometry:' in l][0]
                        size_str = geom_line.split('Geometry:')[1].strip()
                        w, h = map(int, size_str.split('x'))
                        area = w * h
                        
                        if area > max_size:
                            max_size = area
                            main_window = win_id
                except Exception:
                    continue
            
            if not main_window:
                main_window = win_ids[0]

            # Get Tkinter frame window ID
            frame_wid = self.robot_view_frame.winfo_id()
            frame_x11_id = hex(frame_wid)

            # Reparent RViz into frame
            try:
                sp.run(["xdotool", "windowunmap", main_window], timeout=1)
                sp.run(["wmctrl", "-i", "-r", main_window, "-b", "remove,maximized_vert,maximized_horz"], timeout=1)
                sp.run(["xdotool", "windowreparent", main_window, frame_x11_id], timeout=2)
                sp.run(["xdotool", "windowmap", main_window], timeout=1)
            except Exception as e:
                self.log_action(f" Reparenting failed: {e}")

            # Position and resize
            sp.run(["xdotool", "windowmove", main_window, "0", "0"])
            sp.run(["xdotool", "windowsize", main_window, str(frame_width), str(frame_height)])

            try:
                sp.run(["wmctrl", "-i", "-r", main_window, "-b", "add,above"], timeout=1)
            except:
                pass

            sp.run(["xdotool", "windowactivate", main_window])

            if self.rviz_placeholder:
                self.rviz_placeholder.pack_forget()

            self.log_action(f" RViz positioned ({frame_width}x{frame_height})")
            
            # Store window ID and stop auto-positioning
            self.positioned_rviz_window_id = main_window
            
            if hasattr(self, 'rviz_position_timer') and self.rviz_position_timer:
                self.root.after_cancel(self.rviz_position_timer)
                self.rviz_position_timer = None
                self.log_action("ℹ RViz auto-positioning stopped")
            
            return True
            
        except FileNotFoundError:
            self.log_action(" xdotool not found - install with 'sudo apt install xdotool'")
            return False
        except Exception as e:
            self.log_action(f" RViz positioning failed: {e}")
            return False








    def start_rviz_positioning(self):
        """Poll for RViz and position it (initial setup)."""
        def poll_loop():
            # Skip if already positioned
            if hasattr(self, 'positioned_rviz_window_id'):
                self.log_action("RViz already positioned, skipping poll")
                return
            
            success = self.position_rviz_window()
            
            if success:
                self.log_action(" RViz positioned successfully on first attempt")
                # Start periodic checks ONLY if auto-position is enabled
                if self.auto_position_rviz.get():
                    self.periodic_rviz_reposition()
            else:
                # Retry every 2s if not found
                self.log_action(" RViz not found, retrying in 2s...")
                self.root.after(2000, poll_loop)

        # Initial poll after 1s delay (for GUI render)
        self.root.after(1000, poll_loop)




    def manual_position_rviz(self):
        """Manual positioning with reset option"""
        # Clear any previous positioning record
        if hasattr(self, 'positioned_rviz_window_id'):
            delattr(self, 'positioned_rviz_window_id')
        
        # Force immediate positioning
        success = self.position_rviz_window()
        
        if not success:
            self.log_action(" Manual positioning failed - ensure RViz is running")
            


    def periodic_rviz_reposition(self):
        """Auto-snap RViz every 5s if enabled - BUT stop if already positioned."""
        if not self.auto_position_rviz.get():
            return  # User disabled it
        
        # Check if we already successfully positioned it
        if hasattr(self, 'positioned_rviz_window_id'):
            # Verify the window still exists
            try:
                result = sp.run(
                    ["xdotool", "getwindowname", self.positioned_rviz_window_id],
                    capture_output=True, text=True, timeout=1
                )
                if result.returncode == 0:
                    # Window still exists, no need to reposition
                    return
                else:
                    # Window was closed, reset and try again
                    delattr(self, 'positioned_rviz_window_id')
            except Exception:
                # Error checking, assume window is gone
                delattr(self, 'positioned_rviz_window_id')
        
        # Try to position
        success = self.position_rviz_window()
        
        # Only schedule next check if positioning failed
        if not success and self.auto_position_rviz.get():
            self.rviz_position_timer = self.root.after(5000, self.periodic_rviz_reposition)





    def auto_position_rviz_handler(self):
        """Toggle auto-reposition."""
        if self.auto_position_rviz.get():
            if not self.rviz_position_timer:
                self.periodic_rviz_reposition()
            self.log_action(" RViz auto-reposition: Enabled")
        else:
            if self.rviz_position_timer:
                self.root.after_cancel(self.rviz_position_timer)
                self.rviz_position_timer = None
            self.log_action(" RViz auto-reposition: Disabled")






    def update_hand_tracking_display(self):
        """Update hand tracking display widget periodically. Log status changes."""
        try:
            if not hasattr(self, 'hand_display') or self.hand_display is None:
                self.root.after(50, self.update_hand_tracking_display)
                return

            # Fix: Reference hand_display's vars, not self's
            prev_gesture = self.hand_display.gesture_var.get()
            prev_connected = self.hand_display.status_var.get() == "🟢 Connected"

            if not self.hand_tracker:
                if prev_connected:
                    self.log_action(" Hand Tracking: Disconnected")
                self.hand_display.set_disconnected()
                self.root.after(50, self.update_hand_tracking_display)
                return

            # Get current pose and gesture (unchanged)
            pose = self.hand_tracker.get_current_pose()
            gesture = self.hand_tracker.get_current_gesture()

            # Update display (unchanged)
            if pose:
                self.hand_display.update_position(pose.x, pose.y, pose.z)
                self.last_hand_update = time.time()
            else:
                if time.time() - self.last_hand_update > 2.0:
                    self.log_action("Hand Tracking: Stale/Disconnected")
                    self.hand_display.set_disconnected()

            if gesture and gesture.gesture != prev_gesture:
                self.log_action(f"Hand Gesture: {gesture.gesture} at ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f})" if pose else f"👋 Hand Gesture: {gesture.gesture}")
                self.hand_display.update_gesture(gesture.gesture)

            # Real-time Cartesian update (unchanged)
            if self.hand_tracking_enabled.get() and pose and self.control_mode.get() == "cartesian":
                self._update_cartesian_from_hand(pose)

            if not prev_connected and self.hand_display.status_var.get() == "🟢 Connected":
                self.log_action("Hand Tracking: Connected")

        except Exception as e:
            self.logger.error(f"Error updating hand tracking display: {e}")
            if not hasattr(self, '_hand_error_logged'):
                self.log_action(f" Hand Tracking ERROR: {str(e)}")
                self._hand_error_logged = True
                self.root.after(5000, lambda: setattr(self, '_hand_error_logged', False))

        self.root.after(50, self.update_hand_tracking_display)




    def _update_cartesian_from_hand(self, pose):
        """Update Cartesian slider positions from hand tracking (optional real-time mode)"""
        try:
            # Check if any Cartesian slider is being dragged
            if any(self.cartesian_slider_active):
                return
            
            # Get normalized pose
            pose_norm = self.hand_tracker.get_normalized_pose()
            if pose_norm is None:
                return
            
            x_norm, y_norm, z_norm = pose_norm
            
            # Map to robot workspace
            x = 0.05 + x_norm * 0.60
            y = -0.40 + (1.0 - y_norm) * 0.80
            z = 0.05 + z_norm * 0.55
            
            # Update sliders (but don't trigger planning)
            if hasattr(self, 'xyz_vars'):
                self.xyz_vars['X'].set(x)
                self.xyz_vars['Y'].set(y)
                self.xyz_vars['Z'].set(z)
                self.xyz_values_display['X'].config(text=f"{x:.3f}")
                self.xyz_values_display['Y'].config(text=f"{y:.3f}")
                self.xyz_values_display['Z'].config(text=f"{z:.3f}")
        
        except Exception as e:
            self.logger.debug(f"Error updating Cartesian from hand: {e}")





    # Gesture methods REMOVED






    def setup_settings_tab(self, parent_frame):
        """Placeholder for settings tab setup."""
        settings_frame = ttk.Frame(parent_frame, padding="10")
        settings_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(settings_frame, text="Settings tab placeholder.").pack(pady=20)






    def init_socket_server_async(self):
        """Initialize socket server in background thread"""
        def setup_server():
            try:
                self.log_action(" Starting socket server...")
                self.socket_server = SocketServer(host='0.0.0.0', port=5000)
                
                from socket_server import HandTracker
                self.hand_tracker = HandTracker()
                self.socket_server.hand_tracker = self.hand_tracker
                
                self._setup_socket_handlers()
                self._setup_gesture_controls()
                
                self.socket_server.start()
                self.log_action(" Socket server started on port 5000")
                
                # Setup hand control subscriptions after commander is ready
                self.root.after(2000, self.setup_hand_control_subscriptions)
                
            except Exception as e:
                self.log_action(f" Socket server failed: {e}")
        
        threading.Thread(target=setup_server, daemon=True).start()






    def _setup_gesture_controls(self):
            """Map hand gestures to robot actions - REMOVED"""
            pass

    # _gesture_move_to_position REMOVED




    def _setup_socket_handlers(self):
        """Setup socket server handlers for remote commands"""
        if not self.socket_server:
            return
        
        try:
            # Register remote command handlers
            self.socket_server.register_command_handler(
                CommandID.MOVE_JOINTS, self._handle_remote_move_joints)
            self.socket_server.register_command_handler(
                CommandID.MOVE_HOME, self._handle_remote_move_home)
            self.socket_server.register_command_handler(
                CommandID.MOVE_EXTENDED, self._handle_remote_move_extended)
            self.socket_server.register_command_handler(
                CommandID.MOVE_CARTESIAN, self._handle_remote_move_cartesian)
            self.socket_server.register_command_handler(
                CommandID.EMERGENCY_STOP, self._handle_remote_emergency_stop)
            self.socket_server.register_command_handler(
                CommandID.SET_VELOCITY, self._handle_remote_set_velocity)
            self.socket_server.register_command_handler(
                CommandID.CANCEL_GOAL, self._handle_remote_cancel_goal)
            
            # Register callbacks
            self.socket_server.on_client_connected = self._on_remote_client_connected
            self.socket_server.on_client_disconnected = self._on_remote_client_disconnected
            self.socket_server.get_robot_state = self._get_robot_state
            
        except Exception as e:
            print(f"[GUI] Error setting up socket handlers: {e}")






    def _handle_remote_move_joints(self, command: RemoteCommand) -> dict:
        """Handle remote move joints command. Enhanced socket logging."""
        try:
            if not command.joint_values:
                raise ValueError("No joint values provided")

            self.remote_source = "socket"
            incoming_data = f"Joints: {[f'{v:.3f}' for v in command.joint_values]}, Velocity: {command.velocity or 0.3}"
            self.log_action(f" Socket RX: Move Joints - {incoming_data}")

            velocity = command.velocity or 0.3
            success, error_code = self.move_to_joint_positions(command.joint_values, velocity)

            # Update GUI sliders if successful
            if success:
                self.root.after(0, lambda: self._update_sliders_from_joints(command.joint_values))
                self.log_action(f" Socket TX: Joint move SUCCEEDED (code: {error_code})")

            else:
                self.log_action(f" Socket TX: Joint move FAILED (code: {error_code})")

            return {'success': success, 'error_code': error_code}
        except Exception as e:
            self.logger.error(f"Remote move joints error: {e}")
            self.log_action(f" Socket ERROR: {str(e)}")
            return {'success': False, 'error': str(e)}





    def _handle_remote_move_home(self, command: RemoteCommand) -> dict:
        """Handle remote move home command"""
        try:
            self.remote_source = "socket"
            self.log_action(" Remote command: Moving to Home position")
            success, error_code = self.move_to_named_target("home", 0.3)
            
            if success:
                home_pos = self.named_poses['home']
                self.root.after(0, lambda: self._update_sliders_from_joints(home_pos))
            
            return {'success': success, 'error_code': error_code}
        except Exception as e:
            self.logger.error(f"Remote move home error: {e}")
            return {'success': False, 'error': str(e)}

    def _handle_remote_move_extended(self, command: RemoteCommand) -> dict:
        """Handle remote move extended command"""
        try:
            self.remote_source = "socket"
            self.log_action(" Remote command: Moving to Extended position")
            success, error_code = self.move_to_named_target("extended", 0.3)
            
            if success:
                extended_pos = self.named_poses['extended']
                self.root.after(0, lambda: self._update_sliders_from_joints(extended_pos))
            
            return {'success': success, 'error_code': error_code}
        except Exception as e:
            self.logger.error(f"Remote move extended error: {e}")
            return {'success': False, 'error': str(e)}

    def _handle_remote_move_cartesian(self, command: RemoteCommand) -> dict:
        """Handle remote move to cartesian pose command"""
        try:
            if not command.cartesian_pose:
                raise ValueError("No cartesian pose provided")
            
            velocity = command.velocity or 0.3
            self.remote_source = "socket"
            
            # Extract position and orientation
            x, y, z = command.cartesian_pose[0:3]
            roll, pitch, yaw = command.cartesian_pose[3:6] if len(command.cartesian_pose) >= 6 else (0.0, 0.0, 0.0)
            
            # Execute move using the correct method
            success, error_code = self.commander.plan_to_cartesian_pose(x, y, z, roll, pitch, yaw)
            
            if success:
                self.log_action(f" Remote command: Moving to cartesian pose {[f'{v:.3f}' for v in command.cartesian_pose]}")
            
            return {
                'success': success,
                'error_code': error_code if isinstance(error_code, int) else 0
            }
        except Exception as e:
            self.logger.error(f"Remote move cartesian error: {e}")
            return {'success': False, 'error': str(e)}

    def _handle_remote_emergency_stop(self, command: RemoteCommand) -> dict:
        """Handle remote emergency stop command"""
        try:
            self.remote_source = "socket"
            self.log_action(" Remote command: EMERGENCY STOP")
            self.emergency_stop()
            return {'success': True}
        except Exception as e:
            self.logger.error(f"Remote emergency stop error: {e}")
            return {'success': False, 'error': str(e)}

    def _handle_remote_set_velocity(self, command: RemoteCommand) -> dict:
        """Handle remote set velocity command"""
        try:
            if command.velocity is None:
                raise ValueError("No velocity value provided")
            
            # Clamp velocity between 0.1 and 1.0
            velocity = max(0.1, min(1.0, command.velocity))
            self.velocity_scale.set(velocity)
            
            self.log_action(f" Remote command: Velocity set to {velocity:.2f}")
            return {'success': True, 'velocity': velocity}
        except Exception as e:
            self.logger.error(f"Remote set velocity error: {e}")
            return {'success': False, 'error': str(e)}

    def _handle_remote_cancel_goal(self, command: RemoteCommand) -> dict:
        """Handle remote cancel goal command"""
        try:
            self.remote_source = "socket"
            self.log_action(" Remote command: Cancelling active goal")
            self.cancel_current_goals()
            return {'success': True}
        except Exception as e:
            self.logger.error(f"Remote cancel goal error: {e}")
            return {'success': False, 'error': str(e)}

    def _get_robot_state(self) -> dict:
        """Get current robot state for remote clients"""
        try:
            joint_values = self.get_current_joint_values()
            pose = self.get_current_pose()
            
            return {
                'joint_values': joint_values,
                'pose': pose,
                'joint_names': self.joint_names,
                'connected': True
            }
        except Exception as e:
            self.logger.error(f"Get robot state error: {e}")
            return {'connected': False, 'error': str(e)}

    def _on_remote_client_connected(self, client_id: int, address: tuple):
        """Callback when remote client connects"""
        self.log_action(f"Remote client {client_id} connected from {address}")

    def _on_remote_client_disconnected(self, client_id: int, address: tuple):
        """Callback when remote client disconnects"""
        self.log_action(f"Remote client {client_id} disconnected")

    def _update_sliders_from_joints(self, joint_values: list):
        """Update GUI sliders from joint values"""
        try:
            if not hasattr(self, 'joint_vars'):
                return
            
            for i, (var, value) in enumerate(zip(self.joint_vars, joint_values)):
                if i < len(joint_values):
                    var.set(value)
                    if i < len(self.joint_values):
                        self.joint_values[i].config(text=f"{value:.3f}")
        except Exception as e:
            self.logger.error(f"Error updating sliders: {e}")

    def setup_camera_tab(self, parent_frame):
        """Placeholder for camera tab setup."""
        camera_frame = ttk.Frame(parent_frame, padding="10")
        camera_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(camera_frame, text="Camera tab placeholder.").pack(pady=20)

    def update_tf_status(self):
        """Update TF status display."""
        if self.tf_available:
            self.tf_status_var.set(f"TF Status: Connected (Base: {self.base_frame} → {self.end_effector_frame})")
        else:
            self.tf_status_var.set(f"TF Status: Disconnected")
        self.root.after(5000, self.update_tf_status)

    def refresh_tf_status(self):
        """Refresh TF status and frames."""
        new_base_frame = self.base_frame_var.get()
        if new_base_frame != self.base_frame:
            self.base_frame = new_base_frame
        self.check_tf_availability()
        self.update_tf_status()





    def on_closing(self):
        """Handle GUI closing with action log feedback"""
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.log_action(" Shutting down Pegasus Arm Controller...")
            
            # Stop movements
            self.cancel_current_goals()
            self.log_action("Movement goals cancelled")
            
            # Stop camera
            if hasattr(self, 'camera_active') and self.camera_active:
                self.stop_camera_feed()
                self.log_action(" Camera stopped")
            
            # Stop socket server
            if hasattr(self, 'socket_server') and self.socket_server:
                self.socket_server.stop()
                self.log_action(" Socket server stopped")
            
            # Cancel RViz timer
            if hasattr(self, 'rviz_position_timer') and self.rviz_position_timer:
                self.root.after_cancel(self.rviz_position_timer)
            
            # Minimize RViz
            try:
                sp.run(["xdotool", "search", "--class", "rviz", "windowminimize"], timeout=1)
            except:
                pass
            
            self.log_action(" Goodbye!")
            self.root.destroy()









    def query_robot_positions(self):
        """Initialize joint states with home position for MoveIt."""
        try:
            # Use home position as starting point
            current_pos = self.named_poses['home']
            
            # Update internal state (thread-safe)
            self.set_current_joint_values(current_pos)
            
            # Skip publishing - GUI is not a ROS node
            return True
            
        except Exception as e:
            print(f"Warning: Error initializing positions: {str(e)}")
            return False



    def update_displays(self):
        """Update all GUI displays with current values. Log key status changes."""
        try:
            # Only update displays if they've been initialized
            if hasattr(self, 'joint_values') and self.joint_values:
                # Update joint value displays
                joint_values = self.get_current_joint_values()
                for i, value in enumerate(joint_values):
                    if i < len(self.joint_values):
                        self.joint_values[i].config(text=f"{value:.3f}")

            # Update timestamp
            current_time = time.strftime("%H:%M:%S")
            self.timestamp_var.set(f"Timestamp: {current_time}")

            # Update TF status (log changes)
            prev_tf = getattr(self, '_last_tf_logged', False)
            self.update_tf_status()
            if self.tf_available != prev_tf:
                self.log_action(f"TF Status Updated: {'Available' if self.tf_available else 'Unavailable'}")
                self._last_tf_logged = self.tf_available

            # Start hand tracking display updates if not already started
            if not hasattr(self, '_hand_tracking_started'):
                self._hand_tracking_started = True
                self.root.after(50, self.update_hand_tracking_display)

            # Schedule next update
            self.root.after(100, self.update_displays)  # Update every 100ms

        except Exception as e:
            self.logger.error(f"Error updating displays: {str(e)}")
            # Still schedule next update even if there's an error
            self.root.after(100, self.update_displays)

            

    def _wait_for_moveit_services(self):
        """Placeholder for waiting for MoveIt2 services."""
        self.logger.info("Simulated wait for MoveIt2 services (stub).")

          

    def setup_gui(self):
        """Main window layout - simplified without multiple terminal logging"""
        
        def on_gui_configure(event):
            """Called when GUI window moves or resizes"""
            if hasattr(self, 'positioned_rviz_window_id'):
                self.root.after(50, self.position_rviz_window)

        self.root.bind('<Configure>', on_gui_configure)

        self.root.geometry("900x800")
        self.root.resizable(True, True)
        self.root.title("Pegasus Arm Controller")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- MAIN WRAPPER FRAME ---
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- STATUS BAR (TOP) ---
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=2)

        self.tf_status_var = tk.StringVar(value="TF Status: Checking...")
        status_style = "Success.TLabel" if self.tf_available else "Alert.TLabel"
        self.tf_status_label = ttk.Label(status_frame, textvariable=self.tf_status_var, style=status_style)
        self.tf_status_label.pack(fill=tk.X, pady=2)

        # --- MAIN PANELS (LEFT / CENTER / RIGHT) ---
        panels = ttk.Frame(main_frame)
        panels.pack(fill=tk.BOTH, expand=True)

        # LEFT PANEL (controls)
        self.left_panel = ttk.Frame(panels)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(5,5), pady=5)

        # CENTER PANEL (robot view)
        self.center_panel = ttk.Frame(panels)
        self.center_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # RIGHT PANEL (action log)
        self.right_panel = ttk.Frame(panels)
        self.right_panel.config(width=260)
        self.right_panel.pack_propagate(False)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # --- NOTEBOOK (left side) ---
        self.notebook = ttk.Notebook(self.left_panel)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.control_tab = ttk.Frame(self.notebook)
        self.settings_tab = ttk.Frame(self.notebook)
        self.camera_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.control_tab, text="Joint Control")
        self.notebook.add(self.settings_tab, text="Settings")
        self.notebook.add(self.camera_tab, text="Camera")

        self.control_frame = ttk.Frame(self.control_tab)
        self.control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # --- ROBOT VIEW (center) ---
        self.robot_view_frame = ttk.LabelFrame(self.center_panel, text="Robot View", padding=5)
        self.robot_view_frame.pack(fill=tk.BOTH, expand=True)

        rviz_placeholder = ttk.Label(self.robot_view_frame, text="Robot/3D View Placeholder", anchor="center")
        rviz_placeholder.pack(fill=tk.BOTH, expand=True)
        self.rviz_placeholder = rviz_placeholder

        # RViz Controls
        rviz_controls = ttk.Frame(self.center_panel)
        rviz_controls.pack(fill=tk.X, pady=5)

        ttk.Button(rviz_controls, text="Position RViz Now", 
        command=self.manual_position_rviz).pack(side=tk.LEFT, padx=5)

        ttk.Checkbutton(rviz_controls, text="Auto-reposition RViz", variable=self.auto_position_rviz,
                        command=self.auto_position_rviz_handler).pack(side=tk.LEFT, padx=5)

        # --- ACTION LOG (RIGHT) - Clear on startup ---
        log_frame = ttk.LabelFrame(self.right_panel, text="Action Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, height=20, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(self.log_text, orient="vertical", command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scrollbar.set)

        # Clear log and add startup message
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log_action(" Pegasus Arm Controller started")
        self.log_action(" Initializing systems...")

        # --- SETTINGS & CAMERA TABS ---
        self.setup_settings_tab(self.settings_tab)
        self.setup_camera_tab(self.camera_tab)

        # --- CONTROLS ---
        self.setup_control_buttons()

        # Start hand tracking updates
        if hasattr(self, 'hand_display'):
            self.root.after(50, self.update_hand_tracking_display)

        # Start RViz positioning
        self.root.after(1000, self.start_rviz_positioning)
        
        self.log_action(" GUI initialization complete")








    def on_control_mode_changed(self, *args):
        """Rebuilds sliders and manages layout for both modes. Logs mode change."""
        mode = self.control_mode.get()
        self.log_action(f" Control mode switched to: {mode.upper()}")

        # Safe-create frames if missing (redundant now, but safe)
        if not hasattr(self, 'joint_frame'):
            self.joint_frame = ttk.LabelFrame(self.control_frame, text="Joint Positions (rad)", padding="5")
        if not hasattr(self, 'cartesian_input_frame'):
            self.cartesian_input_frame = ttk.LabelFrame(self.control_frame, text="Cartesian Pose", padding="5")

        # Clear old widgets
        for frame in [self.joint_frame, self.cartesian_input_frame]:
            for widget in frame.winfo_children():
                widget.destroy()
        self.joint_labels = []
        self.joint_values = []
        self.joint_sliders = []
        self.joint_vars = []
        self.slider_active = [False] * len(self.get_joint_names() if hasattr(self, 'get_joint_names') else self.joint_names)  # Reset, safe fallback
        self.xyz_vars = {}
        self.rpy_vars = {}
        self.cartesian_slider_active = {'X': False, 'Y': False, 'Z': False}
        self.orientation_slider_active = {'Roll': False, 'Pitch': False, 'Yaw': False}

        # Get current values (safe fallbacks)
        joint_values = self.get_current_joint_values() if hasattr(self, 'get_current_joint_values') else [0.0] * len(self.joint_names)
        current_pose = self.get_current_pose() if hasattr(self, 'get_current_pose') else [0.3, 0.0, 0.3, 0.0, 0.0, 0.0]
        if current_pose is None:
            current_pose = [0.3, 0.0, 0.3, 0.0, 0.0, 0.0]  # Explicit fallback for None

        # JOINT MODE
        if mode == "joint":
            self.log_action(" Displaying Joint Space sliders")
            self.setup_joint_sliders(joint_values)
            self.preset_frame.pack(fill=tk.X, pady=(5,10))
            self.joint_frame.pack(fill=tk.X)
            self.cartesian_input_frame.pack_forget()
            if hasattr(self, 'plan_button'):
                self.plan_button.pack(fill=tk.X, pady=10)

        # CARTESIAN MODE
        else:
            self.log_action(" Displaying Cartesian Space sliders")
            self.setup_cartesian_sliders(current_pose)
            self.preset_frame.pack(fill=tk.X, pady=(5,10))
            self.joint_frame.pack_forget()
            self.cartesian_input_frame.pack(fill=tk.BOTH, expand=True, pady=(5,10))
            if hasattr(self, 'plan_button'):
                self.plan_button.pack(fill=tk.X, pady=10)

        # Update TF/controller status in log if changed
        if self.tf_available != getattr(self, '_last_tf_status', False):
            tf_msg = "🟢" if self.tf_available else "🔴"
            self.log_action(f"{tf_msg} TF Status: {'Available' if self.tf_available else 'Unavailable'}")
            self._last_tf_status = self.tf_available
        if self.controller_available != getattr(self, '_last_controller_status', False):
            ctrl_msg = "🟢" if self.controller_available else "🔴"
            self.log_action(f"{ctrl_msg} Controller Status: {'Active' if self.controller_available else 'Inactive'}")
            self._last_controller_status = self.controller_available

        if hasattr(self, '_update_scroll_region'):
            self.root.after(100, self._update_scroll_region)





    def setup_cartesian_sliders(self, current_pose):
        """Setup Cartesian sliders (X/Y/Z + optional RPY). Initializes from current pose."""
        # Position frame (X/Y/Z)
        pos_frame = ttk.LabelFrame(self.cartesian_input_frame, text="Position (m)", padding="5")
        pos_frame.pack(fill=tk.X, pady=5)

        self.xyz_vars = {}
        self.xyz_values_display = {}
        xyz_labels = ['X', 'Y', 'Z']
        xyz_limits = {
            'X': WORKSPACE_LIMITS['x'],
            'Y': WORKSPACE_LIMITS['y'],
            'Z': WORKSPACE_LIMITS['z']
        }

        for i, label in enumerate(xyz_labels):
            row = ttk.Frame(pos_frame)
            row.pack(fill=tk.X, pady=2)

            ttk.Label(row, text=f"{label}:", width=8).pack(side=tk.LEFT)
            value_label = ttk.Label(row, text=f"{current_pose[i]:.3f}", width=8)
            value_label.pack(side=tk.LEFT, padx=5)
            self.xyz_values_display[label] = value_label

            var = tk.DoubleVar(value=current_pose[i])
            self.xyz_vars[label] = var

            min_lim, max_lim = xyz_limits[label]
            slider = ttk.Scale(
                row, from_=min_lim, to=max_lim, orient=tk.HORIZONTAL,
                variable=var, command=lambda v, lbl=label: self.handle_cartesian_slider_continuous(lbl)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            # Track active sliders (update handler for drag state)
            slider.bind("<ButtonPress-1>", lambda e, lbl=label: self.cartesian_slider_pressed(lbl))
            slider.bind("<ButtonRelease-1>", lambda e, lbl=label: self.cartesian_slider_released(lbl))

        # Add orientation toggle checkbox (new: place in pos_frame for visibility)
        orient_check_frame = ttk.Frame(pos_frame)
        orient_check_frame.pack(fill=tk.X, pady=5)
        ttk.Checkbutton(
            orient_check_frame,
            text="Use Orientation (RPY)",
            variable=self.use_orientation,
            command=self.toggle_orientation_inputs
        ).pack(side=tk.LEFT)

        # Orientation frame (Roll/Pitch/Yaw) - conditional
        self.orientation_frame = ttk.LabelFrame(self.cartesian_input_frame, text="Orientation (rad)", padding="5")
        
        # Safe fallback if use_orientation missing (shouldn't happen after init)
        if not hasattr(self, 'use_orientation'):
            self.use_orientation = tk.BooleanVar(value=True)
        
        if self.use_orientation.get():
            self.orientation_frame.pack(fill=tk.X, pady=5)

            self.rpy_vars = {}
            self.rpy_values_display = {}
            rpy_labels = ['Roll', 'Pitch', 'Yaw']
            rpy_limits = {
                'Roll': WORKSPACE_LIMITS['roll'],
                'Pitch': WORKSPACE_LIMITS['pitch'],
                'Yaw': WORKSPACE_LIMITS['yaw']
            }

            for i, label in enumerate(rpy_labels):
                row = ttk.Frame(self.orientation_frame)
                row.pack(fill=tk.X, pady=2)

                ttk.Label(row, text=f"{label}:", width=8).pack(side=tk.LEFT)
                value_label = ttk.Label(row, text=f"{current_pose[3+i]:.3f}", width=8)
                value_label.pack(side=tk.LEFT, padx=5)
                self.rpy_values_display[label] = value_label

                var = tk.DoubleVar(value=current_pose[3+i])
                self.rpy_vars[label] = var

                min_lim, max_lim = rpy_limits[label]
                slider = ttk.Scale(
                    row, from_=min_lim, to=max_lim, orient=tk.HORIZONTAL,
                    variable=var, command=lambda v, lbl=label: self.handle_orientation_slider_continuous(lbl)
                )
                slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

                slider.bind("<ButtonPress-1>", lambda e, lbl=label: self.orientation_slider_pressed(lbl))
                slider.bind("<ButtonRelease-1>", lambda e, lbl=label: self.orientation_slider_released(lbl))
        else:
            self.rpy_vars = {}
            self.rpy_values_display = {}

        # Bind trace if not already (moved here for safety)
        if not hasattr(self, '_orientation_bound'):
            self.use_orientation.trace('w', self.toggle_orientation_inputs)
            self._orientation_bound = True
        
        # Initial toggle call to set visibility
        self.toggle_orientation_inputs()
        
        # Log setup complete
        self.log_action(" Cartesian sliders setup complete (RPY: Enabled)")



    def handle_cartesian_slider_continuous(self, label):
        """Update display during Cartesian slider drag (no planning)."""
        if label in self.xyz_vars:
            value = self.xyz_vars[label].get()
            self.xyz_values_display[label].config(text=f"{value:.3f}")

    def cartesian_slider_pressed(self, label):
        """Mark Cartesian slider as active (for hand tracking pause)."""
        self.cartesian_slider_active[label] = True  # Assuming self.cartesian_slider_active = {} initialized in init

    def cartesian_slider_released(self, label):
        """Mark Cartesian slider as inactive."""
        self.cartesian_slider_active[label] = False

    def handle_orientation_slider_continuous(self, label):
        """Update display during orientation slider drag."""
        if label in self.rpy_vars:
            value = self.rpy_vars[label].get()
            self.rpy_values_display[label].config(text=f"{value:.3f}")

    def orientation_slider_pressed(self, label):
        """Mark orientation slider as active."""
        self.orientation_slider_active[label] = True  # Assuming self.orientation_slider_active = {} in init

    def orientation_slider_released(self, label):
        """Mark orientation slider as inactive."""
        self.orientation_slider_active[label] = False



    def setup_joint_sliders(self, joint_values):
        """Setup joint sliders (extracted for modularity)."""
        joint_names = self.get_joint_names()
        joint_limits = self.get_joint_limits()
        num_joints = len(joint_values)
        display_names = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]

        for i in range(num_joints):
            row = ttk.Frame(self.joint_frame)
            row.pack(fill=tk.X, pady=2)

            name = display_names[i] if i < len(display_names) else joint_names[i]
            ttk.Label(row, text=f"{name}:", width=18).pack(side=tk.LEFT)

            value_label = ttk.Label(row, text=f"{joint_values[i]:.3f}", width=8)
            value_label.pack(side=tk.LEFT)
            self.joint_values.append(value_label)

            var = tk.DoubleVar(value=joint_values[i])
            self.joint_vars.append(var)

            min_lim, max_lim = joint_limits.get(joint_names[i], (-3.14, 3.14))
            slider = ttk.Scale(
                row, from_=min_lim, to=max_lim, orient=tk.HORIZONTAL,
                variable=var, command=lambda v, idx=i: self.handle_slider_continuous(idx)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            slider.bind("<ButtonPress-1>", lambda e, idx=i: self.slider_pressed(idx))
            slider.bind("<ButtonRelease-1>", lambda e, idx=i: self.slider_released(idx))

            self.joint_sliders.append(slider)


    def toggle_orientation_inputs(self, *args):
        """Toggle visibility of orientation (RPY) sliders based on checkbox."""
        if not hasattr(self, 'orientation_frame'):
            return  # Not in Cartesian mode yet
        
        if self.use_orientation.get():
            self.orientation_frame.pack(fill=tk.X, pady=5)
            self.log_action(" Orientation sliders: Enabled")
        else:
            self.orientation_frame.pack_forget()
            self.log_action("Orientation sliders: Disabled")
        
        # Re-pack Cartesian frame to adjust layout
        if hasattr(self, 'cartesian_input_frame'):
            self.cartesian_input_frame.pack_configure(fill=tk.BOTH, expand=True)



    def get_joint_names(self):
        """Return the list of joint names."""
        return self.joint_names.copy()

    def get_current_joint_values(self):
        """Return current joint values."""
        if hasattr(self, 'current_joint_positions') and self.current_joint_positions is not None:
            return self.current_joint_positions.copy()
        return self._current_values.copy()

    def set_current_joint_values(self, values):
        """Update current joint values (thread-safe)."""
        with self.goal_lock:
            self._current_values = values.copy()
            self.current_joint_positions = values.copy()

    def get_joint_limits(self):
        """Return joint limits from commander."""
        if self.commander and hasattr(self.commander, 'get_joint_limits'):
            return self.commander.get_joint_limits()
        # Fallback limits matching PegasusCommander joint names
        return {
            'joint1': (-6.28, 6.28),
            'joint2': (-0.61, 0.61),
            'joint3': (-1.75, 1.75),
            'joint4': (-1.31, 1.31),
            'joint5': (-6.28, 6.28)
        }
        


    def get_current_pose(self):
        """Get current end effector pose."""
        if self.commander:
            return self.commander.get_current_pose()
        return None

    def move_to_joint_positions(self, joint_goal, velocity):
        """Delegate to commander's move method."""
        if self.commander:
            return self.commander.move_to_joint_positions(joint_goal, velocity)
        elif self.joint_cmd_pub:
            msg = JointState()
            msg.name = self.joint_names
            msg.position = joint_goal
            self.joint_cmd_pub.publish(msg)
            return True, MoveItErrorCodes.SUCCESS
        return False, MoveItErrorCodes.FAILURE

    def move_to_pose(self, pose, velocity):
        """Wrapper for moving to a Cartesian pose."""
        if self.commander and hasattr(self.commander, 'move_to_pose'):
            return self.commander.move_to_pose(pose, velocity)
        elif self.commander and hasattr(self.commander, 'plan_to_cartesian_pose'):
            # Fallback: use plan_to_cartesian_pose if available
            x, y, z = pose[0:3]
            roll, pitch, yaw = pose[3:6]
            success, _ = self.commander.plan_to_cartesian_pose(x, y, z, roll, pitch, yaw)
            return success, MoveItErrorCodes.SUCCESS
        elif self.pose_cmd_pub:
            msg = PoseStamped()
            msg.header.frame_id = self.base_frame
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            
            # Convert RPY to Quaternion
            r = R.from_euler('xyz', [pose[3], pose[4], pose[5]], degrees=False)
            q = r.as_quat()
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            
            self.pose_cmd_pub.publish(msg)
            return True, MoveItErrorCodes.SUCCESS
        return False, MoveItErrorCodes.FAILURE


    def move_to_named_target(self, target_name, velocity):
        """Move to a named target pose."""
        if target_name in self.named_poses:
            joint_goal = self.named_poses[target_name]
            return self.move_to_joint_positions(joint_goal, velocity)
        return False, MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS

    def send_joints_to_arduino(self, joint_values):
        """Delegate to commander's Arduino method."""
        if self.commander:
            return self.commander.send_joints_to_arduino(joint_values)
        return False

    def cancel_current_goals(self):
        """Cancel any active goals."""
        # Publish stop command to headless commander
        if self.node:
            msg = Empty()
            self.stop_cmd_pub.publish(msg)
            self.log_action("Stop command sent")
        else:
            self.log_action("Cannot send stop command: Node not ready")

    def get_available_frames(self):
        """Get available TF frames."""
        if self.commander:
            return self.commander.get_available_frames()
        return []

    def check_tf_availability(self, level_check=False):
        """Check TF availability."""
        if self.commander:
            self.commander.check_tf_availability(level_check)
            self.tf_available = self.commander.tf_available




    def setup_control_buttons(self):
        """Control mode selector + presets + frames setup."""
        
        # Control mode selector
        mode_frame = ttk.LabelFrame(self.control_frame, text="Control Mode", padding=5)
        mode_frame.pack(fill=tk.X, pady=(0,5))

        if not hasattr(self, 'control_mode'):
            self.control_mode = tk.StringVar(value="joint")
            self.control_mode.trace('w', self.on_control_mode_changed)

        ttk.Radiobutton(
            mode_frame, text="Joint Space",
            variable=self.control_mode, value="joint",
            command=self.on_control_mode_changed
        ).pack(side=tk.LEFT, padx=10)

        ttk.Radiobutton(
            mode_frame, text="Cartesian Space",
            variable=self.control_mode, value="cartesian",
            command=self.on_control_mode_changed
        ).pack(side=tk.LEFT, padx=10)
        
        # ADD HAND CONTROL TOGGLE HERE
        self.setup_hand_control_toggle()

        # PRESETS (always top)
        self.preset_frame = ttk.LabelFrame(self.control_frame, text="Presets", padding=5)
        self.preset_frame.pack(fill=tk.X, pady=(5,10))

        ttk.Button(self.preset_frame, text="Home", style="Rounded.TButton",
                command=self.plan_home).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

        ttk.Button(self.preset_frame, text="Extended", style="Rounded.TButton",
                command=self.plan_extended).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

        # Mode-specific frames (create here, but populate in mode change)
        self.joint_frame = ttk.LabelFrame(self.control_frame, text="Joint Positions (rad)", padding="5")
        self.cartesian_input_frame = ttk.LabelFrame(self.control_frame, text="Cartesian Pose", padding="5")

        # PLAN BUTTON (create BEFORE initial mode call)
        self.plan_button = ttk.Button(
            self.control_frame, text="PLAN & EXECUTE",
            style="Success.TButton", command=self.plan_current_mode
        )

        # Initial mode setup (AFTER all creation, to avoid AttributeErrors)
        self.on_control_mode_changed()



    def on_slider_change(self, idx):
        """Called when slider values change - removed arm display update"""
        pass  # Schematic removed, no visualization to update

    def _update_scroll_region(self):
        """Force update of scroll region after layout changes"""
        if hasattr(self, 'main_canvas'):
            self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))

    def slider_pressed(self, idx):
        """Called when a joint slider is clicked"""
        self.slider_active[idx] = True

    def slider_released(self, idx):
        """Called when a joint slider is released"""
        self.slider_active[idx] = False

    def cartesian_slider_pressed(self, idx):
        """Called when a Cartesian slider is clicked"""
        self.cartesian_slider_active[idx] = True

    def cartesian_slider_released(self, idx):
        """Called when a Cartesian slider is released"""
        self.cartesian_slider_active[idx] = False


    def handle_cartesian_slider_continuous(self, idx):
        """Handle continuous Cartesian slider movement - UPDATE DISPLAY ONLY"""
        if not hasattr(self, 'cartesian_slider_active') or not self.cartesian_slider_active[idx]:
            return
        
        try:
            # Get values
            x = self.xyz_vars['X'].get()
            y = self.xyz_vars['Y'].get()
            z = self.xyz_vars['Z'].get()
            
            # Update display labels ONLY
            xyz_labels = ['X', 'Y', 'Z']
            self.xyz_values_display[xyz_labels[idx]].config(text=f"{[x, y, z][idx]:.3f}")
            
            # Get orientation if enabled
            if self.use_orientation.get():
                roll = self.rpy_vars['Roll'].get()
                pitch = self.rpy_vars['Pitch'].get()
                yaw = self.rpy_vars['Yaw'].get()
                
                # Update orientation displays too
                if idx < 3:  # Only update if dragging XYZ sliders
                    self.rpy_values_display['Roll'].config(text=f"{roll:.3f}")
                    self.rpy_values_display['Pitch'].config(text=f"{pitch:.3f}")
                    self.rpy_values_display['Yaw'].config(text=f"{yaw:.3f}")
            
            # NO IK computation or publishing during drag!
            # User must press "PLAN & EXECUTE" button to apply changes
            
        except Exception as e:
            self.logger.debug(f"Error in cartesian slider update: {str(e)}")

    def orientation_slider_pressed(self, idx):
        """Called when an orientation slider is clicked"""
        self.orientation_slider_active[idx] = True

    def orientation_slider_released(self, idx):
        """Called when an orientation slider is released"""
        self.orientation_slider_active[idx] = False



    def handle_orientation_slider_continuous(self, idx):
        """Handle continuous orientation slider movement - UPDATE DISPLAY ONLY"""
        if not hasattr(self, 'orientation_slider_active') or not self.orientation_slider_active[idx]:
            return
        
        try:
            # Get RPY values
            roll = self.rpy_vars['Roll'].get()
            pitch = self.rpy_vars['Pitch'].get()
            yaw = self.rpy_vars['Yaw'].get()
            
            # Update display labels ONLY
            rpy_labels = ['Roll', 'Pitch', 'Yaw']
            current_vals = [roll, pitch, yaw]
            self.rpy_values_display[rpy_labels[idx]].config(text=f"{current_vals[idx]:.3f}")
            
            # NO planning or IK during drag!
            # User must press "PLAN & EXECUTE" button to apply changes
            
        except Exception as e:
            self.logger.debug(f"Error in orientation slider update: {str(e)}")


    def toggle_orientation_inputs(self):
        """Show/hide orientation input fields"""
        if self.use_orientation.get():
            self.orientation_frame.pack(fill=tk.X, pady=5)
        else:
            self.orientation_frame.pack_forget()


    def read_current_cartesian_pose(self):
        """Read current end effector pose and populate input fields"""
        try:
            if not hasattr(self, 'current_pose_cache') or self.current_pose_cache is None:
                self.log_action("Cannot read pose: No data received yet")
                return
            
            pose = self.current_pose_cache
            # pose = [x, y, z, roll, pitch, yaw]
            
            self.xyz_vars['X'].set(round(pose[0], 4))
            self.xyz_vars['Y'].set(round(pose[1], 4))
            self.xyz_vars['Z'].set(round(pose[2], 4))
            
            # Update displays
            self.xyz_values_display['X'].config(text=f"{pose[0]:.3f}")
            self.xyz_values_display['Y'].config(text=f"{pose[1]:.3f}")
            self.xyz_values_display['Z'].config(text=f"{pose[2]:.3f}")
            
            if self.use_orientation.get():
                self.rpy_vars['Roll'].set(round(pose[3], 4))
                self.rpy_vars['Pitch'].set(round(pose[4], 4))
                self.rpy_vars['Yaw'].set(round(pose[5], 4))
                
                self.rpy_values_display['Roll'].config(text=f"{pose[3]:.3f}")
                self.rpy_values_display['Pitch'].config(text=f"{pose[4]:.3f}")
                self.rpy_values_display['Yaw'].config(text=f"{pose[5]:.3f}")
            
            self.log_action(f"✓ Current pose: X={pose[0]:.3f}, Y={pose[1]:.3f}, Z={pose[2]:.3f}")
            
        except Exception as e:
            self.log_action(f"✗ Error reading pose: {str(e)}")
            messagebox.showerror("Error", f"Failed to read current pose: {str(e)}")


    def plan_current_mode(self):
        """Plan in current mode and execute motion. Enhanced logging for success/failure."""
        if not self.node:
            self.log_action("✗ ROS Node not available - planning aborted")
            messagebox.showerror("Error", "ROS Node not initialized.")
            return

        #  ADD THIS CHECK - Prevent planning during execution
        # Note: We rely on status feedback for this now, but for async commands we might just send it
        # if hasattr(self.commander, 'is_executing') and self.commander.is_executing:
        #     self.log_action("⚠ Execution in progress, please wait...")
        #     return

        mode = self.control_mode.get()
        # self.log_action(f" Initiating {mode.upper()} planning...")
        print(f"[DEBUG] plan_current_mode called. Mode: {mode}")

        try:
            if mode == "joint":
                target_joints = [var.get() for var in self.joint_vars]
                self.log_action(f" Joint targets: {[f'{v:.3f}' for v in target_joints]}")
                
                if self.joint_cmd_pub:
                    msg = JointState()
                    msg.name = self.joint_names
                    msg.position = target_joints
                    self.joint_cmd_pub.publish(msg)
                    self.log_action(" Joint command published")
                else:
                    self.log_action("✗ Joint command publisher not available")

            elif mode == "cartesian":
                x = float(self.xyz_vars['X'].get())
                y = float(self.xyz_vars['Y'].get())
                z = float(self.xyz_vars['Z'].get())
                self.log_action(f" Cartesian position targets: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

                roll = pitch = yaw = 0.0
                if self.use_orientation.get():
                    roll = float(self.rpy_vars['Roll'].get())
                    pitch = float(self.rpy_vars['Pitch'].get())
                    yaw = float(self.rpy_vars['Yaw'].get())
                    self.log_action(f" Orientation targets: Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f}")

                # Clamp to workspace (log if clamped)
                x_c, y_c, z_c, r_c, p_c, y_c, was_clamped = clamp_to_workspace(x, y, z, roll, pitch, yaw)
                if was_clamped:
                    self.log_action(f"⚠ Targets clamped: ({x:.3f},{y:.3f},{z:.3f}) → ({x_c:.3f},{y_c:.3f},{z_c:.3f})")

                    # Update GUI with clamped values
                    self.xyz_vars['X'].set(round(x_c, 4))
                    self.xyz_vars['Y'].set(round(y_c, 4))
                    self.xyz_vars['Z'].set(round(z_c, 4))
                    self.xyz_values_display['X'].config(text=f"{x_c:.3f}")
                    self.xyz_values_display['Y'].config(text=f"{y_c:.3f}")
                    self.xyz_values_display['Z'].config(text=f"{z_c:.3f}")

                    if self.use_orientation.get():
                        self.rpy_vars['Roll'].set(round(r_c or 0.0, 4))
                        self.rpy_vars['Pitch'].set(round(p_c or 0.0, 4))
                        self.rpy_vars['Yaw'].set(round(y_c or 0.0, 4))
                        self.rpy_values_display['Roll'].config(text=f"{r_c or 0.0:.3f}")
                        self.rpy_values_display['Pitch'].config(text=f"{p_c or 0.0:.3f}")
                        self.rpy_values_display['Yaw'].config(text=f"{y_c or 0.0:.3f}")

                # Plan (log result)
                if self.pose_cmd_pub:
                    msg = PoseStamped()
                    msg.header.frame_id = self.base_frame
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.pose.position.x = x_c
                    msg.pose.position.y = y_c
                    msg.pose.position.z = z_c
                    
                    # Convert RPY to Quaternion
                    q = R.from_euler('xyz', [r_c or 0.0, p_c or 0.0, y_c or 0.0], degrees=False).as_quat()
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]
                    
                    self.pose_cmd_pub.publish(msg)
                    self.log_action(" Cartesian command published")
                else:
                    self.log_action("✗ Pose command publisher not available")

        except ValueError as e:
            self.log_action(f"✗ Invalid input values: {str(e)}")
            messagebox.showerror("Error", f"Invalid input values: {str(e)}")
        except Exception as e:
            self.log_action(f"✗ Planning exception: {str(e)}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Planning failed: {str(e)}")




    def plan_joint_positions(self):
        """Plan to joint positions - REMOVED, use plan_home/plan_extended instead"""
        self.log_action("This method is deprecated. Use preset buttons instead.")
        return False

    def setup_pose_display(self):
        self.pose_labels = {}
        for coord in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            pose_row = ttk.Frame(self.pose_frame)
            pose_row.pack(fill=tk.X, pady=5)

            label = ttk.Label(pose_row, text=f"{coord}:", width=10, font=("Helvetica", 10))
            label.pack(side=tk.LEFT)

            value = ttk.Label(pose_row, text="N/A" if not self.tf_available else "0.000", width=10, font=("Helvetica", 10))
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
        self.controller_name_var = tk.StringVar(value=self.controller_name)
        controller_entry = ttk.Entry(settings_frame, textvariable=self.controller_name_var, width=30)
        controller_entry.pack(anchor="w", padx=5)
        
        update_controller_button = ttk.Button(
            settings_frame,
            text="Update Controller",
            style="Rounded.TButton",
            command=self.update_controller_name
        )
        update_controller_button.pack(anchor="w", padx=5, pady=5)
        
        # FIXED: Changed from self.load_joint_limits to self.reload_joint_limits
        reload_limits_button = ttk.Button(
            settings_frame,
            text="Reload Joint Limits",
            style="Rounded.TButton",
            command=self.reload_joint_limits  # <-- CHANGED THIS LINE
        )
        reload_limits_button.pack(anchor="w", padx=5, pady=5)

        # Add MoveIt toggle checkbox
        moveit_toggle = ttk.Checkbutton(
            settings_frame,
            text="Enable MoveIt (disable for direct Arduino control)",
            variable=self.moveit_enabled,
            command=self.on_moveit_toggle
        )
        moveit_toggle.pack(anchor="w", padx=5, pady=5)


    def setup_camera_tab(self, parent_frame):
        """Setup the camera tab with live feed and controls"""
        self.camera_frame = ttk.Frame(parent_frame, padding="10")
        self.camera_frame.pack(fill=tk.BOTH, expand=True)
        
        # Camera control frame
        control_frame = ttk.Frame(self.camera_frame)
        control_frame.pack(fill=tk.X, pady=5)
        
        # Start/Stop camera button
        self.camera_active = False
        self.start_camera_button = ttk.Button(
            control_frame,
            text="Start Live Feed",
            style="Success.TButton",
            command=self.toggle_camera_feed
        )
        self.start_camera_button.pack(side=tk.LEFT, padx=5)
        
        # Camera mode selection (REMOVED - RGB only)
        # self.camera_mode = tk.StringVar(value="RGB")
        
        # Camera display frame
        display_frame = ttk.LabelFrame(self.camera_frame, text="Camera Feed", padding="10")
        display_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Camera display label
        self.camera_display = ttk.Label(
            display_frame, 
            text="Camera feed will appear here\nClick 'Start Live Feed' to begin",
            anchor="center",
            font=("Helvetica", 12)
        )
        self.camera_display.pack(expand=True, fill=tk.BOTH)
        
        # Initialize camera variables
        self.camera = None
        self.camera_thread = None
        self.camera_running = False

    def toggle_camera_feed(self):
        """Start or stop the camera feed"""
        if not self.camera_active:
            self.start_camera_feed()
        else:
            self.stop_camera_feed()

    def start_camera_feed(self):
        """Start the camera feed by subscribing to ROS topic"""
        try:
            if not self.node:
                self.log_action("Cannot start camera: ROS node not ready")
                return

            # Subscribe to /cv_image topic
            self.camera_sub = self.node.create_subscription(
                Image,
                '/cv_image',
                self.image_callback,
                10
            )
            
            self.camera_active = True
            self.start_camera_button.config(text="Stop Live Feed", style="Danger.TButton")
            
            self.log_action("Camera feed started (Topic: /cv_image)")
            
        except Exception as e:
            self.log_action(f"Error starting camera: {str(e)}")
            messagebox.showerror("Error", f"Failed to start camera: {str(e)}")

    def stop_camera_feed(self):
        """Stop the camera feed by destroying subscription"""
        try:
            self.camera_active = False
            
            if hasattr(self, 'camera_sub') and self.camera_sub:
                self.node.destroy_subscription(self.camera_sub)
                self.camera_sub = None
            
            self.start_camera_button.config(text="Start Live Feed", style="Success.TButton")
            self.camera_display.config(
                image="",
                text="Camera feed stopped\nClick 'Start Live Feed' to restart"
            )
            
            self.log_action("Camera feed stopped")
            
        except Exception as e:
            self.log_action(f"Error stopping camera: {str(e)}")

    def image_callback(self, msg):
        """Process incoming ROS image"""
        if not self.camera_active:
            return
            
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize and convert to Tkinter format
            # Resize frame to fit display
            height, width = cv_image.shape[:2]
            max_width, max_height = 640, 480
            
            if width > max_width or height > max_height:
                scale = min(max_width/width, max_height/height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Convert to RGB for Tkinter
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            tk_image = ImageTk.PhotoImage(image=pil_image)
            
            # Update label
            self.camera_display.configure(image=tk_image)
            self.camera_display.image = tk_image  # Keep reference
            
        except Exception as e:
            # self.logger.error(f"Image callback error: {e}")
            pass

    # camera_loop REMOVED
    # Obsolete camera methods REMOVED
    # Remaining obsolete camera methods REMOVED



    def setup_keyboard_shortcuts(self):
        self.root.bind("<Up>", lambda e: self.handle_key_press("up"))
        self.root.bind("<Down>", lambda e: self.handle_key_press("down"))
        self.root.bind("<Left>", lambda e: self.handle_key_press("left"))
        self.root.bind("<Right>", lambda e: self.handle_key_press("right"))
        self.root.bind("<h>", lambda e: self.go_to_home())
        self.root.bind("<z>", lambda e: self.go_to_zero())
        self.root.bind("<Escape>", lambda e: self.emergency_stop())
        self.root.bind("<F5>", lambda e: self.refresh_tf_status())




    def log_action(self, message):
        """Enhanced log action with emoji filtering and deduplication"""
        try:
            timestamp = time.strftime('%H:%M:%S')
            
            # Log to console for debugging
            print(f"[{timestamp}] {message}")
            sys.stdout.flush()
            
            # Check for duplicate (prevent spam)
            if hasattr(self, '_last_log_message') and self._last_log_message == message:
                if hasattr(self, '_duplicate_count'):
                    self._duplicate_count += 1
                    if self._duplicate_count > 3:  # Only log first 3 duplicates
                        return
                else:
                    self._duplicate_count = 1
            else:
                self._last_log_message = message
                self._duplicate_count = 0
            
            # Log to GUI text widget
            if hasattr(self, 'log_text') and self.log_text:
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
                self.log_text.see(tk.END)
                
                # Limit log size (keep last 500 lines)
                line_count = int(self.log_text.index('end-1c').split('.')[0])
                if line_count > 500:
                    self.log_text.delete('1.0', f'{line_count - 500}.0')
                
                self.log_text.config(state=tk.DISABLED)
                
        except Exception as e:
            # Fallback to console if GUI logging fails
            print(f"[LOG ERROR] {message} (GUI log failed: {e})")
            sys.stdout.flush()






    def show_temporary_error(self, message):
        """Show a temporary error message that disappears after 2 seconds"""
        # Prevent multiple error dialogs from showing simultaneously
        if self.error_dialog_active:
            return
        
        self.error_dialog_active = True
        
        # Create a temporary error window
        error_window = tk.Toplevel(self.root)
        error_window.title("Error")
        error_window.geometry("400x150")
        error_window.resizable(False, False)
        
        # Center the error window on the main window
        self.root.update_idletasks()
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - 200
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - 75
        error_window.geometry(f"400x150+{x}+{y}")
        
        # Make window visible first, then grab focus
        error_window.transient(self.root)
        error_window.focus_set()
        
        # Delay grab_set until window is fully visible
        def set_grab():
            try:
                if error_window.winfo_exists():
                    error_window.grab_set()
            except Exception:
                pass  # Ignore grab errors
        
        error_window.after(50, set_grab)  # Delay grab by 50ms
        
        # Error message
        error_frame = ttk.Frame(error_window, padding="20")
        error_frame.pack(fill=tk.BOTH, expand=True)
        
        error_label = ttk.Label(
            error_frame, 
            text=message, 
            wraplength=350,
            justify="center",
            font=("Helvetica", 10),
            foreground="red"
        )
        error_label.pack(expand=True, fill=tk.BOTH)
        
        # Countdown label
        countdown_var = tk.StringVar(value="Auto-closing in 2 seconds...")
        countdown_label = ttk.Label(
            error_frame, 
            textvariable=countdown_var,
            font=("Helvetica", 9, "italic")
        )
        countdown_label.pack(pady=(10, 0))
        
        # Close button
        close_button = ttk.Button(
            error_frame,
            text="Close",
            command=lambda: self.close_error_window(error_window),
            style="Rounded.TButton"
        )
        close_button.pack(pady=(10, 0))
        
        # Auto-close after 2 seconds with countdown
        def countdown(seconds_left):
            try:
                if seconds_left > 0 and error_window.winfo_exists():
                    countdown_var.set(f"Auto-closing in {seconds_left} second{'s' if seconds_left != 1 else ''}...")
                    error_window.after(1000, lambda: countdown(seconds_left - 1))
                elif error_window.winfo_exists():
                    self.close_error_window(error_window)
            except Exception:
                pass  # Ignore any countdown errors
        
        # Start countdown
        countdown(2)
        
        # Handle window close event
        error_window.protocol("WM_DELETE_WINDOW", lambda: self.close_error_window(error_window))


    def close_error_window(self, window):
        """Close error window and reset flag"""
        try:
            if window and window.winfo_exists():
                window.grab_release()  # Release grab first
                window.destroy()
        except Exception:
            pass  # Ignore any closing errors
        finally:
            self.error_dialog_active = False

    def status_check(self):
        status = []
        if self.action_server_available:
            status.append("MoveIt OK")
        else:
            status.append("MoveIt Disconnected")
        if self.controller_available:
            status.append("Controller OK")
        else:
            status.append("Controller Disconnected")
        if self.joint_states_available:
            status.append("Joints OK")
        else:
            status.append("Joints Disconnected")
        if self.tf_available:
            status.append("TF OK")
        else:
            status.append("TF Disconnected")
        self.status_var.set(" | ".join(status))
        self.root.after(1000, self.status_check)

    def update_tf_status(self):
        if self.tf_available:
            self.tf_status_var.set(f"TF Status: Connected (Base: {self.base_frame} → {self.end_effector_frame})")
            self.tf_status_label.configure(style="Success.TLabel")
        else:
            self.tf_status_var.set(f"TF Status: Disconnected (Base: {self.base_frame} → {self.end_effector_frame})")
            self.tf_status_label.configure(style="Alert.TLabel")
        self.root.after(5000, self.update_tf_status)

    def refresh_tf_status(self):
        new_base_frame = self.base_frame_var.get()
        if new_base_frame != self.base_frame:
            self.base_frame = new_base_frame
            self.log_action(f"Changed base frame to '{new_base_frame}'")
        
        self.check_tf_availability(level_check=True)
        self.update_tf_status()
        
        frames = self.get_available_frames()
        if frames:
            self.base_frame_dropdown['values'] = frames
            self.log_action(f"Available frames: {', '.join(frames)}")
        else:
            self.log_action("No TF frames available. Check 'robot_state_publisher'.")

    def retry_controllers(self):
        self.log_action("Attempting to activate controllers...")
        success = self.activate_controllers()
        if success:
            self.log_action("Controllers activated successfully")
            messagebox.showinfo("Success", "Controllers activated successfully")
        else:
            self.log_action("Failed to activate controllers. Check controller_manager.")
            messagebox.showerror("Error", "Failed to activate controllers. Please check logs.")

    def update_controller_name(self):
        new_controller = self.controller_name_var.get()
        if new_controller != self.controller_name:
            self.update_controller_name(new_controller)
            self.log_action(f"Updated controller name to '{new_controller}'")
            messagebox.showinfo("Success", f"Controller name updated to '{new_controller}'")

    def on_moveit_toggle(self):
        enabled = self.moveit_enabled.get()
        self.set_moveit_enabled(enabled)
        state = "enabled" if enabled else "disabled"
        self.log_action(f"MoveIt {state}")


    def reload_joint_limits(self):
        """Reload joint limits from the commander"""
        if self.commander:
            self.commander.load_joint_limits()
            self.log_action("Joint limits reloaded")
            messagebox.showinfo("Success", "Joint limits reloaded successfully")
        else:
            self.log_action("Commander not available")
            messagebox.showerror("Error", "Commander not initialized")



    def get_selected_joint_index(self):
        joint_name = self.selected_joint.get()
        joint_names = self.get_joint_names()
        return joint_names.index(joint_name) if joint_name in joint_names else -1


    def emergency_stop_all_publishing(self):
        """Emergency stop - cancel all active publishing"""
        try:
            if self.commander:
                # Stop trajectory execution
                if hasattr(self.commander, 'is_executing'):
                    self.commander.is_executing = False
                
                # Stop any timers
                if hasattr(self.commander, 'execution_timer') and self.commander.execution_timer:
                    self.commander.execution_timer.cancel()
                
                self.log_action(" Emergency stop: All publishing halted")
            
            # Clear movement flag
            if hasattr(self, 'movement_in_progress'):
                self.movement_in_progress.clear()
                
        except Exception as e:
            self.log_action(f"Error in emergency stop: {e}")

    # Bind to ESC key
    def setup_keyboard_shortcuts(self):
        self.root.bind("<Escape>", lambda e: self.emergency_stop_all_publishing())
        # ... other shortcuts ...


    def check_joint_state_sources(self):
        """Diagnostic: Check what's publishing to /joint_states"""
        try:
            if not self.commander:
                return
            
            # Get list of publishers on /joint_states topic
            topic_info = self.commander.get_publishers_info_by_topic('/joint_states')
            
            self.log_action(f"Publishers on /joint_states: {len(topic_info)}")
            for pub in topic_info:
                self.log_action(f"  - Node: {pub.node_name}, Namespace: {pub.node_namespace}")
        except Exception as e:
            self.log_action(f"Error checking publishers: {e}")


    def handle_slider_continuous(self, joint_index):
        """Handle continuous slider movement - UPDATE DISPLAY ONLY, NO PLANNING"""
        if not hasattr(self, 'slider_active') or not self.slider_active[joint_index]:
            return
        
        try:
            joint_goal = [var.get() for var in self.joint_vars]
            joint_name = self.get_joint_names()[joint_index]
            joint_limits = self.get_joint_limits()
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)
            
            # Update display label ONLY
            if joint_index < len(self.joint_values):
                self.joint_values[joint_index].config(text=f"{joint_goal[joint_index]:.3f}")
            
            # Store positions for later use
            self.current_joint_positions = joint_goal.copy()
            
            #  REMOVE ANY PLANNING/PUBLISHING HERE - User must press button
            
        except Exception as e:
            self.logger.error(f"Error in slider continuous update: {str(e)}")



    def handle_slider(self, joint_index):
        """Handle slider movement from button presses or other discrete actions"""
        if self.movement_in_progress.is_set():
            return
        with self.lock:
            if self.movement_in_progress.is_set():
                return
            self.movement_in_progress.set()
        try:
            joint_goal = [var.get() for var in self.joint_vars]
            joint_name = self.get_joint_names()[joint_index]
            joint_limits = self.get_joint_limits()
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)
            self.current_joint_positions = joint_goal.copy()
            self.current_joint_time = time.time()
            self.joint_vars[joint_index].set(joint_goal[joint_index])
            if self.moveit_enabled.get():
                velocity = self.velocity_scale.get()
                success, error_code = self.move_to_joint_positions(joint_goal, velocity)
                if success:
                    try:
                        self.set_current_joint_values(joint_goal)
                    except Exception:
                        pass
            else:
                success = self.send_joints_to_arduino(joint_goal)
                if success:
                    try:
                        self.set_current_joint_values(joint_goal)
                    except Exception:
                        pass
        except Exception as e:
            print(f"Error during slider movement: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def plan_named_pose(self, pose_name):
        """Set sliders to a named pose and invoke the planner."""
        if not hasattr(self, 'named_poses') or pose_name not in self.named_poses:
            self.log_action(f"Named pose '{pose_name}' not found")
            return False
        pose = self.named_poses[pose_name]
        # Update sliders without triggering continuous sends
        for i, val in enumerate(pose):
            if i < len(self.joint_vars):
                self.joint_vars[i].set(val)
        # Give UI a moment to update
        self.root.update_idletasks()
        # Call existing plan flow
        return self.plan_joint_positions()


    def plan_home(self):
        """Move to home position"""
        if not self.commander and not self.joint_cmd_pub:
            self.log_action(" Commander not available")
            messagebox.showerror("Error", "Commander not initialized. Please wait.")
            return
        
        self.log_action(" Planning to Home pose")
        try:
            home_joints = self.named_poses['home']
            
            # Update sliders to show target
            for i, val in enumerate(home_joints):
                if i < len(self.joint_vars):
                    self.joint_vars[i].set(val)
            
            self.root.update_idletasks()  # Update GUI
            
            if self.commander:
                # Plan using commander
                result = self.commander.plan_to_joint_values(home_joints)
                
                if result.get('success'):
                    # Execute trajectory
                    self.commander.execute_live_trajectory(result)
                    self.log_action("Moving to Home position")
                else:
                    error_msg = result.get('message', 'Unknown error')
                    self.log_action(f" Planning failed: {error_msg}")
                    messagebox.showerror("Planning Error", f"Failed to plan: {error_msg}")
            elif self.joint_cmd_pub:
                msg = JointState()
                msg.name = self.joint_names
                msg.position = home_joints
                self.joint_cmd_pub.publish(msg)
                self.log_action(" Moving to Home position (Remote)")
                
        except Exception as e:
            self.log_action(f" Error moving to home: {str(e)}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to move to home: {str(e)}")

    def plan_extended(self):
        """Move to extended position"""
        if not self.commander and not self.joint_cmd_pub:
            self.log_action(" Commander not available")
            messagebox.showerror("Error", "Commander not initialized. Please wait.")
            return
        
        self.log_action(" Planning to Extended pose")
        try:
            # Get extended position and clamp to limits
            extended_joints = self.named_poses['extended'].copy()
            limits = self.get_joint_limits()
            
            # Clamp each joint to its limits
            for i, (val, name) in enumerate(zip(extended_joints, self.joint_names)):
                min_lim, max_lim = limits.get(name, (-3.14, 3.14))
                extended_joints[i] = float(np.clip(val, min_lim, max_lim))
            
            # Update sliders to show target
            for i, val in enumerate(extended_joints):
                if i < len(self.joint_vars):
                    self.joint_vars[i].set(val)
            
            self.root.update_idletasks()  # Update GUI
            
            if self.commander:
                # Plan using commander
                result = self.commander.plan_to_joint_values(extended_joints)
                
                if result.get('success'):
                    # Execute trajectory
                    self.commander.execute_live_trajectory(result)
                    self.log_action(" Moving to Extended position")
                else:
                    error_msg = result.get('message', 'Unknown error')
                    self.log_action(f" Planning failed: {error_msg}")
                    messagebox.showerror("Planning Error", f"Failed to plan: {error_msg}")
            elif self.joint_cmd_pub:
                msg = JointState()
                msg.name = self.joint_names
                msg.position = extended_joints
                self.joint_cmd_pub.publish(msg)
                self.log_action(" Moving to Extended position (Remote)")
                
        except Exception as e:
            self.log_action(f" Error moving to extended: {str(e)}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to move to extended: {str(e)}")






    def move_direction(self, direction):
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            print("Command too soon, please wait...")
            return
        if self.movement_in_progress.is_set():
            print("Movement in progress, please wait...")
            return
                
        with self.lock:
            if self.movement_in_progress.is_set():
                return
            self.movement_in_progress.set()
            
        self.last_command_time = current_time
        try:
            joint_index = self.get_selected_joint_index()
            step = self.step_size.get()
            joint_goal = self.get_current_joint_values()
            joint_limits = self.get_joint_limits()
            joint_name = self.get_joint_names()[joint_index]
            if joint_index < 0 or joint_index >= len(joint_goal):
                print(f"Invalid joint index: {joint_index}")
                return
            if direction in ["up", "right"]:
                joint_goal[joint_index] += step
            elif direction in ["down", "left"]:
                joint_goal[joint_index] -= step
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)
            
            # Store joint positions in a placeholder for later use
            self.current_joint_positions = joint_goal.copy();
            
            if self.moveit_enabled.get():
                velocity = self.velocity_scale.get()
                success, error_code = self.move_to_joint_positions(joint_goal, velocity)
                if success and not self.slider_active[joint_index]:
                    self.joint_vars[joint_index].set(joint_goal[joint_index])
                if success:
                    try:
                        self.set_current_joint_values(joint_goal)
                    except Exception:
                        pass
            else:
                success = self.send_joints_to_arduino(joint_goal)
                if success and not self.slider_active[joint_index]:
                    self.joint_vars[joint_index].set(joint_goal[joint_index])
                if success:
                    try:
                        self.set_current_joint_values(joint_goal)
                    except Exception:
                        pass
        except Exception as e:
            print(f"Error during movement: {str(e)}")
        finally:
            self.movement_in_progress.clear()



    def go_to_home(self):
        if self.movement_in_progress.is_set():
            print("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            success, error_code = self.move_to_named_target("home", self.velocity_scale.get())
            if success:
                joint_values = self.get_current_joint_values()
                # Store joint positions in a placeholder for later use
                self.current_joint_positions = joint_values.copy()
                # Update sliders only if not being manipulated
                for i, (var, value) in enumerate(zip(self.joint_vars, joint_values)):
                    if not self.slider_active[i]:
                        var.set(value)
        except Exception as e:
            print(f"Error moving to home: {str(e)}")
        finally:
            self.movement_in_progress.clear()

    def go_to_zero(self):
        if self.movement_in_progress.is_set():
            print("Movement in progress, please wait...")
            return

        self.movement_in_progress.set()
        self.last_command_time = time.time()

        try:
            zero_position = [0.0] * len(self.get_joint_names())
            velocity = self.velocity_scale.get()
            success, error_code = self.move_to_joint_positions(zero_position, velocity)
            if success:
                # Store joint positions in a placeholder for later use
                self.current_joint_positions = zero_position.copy()
                try:
                    self.set_current_joint_values(zero_position)
                except Exception:
                    pass
                # Update sliders only if not being manipulated
                for i, (var, value) in enumerate(zip(self.joint_vars, zero_position)):
                    if not self.slider_active[i]:
                        var.set(value)
        except Exception as e:
            print(f"Error moving to zero: {str(e)}")
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
            success, error_code = self.move_cartesian(delta, is_rotation, self.velocity_scale.get())
            if success:
                self.log_action(f"Cartesian movement ({direction}) completed")
            else:
                error_msg = MOVEIT_ERROR_CODES.get(error_code, "Unknown error")
                self.log_action(f"Cartesian movement failed: {error_msg}")
                self.show_temporary_error(f"Cartesian movement failed: {error_msg}")
        except Exception as e:
            self.log_action(f"Error during Cartesian movement: {str(e)}")
            self.show_temporary_error(f"Cartesian movement error: {str(e)}")
        finally:
            self.movement_in_progress.clear()


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



    def emergency_stop(self):
        self.log_action("EMERGENCY STOP ACTIVATED")
        self.cancel_current_goals()
        self.movement_in_progress.clear()
        self.root.after(1000, lambda: self.log_action("Emergency stop completed"))
        messagebox.showinfo("Info", "Emergency stop completed")

    def get_joint_limits(self):
        """Return a copy of the joint limits dictionary."""
        return self.joint_limits.copy()

def ros_spin_thread(ros_node):
    """Run ROS spin in a background thread"""
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        try:
            ros_node.get_logger().error(f"ROS spin error: {e}")
        except Exception:
            print(f"ROS spin error: {e}")



def main(args=None):
    import sys
    print("=" * 60)
    print("Starting GUI_commander...")
    print("=" * 60)
    sys.stdout.flush()
    
    logging.basicConfig(level=logging.INFO)

    gui_node = None
    
    try:
        print("[MAIN] Step 1: Initializing ROS...")
        sys.stdout.flush()
        rclpy.init(args=args)
        print("[MAIN] ✓ ROS initialized")
        sys.stdout.flush()
        
        # Create a simple node for the GUI
        gui_node = Node('gui_commander_client')
        
        # Start spinning the node in a background thread
        def ros_spin_thread(node):
            try:
                rclpy.spin(node)
            except Exception as e:
                print(f"ROS spin error: {e}")

        spin_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
        spin_thread.start()
        print("[MAIN] ✓ ROS spin thread started")
        
    except Exception as e:
        print(f"[MAIN] ✗ rclpy.init() failed: {e}")
        import traceback
        traceback.print_exc()
        return

    try:
        print("[MAIN] Step 2: Creating Tkinter root window...")
        sys.stdout.flush()
        root = tk.Tk()
        root.withdraw()  # Hide initially to show before fully loaded
        print("[MAIN] ✓ Tkinter root created")
        sys.stdout.flush()
        
        print("[MAIN] Step 3: Initializing PegasusArmGUI (Client Mode)...")
        sys.stdout.flush()
        # Pass the node to the GUI
        gui = PegasusArmGUI(root, node=gui_node)
        print("[MAIN] ✓ PegasusArmGUI initialized")
        sys.stdout.flush()
        
        # Show window after GUI is initialized
        print("[MAIN] Step 4: Showing window...")
        sys.stdout.flush()
        root.deiconify()  # Show the window
        print("[MAIN] ✓ Window visible")
        sys.stdout.flush()
        
        print("[MAIN] Step 5: Launching Tkinter mainloop...")
        sys.stdout.flush()
        root.mainloop()
        print("[MAIN] ✓ Mainloop exited")
        sys.stdout.flush()
        
    except Exception as e:
        print(f"[MAIN] ✗ GUI error: {e}")
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
    finally:
        print("[MAIN] Cleanup: Destroying node...")
        sys.stdout.flush()
        try:
            if gui_node:
                gui_node.destroy_node()
                print("[MAIN] ✓ GUI node destroyed")
        except Exception as e:
            print(f"[MAIN] Error destroying node: {e}")
        
        print("[MAIN] Cleanup: Shutting down ROS...")
        sys.stdout.flush()
        try:
            rclpy.shutdown()
            print("[MAIN] ✓ ROS shutdown complete")
        except Exception as e:
            print(f"[MAIN] Error shutting down rclpy: {e}")
        
        print("[MAIN] Application exit")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
