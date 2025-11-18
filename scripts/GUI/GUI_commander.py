import sys
import os
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


from PIL import Image, ImageTk 
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose

import serial 

sys.path.append(os.path.expanduser("~/ros2-ws/src/update_pegasus_description/scripts"))
from pegasus_commander import PegasusCommander as ExternalPegasusCommander

# Import socket server
from socket_server import SocketServer, RemoteCommand, CommandID

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
        self.status_var.set("🟢 Connected")
    
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

    def __init__(self, root, wait_for_services=False):
        self.root = root
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
        self.control_mode = tk.StringVar(value="joint")
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
            # RViz positioning vars
            # self.rviz_window_title = "Pegasus Arm RViz"  # Matches launcher env var
            self.auto_position_rviz = tk.BooleanVar(value=True)  # Toggle for auto-snap
            self.rviz_position_timer = None  # For periodic checks
            self.rviz_placeholder = None  # Ref to hide placeholder
            print("[GUI] ⚠ Skipping ExternalPegasusCommander (would block GUI)")
            sys.stdout.flush()
                
        except Exception as e:
            print(f"[GUI] ✗ Warning: Failed to initialize ExternalPegasusCommander: {e}")
            import traceback
            traceback.print_exc()
            sys.stdout.flush()
            self.commander = None
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
        
        print("[GUI] PegasusArmGUI initialization COMPLETE")
        sys.stdout.flush()






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





    def _gesture_home(self, gesture):
        """Gesture callback: Move to home"""
        self.log_action(" Gesture: Open Palm → Planning Home")
        self.root.after(0, self.plan_home)

    def _gesture_stop(self, gesture):
        """Gesture callback: Emergency stop"""
        self.log_action(" Gesture: Closed Fist → EMERGENCY STOP")
        self.root.after(0, self.emergency_stop)

    def _gesture_extended(self, gesture):
        """Gesture callback: Move to extended"""
        self.log_action(" Gesture: Thumb Up → Planning Extended")
        self.root.after(0, self.plan_extended)

    def _gesture_move_to_hand(self, gesture):
        """Gesture callback: Plan to current hand position"""
        if not self.hand_tracker:
            return
        
        # Get normalized pose
        pose_norm = self.hand_tracker.get_normalized_pose()
        if pose_norm is None:
            self.log_action(" Gesture: No valid hand position")
            return
        
        x_norm, y_norm, z_norm = pose_norm
        
        # Map to robot workspace (adjust these mappings based on your setup)
        x = 0.05 + x_norm * 0.60  # 0.05 to 0.65m
        y = -0.40 + (1.0 - y_norm) * 0.80  # -0.40 to 0.40m (inverted Y)
        z = 0.05 + z_norm * 0.55  # 0.05 to 0.60m
        
        self.log_action(f" Gesture: Planning to hand position ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Update Cartesian sliders in GUI thread
        def update_and_plan():
            if hasattr(self, 'xyz_vars'):
                self.xyz_vars['X'].set(x)
                self.xyz_vars['Y'].set(y)
                self.xyz_vars['Z'].set(z)
                self.xyz_values_display['X'].config(text=f"{x:.3f}")
                self.xyz_values_display['Y'].config(text=f"{y:.3f}")
                self.xyz_values_display['Z'].config(text=f"{z:.3f}")
            
            # Switch to Cartesian mode if needed
            if self.control_mode.get() != "cartesian":
                self.control_mode.set("cartesian")
                self.on_control_mode_changed()
        
        self.root.after(0, update_and_plan)






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
                self.log_action("Socket server started on port 5000")
            except Exception as e:
                self.log_action(f" Socket server failed: {e}")
        
        threading.Thread(target=setup_server, daemon=True).start()









    def _setup_gesture_controls(self):
            """Map hand gestures to robot actions"""
            if not self.hand_tracker:
                print("[GUI]  Hand tracker not available")
                return
            
            tracker = self.hand_tracker
            
            # Register gesture actions
            tracker.register_gesture_callback("Open_Palm", self._gesture_home)
            tracker.register_gesture_callback("Closed_Fist", self._gesture_stop)
            tracker.register_gesture_callback("Thumb_Up", self._gesture_extended)
            tracker.register_gesture_callback("Thumb_Down", self._gesture_move_to_hand)
            
            print("[GUI] Gesture controls registered:")
            print("  - Open_Palm → Move to Home")
            print("  - Closed_Fist → Emergency Stop")
            print("  - Thumb_Up → Move to Extended")
            print("  - Thumb_Down → Plan to hand position")






    def _gesture_move_to_position(self):
        """Move robot to current hand position"""
        if not hasattr(self.socket_server, 'hand_tracker'):
            return
        
        pose = self.socket_server.hand_tracker.get_normalized_pose()
        if pose:
            x_norm, y_norm, z_norm = pose
            # Map to robot workspace
            x = 0.05 + x_norm * 0.60  # 0.05 to 0.65m
            y = -0.40 + y_norm * 0.80  # -0.40 to 0.40m
            z = 0.05 + z_norm * 0.55  # 0.05 to 0.60m
            
            self.log_action(f"🤚 Gesture control: Moving to hand position")
            # Plan to Cartesian pose
            if self.commander:
                result = self.commander.plan_to_cartesian_pose(x, y, z, 0.0, 0.0, 0.0)
                if result.get('success'):
                    self.commander.execute_live_trajectory(result)





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
        self.log_action("📐 Cartesian sliders setup complete (RPY: Enabled)")



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
        return False, MoveItErrorCodes.FAILURE

    def move_to_pose(self, pose, velocity):
        """Wrapper for moving to a Cartesian pose."""
        if self.commander and hasattr(self.commander, 'move_to_pose'):
            return self.commander.move_to_pose(pose, velocity)
        elif self.commander and hasattr(self.commander, 'plan_to_cartesian_pose'):
            # Fallback: use plan_to_cartesian_pose if available
            x, y, z = pose[0:3]
            roll, pitch, yaw = pose[3:6] if len(pose) >= 6 else (0.0, 0.0, 0.0)
            result = self.commander.plan_to_cartesian_pose(x, y, z, roll, pitch, yaw)
            return result.get('success', False), MoveItErrorCodes.SUCCESS if result.get('success') else MoveItErrorCodes.FAILURE
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
        if self.commander and hasattr(self.commander, 'current_goal_handle') and self.commander.current_goal_handle:
            try:
                self.commander.current_goal_handle.cancel_goal_async()
                self.log_action("Cancelled active goal")
            except Exception as e:
                self.log_action(f"Error cancelling goal: {e}")

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

        # Ensure control_mode is initialized (fallback if not in __init__)
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
            if not self.commander:
                self.log_action("Commander not available")
                return
            
            pose = self.commander.get_current_pose()
            if pose is None:
                self.log_action("Cannot read pose: TF not available")
                messagebox.showwarning("Warning", "TF transform not available. Check robot state publisher.")
                return
            
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
        if not self.commander:
            self.log_action("✗ Commander not available - planning aborted")
            messagebox.showerror("Error", "Commander not initialized. Please wait.")
            return

        mode = self.control_mode.get()
        self.log_action(f" Initiating {mode.upper()} planning...")

        try:
            if mode == "joint":
                target_joints = [var.get() for var in self.joint_vars]
                self.log_action(f" Joint targets: {[f'{v:.3f}' for v in target_joints]}")
                result = self.commander.plan_to_joint_values(target_joints)

                if result.get('success'):
                    self.commander.execute_live_trajectory(result)
                    self.log_action(" Joint planning SUCCEEDED and executed")
                else:
                    error_msg = result.get('message', 'Unknown error')
                    self.log_action(f"✗ Joint planning FAILED: {error_msg}")
                    messagebox.showerror("Planning Error", error_msg)

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
                    self.log_action(f"Targets clamped: ({x:.3f},{y:.3f},{z:.3f}) → ({x_c:.3f},{y_c:.3f},{z_c:.3f}) | RPY: ({roll:.3f},{pitch:.3f},{yaw:.3f}) → ({r_c:.3f},{p_c:.3f},{y_c:.3f})")

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
                result = self.commander.plan_to_cartesian_pose(x_c, y_c, z_c, r_c or 0.0, p_c or 0.0, y_c or 0.0)

                if result.get('success'):
                    self.commander.execute_live_trajectory(result)
                    self.log_action(" Cartesian planning SUCCEEDED and executed")
                else:
                    error_msg = result.get('message', 'Unknown error')
                    self.log_action(f"✗ Cartesian planning FAILED: {error_msg}")
                    messagebox.showerror("Planning Error", error_msg)

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
        
        # Camera mode selection
        ttk.Label(control_frame, text="Feed Mode:").pack(side=tk.LEFT, padx=(20, 5))
        self.camera_mode = tk.StringVar(value="RGB")
        mode_frame = ttk.Frame(control_frame)
        mode_frame.pack(side=tk.LEFT, padx=5)
        
        # RGB, IR, Depth buttons
        self.rgb_button = ttk.Radiobutton(
            mode_frame, text="RGB", variable=self.camera_mode, 
            value="RGB", command=self.change_camera_mode
        )
        self.rgb_button.pack(side=tk.LEFT, padx=2)
        
        self.ir_button = ttk.Radiobutton(
            mode_frame, text="IR", variable=self.camera_mode, 
            value="IR", command=self.change_camera_mode
        )
        self.ir_button.pack(side=tk.LEFT, padx=2)
        
        self.depth_button = ttk.Radiobutton(
            mode_frame, text="Depth", variable=self.camera_mode, 
            value="Depth", command=self.change_camera_mode
        )
        self.depth_button.pack(side=tk.LEFT, padx=2)
        
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
        """Start the camera feed"""
        try:
            # Try to open camera (0 is usually the default webcam)
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                self.log_action("Failed to open camera")
                messagebox.showerror("Error", "Could not open camera. Please check if camera is connected.")
                return
            
            # Set camera properties for better performance
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            self.camera_active = True
            self.camera_running = True
            self.start_camera_button.config(text="Stop Live Feed", style="Danger.TButton")
            
            # Start camera thread
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            
            self.log_action("Camera feed started")
            
        except Exception as e:
            self.log_action(f"Error starting camera: {str(e)}")
            messagebox.showerror("Error", f"Failed to start camera: {str(e)}")

    def stop_camera_feed(self):
        """Stop the camera feed"""
        try:
            self.camera_running = False
            self.camera_active = False
            
            if self.camera_thread and self.camera_thread.is_alive():
                self.camera_thread.join(timeout=1.0)
            
            if self.camera:
                self.camera.release()
                self.camera = None
            
            self.start_camera_button.config(text="Start Live Feed", style="Success.TButton")
            self.camera_display.config(
                image="",
                text="Camera feed stopped\nClick 'Start Live Feed' to restart"
            )
            
            self.log_action("Camera feed stopped")
            
        except Exception as e:
            self.log_action(f"Error stopping camera: {str(e)}")

    def camera_loop(self):
        """Main camera loop running in separate thread"""
        while self.camera_running and self.camera:
            try:
                ret, frame = self.camera.read()
                if not ret:
                    self.log_action("Failed to read from camera")
                    break
                
                # Process frame based on selected mode
                processed_frame = self.process_camera_frame(frame)
                
                # Convert to PhotoImage for tkinter
                if processed_frame is not None:
                    # Resize frame to fit display
                    height, width = processed_frame.shape[:2]
                    max_width, max_height = 640, 480
                    
                    if width > max_width or height > max_height:
                        scale = min(max_width/width, max_height/height)
                        new_width = int(width * scale)
                        new_height = int(height * scale)
                        processed_frame = cv2.resize(processed_frame, (new_width, new_height))
                    
                    # Convert BGR to RGB for PIL
                    if len(processed_frame.shape) == 3:
                        rgb_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    else:
                        rgb_frame = processed_frame
                    
                    # Convert to PIL Image and then to PhotoImage
                    image = Image.fromarray(rgb_frame)
                    photo = ImageTk.PhotoImage(image)
                    
                    # Update display in main thread
                    self.root.after(0, self.update_camera_display, photo)
                
                # Control frame rate
                time.sleep(1/30)  # ~30 FPS
                
            except Exception as e:
                self.log_action(f"Camera loop error: {str(e)}")
                break
        
        # Cleanup when loop ends
        self.root.after(0, self.stop_camera_feed)

    def process_camera_frame(self, frame):
        """Process camera frame based on selected mode"""
        try:
            mode = self.camera_mode.get()
            
            if mode == "RGB":
                return frame
            elif mode == "IR":
                # Convert to grayscale to simulate IR (since we're using webcam)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Apply a color map to make it look more like IR
                ir_frame = cv2.applyColorMap(gray, cv2.COLORMAP_HOT)
                return ir_frame
            elif mode == "Depth":
                # Simulate depth using edge detection (since we're using webcam)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Use Canny edge detection
                edges = cv2.Canny(gray, 50, 150)
                # Apply color map to edges
                depth_frame = cv2.applyColorMap(edges, cv2.COLORMAP_JET)
                return depth_frame
            else:
                return frame
                
        except Exception as e:
            self.log_action(f"Frame processing error: {str(e)}")
            return frame

    def update_camera_display(self, photo):
        """Update camera display with new photo"""
        try:
            self.camera_display.config(image=photo, text="")
            self.camera_display.image = photo  # Keep a reference
        except Exception as e:
            self.log_action(f"Display update error: {str(e)}")

    def change_camera_mode(self):
        """Handle camera mode change"""
        mode = self.camera_mode.get()
        self.log_action(f"Camera mode changed to: {mode}")
        
        # Note: For actual depth cameras (like RealSense), you would
        # initialize different streams here instead of simulating

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
        """Log action to GUI action log (centralized logging)"""
        try:
            timestamp = time.strftime('%H:%M:%S')
            
            # Log to console for debugging
            print(f"[{timestamp}] {message}")
            sys.stdout.flush()
            
            # Log to GUI text widget
            if hasattr(self, 'log_text') and self.log_text:
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
                self.log_text.see(tk.END)
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
        """Handle continuous slider movement - send to controller"""
        if not hasattr(self, 'slider_active') or not self.slider_active[joint_index]:
            return
        
        try:
            joint_goal = [var.get() for var in self.joint_vars]
            joint_name = self.get_joint_names()[joint_index]
            joint_limits = self.get_joint_limits()
            min_limit, max_limit = joint_limits.get(joint_name, (-3.14, 3.14))
            joint_goal[joint_index] = np.clip(joint_goal[joint_index], min_limit, max_limit)
            
            # Update display label
            if joint_index < len(self.joint_values):
                self.joint_values[joint_index].config(text=f"{joint_goal[joint_index]:.3f}")
            
            # Store positions
            self.current_joint_positions = joint_goal.copy()
            
            # **NEW: Send simple trajectory to controller for real-time preview**
            if self.commander and hasattr(self.commander, '_publish_simple_preview'):
                self.commander._publish_simple_preview(joint_goal)
            
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
        if not self.commander:
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
                
        except Exception as e:
            self.log_action(f" Error moving to home: {str(e)}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to move to home: {str(e)}")

    def plan_extended(self):
        """Move to extended position"""
        if not self.commander:
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

class PegasusCommander(Node):  

    def __init__(self):
        print("[PegasusCommander] Calling super().__init__...")
        sys.stdout.flush()
        
        # Initialize node with minimal blocking
        try:
            super().__init__('pegasus_commander')
            print("[PegasusCommander] super().__init__() complete")
        except Exception as e:
            print(f"[PegasusCommander] Warning: super().__init__() raised: {e}")
        
        sys.stdout.flush()
        self.logger = self.get_logger()
        self.load_joint_limits()

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
            'home': [0.0, -0.26, 0.0, 0.0, 0.0],
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
        self.moveit_enabled = False

        # Arduino serial setup
        self.arduino_serial = None
        self.arduino_port = portname
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
                self.get_logger().error(f" Trajectory ABORTED: {error_str}")
            elif status == 6:  # PREEMPTED
                self.get_logger().warn(" Trajectory PREEMPTED (cancelled)")
            else:
                self.get_logger().warn(f" Trajectory ended with status {status}")
            
        except Exception as e:
            self.get_logger().error(f" Result callback error: {str(e)}")
        finally:
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

    def plan_to_cartesian_pose(self, x, y, z, roll=None, pitch=None, yaw=None, show_in_rviz=True):
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
            result = self.plan_to_joint_state(joint_solution)
            
            if not result['success']:
                return result
            
            # Show in RViz if requested
            if show_in_rviz and 'trajectory' in result:
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





    def plan_to_cartesian_pose(self, x, y, z, roll, pitch, yaw):
        """Plan and execute a motion to a Cartesian pose using MoveIt IK and motion planning."""
        self.get_logger().info(
            f"Planning to Cartesian pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}"
        )

        # 1️ Solve IK first
        joint_solution = self.solve_ik(x, y, z, roll, pitch, yaw)
        if joint_solution is None:
            msg = "IK solver failed — pose unreachable."
            self.get_logger().error(msg)
            return {"success": False, "message": msg}

        # 2️ Validate within joint limits
        for i, name in enumerate(self.joint_names):
            low, high = self.joint_limits.get(name, (-6.28, 6.28))
            if not (low <= joint_solution[i] <= high):
                self.get_logger().warn(f"Joint {name} out of limits: {joint_solution[i]:.3f}")
                joint_solution[i] = max(min(joint_solution[i], high), low)

        # 3️ Call your existing joint planner
        result = self.plan_to_joint_values(joint_solution)

        # 4️Verify and visualize
        if result["success"]:
            try:
                # Publish JointTrajectory directly to trajectory_pub for RViz visualization
                if "raw_trajectory" in result and result["raw_trajectory"] is not None:
                    self.trajectory_pub.publish(result["raw_trajectory"])
                    self.get_logger().info("Published trajectory for visualization")
            except Exception as e:
                self.get_logger().debug(f"Could not publish trajectory: {e}")

            # 5️ Execute
            self.execute_live_trajectory(result)
            self.get_logger().info("Trajectory execution started.")
            return {"success": True, "message": "Cartesian plan executed successfully"}
        else:
            self.get_logger().error(f"Cartesian planning failed: {result['message']}")
            return result


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






    def initialize_services_and_topics(self):
        # Skip blocking waits - just create clients without waiting
        # The actual availability will be checked asynchronously via the timer
        self.controller_manager_available = False

        # === Action Clients ===
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_action_client = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')

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



    def plan_to_joint_values(self, joint_values_str):
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
            total_time = 2.0  # seconds (adjustable)
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

    gui = None
    commander = None
    spin_thread = None
    
    try:
        print("[MAIN] Step 1: Initializing ROS...")
        sys.stdout.flush()
        rclpy.init(args=args)
        print("[MAIN] ✓ ROS initialized")
        sys.stdout.flush()
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
        
        print("[MAIN] Step 3: Initializing PegasusArmGUI (without commander)...")
        sys.stdout.flush()
        gui = PegasusArmGUI(root, wait_for_services=False)
        print("[MAIN] ✓ PegasusArmGUI initialized")
        sys.stdout.flush()
        
        # Show window after GUI is initialized
        print("[MAIN] Step 4: Showing window...")
        sys.stdout.flush()
        root.deiconify()  # Show the window
        print("[MAIN] ✓ Window visible")
        sys.stdout.flush()
        
        # Create commander in a separate thread to avoid blocking GUI
        print("[MAIN] Step 5: Creating ROS Commander node in background thread...")
        sys.stdout.flush()
        def init_commander():
            try:
                global commander
                commander = PegasusCommander()
                gui.commander = commander
                print("[MAIN] ✓ PegasusCommander created and linked to GUI")
                sys.stdout.flush()
                # Start ROS spin thread
                spin_thread = threading.Thread(target=ros_spin_thread, args=(commander,), daemon=True)
                spin_thread.start()
                print("[MAIN] ✓ ROS spin thread started")
                sys.stdout.flush()
            except Exception as e:
                print(f"[MAIN] ✗ Error initializing commander: {e}")
                import traceback
                traceback.print_exc()
                sys.stdout.flush()
        
        commander_thread = threading.Thread(target=init_commander, daemon=True)
        commander_thread.start()
        print("[MAIN] ✓ Commander initialization thread started")
        sys.stdout.flush()
        
        print("[MAIN] Step 6: Launching Tkinter mainloop...")
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
            if 'commander' in globals() and commander:
                commander.destroy_node()
                print("[MAIN] ✓ Commander node destroyed")
        except Exception as e:
            print(f"[MAIN] Error destroying commander: {e}")
        
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