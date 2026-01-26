"""
CV Pegasus Bridge - Filters and normalizes Kinect hand tracking data for robot control.

This node interprets waypoints from handcv.py and publishes
normalized commands for the GUI_commander to execute.

Control Mode: Absolute (Direct Mapping)
- 30Hz Update Rate
- OneEuroFilter smoothing

Coordinate Mapping:
- Kinect X (horizontal) -> Robot Y (Left/Right)
- Kinect Y (vertical) -> Robot Z (Up/Down)
- Kinect Z (depth) -> Robot X (Forward/Back)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import numpy as np
import math

class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        """Initialize the one euro filter."""
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def filter(self, t, x):
        """Compute the filtered signal."""
        t_e = t - self.t_prev
        
        # Avoid division by zero
        if t_e <= 0.0:
            return self.x_prev

        # The filtered derivative of the signal.
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal.
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat

class CvPegasusBridge(Node):
    def __init__(self):
        super().__init__('cv_pegasus_bridge')
        
        # Robot workspace limits (meters)
        self.declare_parameter('x_limits', [0.05, 0.65])
        self.declare_parameter('y_limits', [-0.40, 0.40])
        self.declare_parameter('z_limits', [0.05, 0.60])
        
        # Kinect camera workspace (pixels/mm)
        self.declare_parameter('kinect_x_range', [0.0, 1920.0])
        self.declare_parameter('kinect_y_range', [0.0, 1080.0])
        self.declare_parameter('kinect_z_range', [0.5, 2.0])
        
        # Filter parameters (Maintained from "Smoothness" update)
        self.declare_parameter('filter_min_cutoff', 0.5)  # Reduced for smoothness
        self.declare_parameter('filter_beta', 0.05)       # Reduced to eliminate jitter
        self.declare_parameter('filter_d_cutoff', 1.0)
        
        self.x_limits = self.get_parameter('x_limits').value
        self.y_limits = self.get_parameter('y_limits').value
        self.z_limits = self.get_parameter('z_limits').value
        
        self.kinect_x_range = self.get_parameter('kinect_x_range').value
        self.kinect_y_range = self.get_parameter('kinect_y_range').value
        self.kinect_z_range = self.get_parameter('kinect_z_range').value
        
        self.filter_params = {
            'min_cutoff': self.get_parameter('filter_min_cutoff').value,
            'beta': self.get_parameter('filter_beta').value,
            'd_cutoff': self.get_parameter('filter_d_cutoff').value
        }
        
        # Calculate Robot Home (Center of Workspace)
        self.robot_home = [
            (self.x_limits[0] + self.x_limits[1]) / 2.0,
            (self.y_limits[0] + self.y_limits[1]) / 2.0,
            (self.z_limits[0] + self.z_limits[1]) / 2.0
        ]
        
        # Calculate Scaling Factors (Robot Range / Kinect Range)
        # Note: Directions are handled in the calculation logic
        self.scale_x = (self.y_limits[1] - self.y_limits[0]) / (self.kinect_x_range[1] - self.kinect_x_range[0])
        self.scale_y = (self.z_limits[1] - self.z_limits[0]) / (self.kinect_y_range[1] - self.kinect_y_range[0])
        self.scale_z = (self.x_limits[1] - self.x_limits[0]) / (self.kinect_z_range[1] - self.kinect_z_range[0])

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/waypoint', self.waypoint_callback, 10)
            
        # Publishers
        self.cartesian_cmd_pub = self.create_publisher(
            PoseStamped, '/hand_control/cartesian_command', 10)
        self.control_active_pub = self.create_publisher(
            Bool, '/hand_control/control_active', 10)
        
        # State variables
        self.current_waypoint = None
        self.last_waypoint_time = self.get_clock().now()
        self.kinect_ref_point = None # [x, y, z] of the first detected hand
        self.filters = {}
        
        # Timer (30Hz for smooth updates)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.get_logger().info(f"CV Pegasus Bridge initialized (Relative Control Mode). Home: {self.robot_home}")

    def waypoint_callback(self, msg):
        """Process incoming waypoint data from Kinect."""
        # Filter invalid depth values
        if msg.pose.position.z <= 0.0 or not np.isfinite(msg.pose.position.z):
            return
        
        self.current_waypoint = msg
        self.last_waypoint_time = self.get_clock().now()

    def timer_callback(self):
        """Control loop."""
        # Safety timeout (0.2s)
        time_diff = (self.get_clock().now() - self.last_waypoint_time).nanoseconds / 1e9
        if self.current_waypoint is None or time_diff > 0.2:
            self.control_active_pub.publish(Bool(data=False))
            self.kinect_ref_point = None # Reset reference on timeout
            return
            
        # Get current kinect position
        k_x = self.current_waypoint.pose.position.x
        k_y = self.current_waypoint.pose.position.y
        k_z = self.current_waypoint.pose.position.z
        
        # Set reference point if not set
        if self.kinect_ref_point is None:
            self.kinect_ref_point = [k_x, k_y, k_z]
            self.get_logger().info(f"Reference point set: {self.kinect_ref_point}")
            
        # Calculate Deltas
        d_x = k_x - self.kinect_ref_point[0]
        d_y = k_y - self.kinect_ref_point[1]
        d_z = k_z - self.kinect_ref_point[2]
        
        # Map to Robot Coordinates (Relative to Home)
        # Robot Y (Left/Right) <- -Delta Kinect X (Left is usually smaller X in image coords? 
        # Wait, usually X increases left to right in image. 
        # In previous code: x_norm 0->1. Robot Y: Max->Min. 
        # So Increasing Kinect X -> Decreasing Robot Y.
        # So Delta Robot Y = - Delta Kinect X * Scale
        raw_y = self.robot_home[1] - d_x * self.scale_x
        
        # Robot Z (Up/Down) <- -Delta Kinect Y (Y increases downwards in image)
        # Previous: y_norm 0->1. Robot Z: Max->Min.
        # So Increasing Kinect Y -> Decreasing Robot Z.
        # So Delta Robot Z = - Delta Kinect Y * Scale
        raw_z = self.robot_home[2] - d_y * self.scale_y
        
        # Robot X (Forward/Back) <- Delta Kinect Z (Depth)
        # Previous: z_norm 0->1. Robot X: Min->Max.
        # So Increasing Kinect Z -> Increasing Robot X.
        # So Delta Robot X = + Delta Kinect Z * Scale
        raw_x = self.robot_home[0] + d_z * self.scale_z
        
        # Clamp to limits
        raw_x = np.clip(raw_x, self.x_limits[0], self.x_limits[1])
        raw_y = np.clip(raw_y, self.y_limits[0], self.y_limits[1])
        raw_z = np.clip(raw_z, self.z_limits[0], self.z_limits[1])
        
        # Apply One Euro Filter
        t = self.get_clock().now().nanoseconds / 1e9
        if not self.filters:
            self.filters['x'] = OneEuroFilter(t, raw_x, **self.filter_params)
            self.filters['y'] = OneEuroFilter(t, raw_y, **self.filter_params)
            self.filters['z'] = OneEuroFilter(t, raw_z, **self.filter_params)
            x, y, z = raw_x, raw_y, raw_z
        else:
            x = self.filters['x'].filter(t, raw_x)
            y = self.filters['y'].filter(t, raw_y)
            z = self.filters['z'].filter(t, raw_z)
            
        # Publish Command
        cmd = PoseStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.pose.position.x = float(x)
        cmd.pose.position.y = float(y)
        cmd.pose.position.z = float(z)
        
        # Orientation (Fixed Down)
        cmd.pose.orientation.x = 1.0
        cmd.pose.orientation.y = 0.0
        cmd.pose.orientation.z = 0.0
        cmd.pose.orientation.w = 0.0
        
        self.cartesian_cmd_pub.publish(cmd)
        self.control_active_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = CvPegasusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()