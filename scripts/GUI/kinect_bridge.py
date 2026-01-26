#!/usr/bin/env python3
"""
Kinect Hand Tracking Bridge for Robot Control
Subscribes to /waypoint (hand position) and /hand_gesture topics
Forwards data to GUI socket server for gesture-based control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import socket
import json
import time
import logging
import threading


class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        
        # Setup logging
        self.logger = logging.getLogger('KinectBridge')
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # Socket connection parameters
        self.gui_host = 'localhost'  # Change if GUI is on different machine
        self.gui_port = 5000
        self.socket = None
        self.connected = False
        self.connection_lock = threading.Lock()
        
        # ROS subscriptions
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.position_callback,
            10
        )
        
        self.gesture_sub = self.create_subscription(
            String,
            '/hand_gesture',
            self.gesture_callback,
            10
        )
        
        # Connect to GUI socket server
        self.connect_to_gui()
        
        # Timer to reconnect if connection lost
        self.reconnect_timer = self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info("Kinect Bridge initialized")
        self.get_logger().info(f"Listening to /waypoint and /hand_gesture")
        self.get_logger().info(f"Forwarding to GUI at {self.gui_host}:{self.gui_port}")
    
    def connect_to_gui(self):
        """Establish socket connection to GUI"""
        try:
            with self.connection_lock:
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect((self.gui_host, self.gui_port))
                self.connected = True
                self.logger.info(f"✓ Connected to GUI at {self.gui_host}:{self.gui_port}")
        
        except Exception as e:
            self.connected = False
            self.logger.error(f"Failed to connect to GUI: {e}")
            self.logger.info("Will retry connection...")
    
    def check_connection(self):
        """Periodically check and reconnect if needed"""
        if not self.connected:
            self.logger.info("Attempting to reconnect to GUI...")
            self.connect_to_gui()
    
    

    def send_data(self, data: dict):
        """Send data to GUI socket server"""
        if not self.connected:
            return False
        
        try:
            with self.connection_lock:
                json_data = json.dumps(data).encode('utf-8')
                # Add newline delimiter for message boundary
                json_data += b'\n'
                self.socket.sendall(json_data)
                return True
        
        except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
            self.logger.warn(f"Connection lost: {e}")
            self.connected = False
            return False
        
        except Exception as e:
            self.logger.error(f"Error sending data: {e}")
            return False





    def position_callback(self, msg: PoseStamped):
        """Handle hand position data from /waypoint topic"""
        try:
            # Extract position data
            position_data = {
                'type': 'position',  # ADD THIS
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                }
            }
            
            # Send to GUI
            success = self.send_data(position_data)
            
            if success:
                self.logger.debug(
                    f"Position: x={msg.pose.position.x:.1f}, "
                    f"y={msg.pose.position.y:.1f}, "
                    f"z={msg.pose.position.z:.3f}m"
                )
        
        except Exception as e:
            self.logger.error(f"Error in position callback: {e}")
    
    def gesture_callback(self, msg: String):
        """Handle hand gesture data from /hand_gesture topic"""
        try:
            # Extract gesture data
            gesture_data = {
                'type': 'gesture',  # ADD THIS
                'data': msg.data  # "Open_Palm", "Closed_Fist", "Thumb_Up", etc.
            }
            
            # Send to GUI
            success = self.send_data(gesture_data)
            
            if success and msg.data != "None":
                self.logger.info(f"Gesture detected: {msg.data}")
        
        except Exception as e:
            self.logger.error(f"Error in gesture callback: {e}")




 
    def shutdown(self):
        """Clean shutdown"""
        self.logger.info("Shutting down Kinect Bridge...")
        with self.connection_lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
        self.connected = False


def main(args=None):
    rclpy.init(args=args)
    
    bridge = KinectBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Keyboard interrupt received")
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
