#!/usr/bin/env python3

import socket
import threading
import json
import logging
from enum import Enum
from typing import Callable, Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, field, asdict


class CommandID(Enum):
    """Enumeration of remote command IDs"""
    MOVE_JOINTS = 1
    MOVE_HOME = 2
    MOVE_EXTENDED = 3
    MOVE_CARTESIAN = 4
    EMERGENCY_STOP = 5
    SET_VELOCITY = 6
    CANCEL_GOAL = 7


@dataclass
class RemoteCommand:
    """Data class for remote commands"""
    command_id: int
    joint_values: Optional[List[float]] = None
    cartesian_pose: Optional[List[float]] = None
    velocity: Optional[float] = None
    
    @classmethod
    def from_dict(cls, data: dict) -> 'RemoteCommand':
        """Create RemoteCommand from dictionary"""
        return cls(
            command_id=data.get('command_id'),
            joint_values=data.get('joint_values'),
            cartesian_pose=data.get('cartesian_pose'),
            velocity=data.get('velocity')
        )


class SocketServer:
    """Remote control server for robot commands"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 5000):
        """Initialize socket server
        
        Args:
            host: Server host address (default: 0.0.0.0)
            port: Server port (default: 5000)
        """
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.server_thread = None
        self.client_threads: Dict[int, threading.Thread] = {}
        self.client_id_counter = 0
        self.client_lock = threading.Lock()
        
        # Command handlers
        self.command_handlers: Dict[int, Callable] = {}
        
        # Callbacks
        self.state_callback: Optional[Callable] = None
        self.client_connected_callback: Optional[Callable] = None
        self.client_disconnected_callback: Optional[Callable] = None
        
        # Logger
        self.logger = logging.getLogger('SocketServer')
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)
    
    def register_command_handler(self, command_id: int, handler: Callable) -> None:
        """Register a handler for a specific command ID
        
        Args:
            command_id: Command ID to handle
            handler: Callable that handles the command
        """
        self.command_handlers[command_id] = handler
        self.logger.info(f"Registered handler for command ID {command_id}")
    
    def set_state_callback(self, callback: Callable) -> None:
        """Set callback for getting robot state
        
        Args:
            callback: Callable that returns robot state dict
        """
        self.state_callback = callback
    
    def set_client_connected_callback(self, callback: Callable) -> None:
        """Set callback for client connections
        
        Args:
            callback: Callable(client_id, address)
        """
        self.client_connected_callback = callback
    
    def set_client_disconnected_callback(self, callback: Callable) -> None:
        """Set callback for client disconnections
        
        Args:
            callback: Callable(client_id, address)
        """
        self.client_disconnected_callback = callback
    
    def start(self) -> None:
        """Start the socket server"""
        if self.running:
            self.logger.warn("Server already running")
            return
        
        self.running = True
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()
        self.logger.info(f"Socket server started on {self.host}:{self.port}")
    
    def stop(self) -> None:
        """Stop the socket server"""
        self.running = False
        
        # Close all client threads
        with self.client_lock:
            for thread in self.client_threads.values():
                if thread.is_alive():
                    thread.join(timeout=1.0)
            self.client_threads.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception as e:
                self.logger.error(f"Error closing server socket: {e}")
        
        self.logger.info("Socket server stopped")
    
    def _run_server(self) -> None:
        """Main server loop"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.logger.info(f"Server listening on {self.host}:{self.port}")
            
            while self.running:
                try:
                    self.server_socket.settimeout(1.0)
                    client_socket, client_address = self.server_socket.accept()
                    
                    # Assign client ID
                    with self.client_lock:
                        client_id = self.client_id_counter
                        self.client_id_counter += 1
                    
                    # Call connected callback
                    if self.client_connected_callback:
                        try:
                            self.client_connected_callback(client_id, client_address)
                        except Exception as e:
                            self.logger.error(f"Error in connected callback: {e}")
                    
                    # Start client handler thread
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_id, client_socket, client_address),
                        daemon=True
                    )
                    
                    with self.client_lock:
                        self.client_threads[client_id] = client_thread
                    
                    client_thread.start()
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        self.logger.error(f"Accept error: {e}")
        
        except Exception as e:
            self.logger.error(f"Server error: {e}")
        finally:
            self.running = False
    

    def _handle_client(self, client_id: int, client_socket: socket.socket, client_address: Tuple) -> None:
        """Handle individual client connection"""
        try:
            self.logger.info(f"Client {client_id} connected from {client_address}")
            client_socket.settimeout(1.0)  # Shorter timeout for line reading

            buffer = b''  # Buffer for partial messages
            while self.running:
                try:
                    # Read data (handle partial messages)
                    chunk = client_socket.recv(4096)
                    if not chunk:
                        break
                    buffer += chunk

                    # Process complete lines (JSON + \n)
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line = line.decode('utf-8').strip()
                        if not line:
                            continue

                        # Parse JSON
                        try:
                            parsed_data = json.loads(line)
                            
                            # Check if it's hand tracking data (by 'type')
                            if 'type' in parsed_data and parsed_data['type'] in ['position', 'gesture']:
                                # Use shared hand tracker (attached to server by GUI)
                                hand_tracker = getattr(self, 'hand_tracker', None)
                                
                                if hand_tracker:
                                    # Parse hand tracking data
                                    hand_tracker.parse_message(parsed_data)
                                    # Send acknowledgment
                                    response = {'success': True, 'type': 'hand_tracking_ack'}
                                    client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                                    self.logger.debug(f"Hand tracking ack sent to client {client_id}")
                                else:
                                    # Hand tracker not initialized yet
                                    self.logger.warn("Hand tracker not attached - ignoring hand tracking data")
                                    response = {'success': False, 'error': 'Hand tracker not initialized'}
                                    client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                            else:
                                # Assume it's a remote command
                                command = RemoteCommand.from_dict(parsed_data)
                                response = self._execute_command(command)
                                client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                                self.logger.debug(f"Command response sent to client {client_id}: {response}")
                        
                        except json.JSONDecodeError as e:
                            self.logger.warn(f"Invalid JSON from client {client_id}: {e}")
                            response = {'success': False, 'error': 'Invalid JSON format'}
                            client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))
                
                except socket.timeout:
                    continue
                except (ConnectionResetError, BrokenPipeError) as e:
                    self.logger.info(f"Client {client_id} connection closed: {e}")
                    break
                except Exception as e:
                    self.logger.error(f"Error handling message from client {client_id}: {e}")
                    break
            
            self.logger.info(f"Client {client_id} disconnected")
            
            # Call disconnected callback
            if self.client_disconnected_callback:
                try:
                    self.client_disconnected_callback(client_id, client_address)
                except Exception as e:
                    self.logger.error(f"Error in disconnected callback: {e}")
                
        except Exception as e:
            self.logger.error(f"Unexpected error in client handler {client_id}: {e}")
        finally:
            try:
                client_socket.close()
            except Exception:
                pass
            # Clean up thread reference
            with self.client_lock:
                self.client_threads.pop(client_id, None)
    
    def _execute_command(self, command: RemoteCommand) -> Dict[str, Any]:
        """Execute a remote command
        
        Args:
            command: RemoteCommand to execute
            
        Returns:
            Response dictionary
        """
        try:
            # Check if handler exists
            if command.command_id not in self.command_handlers:
                return {
                    'success': False,
                    'error': f'Unknown command ID: {command.command_id}'
                }
            
            # Execute handler
            handler = self.command_handlers[command.command_id]
            response = handler(command)
            
            return response if response else {'success': True}
        
        except Exception as e:
            self.logger.error(f"Command execution error: {e}")
            return {'success': False, 'error': str(e)}
    
    def get_robot_state(self) -> Dict[str, Any]:
        """Get current robot state via callback
        
        Returns:
            Robot state dictionary
        """
        if self.state_callback:
            try:
                return self.state_callback()
            except Exception as e:
                self.logger.error(f"Error getting robot state: {e}")
                return {'connected': False, 'error': str(e)}
        
        return {'connected': False, 'error': 'No state callback registered'}


# ============================================================================
# HAND TRACKING DATA PARSER
# Parses Kinect-based hand position and gesture data from socket stream
# ============================================================================

from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Callable

@dataclass
class HandPose:
    """Represents hand position from Kinect camera"""
    x: float  # Pixel coordinate (0-1920)
    y: float  # Pixel coordinate (0-1080)
    z: float  # Depth in meters (0.5-3.0)
    timestamp: Optional[float] = None  # Optional timestamp
    
    def is_valid(self) -> bool:
        """Check if pose values are within reasonable bounds"""
        return (0 <= self.x <= 1920 and 
                0 <= self.y <= 1080 and 
                0.3 <= self.z <= 5.0)  # Allow slight extension beyond typical


@dataclass
class HandGesture:
    """Represents hand gesture classification"""
    gesture: str  # e.g., "Open_Palm", "Closed_Fist", "Thumb_Up", "None"
    timestamp: Optional[float] = None
    
    def is_detected(self) -> bool:
        """Check if hand is actually detected (not 'None')"""
        return self.gesture != "None" and self.gesture != ""


class HandTracker:
    """Parse and manage hand tracking data from Kinect socket stream"""
    
    def __init__(self):
        self.current_pose: Optional[HandPose] = None
        self.current_gesture: Optional[HandGesture] = None
        self.pose_lock = threading.Lock()
        self.gesture_lock = threading.Lock()
        self.logger = logging.getLogger('HandTracker')
        
        # Gesture callbacks for control actions
        self.gesture_callbacks: Dict[str, Callable] = {}
    
    def parse_message(self, data: dict) -> bool:
        """Parse incoming message and classify as pose or gesture
        
        Args:
            data: Parsed JSON data from socket
            
        Returns:
            True if successfully parsed, False if malformed
        """
        try:
            # Check if it's a POSE message (contains position fields)
            if 'position' in data:
                return self._parse_pose_message(data)
            
            # Check if it's a GESTURE message (contains data string)
            elif 'data' in data and isinstance(data.get('data'), str):
                return self._parse_gesture_message(data)
            
            else:
                self.logger.debug(f"Unknown message format: {data}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error parsing message: {e}")
            return False
    
    def _parse_pose_message(self, data: dict) -> bool:
        """Parse hand position message (PoseStamped)
        
        Expected format:
        {
            "position": {"x": float, "y": float, "z": float},
            "header": {"stamp": {"sec": int, "nanosec": int}},
            "frame_id": "kinect2_rgb_optical_frame"
        }
        """
        try:
            pos = data.get('position', {})
            x = float(pos.get('x', 0))
            y = float(pos.get('y', 0))
            z = float(pos.get('z', 0))
            
            # Extract optional timestamp
            timestamp = None
            header = data.get('header', {})
            stamp = header.get('stamp', {})
            if stamp:
                timestamp = stamp.get('sec', 0) + stamp.get('nanosec', 0) * 1e-9
            
            # Create pose object
            pose = HandPose(x=x, y=y, z=z, timestamp=timestamp)
            
            # Validate
            if not pose.is_valid():
                self.logger.warn(f"Invalid pose values: x={x}, y={y}, z={z}")
                return False
            
            # Store (thread-safe)
            with self.pose_lock:
                self.current_pose = pose
            
            self.logger.debug(f"Pose updated: x={x:.1f}, y={y:.1f}, z={z:.2f}m")
            return True
            
        except Exception as e:
            self.logger.error(f"Error parsing pose message: {e}")
            return False
    
    def _parse_gesture_message(self, data: dict) -> bool:
        """Parse hand gesture message (String)
        
        Expected format:
        {
            "data": "Open_Palm" | "Closed_Fist" | "Thumb_Up" | "Thumb_Down" | "None"
        }
        """
        try:
            gesture_str = str(data.get('data', 'None')).strip()
            
            # Extract optional timestamp
            timestamp = None
            header = data.get('header', {})
            stamp = header.get('stamp', {})
            if stamp:
                timestamp = stamp.get('sec', 0) + stamp.get('nanosec', 0) * 1e-9
            
            # Create gesture object
            gesture = HandGesture(gesture=gesture_str, timestamp=timestamp)
            
            # Store (thread-safe)
            with self.gesture_lock:
                self.current_gesture = gesture
            
            self.logger.info(f"Gesture detected: {gesture_str}")
            
            # Trigger callback if registered
            if gesture_str in self.gesture_callbacks:
                try:
                    self.gesture_callbacks[gesture_str](gesture)
                except Exception as e:
                    self.logger.error(f"Error in gesture callback: {e}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error parsing gesture message: {e}")
            return False
    
    def get_current_pose(self) -> Optional[HandPose]:
        """Get latest hand pose (thread-safe)"""
        with self.pose_lock:
            return self.current_pose
    
    def get_current_gesture(self) -> Optional[HandGesture]:
        """Get latest hand gesture (thread-safe)"""
        with self.gesture_lock:
            return self.current_gesture
    


    def get_normalized_pose(self, screen_width: int = 1920, 
                           screen_height: int = 1080) -> Optional[Tuple[float, float, float]]:
        """Get hand pose normalized to [0, 1] range
        
        Useful for UI cursor control
        
        Args:
            screen_width: Kinect RGB frame width (default 1920)
            screen_height: Kinect RGB frame height (default 1080)
            
        Returns:
            Tuple of (x_norm, y_norm, z_norm) in range [0, 1] or None
        """
        pose = self.get_current_pose()
        if pose is None:
            return None
        
        try:
            x_norm = max(0, min(1, pose.x / screen_width))
            y_norm = max(0, min(1, pose.y / screen_height))
            # Normalize depth: assume useful range is 0.5m to 2.0m (150cm working range)
            z_norm = max(0, min(1, (pose.z - 0.5) / 1.5))
            return (x_norm, y_norm, z_norm)
        except Exception as e:
            self.logger.error(f"Error normalizing pose: {e}")
            return None




    def register_gesture_callback(self, gesture: str, callback: Callable) -> None:
        """Register callback for specific gesture
        
        Args:
            gesture: Gesture string to listen for
            callback: Function(gesture: HandGesture) to call when detected
        """
        self.gesture_callbacks[gesture] = callback
        self.logger.info(f"Registered callback for gesture: {gesture}")
    
    def is_hand_detected(self) -> bool:
        """Check if hand is currently detected
        
        Returns:
            True if both pose and gesture are valid (and gesture != 'None')
        """
        pose = self.get_current_pose()
        gesture = self.get_current_gesture()
        return pose is not None and gesture is not None and gesture.is_detected()


if __name__ == '__main__':
    # Test the socket server
    logging.basicConfig(level=logging.INFO)
    
    def test_handler(command: RemoteCommand) -> Dict:
        print(f"Received command: {command}")
        return {'success': True, 'message': 'Command received'}
    
    server = SocketServer('localhost', 5000)
    server.register_command_handler(CommandID.MOVE_JOINTS.value, test_handler)
    server.start()
    
    try:
        while True:
            threading.Event().wait(1.0)
    except KeyboardInterrupt:
        server.stop()
        print("Server stopped")