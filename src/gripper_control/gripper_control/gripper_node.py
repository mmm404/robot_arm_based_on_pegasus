import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import time

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        
        # Declare parameters
        self.declare_parameter('esp_ip', '192.168.4.1')
        self.esp_ip = self.get_parameter('esp_ip').value
        
        # Subscription
        self.subscription = self.create_subscription(
            String,
            '/right_gesture',
            self.gesture_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.last_command = None
        self.get_logger().info(f'Gripper Node started. ESP32 IP: {self.esp_ip}')

    def gesture_callback(self, msg):
        gesture = msg.data
        command = None
        
        if gesture == "Open_Palm":
            command = "open"
        elif gesture == "Closed_Fist":
            command = "close"
            
        if command and command != self.last_command:
            self.send_command(command)
            self.last_command = command

    def send_command(self, command):
        url = f"http://{self.esp_ip}/{command}"
        try:
            self.get_logger().info(f"Sending command: {command} to {url}")
            # Use a short timeout to prevent blocking the node for too long
            response = requests.get(url, timeout=2.0)
            if response.status_code == 200:
                self.get_logger().debug(f"Successfully sent {command}")
            else:
                self.get_logger().warn(f"Failed to send {command}. Status: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error sending command to ESP32: {e}")

def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
