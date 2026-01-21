#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('test_joint_state_pub')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_positions = [0.0, -0.26, -0.785, -1.27, 0.0]
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz

    def timer_cb(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.pub.publish(msg)
        # optional: small variation so transforms change slightly
        # self.joint_positions[0] += 0.0

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
