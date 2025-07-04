#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import MultiThreadedExecutor
import sys
import time

class ControllerChecker(Node):
    def __init__(self):
        super().__init__('controller_checker')
        self.cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller_manager service...')
        self.get_logger().info('Controller manager service available')

    def check_controllers(self):
        req = ListControllers.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            for controller in future.result().controller:
                if controller.name == 'pegasus_arm_controller' and controller.state == 'active':
                    self.get_logger().info('pegasus_arm_controller is active')
                    return True
        self.get_logger().warn('pegasus_arm_controller is not active')
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ControllerChecker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        for _ in range(10):  # Retry up to 10 times
            if node.check_controllers():
                sys.exit(0)
            time.sleep(2.0)
        node.get_logger().error('Failed to verify pegasus_arm_controller active state')
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()