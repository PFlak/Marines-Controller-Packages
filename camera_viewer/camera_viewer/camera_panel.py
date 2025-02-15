import os
from collections import namedtuple
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.node import HIDDEN_NODE_PREFIX
from custom_interfaces.srv import CameraList


class CameraPanel(Node):

    def __init__(self):
        super().__init__('Camera_Panel')
        self.camera_list_cli = self.create_client(CameraList, 'CameraList')
        while not self.camera_list_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        print(self.get_camera_names_list())

    def get_camera_names_list(self) -> List[str]:
        future = self.camera_list_cli.call_async(CameraList.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().list


def main(args=None):
    rclpy.init(args=args)
    manager = CameraPanel()

    rclpy.spin(manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
