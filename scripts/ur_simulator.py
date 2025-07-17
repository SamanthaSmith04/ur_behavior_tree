#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import ur_msgs
import ur_dashboard_msgs
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates, RobotModeDataMsg, Digital, Analog
from ur_dashboard_msgs.srv import GetRobotMode
from ur_dashboard_msgs.action import SetMode
from std_srvs.srv import Trigger

class RobotServiceNode(Node):
    def __init__(self):
        super().__init__('ur_simulator')

        self.create_service(SetIO, '/io_and_status_controller/set_io', self.set_io_callback)
        self.create_service(GetRobotMode, '/ur_dashboard_client/robot_mode', self.get_robot_mode_callback)
        self._action_server = ActionServer(
            self,
            ur_dashboard_msgs.action.SetMode,
            '/ur_dashboard_client/set_mode',
            self.set_mode_callback
        )
        self.io = {}

    async def set_io_callback(self, request, response):
        self.get_logger().info(f'Setting IO: {request.fun} {request.pin} {request.state}')

        response.success = True
        return response

    async def get_robot_mode_callback(self, request, response):
        self.get_logger().info('Getting robot mode')
        response.mode = RobotModeDataMsg()
        response.mode.robot_mode = RobotModeDataMsg.RUNNING
        response.answer = "Robot is running"
        response.success = True

    async def set_mode_callback(self, goal_handle):
        self.get_logger().info(f'Setting mode: {goal_handle.request.target_robot_mode}')
        goal_handle.succeed()
        result = ur_dashboard_msgs.action.SetMode.Result()
        result.success = True
        result.message = "Robot is set to RUNNING mode"
        return result

    

def main(args=None):
    rclpy.init(args=args)
    node = RobotServiceNode()
    node.get_logger().info('Started UR simulator')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
