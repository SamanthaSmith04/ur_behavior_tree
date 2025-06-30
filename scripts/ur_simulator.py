#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import ur_msgs
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates, RobotModeDataMsg, Digital, Analog
from std_srvs.srv import Trigger

class RobotServiceNode(Node):
    def __init__(self):
        super().__init__('ur_simulator')

        self.create_service(SetIO, '/io_and_status_controller/set_io', self.set_io_callback)
        self.io = {}

    async def set_io_callback(self, request, response):
        self.get_logger().info(f'Setting IO: {request.fun} {request.pin} {request.state}')
        if request.fun == 1: # FUN_SET_DIGITAL_OUT
            self.io[request.pin] = Digital(state=request.state)
        elif request.fun == 2: # FUN_SET_FLAG
            self.io[request.pin] = Analog(state=request.state)
        elif request.fun == 3: # FUN_SET_ANALOG_OUT
            self.io[request.pin] = Analog(state=request.state)

        response.success = True
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotServiceNode()
    node.get_logger().info('Started UR simulator')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
