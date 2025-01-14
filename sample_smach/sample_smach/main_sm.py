#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import smach

from ai_robot_book_interfaces.srv import StringCommand



def main():
    rclpy.init()

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add("VOICE", Voice(), transitions={"succeeded": "NAVIGATION", "failed": "VOICE"})    
        smach.StateMachine.add("NAVIGATION", Navigation(), transitions={"succeeded": "VISION", "failed": "NAVIGATION"})
        smach.StateMachine.add("VISION", Vision(), transitions={"succeeded": "MANIPULATION", "failed": "VISION"})    
        smach.StateMachine.add("MANIPULATION", Manipulation(), transitions={"failed": "VISION", "exit": "succeeded"})    

    outcome = sm_top.execute()


class Voice(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=["target_object"], outcomes=["succeeded", "failed"])

        # Create node    
        self.node = Node("VoiceState")
        self.logger = self.node.get_logger()    
    
        self.cli = self.node.create_client(StringCommand, 'voice/command')    
    
        while not self.cli.wait_for_service(timeout_sec=1.0):    
            self.logger.info('service not available, waiting again...')    
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info("Start voice recognition")

        result = self.send_request()

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.req.command = "check"    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="succeeded":
            return True
        else:
            return False


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object"], outcomes=["succeeded", "failed"])

        # Define logger
        self.node = Node("NavigationState")
        self.logger = self.node.get_logger()    
    
        self.cli = self.node.create_client(StringCommand, 'navigation/command')    
    
        while not self.cli.wait_for_service(timeout_sec=1.0):    
            self.logger.info('service not available, waiting again...')    
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):

        result = self.send_request()

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.req.command = "check"    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="succeeded":
            return True
        else:
            return False


class Vision(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object"], output_keys=["target_object_pos"], outcomes=["succeeded", "failed"])

        # Define logger
        self.node = Node("VisionState")
        self.logger = self.node.get_logger()    
    
        self.cli = self.node.create_client(StringCommand, 'vision/command')    
    
        while not self.cli.wait_for_service(timeout_sec=1.0):    
            self.logger.info('service not available, waiting again...')    
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        result = self.send_request()

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.req.command = "check"    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="succeeded":
            return True
        else:
            return False


class Manipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object_pos"], outcomes=["exit", "failed"])

        # Define logger
        self.node = Node("ManipulationState")
        self.logger = self.node.get_logger()    
    
        self.cli = self.node.create_client(StringCommand, 'vision/command')    
    
        while not self.cli.wait_for_service(timeout_sec=1.0):    
            self.logger.info('service not available, waiting again...')    
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info(f"Try to grasp the target object")

        result = self.send_request()

        if result:
            return "exit"
        else:
            return "failed"

    def send_request(self):    
        self.req.command = "check"    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="succeeded":
            return True
        else:
            return False


if __name__ == '__main__':
    main()
