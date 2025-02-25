import rclpy
from rclpy.node import Node
import smach
from ai_robot_book_interfaces.srv import StringCommand


class MainSM2(Node):
    def __init__(self):
        super().__init__('main_sm2')

    def execute(self):
        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            smach.StateMachine.add(
                "VOICE", Voice(self),
                transitions={"succeeded": "NAVIGATION", "failed": "VOICE"})
            smach.StateMachine.add(
                "NAVIGATION", Navigation(self),
                transitions={"succeeded": "VISION", "failed": "NAVIGATION"})
            smach.StateMachine.add(
                "VISION", Vision(self),
                transitions={"succeeded": "MANIPULATION", "failed": "VISION"})
            smach.StateMachine.add(
                "MANIPULATION", Manipulation(self),
                transitions={"failed": "VISION", "exit": "succeeded"})
        outcome = sm_top.execute()
        self.get_logger().info(f'outcom: {outcome}')


class Voice(smach.State):
    def __init__(self, node):
        super().__init__(
            output_keys=["target_object"],
            outcomes=["succeeded", "failed"])

        # Create node
        self.node = node
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'voice/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info(self.__class__.__name__ + ' サービスが無効です．再び待機します...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info("音声認識を開始せよ")

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

        if response.answer == "succeeded":
            return True
        else:
            return False


class Navigation(smach.State):
    def __init__(self, node):
        super().__init__(
            input_keys=["target_object"],
            outcomes=["succeeded", "failed"])

        # Define logger
        self.node = node
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'navigation/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info(self.__class__.__name__ + ' サービスが無効です．再び待機します...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info("ナビゲーションを開始せよ")

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

        if response.answer == "succeeded":
            return True
        else:
            return False


class Vision(smach.State):
    def __init__(self, node):
        super().__init__(
            input_keys=["target_object"],
            output_keys=["target_object_pos"],
            outcomes=["succeeded", "failed"])

        # Define logger
        self.node = node
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'vision/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info(self.__class__.__name__ + ' サービスが無効です．再び待機します...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info("対処物体の位置検出を開始せよ")

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

        if response.answer == "succeeded":
            return True
        else:
            return False


class Manipulation(smach.State):
    def __init__(self, node):
        super().__init__(
            input_keys=["target_object_pos"],
            outcomes=["exit", "failed"])

        # Define logger
        self.node = node
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'vision/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info(self.__class__.__name__ + ' サービスが無効です．再び待機します...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('対象物体の把持を開始せよ')

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

        if response.answer == "succeeded":
            return True
        else:
            return False


def main():
    rclpy.init()
    node = MainSM2()
    node.execute()


if __name__ == '__main__':
    main()
