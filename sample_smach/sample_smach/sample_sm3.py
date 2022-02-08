import time
import rclpy
from rclpy.node import Node
import smach


class StateMachine3(Node):
    def __init__(self):
        super().__init__('state_machine3')

    def execute(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome4'])
        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add(
                'FOO', Foo(self),
                transitions={'outcome1': 'BAR', 'outcome2': 'outcome4'})
            smach.StateMachine.add(
                'BAR', Bar(self),
                transitions={'outcome1': 'FOO'})

        # Execute SMACH plan
        outcome = sm.execute()
        self.get_logger().info(f'outcom: {outcome}')


# define state Foo
class Foo(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['outcome1', 'outcome2'])
        self.counter = 0
        self.logger = node.get_logger()

    def execute(self, userdata):
        self.logger.info('状態FOOを実行中')
        time.sleep(1)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['outcome1'])
        self.logger = node.get_logger()

    def execute(self, userdata):
        self.logger.info('状態BARを実行中')
        time.sleep(1)
        return 'outcome1'


def main():
    rclpy.init()
    node = StateMachine3()
    node.execute()


if __name__ == '__main__':
    main()
