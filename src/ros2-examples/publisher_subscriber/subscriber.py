#!/usr/bin/env python3

"""
Simple subscriber example for ROS 2
This node subscribes to messages from a topic and processes them
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription to the 'robot_commands' topic
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.listener_callback,
            10  # Queue size
        )

        # Prevent unused variable warning
        self.subscription  # This line prevents linter warnings

        # Log initialization
        self.get_logger().info('Simple Subscriber Node Initialized')

    def listener_callback(self, msg):
        """
        Callback function that executes when a message is received
        Args:
            msg: The received message of type String
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Here you could add additional processing of the received message
        self.process_command(msg.data)

    def process_command(self, command):
        """
        Process the received command
        Args:
            command: The command string received from the publisher
        """
        # Example processing logic
        if 'Hello' in command:
            self.get_logger().info(f'Processing greeting command: {command}')
        else:
            self.get_logger().info(f'Processing unknown command: {command}')


def main(args=None):
    """Main function that initializes the node and spins it"""
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Interrupted by user')
    finally:
        # Clean up and shut down
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()