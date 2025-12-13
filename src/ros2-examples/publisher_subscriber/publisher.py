#!/usr/bin/env python3

"""
Simple publisher example for ROS 2
This node publishes messages to a topic at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher with a topic name and queue size
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)

        # Create a timer to publish messages at regular intervals
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of published messages
        self.i = 0

        # Log initialization
        self.get_logger().info('Simple Publisher Node Initialized')

    def timer_callback(self):
        """Callback function that executes when the timer expires"""
        # Create a String message
        msg = String()
        msg.data = f'Hello Robot: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function that initializes the node and spins it"""
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the publisher node
    simple_publisher = SimplePublisher()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        simple_publisher.get_logger().info('Interrupted by user')
    finally:
        # Clean up and shut down
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()