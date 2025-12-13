#!/usr/bin/env python3

"""
Service client example for ROS 2
This node calls a service provided by a server
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

        # Log initialization
        self.get_logger().info('Service Client Node Initialized')

    def send_request(self, a, b):
        """
        Send a request to the service
        Args:
            a: First integer
            b: Second integer
        Returns:
            The response from the service
        """
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)

        # Log the request
        self.get_logger().info(f'Sending request: {a} + {b}')

        # Return the future object
        return self.future


def main(args=None):
    """Main function that initializes the node and sends requests"""
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the client node
    minimal_client = MinimalClient()

    # Check if command line arguments are provided
    if len(sys.argv) == 3:
        try:
            # Parse the command line arguments
            a = int(sys.argv[1])
            b = int(sys.argv[2])
        except ValueError:
            minimal_client.get_logger().error('Invalid arguments. Please provide two integers.')
            return
    else:
        # Use default values if no arguments provided
        a = 1
        b = 2
        minimal_client.get_logger().info('Using default values: a=1, b=2')

    try:
        # Send the request
        future = minimal_client.send_request(a, b)

        # Wait for the response
        rclpy.spin_until_future_complete(minimal_client, future)

        # Get the response
        response = future.result()

        # Log the result
        minimal_client.get_logger().info(f'Result: {a} + {b} = {response.sum}')

    except KeyboardInterrupt:
        minimal_client.get_logger().info('Interrupted by user')
    finally:
        # Clean up and shut down
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()