#!/usr/bin/env python3

"""
Service server example for ROS 2
This node provides a service that can be called by clients
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service with service name and callback function
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Log initialization
        self.get_logger().info('Service Server Node Initialized')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that executes when the service is called
        Args:
            request: The service request containing two integers
            response: The service response that will contain the sum
        Returns:
            response: The response with the calculated sum
        """
        # Calculate the sum
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(f'Returning {response.sum} (from {request.a} + {request.b})')

        # Return the response
        return response


def main(args=None):
    """Main function that initializes the node and spins it"""
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the service server node
    minimal_service = MinimalService()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Interrupted by user')
    finally:
        # Clean up and shut down
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()