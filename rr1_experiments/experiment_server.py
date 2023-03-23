#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rr1_interfaces.srv import Experiment


class ExperimentServiceServerNode(Node):
    """Experiment service server Python Node that will simply return the request message as a response."""
    def __init__(self):
        super().__init__("experiment_server_py")

        self.server_ = self.create_service(
            Experiment, "experiment_service", self.callback_service_call)
        self.get_logger().info("Experiment service server node started. Waiting for requests...")
    
    def callback_service_call(self, request, response):
        """Runs when a service has been called with a request."""
        response.payload = request.payload
        return response


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = ExperimentServiceServerNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication

if __name__ == "__main__":
    main()
