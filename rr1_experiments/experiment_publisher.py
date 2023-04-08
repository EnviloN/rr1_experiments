#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

from rr1_interfaces.msg import *

class ExperimentPublisherNode(Node):
    def __init__(self):
        super().__init__("experiment_publisher_py")

        self.declare_parameter("timer_delta_ms", 1000)
        self.declare_parameter("message_count", 10)
        self.declare_parameter("payload_size", 2)

        self._timer_delta_s = self.get_parameter("timer_delta_ms").value / 1000
        self._payload_size = 2**self.get_parameter("payload_size").value
        self._msg_cnt = self.get_parameter("message_count").value

        self._counter = 0
        time.sleep(5)
        self._interface = globals()["Experiment{}B".format(self._payload_size)]
        self._publisher = self.create_publisher(self._interface, "experiment", 1)

        self._timer = self.create_timer(self._timer_delta_s, self.experiment_call)
        self._payload = [(255).to_bytes(1, 'big')] * self._payload_size

        self.get_logger().info("Experiment publisher node started.")
        self.get_logger().info(f"Sending experimental messages every {self._timer_delta_s} seconds...")
    

    def experiment_call(self):
        if self._counter >= self._msg_cnt:
            self.destroy_publisher(self._publisher)
            self.destroy_timer(self._timer)
            # self.get_logger().info("Experiment finished.")
            return
                
        message = self._interface()
        message.payload = self._payload
        message.timestamp = time.time()

        self._publisher.publish(message)

        self._counter += 1


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = ExperimentPublisherNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication

if __name__ == "__main__":
    main()
