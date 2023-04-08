#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from rr1_interfaces.msg import *

class ExperimentSubscriberNode(Node):
    def __init__(self):
        super().__init__("experiment_subscriber_py")

        # Declaring a parameter with default value
        # The type is inferred from default value
        self.declare_parameter("logfile", "log/log.txt")
        self.declare_parameter("unity", False)
        self.declare_parameter("payload_size", 2)
        self.declare_parameter("message_count", 10)

        self._logfile_path = self.get_parameter("logfile").value
        self._payload_size = 2**self.get_parameter("payload_size").value
        self._is_unity_client = self.get_parameter("unity").value
        self._msg_cnt = self.get_parameter("message_count").value

        self._counter = 0
        self._interface = globals()["Experiment{}B".format(self._payload_size)]
        self._logfile = open(self._logfile_path, "a")

        self._subscriber = self.create_subscription(self._interface, "experiment", self.experiment_callback, 1)
        self.get_logger().info("Experiment subscriber node started.")
    

    def experiment_callback(self, msg):
        exec_time_ms = round((time.time() - msg.timestamp) * 1000, 6)
        self._logfile.write("{},{},{}\n".format(self._is_unity_client,self._payload_size,exec_time_ms))

        self._counter += 1
        if self._counter >= self._msg_cnt:
            self._logfile.close()
            self.destroy_subscription(self._subscriber)
            self.get_logger().info("Experiment finished.")
            return


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = ExperimentSubscriberNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication

if __name__ == "__main__":
    main()
