#!/usr/bin/env python3
import random, sys, time
import rclpy
from rclpy.node import Node
from functools import partial

from rr1_interfaces.srv import Experiment


class ExperimentServiceClientNode(Node):
    """A template for a service client Python Node."""
    def __init__(self):
        super().__init__("experiment_client_py")

        # Declaring a parameter with default value
        # The type is inferred from default value
        self.declare_parameter("timer_delta_ms", 1000)
        self.declare_parameter("logfile", "log/log.txt")
        self.declare_parameter("message_count", 10)
        self.declare_parameter("unity", False)
        self.declare_parameter("max_payload", 4096)

        self._timer_delta_s = self.get_parameter("timer_delta_ms").value / 1000
        self._logfile_path = self.get_parameter("logfile").value
        self._max_payload = self.get_parameter("max_payload").value
        self._is_unity_client = self.get_parameter("unity").value
        self._msg_cnt = self.get_parameter("message_count").value
        self._counter = 0

        self._logfile = open(self._logfile_path, "a")

        self._client = self.create_client(Experiment, "experiment_service")
        self.wait_for_server()  # Wait until the server is available

        self._timer = self.create_timer(self._timer_delta_s, self.experiment_service_call)
        self._payload = [(255).to_bytes(1, 'big')]

        self.get_logger().info("Experiment service client node started.")
        self.get_logger().info(f"Sending experimental service calls every {self._timer_delta_s} seconds...")
    

    def experiment_service_call(self):
        """Calls service server asynchronously with given arguments."""
        if self._counter >= self._msg_cnt:
            self._counter = 0
            self._payload += self._payload

            if len(self._payload) > self._max_payload:
                self._logfile.close()
                self.get_logger().info("Logging file closed.")
                
        # Create a request
        request = Experiment.Request()
        request.payload = self._payload

        # Call the service
        time_start = time.time()
        future = self._client.call_async(request)

        # Add a callback for the response
        future.add_done_callback(partial(self.callback_done, start=time_start))

        self._counter += 1

    def callback_done(self, future, start):
        """Runs when a response from the server is received.
        
        Note: The arguments for the request are included, so when the callback
        is called we know what the request has been.
        """
        try:
            response = future.result()
            exec_time_ms = round((time.time() - start) * 1000, 6)
                
            self._logfile.write("{},{},{}\n".format(self._is_unity_client,len(response.payload),exec_time_ms))
            self.get_logger().info("Execution time for {} bytes: {}ms".format(len(response.payload), exec_time_ms))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def wait_for_server(self):
        """Waits for the server in a loop."""
        while not self._client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = ExperimentServiceClientNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication

if __name__ == "__main__":
    main()
