"""
ROS 2 node that subscribes to a JointState topic, 
buffers the incoming messages to introduce an artificial time delay, 
injects Gaussian noise into the joint position measurements, 
and then republishes the modified messages through the /delayed_joint_states topic.
"""


# Import necessary ROS 2 and Python libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque
import numpy as np


class Injector(Node):
    def __init__(self):
        """
        Initializes the Injector node.
        Sets up ROS 2 parameters for delay and noise, creates the publisher 
        and subscriber, initializes the message buffer, and starts the timer 
        that checks for messages ready to be published.
        """
        super().__init__('injector')
        
        # Enforce the use of simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('delay_ms', 100)
        delay_ms = self.get_parameter('delay_ms').value
        self.delay_sec = delay_ms / 1000.0
        self.get_logger().info(f'Injector: {delay_ms} ms delay active')

        # Initialize a double-ended queue to act as a First-In-First-Out (FIFO) buffer
        self.buffer = deque()

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)
        self.pub = self.create_publisher(
            JointState, '/delayed_joint_states', 10)

        # Timer that triggers at 100Hz (every 0.01s) to process and publish due messages
        self.timer = self.create_timer(0.01, self.publish_due_messages)

        # Declare and fetch the measurement noise standard deviation
        self.declare_parameter('measurement_noise_std', 0.0316)  
        self.noise_std = self.get_parameter('measurement_noise_std').value



    def callback(self, msg):
        """
        Subscriber callback for incoming JointState messages to be buffered.
        Grabs the current simulation time and appends a tuple of (timestamp, message) 
        to the right side of the buffer.
        """
        # Switched from time.time() to self.get_clock().now() to get the current time in seconds using the node's clock
        now = self.get_clock().now().nanoseconds / 1e9

        self.buffer.append((now, msg))



    def publish_due_messages(self):
        """
        Timer callback that executes continuously.
        Checks the oldest messages in the buffer. If a message has been in the buffer 
        longer than the specified delay_sec, it applies Gaussian noise to the position 
        data and publishes it.
        """
        # Fix: Switched from time.time() to self.get_clock().now() to ensure the node respects ROS 2 simulation time (use_sim_time) instead of system wall time
        now = self.get_clock().now().nanoseconds / 1e9
        while self.buffer:
            # Look at the oldest message without removing it yet
            arrival_time, msg = self.buffer[0]
            
            if now - arrival_time >= self.delay_sec:
                # Fix: Added actual delay calculation to measure the exact time the message spent in the buffer
                actual = now - arrival_time

                # Fix: Added throttled logging to monitor the actual buffer delay against the target delay without overloading the console output
                self.get_logger().info(
                    f'buffered {actual:.3f}s (target {self.delay_sec:.3f}s)',
                    throttle_duration_sec=2.0
                )

                # Inject Gaussian noise into the position measurements
                if self.noise_std > 0.0:
                    noisy = list(msg.position)
                    noisy = [p + np.random.normal(0, self.noise_std) for p in noisy]
                    msg.position = noisy
                    
                # Publish the modified message and remove it from the buffer
                self.pub.publish(msg)
                self.buffer.popleft()
            else:
                break



def main():
    """
    Initialize and spin up the node
    and handle shutdowns.
    """
    rclpy.init()
    node = Injector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


