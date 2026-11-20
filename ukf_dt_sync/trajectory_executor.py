"""
ROS 2 Action Client node for commanding an N-DOF robotic manipulator through a
cyclic sequence of joint-space waypoints to generate continuous reference motion.

Reads joint names, flattened waypoints, and target action topic dynamically from
ROS 2 parameters and communicates with a FollowJointTrajectory Action Server
(e.g., /xarm7_controller/follow_joint_trajectory):
- Service Clients:
  - <controller_action_topic>/_action/send_goal
  - <controller_action_topic>/_action/get_result
  - <controller_action_topic>/_action/cancel_goal

- Subscribes to (Topics):
  - <controller_action_topic>/_action/feedback
  - <controller_action_topic>/_action/status
"""


# Import necessary ROS 2 libraries and message types
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter
import time


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        # Declare and read parameters
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('controller_action_topic', '/default_controller/follow_joint_trajectory')
        # Declare an empty array that strictly enforce that it must be populated with doubles
        self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_duration_sec', 1.5)
        
        self.joint_names = self.get_parameter('joint_names').value
        action_topic = self.get_parameter('controller_action_topic').value
        self.duration_sec = self.get_parameter('waypoint_duration_sec').value


        # Retrieve and reshape the flattened waypoints array
        flat_waypoints = self.get_parameter('waypoints').value
        dof = len(self.joint_names)
        
        # Chunk the 1D array into a list of lists using list comprehension
        self.waypoints = [flat_waypoints[i:i + dof] for i in range(0, len(flat_waypoints), dof)]

        
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            action_topic
        )
        self.get_logger().info('Waiting for trajectory action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server found. Starting trajectory loop.')
        time.sleep(5.0)
        self.get_logger().info('Starting trajectory loop.')

    def send_waypoint(self, positions, duration_sec=4.0):
        """Send a single waypoint and wait for completion."""
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points = [point]
        goal.trajectory = trajectory

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def run_loop(self):
        """Cycle through waypoints continuously."""
        self.get_logger().info('Trajectory loop started — Ctrl+C to stop')
        cycle = 0
        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f'Cycle {cycle}')
            for i, wp in enumerate(self.waypoints):
                if not rclpy.ok():
                    break
                self.get_logger().info(f'  Moving to waypoint {i+1}/{len(self.waypoints)}')
                self.send_waypoint(wp, duration_sec=self.duration_sec)
                time.sleep(0.3)  # brief pause between waypoints

def main():
    rclpy.init()
    node = TrajectoryExecutor()
    node.run_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
