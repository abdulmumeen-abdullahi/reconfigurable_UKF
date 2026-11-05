"""
ROS 2 node that acts as an Action Client to control Ufractory xArm7 robotic arm. 
It cyclically commands the arm through a predefined set of joint-space waypoints to 
generate continuous movement.

Communication / Topics:
It communicates 
with the `/xarm7_controller/follow_joint_trajectory` Action Server.
- Publishes to: 
    - `/xarm7_controller/follow_joint_trajectory/_action/send_goal`
    - `/xarm7_controller/follow_joint_trajectory/_action/cancel_goal`
- Subscribes to: 
    - `/xarm7_controller/follow_joint_trajectory/_action/get_result`
    - `/xarm7_controller/follow_joint_trajectory/_action/feedback`
    - `/xarm7_controller/follow_joint_trajectory/_action/status`
"""

# Import necessary ROS 2 libraries and message types
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

# JOINT_NAMES = [
#     'base_link_to_link1',
#     'link1_to_link2',
#     'link2_to_link3',
#     'link3_to_link4',
#     'link4_to_link5',
#     'link5_to_link6',
#     'link6_to_link7',
# ]

# # Six safe waypoints in joint space (radians)
# # For the arm to cycle through them continuously
# WAYPOINTS = [
#     # Pose A — Upward
#     [0.4,  -0.5,  -1.4,  0.1,  -0.5,  -0.2,  0.4],

#     # Pose B — Raised home with a little backward tilt
#     [ 0.0,  -0.9,  -0.3,  -0.4,  0.0,   1.0,  0.0],

#     # Pose C — The 1.5 Radian Sweep (Leftward sweep)
#     [ 1.5,  -0.5,  -0.5,  -0.3,  0.5,   0.8,  0.8],

#     # Pose D — The -1.5 Radian Sweep (Rightward sweep)
#     [-1.5,  -0.5,   0.5,  -0.3, -0.5,   0.8, -0.8],

#     # Pose E — Custom pose from MoveIt image
#     [ 0.593, 0.419,-0.785,-0.192,-0.838, 0.541, 0.279],

#     # Return to home
#     [ 0.0,  -0.9,  -0.3,  -0.4,  0.0,   1.0,  0.0],
# ]


class TrajectoryExecutor(Node):
    """
        Initializes the TrajectoryExecutor node.
        Creates an Action Client targeting the joint trajectory controller and 
        halts initialization until the action server is confirmed to be online.
        """
    def __init__(self):
        super().__init__('trajectory_executor')
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm7_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for trajectory action server...')

        # Block node execution until the action server is available
        self._client.wait_for_server()
        self.get_logger().info('Action server found. Starting trajectory loop.')



    def send_waypoint(self, positions, duration_sec=1.5, max_retries=10, retry_delay=0.5):
        """
        Constructs a joint trajectory goal for a single waypoint and sends it to the action server.
        Waits for the execution to complete before returning.
        
        Args:
            positions (list): A list of 7 joint angles (in radians) corresponding to JOINT_NAMES.
            duration_sec (float): How long the robot should take to reach the waypoint.
            max_retries (int): Number of times to attempt sending the goal if rejected.
            retry_delay (float): Seconds to wait before attempting another send.
        """
        # Fix: Wrapped the goal transmission in a retry loop (max_retries). This prevents the node 
        # from silently failing or skipping waypoints if the controller briefly rejects the goal
        for attempt in range(max_retries):
            goal = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = JOINT_NAMES

            # Create a single trajectory point with zero velocity at the target position
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 7

            # Convert float seconds into ROS Duration message format
            point.time_from_start = Duration(
                sec=int(duration_sec),
                nanosec=int((duration_sec % 1) * 1e9)
            )

            trajectory.points = [point]
            goal.trajectory = trajectory

            # Send the goal asynchronously and block until the server accepts/rejects it
            future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            # Fix: Added explicit handling of goal acceptance. If accepted, it waits for the 
            # physical execution to finish. If rejected, it logs a warning and retries
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                return True

            self.get_logger().warn(f'Goal rejected, retry {attempt+1}/{max_retries}')
            time.sleep(retry_delay)

        self.get_logger().error('Goal rejected after max retries — controller never came up')
        return False



    def run_loop(self):
        """
        Cycle through the predefined WAYPOINTS list, sending them 
        one by one to the robot until the node is shut down using (Ctrl+C).
        """
        self.get_logger().info('Trajectory loop started — Ctrl+C to stop')
        cycle = 0
        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f'Cycle {cycle}')
            for i, wp in enumerate(WAYPOINTS):
                if not rclpy.ok():
                    break

                self.get_logger().info(f'  Moving to waypoint {i+1}/{len(WAYPOINTS)}')
                
                # Set the speed of motion between waypoints
                self.send_waypoint(wp, duration_sec=1.5)

                # Brief pause between waypoints to simulate a real-world robotic operation and allow for any necessary processing
                time.sleep(0.3)

def main():
    rclpy.init()
    node = TrajectoryExecutor()
    node.run_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
