"""
This node implements an Unscented Kalman Filter (UKF) to estimate the joint states 
(position and velocity) of a robotic arm, specifically handling delayed measurements 
by synchronizing with historical control commands. 

It uses a control-aware process model with a velocity ramp to accurately simulate 
real-world acceleration and deceleration. When a delayed measurement arrives, the 
filter piecewise-integrates the past trajectory using the exact sequence of 
historical commands, applies the measurement update, and calculates filter 
consistency (NIS). A high-frequency timer then predicts this estimate to the 
current wall-time for downstream controllers.

- Subscribes to:
  - `/delayed_joint_states` (sensor_msgs/JointState): The delayed, noisy measurements.
  - `/xarm7_controller/controller_state` (control_msgs/JointTrajectoryControllerState): To read the commanded reference velocities.
- Publishes to:
  - `/predicted_joint_states` (sensor_msgs/JointState): The real-time, zero-delay state estimate.
  - `/ukf_nis` (std_msgs/Float64MultiArray): The Normalized Innovation Squared for filter tuning validation.
"""

# Import necessary libraries
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState  # For control effort
from std_msgs.msg import Float64MultiArray
from collections import deque # To handle delayed measurements
from functools import partial
from ukf_dt_sync.ukf import UnscentedKalmanFilter # Import the UKF implementation from ukf.py



def _velocity_ramp(v_curr, u, dt, tau):
    """
    Exact integration of a first-order velocity relaxation: dv/dt = (u - v) / tau.
    Returns the average velocity over the time step and the final velocity.
    to ensure that position integrates correctly rather than just predicting from an endpoint.
    """
    if tau <= 0.0 or dt <= 0.0:
        v_next = u
        avg_v = 0.5 * (v_curr + u) if dt > 0.0 else v_curr
        return avg_v, v_next

    alpha = np.exp(-dt / tau)
    v_next = u + (v_curr - u) * alpha
    avg_v = u + (v_curr - u) * (tau / dt) * (1.0 - alpha)
    return avg_v, v_next



def control_aware_fx(x, dt, u=None, tau=0.15):
    """
    Control-Aware Process Model (fx).
    Fix: Upgraded to a Ramp-aware control process model. Instead of jumping straight 
    to the commanded velocity or assuming constant velocity, 
    this blends velocity toward the command 'u' with a time constant 'tau'. 
    This captures the actual acceleration/deceleration ramps executed by the physical robot.
    """
    n = len(x) // 2
    x_next = np.copy(x)

    if u is not None and len(u) == n:
        avg_v, v_next = _velocity_ramp(x[n:], u, dt, tau)
        x_next[:n] += avg_v * dt
        x_next[n:] = v_next
    else:
        # Fallback to constant velocity if no commands are received
        x_next[:n] += x[n:] * dt

    return x_next



def position_only_hx(x):
    """
    Measurement Model (hx) that strips the velocities from the position measured by encoder and returns 
    only the position half of the state vector.
    """
    n = len(x) // 2
    return x[:n]


class UKFPredictorNode(Node):
    def __init__(self):
        """
        Sets up parameters, the UKF instance, dynamic covariance matrices, 
        publishers, subscribers, and the command history buffer.
        """
        super().__init__('ukf_predictor_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # self.declare_parameter('num_joints', 7)
        self.declare_parameter('dof', Parameter.Type.INTEGER)
        self.declare_parameter('predict_rate_hz', 100.0)

        # Adjusted Alpha, Beta, and expanded Kappa for a 14-dimensional state spread.
        # to prevent negative weights from blowing the covariance to infinity.
        self.declare_parameter('ukf.alpha', 1e-3)  
        self.declare_parameter('ukf.beta', 2.0)
        self.declare_parameter('ukf.kappa', 1.0) 

        # Buffer to handle delayed measurements by tracking past commands
        self.u_history = deque()

        self._last_publish_time = None
        self._startup_check_timer = self.create_timer(2.0, self._check_alive)
        
        self.declare_parameter('ukf.process_noise_std', 0.2) 
        self.declare_parameter('ukf.measurement_noise_std', 0.0316) 
        self.declare_parameter('measurement_topic', '/delayed_joint_states')
        self.declare_parameter('prediction_topic', '/predicted_joint_states')
        self.declare_parameter('controller_state_topic', '/xarm7_controller/controller_state')

        # Read the delay_ms parameter passed by the launch file
        self.declare_parameter('delay_ms', 100)

        delay_ms_val = self.get_parameter('delay_ms').value
        # Calculate max latency: delay_ms (converted to seconds) + 1.0 second buffer
        self.max_expected_latency = (delay_ms_val / 1000.0) + 1.0
        

        self.n_j = self.get_parameter('dof').value
        rate = self.get_parameter('predict_rate_hz').value
        alpha = self.get_parameter('ukf.alpha').value
        beta = self.get_parameter('ukf.beta').value
        kappa = self.get_parameter('ukf.kappa').value
        q_std = self.get_parameter('ukf.process_noise_std').value

        # For scaling with velocity
        self.base_q_std = q_std 
        self.q_std = q_std 
        # For scaling with velocity: Declare and fetch the v_scale parameter (how much Q increases per 1 rad/s of speed)
        self.declare_parameter('ukf.v_scale', 0.5) 
        self.v_scale = self.get_parameter('ukf.v_scale').value


        r_std = self.get_parameter('ukf.measurement_noise_std').value
        self.q_std = q_std
        self.declare_parameter('ukf.velocity_time_constant', 0.15)
        self.vel_tau = self.get_parameter('ukf.velocity_time_constant').value
        

        self.dim_x = 2 * self.n_j
        self.dim_z = self.n_j
        self.dt = 1.0 / rate

        # Initialize the UKF, using partial to pass the tau parameter into the process model
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, dim_z=self.dim_z,
            fx=partial(control_aware_fx, tau=self.vel_tau),
            hx=position_only_hx,
            alpha=alpha, beta=beta, kappa=kappa,
        )
        
        # Kinematically coupled Q matrix initialization using discrete-time kinematic variance formulation.
        self.ukf.Q = np.zeros((self.dim_x, self.dim_x))
        q_var = q_std ** 2
        for i in range(self.n_j):
            self.ukf.Q[i, i] = (self.dt**3 / 3) * q_var
            self.ukf.Q[i, self.n_j + i] = (self.dt**2 / 2) * q_var
            self.ukf.Q[self.n_j + i, i] = (self.dt**2 / 2) * q_var
            self.ukf.Q[self.n_j + i, self.n_j + i] = self.dt * q_var
        
        self.ukf.R = np.eye(self.dim_z) * (r_std ** 2)
        self.ukf.x = np.zeros(self.dim_x)
        self.ukf.P = np.eye(self.dim_x) * 0.01

        self._initialized = False
        self._joint_names = [f'joint_{i + 1}' for i in range(self.n_j)]
        self.ukf_time = 0.0 
        
        # State variable to hold the latest commanded velocities (u)
        self.latest_u = np.zeros(self.n_j)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        meas_topic = self.get_parameter('measurement_topic').value
        pred_topic = self.get_parameter('prediction_topic').value
        ctrl_topic = self.get_parameter('controller_state_topic').value
        self.meas_sub = self.create_subscription(JointState, meas_topic, self._measurement_cb, qos)
        
        # Subscribe to the controller state to read commanded effort dynamically
        self.ctrl_sub = self.create_subscription(JointTrajectoryControllerState, ctrl_topic, self._control_cb, qos)
        
        self.pred_pub = self.create_publisher(JointState, pred_topic, qos)
        self.nis_pub = self.create_publisher(Float64MultiArray, '/ukf_nis', qos)
        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(f'Standard UKF started: rate={rate} Hz')



    def _control_cb(self, msg):
        """
        Callback for controller state
        """
        ref = getattr(msg, 'reference', None) or getattr(msg, 'desired', None)
        if ref is None or len(ref.velocities) != self.n_j:
            return
        new_u = np.array(ref.velocities)
        t_now = self.get_clock().now().nanoseconds / 1e9

        # Extracts commanded velocities and buffers them chronologically (`u_history`).
        # Only logs new segments when the command actually changes to keep the buffer compact
        if self.u_history and np.allclose(new_u, self.u_history[-1][1], atol=1e-6):
            self.latest_u = new_u
            return

        self.u_history.append((t_now, new_u))
        self.latest_u = new_u

        # Safely prune data older than dynamically calculated latency
        cutoff = t_now - self.max_expected_latency
        while self.u_history and self.u_history[0][0] < cutoff:
            self.u_history.popleft()



    # To fetch the commanded velocity that was active at a specific historical time 't'
    def _u_at(self, t):
        for t_u, u_val in reversed(self.u_history):
            if t_u <= t:
                return u_val
        if self.u_history:
            return self.u_history[0][1]
        return self.latest_u

   

    def _u_segments(self, t_start, t_end):
        """
        Fix: Splits the time interval [t_start, t_end] at every recorded command change.
        Returns a list of sub-intervals so the UKF can piecewise-integrate using the 
        exact command that was active during that specific window, rather than assuming 
        a single command spanned the entire delayed gap.
        """
        if t_end <= t_start:
            return []

        seg_u = self._u_at(t_start)
        changes = sorted(
            ((t, u) for (t, u) in self.u_history if t_start < t < t_end),
            key=lambda c: c[0]
        )

        segments = []
        seg_start = t_start
        for t_change, u_change in changes:
            segments.append((seg_start, t_change, seg_u))
            seg_start, seg_u = t_change, u_change
        segments.append((seg_start, t_end, seg_u))
        return segments



    def _measurement_cb(self, msg):
        """
        Triggered when a delayed measurement arrives. It calculates the time gap, 
        advances the UKF state through history to match the measurement timestamp 
        (using piecewise prediction), applies the update, and logs the NIS.
        """
        if len(msg.position) < self.n_j:
            self.get_logger().warn(f'Got {len(msg.position)} positions, expected {self.n_j}, skipping', throttle_duration_sec=2.0)
            return
        t_meas = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        z = np.array(msg.position[:self.n_j])

        if not self._initialized:
            self.ukf.x[:self.n_j] = z
            self.ukf.x[self.n_j:] = self.latest_u
            self.ukf_time = t_meas
            self._joint_names = list(msg.name[:self.n_j])
            self._initialized = True
            return

        dt_ukf = t_meas - self.ukf_time
        if dt_ukf <= 0.0:
            return
        if dt_ukf > self.max_expected_latency:
            self.get_logger().warn(f'Measurement gap too large (dt={dt_ukf:.2f}s, max expected={self.max_expected_latency}s). Re-initializing.')
            self._initialized = False
            return

        # enable dynamic process noise (Q) & piecewise integration.
        # Iterates through historical command segments. For each segment, it recalculates Q 
        # based on the exact segment duration (seg_dt) and predicts the state forward.
        q_var = self.q_std ** 2
        for seg_start, seg_end, u_seg in self._u_segments(self.ukf_time, t_meas):
            seg_dt = seg_end - seg_start
            if seg_dt <= 0.0:
                continue

            # ADAPTIVE Q SCALING
            # Calculate the overall speed of the arm (Euclidean norm of velocities)
            velocity_magnitude = np.linalg.norm(u_seg)
            
            # Scale the standard deviation based on speed
            dynamic_q_std = self.base_q_std + (self.v_scale * velocity_magnitude)
            dynamic_q_std = min(dynamic_q_std, 0.5) # PREVENTS MATH EXPLOSIONS
            # Square it to get the variance for the Q matrix
            q_var = dynamic_q_std ** 2

            # Populate the kinematically coupled Q matrix
            Q = np.zeros((self.dim_x, self.dim_x))
            for i in range(self.n_j):
                Q[i, i] = (seg_dt**3 / 3) * q_var
                Q[i, self.n_j + i] = (seg_dt**2 / 2) * q_var
                Q[self.n_j + i, i] = (seg_dt**2 / 2) * q_var
                Q[self.n_j + i, self.n_j + i] = seg_dt * q_var


            # Apply Q and advance the filter state forward in time
            self.ukf.Q = Q
            self.ukf.predict(seg_dt, u=u_seg)

    
        # Apply the measurement update
        self.ukf.update(z)
        self.ukf_time = t_meas

        # NIS logging
        # Computes the Normalized Innovation Squared for the total state 
        # and per-joint to validate if the UKF tuning (Q and R matrices) is statistically consistent
        nis_total = float(self.ukf.y @ np.linalg.solve(self.ukf.S, self.ukf.y))
        nis_per_joint = (self.ukf.y ** 2) / np.diag(self.ukf.S)
        nis_msg = Float64MultiArray()
        nis_msg.data = [nis_total] + nis_per_joint.tolist()  # index 0 = total, 1..7 = joint1..7
        self.nis_pub.publish(nis_msg)



    # publish timer
    def _timer_cb(self):
        if not self._initialized:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        dt_extrapolate = now - self.ukf_time

        
        max_extrapolate = self.max_expected_latency + 0.5 
        if dt_extrapolate < 0.0 or dt_extrapolate > max_extrapolate:
            dt_extrapolate = 0.0

        x_now = np.copy(self.ukf.x)

        if dt_extrapolate > 0:
            # Real-time Prediction. Safely projects the internal UKF state, which is currently 
            # at the past at `ukf_time`, forward to the current wall-time using the velocity 
            # ramp model, providing a zero-delay estimate for downstream controllers
            avg_v, v_next = _velocity_ramp(
                x_now[self.n_j:], self.latest_u, dt_extrapolate, self.vel_tau
            )
            x_now[:self.n_j] += avg_v * dt_extrapolate
            x_now[self.n_j:] = v_next

        self._publish(x_now)



    # Formats the predicted (extrapolated) state array into a JointState message and publishes it
    def _publish(self, x):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self._joint_names
        out.position = x[:self.n_j].tolist()
        out.velocity = x[self.n_j:].tolist()
        self.pred_pub.publish(out)
        self._last_publish_time = self.get_clock().now()

    
    
    # Checker to ensure the filter hasn't silently failed to publish
    def _check_alive(self):
        if self._initialized and self._last_publish_time is None:
            self.get_logger().error('Filter initialized but has never published: predictor is likely dead')
        self._startup_check_timer.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = UKFPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()