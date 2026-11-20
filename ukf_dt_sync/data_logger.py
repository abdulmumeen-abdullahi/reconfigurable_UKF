
"""
ROS 2 benchmarking node to evaluate the performance of an Unscented Kalman Filter (UKF).
It subscribes to ground-truth (/joint_states), delayed (/delayed_joint_states), and
UKF-predicted (/predicted_joint_states) joint states alongside NIS diagnostics (/ukf_nis),
sampling at 10 Hz over a fixed 60-second trial window.

At the end of the trial, it computes per-joint Root Mean Square Error (RMSE), percentage
improvement, overall Mean Absolute Error (MAE), and 95% chi-square NIS consistency metrics.
Summary metrics are appended to CSV files while time-series trajectories are exported to
per-delay CSV logs, after which the node triggers rclpy.shutdown().
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import numpy as np
import csv
import time
import os
from std_msgs.msg import Float64MultiArray
from scipy.stats import chi2



class DataLogger(Node):
    def __init__(self):
        """
        Sets up parameters, initializes data storage lists, creates QoS profiles,
        and establishes subscriptions to ground truth, predicted, delayed, and NIS topics
        Starts a timer to periodically sample and log the errors
        """
        super().__init__('data_logger')
        self.declare_parameter('delay_ms', 100)
        self.delay_ms = self.get_parameter('delay_ms').value
       
        # Declare the parameter for ROS 2 to look for in the YAML config
        self.declare_parameter('joint_names', [''])
        self._joint_names = self.get_parameter('joint_names').value

        # Declare and read the parameter for results default path
        self.declare_parameter('results_output_dir', '~/Projects/xarm7_UKF_experiment/src/experiment_results')
        raw_dir = self.get_parameter('results_output_dir').get_parameter_value().string_value
        
        # Expand the '~' to the full absolute path
        self.output_dir = os.path.expanduser(raw_dir)
        
        # AUTO-CREATE THE FOLDER
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Data Logger initialized. Saving results to: {self.output_dir}")

        self.ground_truth = None
        self.predicted = None
        self.delayed = None
        self.errors_kf = []
        self.errors_baseline = []
        self.nis_samples = []
        self.time_series_data = []

        # Best Effort QoS profile to match the UKF predictor's publishing settings,
        # ensuring messages are not dropped due to QoS incompatibility
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_gt = self.create_subscription(
            JointState, '/joint_states', self.cb_gt, 10)

        self.sub_pred = self.create_subscription(
            JointState, '/predicted_joint_states', self.cb_pred, qos_best_effort)

        self.sub_nis = self.create_subscription(
            Float64MultiArray, '/ukf_nis', self.cb_nis, qos_best_effort)

        self.sub_del = self.create_subscription(
            JointState, '/delayed_joint_states', self.cb_del, 10)

        # Timer to sample the state of all topics at 10Hz
        self.timer = self.create_timer(0.1, self.log_sample)

        self.start_time = time.time()
        self.duration = 60.0
        self.done = False
        self.get_logger().info(f'Logger started — {self.delay_ms}ms scenario, 60s trial')

    def _match(self, msg):
        """
        Matches a JointState message's positions to self._joint_names BY NAME,
        regardless of what order (or how many extra joints) the message itself
        contains. Returns None (and warns) if an expected joint is missing.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            return np.array([name_to_pos[n] for n in self._joint_names])
        except KeyError as e:
            self.get_logger().warn(f'Missing joint {e} in message', throttle_duration_sec=5.0)
            return None

    def cb_gt(self, msg):
        self.ground_truth = self._match(msg)

    def cb_pred(self, msg):
        self.predicted = self._match(msg)

    def cb_del(self, msg):
        self.delayed = self._match(msg)

    def cb_nis(self, msg):
        self.nis_samples.append(msg.data)

    def log_sample(self):
        """
        Timer callback that executes at 10Hz.
        Checks if the trial duration is complete. If so, triggers the save_results function.
        Otherwise, if all data streams have been received at least once, it calculates
        and stores the squared errors for both the UKF and the baseline.
        """
        if self.done:
            return
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.save_results()
            self.done = True
            return


        if any(x is None for x in [self.ground_truth, self.predicted, self.delayed]):
            return

        # Flatten arrays into a single row: [time, true1...true7, del1...del7, pred1...pred7]
        row = [elapsed] + list(self.ground_truth) + list(self.delayed) + list(self.predicted)

        self.time_series_data.append(row)
        self.errors_kf.append((self.predicted - self.ground_truth) ** 2)
        self.errors_baseline.append((self.delayed - self.ground_truth) ** 2)

    def save_results(self):
        """
        Calculates RMSE and MAE for the position errors, calculates chi-square bounds
        for the NIS samples, and appends all metrics to their respective CSV files
        """
        if len(self.errors_kf) == 0 or len(self.errors_baseline) == 0:
            self.get_logger().error(
                'No samples collected in this trial — check that '
                '/joint_states, /delayed_joint_states, and /predicted_joint_states '
                'are all actually publishing before the 60s window closes, and that '
                'JOINT_NAMES matches what those topics actually publish '
                '(ros2 topic echo /joint_states --once).'
            )
            rclpy.shutdown()
            return

        errors_kf = np.array(self.errors_kf)
        errors_bl = np.array(self.errors_baseline)
        rmse_ukf = np.sqrt(np.mean(errors_kf, axis=0))
        rmse_bl = np.sqrt(np.mean(errors_bl, axis=0))
        mean_mae_ukf = float(np.mean(np.sqrt(errors_kf)))
        mean_mae_bl = float(np.mean(np.sqrt(errors_bl)))

        filepath = os.path.join(self.output_dir, 'UKF_pred_results.csv')

        # Check if file exists
        file_exists = os.path.isfile(filepath)

        with open(filepath, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['Delay_ms', 'Joint', 'UKF_RMSE_rad', 'Baseline_RMSE_rad', 'Improvement_%'])

            for i in range(len(self._joint_names)):
                imp = ((rmse_bl[i] - rmse_ukf[i]) / rmse_bl[i]) * 100 if rmse_bl[i] > 0 else 0.0
                writer.writerow([
                    self.delay_ms,
                    self._joint_names[i],
                    f'{rmse_ukf[i]:.6f}',
                    f'{rmse_bl[i]:.6f}',
                    f'{imp:.1f}'
                ])
            writer.writerow([self.delay_ms, 'MEAN_MAE', f'{mean_mae_ukf:.6f}', f'{mean_mae_bl:.6f}', ''])

        self.get_logger().info(f'Saved: {filepath}')
        self.get_logger().info(f'UKF RMSE per joint: {np.round(rmse_ukf, 6)}')
        self.get_logger().info(f'Baseline RMSE per joint: {np.round(rmse_bl, 6)}')

        # --- Normalized Innovation Squared (NIS) Statistical Validation ---
        if len(self.nis_samples) > 0:
            nis_arr = np.array(self.nis_samples)          # shape (N, 1 + n_joints)
            nis_total_samples = nis_arr[:, 0]
            nis_joint_samples = nis_arr[:, 1:]

            n_j = len(self._joint_names)
            lo_n, hi_n = chi2.ppf(0.025, df=n_j), chi2.ppf(0.975, df=n_j)
            lo1, hi1 = chi2.ppf(0.025, df=1), chi2.ppf(0.975, df=1)

            mean_nis_total = float(np.mean(nis_total_samples))
            frac_consistent_total = float(
                np.mean((nis_total_samples >= lo_n) & (nis_total_samples <= hi_n))
            )

            mean_nis_joint = np.mean(nis_joint_samples, axis=0)
            frac_consistent_joint = np.mean(
                (nis_joint_samples >= lo1) & (nis_joint_samples <= hi1), axis=0
            )

            nis_filepath = os.path.join(self.output_dir, 'UKF_nis_results.csv')
    
            # Check if file exists
            nis_file_exists = os.path.isfile(nis_filepath)
            

            with open(nis_filepath, 'a', newline='') as f:
                writer = csv.writer(f)
                if not nis_file_exists:
                    writer.writerow(['Delay_ms', 'Channel', 'Mean_NIS', 'Frac_Within_95CI'])
                for i in range(n_j):
                    writer.writerow([
                        self.delay_ms, self._joint_names[i],
                        f'{mean_nis_joint[i]:.4f}', f'{frac_consistent_joint[i]:.3f}'
                    ])
                writer.writerow([
                    self.delay_ms, f'TOTAL_{n_j}DOF',
                    f'{mean_nis_total:.4f}', f'{frac_consistent_total:.3f}'
                ])

            self.get_logger().info(f'NIS saved: {nis_filepath}')
            self.get_logger().info(f'Mean NIS per joint: {np.round(mean_nis_joint, 4)}')
        else:
            self.get_logger().warn('No NIS samples collected — check /ukf_nis is publishing.')

        # Define the subfolder path inside your base output directory
        timeseries_dir = os.path.join(self.output_dir, 'UKF_timeseries_plot')

        # Ensure this subfolder exists
        os.makedirs(timeseries_dir, exist_ok=True)

        # Join the subfolder with dynamic filename
        ts_filepath = os.path.join(timeseries_dir, f'{self.delay_ms}ms.csv')
        
        # Create headers dynamically based on JOINT_NAMES
        headers = ['time_sec']
        headers += [f'{name}_true' for name in self._joint_names]
        headers += [f'{name}_delayed' for name in self._joint_names]
        headers += [f'{name}_pred' for name in self._joint_names]

        with open(ts_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(self.time_series_data)
            
        self.get_logger().info(f'Time-series saved: {ts_filepath}')

        # exit the process on its own, which the
        # launch file's auto-shutdown event handler depends on.
        rclpy.shutdown()


def main():
    """
    Spins up the DataLogger node to collect
    60 seconds of data, and handles shutdowns.
    """
    rclpy.init()
    node = DataLogger()
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
