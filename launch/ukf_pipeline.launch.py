import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #### ===> Require the user to pass their specific config file
    config_file_arg = DeclareLaunchArgument(
        'config_file'
    )
    config_file = LaunchConfiguration('config_file')

    delay_ms_arg = DeclareLaunchArgument(
        'delay_ms',
        default_value='100',
        description='Latency in milliseconds to inject'
    )
    delay_ms = LaunchConfiguration('delay_ms')

    # Generic TF Bridges
    predicted_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='predicted_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'predicted/world'] 
    )

    measured_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='measured_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'measured/world']
    )

    # ============================
    # Core Architecture Nodes
    injector_node = Node(
        package='ukf_dt_sync',
        executable='injector',
        name='injector',
        parameters=[{'delay_ms': delay_ms}]
    )

    data_logger_node = Node(
        package='ukf_dt_sync',
        executable='data_logger',
        name='data_logger',
        parameters=[config_file, {'delay_ms': delay_ms}]
    )

    trajectory_executor_node = Node(
        package='ukf_dt_sync',
        executable='trajectory_executor',
        name='trajectory_executor',
        parameters=[config_file]
    )

    ukf_predictor_node = Node(
        package='ukf_dt_sync',
        executable='ukf_predictor',
        name='ukf_predictor',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'delay_ms': delay_ms,
            }
        ]
    )

    shutdown_on_logger_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=data_logger_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        config_file_arg,
        delay_ms_arg,
        predicted_tf_bridge, 
        measured_tf_bridge,
        injector_node,
        data_logger_node,
        trajectory_executor_node,
        ukf_predictor_node,
        shutdown_on_logger_exit,
    ])