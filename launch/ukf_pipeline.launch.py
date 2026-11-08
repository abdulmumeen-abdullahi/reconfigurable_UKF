# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
# from launch.event_handlers import OnProcessExit
# from launch.events import Shutdown
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch.conditions import IfCondition
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     #### ===> Require the user to pass their specific config file
#     config_file_arg = DeclareLaunchArgument(
#         'config_file'
#     )
#     config_file = LaunchConfiguration('config_file')

#     delay_ms_arg = DeclareLaunchArgument(
#         'delay_ms',
#         default_value='100',
#         description='Latency in milliseconds to inject'
#     )
#     delay_ms = LaunchConfiguration('delay_ms')

#     # Generic TF Bridges
#     predicted_tf_bridge = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='predicted_tf_bridge',
#         arguments=['0', '0', '0', '0', '0', '0', 'world', 'predicted/world'] 
#     )

#     measured_tf_bridge = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='measured_tf_bridge',
#         arguments=['0', '0', '0', '0', '0', '0', 'world', 'measured/world']
#     )


#     use_rviz_arg = DeclareLaunchArgument(
#     'use_rviz', default_value='true',
#     description='Launch RViz with the digital-twin (ghost) view'
#     )
#     rviz_config_arg = DeclareLaunchArgument(
#         'rviz_config',
#         default_value=PathJoinSubstitution(
#             [FindPackageShare('ukf_dt_sync'), 'rviz', 'ukf_ghosts.rviz']
#         ),
#         description='RViz config file for the twin view'
#     )
#     use_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time', default_value='true',
#         description='Use /clock (simulation) time'
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', LaunchConfiguration('rviz_config')],
#         parameters=[{
#             'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
#         }],
#         output='screen',
#         condition=IfCondition(LaunchConfiguration('use_rviz')),
#     )

#     # ============================
#     # Core Architecture Nodes
#     injector_node = Node(
#         package='ukf_dt_sync',
#         executable='injector',
#         name='injector',
#         parameters=[{'delay_ms': delay_ms}]
#     )

#     data_logger_node = Node(
#         package='ukf_dt_sync',
#         executable='data_logger',
#         name='data_logger',
#         parameters=[config_file, {'delay_ms': delay_ms}]
#     )

#     trajectory_executor_node = Node(
#         package='ukf_dt_sync',
#         executable='trajectory_executor',
#         name='trajectory_executor',
#         parameters=[config_file]
#     )

#     ukf_predictor_node = Node(
#         package='ukf_dt_sync',
#         executable='ukf_predictor',
#         name='ukf_predictor',
#         parameters=[
#             config_file,
#             {
#                 'use_sim_time': True,
#                 'delay_ms': delay_ms,
#             }
#         ]
#     )

#     shutdown_on_logger_exit = RegisterEventHandler(
#         OnProcessExit(
#             target_action=data_logger_node,
#             on_exit=[EmitEvent(event=Shutdown())],
#         )
#     )

#     return LaunchDescription([
#         config_file_arg,
#         delay_ms_arg,
#         use_rviz_arg,
#         rviz_config_arg,
#         use_sim_time_arg,
#         predicted_tf_bridge,
#         measured_tf_bridge,
#         injector_node,
#         data_logger_node,
#         trajectory_executor_node,
#         ukf_predictor_node,
#         rviz_node,
#         shutdown_on_logger_exit,
#     ])




import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock (simulation) time'
    )

    # Robot Description for Ghosts
    robot_xacro_arg = DeclareLaunchArgument(
        'robot_xacro',
        default_value=PathJoinSubstitution(
            [FindPackageShare('ufactory_xarm7_description'), 'urdf', 'xarm7.urdf.xacro']),
        description='Robot xacro used for the ghost models'
    )

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

    # Robot State Publishers for Ghosts
    predicted_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_predicted',
        namespace='predicted',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('robot_xacro')]), value_type=str),
            'frame_prefix': 'predicted/',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('joint_states', '/predicted_joint_states')]
    )

    measured_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_measured',
        namespace='measured',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('robot_xacro')]), value_type=str),
            'frame_prefix': 'measured/',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('joint_states', '/delayed_joint_states')] # Ensure this matches your actual delayed topic
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz with the digital-twin (ghost) view'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('ukf_dt_sync'), 'rviz', 'ukf_ghosts.rviz']
        ),
        description='RViz config file for the twin view'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
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
        use_sim_time_arg,
        robot_xacro_arg,
        use_rviz_arg,
        rviz_config_arg,
        predicted_tf_bridge,
        measured_tf_bridge,
        predicted_rsp,
        measured_rsp,
        injector_node,
        data_logger_node,
        trajectory_executor_node,
        ukf_predictor_node,
        rviz_node,
        shutdown_on_logger_exit,
    ])