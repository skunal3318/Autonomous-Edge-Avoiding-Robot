import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    namePackage = 'edge_robot_description'
    robotName = 'edge_robot'

    model_path = os.path.join(
        get_package_share_directory(namePackage),
        'urdf',
        'model.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -v -v4 empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotName,
            '-topic', 'robot_description'
        ],
        output='screen'
    )


    rviz_config = os.path.join(
        get_package_share_directory(namePackage),
        'rviz',
        'urdf_config.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    bridge_params = os.path.join(
        get_package_share_directory('edge_robot_bringup'),
        'config',
        'gazebo_bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazeboLaunch,
        robot_state_publisher,
        spawn_robot,
        # joint_state_gui,
        bridge,
        rviz
    ])