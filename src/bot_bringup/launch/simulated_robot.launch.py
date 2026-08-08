import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bot_description_dir = get_package_share_directory('bot_description')
    bot_controller_dir = get_package_share_directory('bot_controller')

    xacro_file = os.path.join(bot_description_dir, 'urdf', 'bot.urdf.xacro')
    world_file = os.path.join(bot_description_dir, 'worlds', 'edge_world.sdf')
    rviz_config_file = os.path.join(bot_description_dir, 'rviz', 'edge_avoider.rviz')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with the scan/TF/state debug view')

    # 1. Start Gazebo Sim 8 with our world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. robot_state_publisher: publishes TF from the URDF + joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. Spawn the robot into the running world
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'edge_bot',
            '-topic', 'robot_description',
            '-z', '0.10'
        ],
        output='screen'
    )

    # 4. ROS <-> Gazebo bridge (cmd_vel, odom, tf, scan, joint_states, clock)
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, 'launch', 'bridge.launch.py')
        )
    )

    # 5. Edge-avoidance behavior node
    edge_avoider = Node(
        package='bot_script',
        executable='edge_avoider',
        output='screen'
    )

    # 6. RViz2 debug view: robot model, TF, /scan, steering marker, state
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        edge_avoider,
        rviz,
    ])
