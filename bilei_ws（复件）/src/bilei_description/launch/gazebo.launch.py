import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('bilei_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'bilei.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'room.world'),
            description='Gazebo 世界文件'),

        # 1) Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items(),
        ),

        # 2) 机器人描述
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 3) 生成机器人
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                               '-entity', 'bilei',
                               '-z', '0.08'],
                    output='screen',
                ),
            ],
        ),

        # 4) 超声波 LaserScan -> Range 转换
        Node(
            package='bilei_description',
            executable='range_bridge.py',
            name='range_bridge',
            output='screen',
        ),

        # 5) 占据栅格建图
        Node(
            package='bilei_description',
            executable='occupancy_mapper.py',
            name='occupancy_mapper',
            output='screen',
            condition=IfCondition(LaunchConfiguration('mapping')),
        ),

        # 6) 可选 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
