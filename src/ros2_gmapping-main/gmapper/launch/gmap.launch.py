import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gmapper_share = get_package_share_directory('gmapper')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    rviz_config = PathJoinSubstitution([gmapper_share, 'rviz', 'gmapper.rviz'])
    nav2_params = PathJoinSubstitution([gmapper_share, 'config', 'nav2_params.yaml'])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        Node(
            package='gmapper',
            executable='gmap',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        navigation,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])